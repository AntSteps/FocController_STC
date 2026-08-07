#include "calibration_service.h"
#include "param_storage.h"
#include "calibration.h"
#include "encoder.h"
#include "svpwm.h"
#include "motor.h"
#include "timer.h"
#include "uart_controller.h"
#include "telemetry_service.h"
#include <string.h>

#define CALIB_SERVICE_TWO_PI          6.28318530718f
#define CALIB_SERVICE_VOLTAGE         0.50f
#define CALIB_SERVICE_STEP_RAD        0.05f
#define CALIB_SERVICE_STEP_DELAY_MS   20ul
#define CALIB_SERVICE_LOCK_DELAY_MS   1000ul
#define CALIB_SERVICE_SETTLE_DELAY_MS 500ul
#define CALIB_SERVICE_MIN_RMS_RAD     0.001f

typedef enum
{
    CALIB_FLOW_IDLE = 0,
    CALIB_FLOW_LOCK_WAIT = 1,
    CALIB_FLOW_DIR_SWEEP_SET = 2,
    CALIB_FLOW_DIR_SWEEP_WAIT = 3,
    CALIB_FLOW_DIR_SETTLE_WAIT = 4,
    CALIB_FLOW_FIT_SWEEP_SET = 5,
    CALIB_FLOW_FIT_SWEEP_WAIT = 6,
    CALIB_FLOW_REVERSE_SWEEP_SET = 7,
    CALIB_FLOW_REVERSE_SWEEP_WAIT = 8,
    CALIB_FLOW_FINALIZE = 9
} CalibrationFlowState;

typedef struct
{
    uint8_t calib_state;
    uint8_t progress;
    uint8_t flow_state;
    uint32_t deadline_ms;
    int pole_pairs;
    Motor_Calibration result;
    float theta;
    float theta_end;
    float start_mech;
    float end_mech;
    float sum_diff;
    float sum_diff_sq;
    float sum_sin_1;
    float sum_cos_1;
    float sum_sin_2;
    float sum_cos_2;
    int sample_count;
    int8_t sweep_direction;
} CalibrationServiceContext;

static CalibrationServiceContext s_calib_ctx;

static float CalibrationService_NormalizePmPi(float angle)
{
    while (angle <= -3.14159265359f)
    {
        angle += CALIB_SERVICE_TWO_PI;
    }
    while (angle > 3.14159265359f)
    {
        angle -= CALIB_SERVICE_TWO_PI;
    }
    return angle;
}

static float CalibrationService_Normalize0To2Pi(float angle)
{
    while (angle < 0.0f)
    {
        angle += CALIB_SERVICE_TWO_PI;
    }
    while (angle >= CALIB_SERVICE_TWO_PI)
    {
        angle -= CALIB_SERVICE_TWO_PI;
    }
    return angle;
}

static void CalibrationService_StopMotor(void)
{
    SVPWM_Update(0.0f, 0.0f);
    Motor_SetControlMode(MOTOR_CONTROL_MODE_STOP);
    Motor_SetSuspended(FALSE);
}

static void CalibrationService_SetFailure(uint8_t fail_state)
{
    s_calib_ctx.calib_state = fail_state;
    s_calib_ctx.flow_state = CALIB_FLOW_IDLE;
    CalibrationService_StopMotor();
}

static void CalibrationService_UpdateSweepProgress(uint8_t start_progress,
                                                   uint8_t end_progress,
                                                   float theta,
                                                   float theta_end)
{
    float ratio;

    if (theta_end <= 0.0f)
    {
        s_calib_ctx.progress = end_progress;
        return;
    }

    ratio = theta / theta_end;
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    else if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }

    s_calib_ctx.progress = (uint8_t)(start_progress + (uint8_t)((float)(end_progress - start_progress) * ratio));
}

void CalibrationService_Init(void)
{
    memset(&s_calib_ctx, 0, sizeof(s_calib_ctx));
    s_calib_ctx.calib_state = PROTOCOL_CALIB_STATE_IDLE;
}

ProtocolResultCode CalibrationService_Start(void)
{
    ParamStorage_Data_t *params;

    if (CalibrationService_IsBusy())
    {
        return PROTOCOL_RESULT_BUSY;
    }

    params = ParamStorage_Get();
    if (params->motor_pole_pairs <= 0)
    {
        return PROTOCOL_RESULT_INVALID_VALUE;
    }

    memset(&s_calib_ctx, 0, sizeof(s_calib_ctx));
    s_calib_ctx.calib_state = PROTOCOL_CALIB_STATE_RUNNING;
    s_calib_ctx.progress = 1u;
    s_calib_ctx.flow_state = CALIB_FLOW_LOCK_WAIT;
    s_calib_ctx.deadline_ms = Timer_GetTickMs() + CALIB_SERVICE_LOCK_DELAY_MS;
    s_calib_ctx.pole_pairs = params->motor_pole_pairs;
    s_calib_ctx.result.pole_pairs = params->motor_pole_pairs;
    s_calib_ctx.result.dir = 1;

    Motor_SetControlMode(MOTOR_CONTROL_MODE_STOP);
    Motor_SetSuspended(TRUE);
    SVPWM_Update(0.0f, CALIB_SERVICE_VOLTAGE);

    return PROTOCOL_RESULT_OK;
}

BOOL CalibrationService_IsBusy(void)
{
    return (BOOL)(s_calib_ctx.calib_state == PROTOCOL_CALIB_STATE_RUNNING);
}

uint8_t CalibrationService_GetState(void)
{
    return s_calib_ctx.calib_state;
}

uint8_t CalibrationService_GetProgress(void)
{
    return s_calib_ctx.progress;
}

void CalibrationService_Poll(void)
{
    uint32_t now_ms;

    if (s_calib_ctx.calib_state != PROTOCOL_CALIB_STATE_RUNNING)
    {
        return;
    }

    now_ms = Timer_GetTickMs();
    if (now_ms < s_calib_ctx.deadline_ms)
    {
        return;
    }

    switch (s_calib_ctx.flow_state)
    {
    case CALIB_FLOW_LOCK_WAIT:
        Encoder_Update();
        if (!Encoder_Get_DataGood())
        {
            CalibrationService_SetFailure(PROTOCOL_CALIB_STATE_FAIL_ENCODER);
            return;
        }
        s_calib_ctx.start_mech = Encoder_Get_Angle();
        s_calib_ctx.theta = 0.0f;
        s_calib_ctx.progress = 5u;
        s_calib_ctx.flow_state = CALIB_FLOW_DIR_SWEEP_SET;
        break;

    case CALIB_FLOW_DIR_SWEEP_SET:
        if (s_calib_ctx.theta > CALIB_SERVICE_TWO_PI)
        {
            s_calib_ctx.deadline_ms = now_ms + CALIB_SERVICE_SETTLE_DELAY_MS;
            s_calib_ctx.progress = 30u;
            s_calib_ctx.flow_state = CALIB_FLOW_DIR_SETTLE_WAIT;
            break;
        }

        SVPWM_Update(s_calib_ctx.theta, CALIB_SERVICE_VOLTAGE);
        CalibrationService_UpdateSweepProgress(5u, 30u, s_calib_ctx.theta, CALIB_SERVICE_TWO_PI);
        s_calib_ctx.deadline_ms = now_ms + CALIB_SERVICE_STEP_DELAY_MS;
        s_calib_ctx.flow_state = CALIB_FLOW_DIR_SWEEP_WAIT;
        break;

    case CALIB_FLOW_DIR_SWEEP_WAIT:
        s_calib_ctx.theta += CALIB_SERVICE_STEP_RAD;
        s_calib_ctx.flow_state = CALIB_FLOW_DIR_SWEEP_SET;
        break;

    case CALIB_FLOW_DIR_SETTLE_WAIT:
        Encoder_Update();
        if (!Encoder_Get_DataGood())
        {
            CalibrationService_SetFailure(PROTOCOL_CALIB_STATE_FAIL_ENCODER);
            return;
        }

        s_calib_ctx.end_mech = Encoder_Get_Angle();
        {
            float mech_delta;

            mech_delta = s_calib_ctx.end_mech - s_calib_ctx.start_mech;
            if (mech_delta > (CALIB_SERVICE_TWO_PI * 0.5f))
            {
                mech_delta -= CALIB_SERVICE_TWO_PI;
            }
            if (mech_delta < -(CALIB_SERVICE_TWO_PI * 0.5f))
            {
                mech_delta += CALIB_SERVICE_TWO_PI;
            }

            s_calib_ctx.result.dir = (mech_delta < 0.0f) ? -1 : 1;
        }

        s_calib_ctx.theta = 0.0f;
        s_calib_ctx.theta_end = CALIB_SERVICE_TWO_PI * (float)s_calib_ctx.pole_pairs;
        s_calib_ctx.sum_diff = 0.0f;
        s_calib_ctx.sum_diff_sq = 0.0f;
        s_calib_ctx.sum_sin_1 = 0.0f;
        s_calib_ctx.sum_cos_1 = 0.0f;
        s_calib_ctx.sum_sin_2 = 0.0f;
        s_calib_ctx.sum_cos_2 = 0.0f;
        s_calib_ctx.sample_count = 0;
        s_calib_ctx.sweep_direction = 1;
        s_calib_ctx.progress = 35u;
        s_calib_ctx.flow_state = CALIB_FLOW_FIT_SWEEP_SET;
        break;

    case CALIB_FLOW_FIT_SWEEP_SET:
        if (s_calib_ctx.theta > s_calib_ctx.theta_end)
        {
            s_calib_ctx.theta = s_calib_ctx.theta_end;
            s_calib_ctx.sweep_direction = -1;
            s_calib_ctx.progress = 65u;
            s_calib_ctx.deadline_ms = now_ms + CALIB_SERVICE_SETTLE_DELAY_MS;
            s_calib_ctx.flow_state = CALIB_FLOW_REVERSE_SWEEP_SET;
            break;
        }

        SVPWM_Update(s_calib_ctx.theta, CALIB_SERVICE_VOLTAGE);
        CalibrationService_UpdateSweepProgress(35u, 65u, s_calib_ctx.theta, s_calib_ctx.theta_end);
        s_calib_ctx.deadline_ms = now_ms + CALIB_SERVICE_STEP_DELAY_MS;
        s_calib_ctx.flow_state = CALIB_FLOW_FIT_SWEEP_WAIT;
        break;

    case CALIB_FLOW_FIT_SWEEP_WAIT:
        Encoder_Update();
        if (!Encoder_Get_DataGood())
        {
            CalibrationService_SetFailure(PROTOCOL_CALIB_STATE_FAIL_ENCODER);
            return;
        }

        {
            float mech_rad;
            float elec_raw;
            float diff;
            float theta_ref;

            mech_rad = Encoder_Get_Angle();
            elec_raw = mech_rad * (float)s_calib_ctx.result.dir * (float)s_calib_ctx.pole_pairs;
            elec_raw = CalibrationService_Normalize0To2Pi(elec_raw);
            theta_ref = CalibrationService_Normalize0To2Pi(s_calib_ctx.theta);
            diff = CalibrationService_NormalizePmPi(elec_raw - theta_ref);

            s_calib_ctx.sum_diff += diff;
            s_calib_ctx.sum_diff_sq += diff * diff;
            s_calib_ctx.sum_sin_1 += diff * (float)sin((double)mech_rad);
            s_calib_ctx.sum_cos_1 += diff * (float)cos((double)mech_rad);
            s_calib_ctx.sum_sin_2 += diff * (float)sin((double)(2.0f * mech_rad));
            s_calib_ctx.sum_cos_2 += diff * (float)cos((double)(2.0f * mech_rad));
            s_calib_ctx.sample_count++;

            {
                ProtocolReply sample_reply;
                TelemetryService_BuildCalibSampleReply(&sample_reply,
                                                       (uint16_t)s_calib_ctx.sample_count,
                                                       s_calib_ctx.sweep_direction,
                                                       s_calib_ctx.theta,
                                                       mech_rad,
                                                       diff);
                (void)ProtocolController_SendReply(&sample_reply);
            }
        }

        s_calib_ctx.theta += CALIB_SERVICE_STEP_RAD;
        s_calib_ctx.flow_state = CALIB_FLOW_FIT_SWEEP_SET;
        break;

    case CALIB_FLOW_REVERSE_SWEEP_SET:
        if (s_calib_ctx.theta < 0.0f)
        {
            s_calib_ctx.flow_state = CALIB_FLOW_FINALIZE;
            break;
        }

        SVPWM_Update(s_calib_ctx.theta, CALIB_SERVICE_VOLTAGE);
        CalibrationService_UpdateSweepProgress(65u,
                                               95u,
                                               s_calib_ctx.theta_end - s_calib_ctx.theta,
                                               s_calib_ctx.theta_end);
        s_calib_ctx.deadline_ms = now_ms + CALIB_SERVICE_STEP_DELAY_MS;
        s_calib_ctx.flow_state = CALIB_FLOW_REVERSE_SWEEP_WAIT;
        break;

    case CALIB_FLOW_REVERSE_SWEEP_WAIT:
        Encoder_Update();
        if (!Encoder_Get_DataGood())
        {
            CalibrationService_SetFailure(PROTOCOL_CALIB_STATE_FAIL_ENCODER);
            return;
        }

        {
            float mech_rad;
            float elec_raw;
            float theta_ref;
            float diff;

            mech_rad = Encoder_Get_Angle();
            elec_raw = mech_rad * (float)s_calib_ctx.result.dir * (float)s_calib_ctx.pole_pairs;
            elec_raw = CalibrationService_Normalize0To2Pi(elec_raw);
            theta_ref = CalibrationService_Normalize0To2Pi(s_calib_ctx.theta);
            diff = CalibrationService_NormalizePmPi(elec_raw - theta_ref);

            s_calib_ctx.sum_diff += diff;
            s_calib_ctx.sum_diff_sq += diff * diff;
            s_calib_ctx.sum_sin_1 += diff * (float)sin((double)mech_rad);
            s_calib_ctx.sum_cos_1 += diff * (float)cos((double)mech_rad);
            s_calib_ctx.sum_sin_2 += diff * (float)sin((double)(2.0f * mech_rad));
            s_calib_ctx.sum_cos_2 += diff * (float)cos((double)(2.0f * mech_rad));
            s_calib_ctx.sample_count++;

            {
                ProtocolReply sample_reply;
                TelemetryService_BuildCalibSampleReply(&sample_reply,
                                                       (uint16_t)s_calib_ctx.sample_count,
                                                       s_calib_ctx.sweep_direction,
                                                       s_calib_ctx.theta,
                                                       mech_rad,
                                                       diff);
                (void)ProtocolController_SendReply(&sample_reply);
            }
        }

        s_calib_ctx.theta -= CALIB_SERVICE_STEP_RAD;
        s_calib_ctx.flow_state = CALIB_FLOW_REVERSE_SWEEP_SET;
        break;

    case CALIB_FLOW_FINALIZE:
        {
            ParamStorage_Data_t *params;

            if (s_calib_ctx.sample_count <= 0)
            {
                CalibrationService_SetFailure(PROTOCOL_CALIB_STATE_FAIL_INTERNAL);
                return;
            }

            {
                float mean_diff;
                float coef_sin_1;
                float coef_cos_1;
                float coef_sin_2;
                float coef_cos_2;
                float var_residual;
                float reduction_h1;
                float reduction_h2;
                float rms_offset_only;
                float rms_h1;
                float rms_h2;

                mean_diff = s_calib_ctx.sum_diff / (float)s_calib_ctx.sample_count;
                coef_sin_1 = 2.0f * s_calib_ctx.sum_sin_1 / (float)s_calib_ctx.sample_count;
                coef_cos_1 = 2.0f * s_calib_ctx.sum_cos_1 / (float)s_calib_ctx.sample_count;
                coef_sin_2 = 2.0f * s_calib_ctx.sum_sin_2 / (float)s_calib_ctx.sample_count;
                coef_cos_2 = 2.0f * s_calib_ctx.sum_cos_2 / (float)s_calib_ctx.sample_count;

                s_calib_ctx.result.electrical_offset_rad = mean_diff;
                s_calib_ctx.result.harmonic_amp_1 = (float)sqrt((double)(coef_sin_1 * coef_sin_1 + coef_cos_1 * coef_cos_1));
                s_calib_ctx.result.harmonic_phase_1 = (float)atan2((double)coef_cos_1, (double)coef_sin_1);
                s_calib_ctx.result.harmonic_amp_2 = (float)sqrt((double)(coef_sin_2 * coef_sin_2 + coef_cos_2 * coef_cos_2));
                s_calib_ctx.result.harmonic_phase_2 = (float)atan2((double)coef_cos_2, (double)coef_sin_2);

                var_residual = s_calib_ctx.sum_diff_sq / (float)s_calib_ctx.sample_count - mean_diff * mean_diff;
                if (var_residual < 0.0f)
                {
                    var_residual = 0.0f;
                }

                reduction_h1 = 0.5f * s_calib_ctx.result.harmonic_amp_1 * s_calib_ctx.result.harmonic_amp_1;
                reduction_h2 = 0.5f * s_calib_ctx.result.harmonic_amp_2 * s_calib_ctx.result.harmonic_amp_2;
                rms_offset_only = (float)sqrt((double)var_residual);

                if (var_residual > reduction_h1)
                {
                    rms_h1 = (float)sqrt((double)(var_residual - reduction_h1));
                }
                else
                {
                    rms_h1 = 0.0f;
                }

                if (var_residual > (reduction_h1 + reduction_h2))
                {
                    rms_h2 = (float)sqrt((double)(var_residual - reduction_h1 - reduction_h2));
                }
                else
                {
                    rms_h2 = 0.0f;
                }

                if (rms_h1 >= rms_offset_only)
                {
                    s_calib_ctx.result.harmonic_amp_1 = 0.0f;
                    s_calib_ctx.result.harmonic_phase_1 = 0.0f;
                    s_calib_ctx.result.harmonic_amp_2 = 0.0f;
                    s_calib_ctx.result.harmonic_phase_2 = 0.0f;
                }
                else if (rms_h2 >= rms_h1)
                {
                    s_calib_ctx.result.harmonic_amp_2 = 0.0f;
                    s_calib_ctx.result.harmonic_phase_2 = 0.0f;
                }

                if (rms_offset_only < CALIB_SERVICE_MIN_RMS_RAD)
                {
                    s_calib_ctx.result.harmonic_amp_1 = 0.0f;
                    s_calib_ctx.result.harmonic_phase_1 = 0.0f;
                    s_calib_ctx.result.harmonic_amp_2 = 0.0f;
                    s_calib_ctx.result.harmonic_phase_2 = 0.0f;
                }
            }

            params = ParamStorage_Get();
            params->motor_calib = s_calib_ctx.result;
            if (!ParamStorage_Save())
            {
                CalibrationService_SetFailure(PROTOCOL_CALIB_STATE_FAIL_INTERNAL);
                return;
            }

            Motor_Calibration_Init(&params->motor_calib);
            CalibrationService_StopMotor();
            s_calib_ctx.progress = 100u;
            s_calib_ctx.calib_state = PROTOCOL_CALIB_STATE_SUCCESS;
            s_calib_ctx.flow_state = CALIB_FLOW_IDLE;
        }
        break;

    default:
        CalibrationService_SetFailure(PROTOCOL_CALIB_STATE_FAIL_INTERNAL);
        break;
    }
}
