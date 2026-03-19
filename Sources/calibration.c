#include "calibration.h"

#define CALIB_TWO_PI          6.28318530718f
#define CALIB_INV_TWO_PI      0.15915494309f
#define CALIB_RAD_TO_DEG      57.2957795131f
#define CALIB_VOLTAGE         0.40f
#define CALIB_STEP_RAD        0.05f
#define CALIB_STEP_DELAY_MS   10
#define CALIB_LOCK_DELAY_MS   1000
#define CALIB_MIN_RMS_RAD     0.001f

static Motor_Calibration g_motor_cal;
static float data s_electrical_scale = 0.0f;
static float data s_mechanical_scale = 1.0f;
static float data s_electrical_offset_rad = 0.0f;
static float data s_mechanical_offset_rad = 0.0f;
static float s_harmonic_amp_1_mech = 0.0f;
static float s_harmonic_amp_2_mech = 0.0f;

static void Calibration_UpdateRuntimeCache(void)
{
    float inv_pole_pairs;

    s_electrical_scale = (float)g_motor_cal.dir * (float)g_motor_cal.pole_pairs;
    s_mechanical_scale = (float)g_motor_cal.dir;
    s_electrical_offset_rad = g_motor_cal.electrical_offset_rad;

    if (g_motor_cal.pole_pairs > 0)
    {
        inv_pole_pairs = 1.0f / (float)g_motor_cal.pole_pairs;
    }
    else
    {
        inv_pole_pairs = 0.0f;
    }

    s_mechanical_offset_rad = g_motor_cal.electrical_offset_rad * inv_pole_pairs;
    s_harmonic_amp_1_mech = g_motor_cal.harmonic_amp_1 * inv_pole_pairs;
    s_harmonic_amp_2_mech = g_motor_cal.harmonic_amp_2 * inv_pole_pairs;
}

static float Calibration_Normalize_0_2pi(float angle)
{
    while (angle < 0.0f)
    {
        angle += CALIB_TWO_PI;
    }
    while (angle >= CALIB_TWO_PI)
    {
        angle -= CALIB_TWO_PI;
    }
    return angle;
}

static float Calibration_Normalize_0_2pi_Fast(float angle)
{
    int turns;

    turns = (int)(angle * CALIB_INV_TWO_PI);
    if (angle < 0.0f)
    {
        turns--;
    }

    angle -= CALIB_TWO_PI * (float)turns;

    if (angle >= CALIB_TWO_PI)
    {
        angle -= CALIB_TWO_PI;
    }
    else if (angle < 0.0f)
    {
        angle += CALIB_TWO_PI;
    }

    return angle;
}

static float Calibration_Normalize_MechanicalFast(float angle)
{
    if (angle < 0.0f)
    {
        angle += CALIB_TWO_PI;
        if (angle < 0.0f)
        {
            angle += CALIB_TWO_PI;
        }
    }
    else if (angle >= CALIB_TWO_PI)
    {
        angle -= CALIB_TWO_PI;
        if (angle >= CALIB_TWO_PI)
        {
            angle -= CALIB_TWO_PI;
        }
    }

    return angle;
}

static float Calibration_Normalize_pm_pi(float angle)
{
    while (angle <= -3.14159265359f)
    {
        angle += CALIB_TWO_PI;
    }
    while (angle > 3.14159265359f)
    {
        angle -= CALIB_TWO_PI;
    }
    return angle;
}

static float Calibration_GetElectricalRaw(float mechanical_rad)
{
    float elec_raw;

    elec_raw = mechanical_rad * (float)g_motor_cal.dir * (float)g_motor_cal.pole_pairs;
    return Calibration_Normalize_0_2pi(elec_raw);
}

static float Calibration_GetHarmonicError(float mechanical_rad)
{
    float harmonic_error;

    harmonic_error = g_motor_cal.electrical_offset_rad;
    harmonic_error += g_motor_cal.harmonic_amp_1 * (float)sin(mechanical_rad + g_motor_cal.harmonic_phase_1);
    harmonic_error += g_motor_cal.harmonic_amp_2 * (float)sin(2.0f * mechanical_rad + g_motor_cal.harmonic_phase_2);

    return harmonic_error;
}

void Motor_Calibration_Init(const Motor_Calibration *calib)
{
    if (calib != NULL)
    {
        /* 使用传入的参数 */
        g_motor_cal = *calib;
    }
    else
    {
        /* 使用默认参数 */
        g_motor_cal.pole_pairs = MOTOR_CALIB_DEFAULT_POLE_PAIRS;
        g_motor_cal.dir = 1;
        g_motor_cal.electrical_offset_rad = 0.0f;
        g_motor_cal.harmonic_amp_1 = 0.0f;
        g_motor_cal.harmonic_phase_1 = 0.0f;
        g_motor_cal.harmonic_amp_2 = 0.0f;
        g_motor_cal.harmonic_phase_2 = 0.0f;
    }

    Calibration_UpdateRuntimeCache();
}

Motor_Calibration Motor_Calibration_AutoCalibrate(int pole_pairs)
{
    Motor_Calibration result;
    float sum_diff;
    float sum_diff_sq;
    float sum_sin_1;
    float sum_cos_1;
    float sum_sin_2;
    float sum_cos_2;
    float coef_sin_1;
    float coef_cos_1;
    float coef_sin_2;
    float coef_cos_2;
    int sample_count;
    float mech_rad;
    float diff;
    float theta;
    float mean_diff;
    float start_mech;
    float end_mech;
    float mech_delta;
    float elec_raw;
    float theta_end;
    float rms_offset_only;
    float rms_h1;
    float rms_h2;
    float var_residual;
    float reduction_h1;
    float reduction_h2;
    float overall_confidence;
    float harmonic2_gain;

    /* 初始化结果为默认值 */
    result.pole_pairs = pole_pairs;
    result.dir = 1;
    result.electrical_offset_rad = 0.0f;
    result.harmonic_amp_1 = 0.0f;
    result.harmonic_phase_1 = 0.0f;
    result.harmonic_amp_2 = 0.0f;
    result.harmonic_phase_2 = 0.0f;

    if (pole_pairs <= 0)
    {
        return result;
    }

    /* 1) 锁定电角度0，给转子建立稳定参考 */
    SVPWM_Update(0.0f, CALIB_VOLTAGE);
    delay_ms(CALIB_LOCK_DELAY_MS);

    Encoder_Update();
    start_mech = Encoder_Get_Angle();

    /* 2) 沿一个电角周期慢速扫角，用于检测编码器安装方向 */
    theta = 0.0f;
    while (theta <= CALIB_TWO_PI)
    {
        SVPWM_Update(theta, CALIB_VOLTAGE);
        delay_ms(CALIB_STEP_DELAY_MS);
        theta += CALIB_STEP_RAD;
    }

    /* 等待电机停稳 */
    delay_ms(500);
    Encoder_Update();
    end_mech = Encoder_Get_Angle();

    /* 计算位移差（处理跨越0点的情况） */
    mech_delta = end_mech - start_mech;
    if (mech_delta > CALIB_TWO_PI / 2.0f) mech_delta -= CALIB_TWO_PI;
    if (mech_delta < -CALIB_TWO_PI / 2.0f) mech_delta += CALIB_TWO_PI;

    if (mech_delta < 0.0f) {
        result.dir = -1;
    } else {
        result.dir = 1;
    }

    /* 3) 扫完整机械一圈，拟合编码器偏心导致的一次/二次机械谐波 */
    sum_diff_sq = 0.0f;
    sum_sin_1 = 0.0f;
    sum_cos_1 = 0.0f;
    sum_sin_2 = 0.0f;
    sum_cos_2 = 0.0f;
    sample_count = 0;
    theta = 0.0f;
    theta_end = CALIB_TWO_PI * (float)pole_pairs;

    while (theta <= theta_end)
    {
        SVPWM_Update(theta, CALIB_VOLTAGE);
        delay_ms(CALIB_STEP_DELAY_MS);

        Encoder_Update();
        mech_rad = Encoder_Get_Angle();

        /* 使用临时计算的电角度（基于已检测的方向） */
        elec_raw = mech_rad * (float)result.dir * (float)pole_pairs;
        while (elec_raw < 0.0f) elec_raw += CALIB_TWO_PI;
        while (elec_raw >= CALIB_TWO_PI) elec_raw -= CALIB_TWO_PI;

        diff = Calibration_Normalize_pm_pi(elec_raw - theta);
        sum_diff += diff;
        sum_diff_sq += diff * diff;
        sum_sin_1 += diff * (float)sin((double)mech_rad);
        sum_cos_1 += diff * (float)cos((double)mech_rad);
        sum_sin_2 += diff * (float)sin((double)(2.0f * mech_rad));
        sum_cos_2 += diff * (float)cos((double)(2.0f * mech_rad));

        sample_count++;
        theta += CALIB_STEP_RAD;
    }

    if (sample_count > 0)
    {
        mean_diff = sum_diff / (float)sample_count;
        coef_sin_1 = 2.0f * sum_sin_1 / (float)sample_count;
        coef_cos_1 = 2.0f * sum_cos_1 / (float)sample_count;
        coef_sin_2 = 2.0f * sum_sin_2 / (float)sample_count;
        coef_cos_2 = 2.0f * sum_cos_2 / (float)sample_count;

        result.electrical_offset_rad = mean_diff;
        result.harmonic_amp_1 = (float)sqrt((double)(coef_sin_1 * coef_sin_1 + coef_cos_1 * coef_cos_1));
        result.harmonic_phase_1 = (float)atan2((double)coef_cos_1, (double)coef_sin_1);
        result.harmonic_amp_2 = (float)sqrt((double)(coef_sin_2 * coef_sin_2 + coef_cos_2 * coef_cos_2));
        result.harmonic_phase_2 = (float)atan2((double)coef_cos_2, (double)coef_sin_2);

        var_residual = sum_diff_sq / (float)sample_count - mean_diff * mean_diff;
        if (var_residual < 0.0f)
        {
            var_residual = 0.0f;
        }

        reduction_h1 = 0.5f * result.harmonic_amp_1 * result.harmonic_amp_1;
        reduction_h2 = 0.5f * result.harmonic_amp_2 * result.harmonic_amp_2;

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
            result.harmonic_amp_1 = 0.0f;
            result.harmonic_phase_1 = 0.0f;
            rms_h1 = rms_offset_only;
            rms_h2 = rms_offset_only;
            result.harmonic_amp_2 = 0.0f;
            result.harmonic_phase_2 = 0.0f;
        }
        else if (rms_h2 >= rms_h1)
        {
            result.harmonic_amp_2 = 0.0f;
            result.harmonic_phase_2 = 0.0f;
            rms_h2 = rms_h1;
        }

        if (rms_offset_only > CALIB_MIN_RMS_RAD)
        {
            overall_confidence = 1.0f - rms_h2 / rms_offset_only;
        }
        else
        {
            overall_confidence = 0.0f;
        }

        if (rms_h1 > CALIB_MIN_RMS_RAD)
        {
            harmonic2_gain = 1.0f - rms_h2 / rms_h1;
        }
        else
        {
            harmonic2_gain = 0.0f;
        }

        if (overall_confidence < 0.0f)
        {
            overall_confidence = 0.0f;
        }
        if (harmonic2_gain < 0.0f)
        {
            harmonic2_gain = 0.0f;
        }
    }
    else
    {
        result.electrical_offset_rad = 0.0f;
        result.harmonic_amp_1 = 0.0f;
        result.harmonic_phase_1 = 0.0f;
        result.harmonic_amp_2 = 0.0f;
        result.harmonic_phase_2 = 0.0f;
    }

    /* 停止输出，避免持续发热 */
    SVPWM_Update(0.0f, 0.0f);

    return result;
}

const Motor_Calibration* Motor_Calibration_GetCurrent(void)
{
    return &g_motor_cal;
}

float Motor_Calibration_GetElectricalAngle(float mechanical_rad)
{
    float elec;

    elec = mechanical_rad * s_electrical_scale - s_electrical_offset_rad;
    elec -= g_motor_cal.harmonic_amp_1 * (float)sin(mechanical_rad + g_motor_cal.harmonic_phase_1);
    elec -= g_motor_cal.harmonic_amp_2 * (float)sin(2.0f * mechanical_rad + g_motor_cal.harmonic_phase_2);

    return Calibration_Normalize_0_2pi_Fast(elec);
}

float Motor_Calibration_GetMechanicalAngle(float mechanical_rad)
{
    float mech;

    mech = mechanical_rad * s_mechanical_scale - s_mechanical_offset_rad;
    mech -= s_harmonic_amp_1_mech * (float)sin(mechanical_rad + g_motor_cal.harmonic_phase_1);
    mech -= s_harmonic_amp_2_mech * (float)sin(2.0f * mechanical_rad + g_motor_cal.harmonic_phase_2);

    return Calibration_Normalize_MechanicalFast(mech);
}