#include "motor.h"
#include "encoder.h"
#include "calibration.h"
#include "svpwm.h"

#define MOTOR_Q_AXIS_BASE_VOLTAGE 0.5f
#define MOTOR_PRBS_AMPLITUDE 0.2f
#define MOTOR_CONTROL_PERIOD_US 400UL
#define MOTOR_PRBS_UPDATE_PERIOD_US 10000UL
#define MOTOR_PRBS_START_DELAY_MS 500UL

static uint16_t s_lfsr = 0xACE1u;
static float s_prbs_out = 0.0f;
static float s_last_q_axis = MOTOR_Q_AXIS_BASE_VOLTAGE;
static float s_last_elec_angle = 0.0f;
static uint32_t s_prbs_elapsed_us = 0;
static uint32_t s_stable_elapsed_us = 0;
static uint8_t s_prbs_enabled = 0;

static void PRBS_Update(void)
{
    uint8_t lsb = s_lfsr & 1;

    s_lfsr >>= 1;

    if (lsb)
    {
        s_lfsr ^= 0xB400u;
    }

    s_prbs_out = lsb ? MOTOR_PRBS_AMPLITUDE : -MOTOR_PRBS_AMPLITUDE;
}

static void Motor_PRBS_Service(void)
{
    if (!s_prbs_enabled)
    {
        s_stable_elapsed_us += MOTOR_CONTROL_PERIOD_US;
        if (s_stable_elapsed_us >= (MOTOR_PRBS_START_DELAY_MS * 1000UL))
        {
            s_prbs_enabled = 1;
            s_prbs_elapsed_us = 0;
            PRBS_Update();
        }
        return;
    }

    s_prbs_elapsed_us += MOTOR_CONTROL_PERIOD_US;
    while (s_prbs_elapsed_us >= MOTOR_PRBS_UPDATE_PERIOD_US)
    {
        s_prbs_elapsed_us -= MOTOR_PRBS_UPDATE_PERIOD_US;
        PRBS_Update();
    }
}

void Motor_Commutation_Update(void)
{
    float elec_angle;
    float q_axis;

    Motor_PRBS_Service();
    elec_angle = Motor_Calibration_GetElectricalAngle(Encoder_UpdateAndGetAngle());
    q_axis = MOTOR_Q_AXIS_BASE_VOLTAGE + (s_prbs_enabled ? s_prbs_out : 0.0f);
    SVPWM_UpdateQAxis(elec_angle, q_axis);

    s_last_elec_angle = elec_angle;
    s_last_q_axis = q_axis;
}

void Motor_Print_LastSample(void)
{
    uint8_t frame[12];
    uint8_t *p_float;
    int i;

    if (!s_prbs_enabled)
    {
        return;
    }

    // 帧头: 0xAA 0x55
    frame[0] = 0xAA;
    frame[1] = 0x55;

    // q_axis (float -> 4 bytes, little-endian)
    p_float = (uint8_t *)&s_last_q_axis;
    frame[2] = p_float[0];
    frame[3] = p_float[1];
    frame[4] = p_float[2];
    frame[5] = p_float[3];

    // elec_angle (float -> 4 bytes, little-endian)
    p_float = (uint8_t *)&s_last_elec_angle;
    frame[6] = p_float[0];
    frame[7] = p_float[1];
    frame[8] = p_float[2];
    frame[9] = p_float[3];

    // 帧尾: 0x55 0xAA
    frame[10] = 0x55;
    frame[11] = 0xAA;

    // 发送 12 字节
    for (i = 0; i < 12; i++)
    {
        putchar(frame[i]);
    }
}