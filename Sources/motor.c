#include "motor.h"
#include "encoder.h"
#include "calibration.h"
#include "svpwm.h"
#include "control.h"

// PID执行周期计数（5kHz / 300Hz ≈ 16.67，取17）
#define PID_PERIOD_COUNT    17

// 控制相关
static uint8_t s_pid_counter = 0;     // PID计数器，用于分频
static float s_cached_angle = 0.0f;   // 缓存的角度（用于速度计算）
static float s_q_axis_voltage = 0.0f; // Q轴电压占空比（由速度环计算）
static uint8_t s_control_mode = MOTOR_CONTROL_MODE_STOP;
static BOOL s_motor_suspended = FALSE;

void Motor_SetControlMode(uint8_t control_mode)
{
    if (control_mode == s_control_mode)
    {
        return;
    }

    s_control_mode = control_mode;
    s_pid_counter = 0u;

    if (s_control_mode == MOTOR_CONTROL_MODE_STOP)
    {
        s_q_axis_voltage = 0.0f;
        SpeedController_SetTarget(0.0f);
    }

    SpeedController_Reset();
}

uint8_t Motor_GetControlMode(void)
{
    return s_control_mode;
}

void Motor_SetSuspended(BOOL suspended)
{
    s_motor_suspended = suspended;
    if (suspended)
    {
        s_q_axis_voltage = 0.0f;
        SVPWM_Update(0.0f, 0.0f);
    }
}

BOOL Motor_IsSuspended(void)
{
    return s_motor_suspended;
}

void Motor_Update(void)
{
    float elec_angle;

    if (s_motor_suspended)
    {
        return;
    }

    s_cached_angle = Encoder_UpdateAndGetAngle();
    elec_angle = Motor_Calibration_GetElectricalAngle(s_cached_angle);

    if (s_control_mode == MOTOR_CONTROL_MODE_SPEED)
    {
        if (++s_pid_counter >= PID_PERIOD_COUNT)
        {
            s_pid_counter = 0u;
            s_q_axis_voltage = SpeedController_Update(s_cached_angle);
        }
    }
    else
    {
        s_q_axis_voltage = 0.0f;
    }

    SVPWM_UpdateQAxis(elec_angle, s_q_axis_voltage);
}

float Motor_GetCachedAngle(void)
{
    return s_cached_angle;
}

float Motor_GetQAxisVoltage(void)
{
    return s_q_axis_voltage;
}

void Motor_SetQAxisVoltageDirect(float voltage)
{
    s_q_axis_voltage = voltage;
}
