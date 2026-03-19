#include "motor.h"
#include "encoder.h"
#include "calibration.h"
#include "svpwm.h"
#include "control.h"


// 控制相关
static uint8_t s_pid_counter = 0;     // PID计数器，用于分频
static float s_cached_angle = 0.0f;   // 缓存的角度（用于速度计算）
static float s_q_axis_voltage = 0.0f; // Q轴电压占空比（由速度环计算）

// PID执行周期计数（5kHz / 300Hz ≈ 16.67，取17）
#define PID_PERIOD_COUNT    17



void Motor_Update(void)
{
    float elec_angle;

    // 获取当前机械角度并缓存
    s_cached_angle = Encoder_UpdateAndGetAngle();

    // 转换为电角度
    elec_angle = Motor_Calibration_GetElectricalAngle(s_cached_angle);

    // 计数器分频，300Hz执行PID
    if (++s_pid_counter >= PID_PERIOD_COUNT)
    {
        s_pid_counter = 0;

        // 执行速度环控制（传入机械角度，内部进行差分计算）
        s_q_axis_voltage = SpeedController_Update(s_cached_angle);
    }

    // 更新SVPWM，使用速度环输出的Q轴电压
    SVPWM_UpdateQAxis(elec_angle, s_q_axis_voltage);
}

// 获取当前缓存的机械角度（供其他模块使用）
float Motor_GetCachedAngle(void)
{
    return s_cached_angle;
}

// 获取当前Q轴电压（调试用）
float Motor_GetQAxisVoltage(void)
{
    return s_q_axis_voltage;
}

// 设置直接的Q轴电压（用于测试，绕过速度环）
void Motor_SetQAxisVoltageDirect(float voltage)
{
    s_q_axis_voltage = voltage;
}
