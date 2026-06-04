#include "motor.h"
#include "encoder.h"
#include "calibration.h"
#include "svpwm.h"
#include "control.h"

// PID执行周期计数（5kHz / 300Hz ≈ 16.67，取17）
#define PID_PERIOD_COUNT    17

// 开环测试开关：置1后忽略编码器电角度与速度环，直接自增电角度驱动电机旋转
#define MOTOR_OPEN_LOOP_TEST_ENABLE        0
// 开环测试时每个5kHz周期增加的电角度(rad)，可改符号切换转向
#define MOTOR_OPEN_LOOP_TEST_STEP_RAD      0.010f
// 开环测试时固定输出的Q轴电压幅值
#define MOTOR_OPEN_LOOP_TEST_Q_VOLTAGE     0.8f
#define MOTOR_OPEN_LOOP_TEST_TWO_PI        6.28318530718f

// 控制相关
static uint8_t s_pid_counter = 0;     // PID计数器，用于分频
static float s_cached_angle = 0.0f;   // 缓存的角度（用于速度计算）
static float s_q_axis_voltage = 0.0f; // Q轴电压占空比（由速度环计算）
static uint8_t s_control_mode = MOTOR_CONTROL_MODE_STOP;
static BOOL s_motor_suspended = FALSE;
static uint16_t count =0;
#if MOTOR_OPEN_LOOP_TEST_ENABLE
static float s_open_loop_elec_angle = 0.0f;

static float Motor_NormalizeAngle0To2Pi(float angle)
{
    while (angle >= MOTOR_OPEN_LOOP_TEST_TWO_PI)
    {
        angle -= MOTOR_OPEN_LOOP_TEST_TWO_PI;
    }

    while (angle < 0.0f)
    {
        angle += MOTOR_OPEN_LOOP_TEST_TWO_PI;
    }

    return angle;
}
#endif

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
    count++;
    if (count%1000==0){
        MOTOR_STATUS=!MOTOR_STATUS;
    }
#if MOTOR_OPEN_LOOP_TEST_ENABLE
    s_open_loop_elec_angle = Motor_NormalizeAngle0To2Pi(s_open_loop_elec_angle +
                                                        MOTOR_OPEN_LOOP_TEST_STEP_RAD);
    s_q_axis_voltage = MOTOR_OPEN_LOOP_TEST_Q_VOLTAGE;
    SVPWM_UpdateQAxis(s_open_loop_elec_angle, s_q_axis_voltage);
    return;
#else
    if (s_motor_suspended)
    {
        return;
    }
    {
        float elec_angle;
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
#endif
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
