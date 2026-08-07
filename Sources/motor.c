#include "motor.h"
#include "encoder.h"
#include "calibration.h"
#include "svpwm.h"
#include "control.h"
#include "adc.h"

// 开环测试开关：置1后忽略编码器电角度与速度环，直接自增电角度驱动电机旋转
#define MOTOR_OPEN_LOOP_TEST_ENABLE        0
// 开环测试时每个5kHz周期增加的电角度(rad)，可改符号切换转向
#define MOTOR_OPEN_LOOP_TEST_STEP_RAD      0.010f
// 开环测试时固定输出的Q轴电压幅值
#define MOTOR_OPEN_LOOP_TEST_Q_VOLTAGE     0.8f
#define MOTOR_OPEN_LOOP_TEST_TWO_PI        6.28318530718f
#define MOTOR_MIN_BUS_VOLTAGE              1.0f

// 控制相关
static float s_cached_angle = 0.0f;   // 缓存的角度（用于速度计算）
static float s_cached_mechanical_angle = 0.0f;
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
    if (count >= 1000u){
        count = 0u;
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
        float mech_angle;
        float speed_feedback;
        float bus_voltage;
        s_cached_angle = Encoder_UpdateAndGetAngle();

        Motor_Calibration_GetAngles(s_cached_angle, &mech_angle, &elec_angle);
        s_cached_mechanical_angle = mech_angle;

        if (s_control_mode == MOTOR_CONTROL_MODE_SPEED)
        {
            (void)SpeedController_UpdateObserver(mech_angle);
            if (SpeedController_TryGetAveragedSpeed(&speed_feedback))
            {
                bus_voltage = ADC_GetBusVoltage();
                if (bus_voltage > MOTOR_MIN_BUS_VOLTAGE) {
                    /* 电压→调制度：Vout / Vbus */
                    s_q_axis_voltage = SpeedController_UpdateFromSpeed(speed_feedback,
                                                                       bus_voltage)
                                     / bus_voltage;
                } else {
                    (void)SpeedController_UpdateFromSpeed(speed_feedback, 0.0f);
                    s_q_axis_voltage = 0.0f;
                }
            }
        }
        else
        {
            s_q_axis_voltage = 0.0f;
        }

        /* 占空比限幅（基于实际PWM周期） */
        if (s_q_axis_voltage > SVPWM_LINEAR_MODULATION_MAX) {
            s_q_axis_voltage = SVPWM_LINEAR_MODULATION_MAX;
        } else if (s_q_axis_voltage < -SVPWM_LINEAR_MODULATION_MAX) {
            s_q_axis_voltage = -SVPWM_LINEAR_MODULATION_MAX;
        }

        SVPWM_UpdateQAxis(elec_angle, s_q_axis_voltage);
    }
#endif
}

float Motor_GetCachedAngle(void)
{
    return s_cached_angle;
}

float Motor_GetCachedMechanicalAngle(void)
{
    return s_cached_mechanical_angle;
}

float Motor_GetQAxisVoltage(void)
{
    return s_q_axis_voltage;
}

void Motor_SetQAxisVoltageDirect(float voltage)
{
    s_q_axis_voltage = voltage;
}
