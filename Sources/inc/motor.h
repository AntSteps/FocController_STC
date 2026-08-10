#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "def.h"

#define MOTOR_MIN_BUS_VOLTAGE  1.0f

typedef enum
{
    MOTOR_CONTROL_MODE_STOP = 0,
    MOTOR_CONTROL_MODE_SPEED = 1
} MotorControlMode;

// 换相更新（在定时器中断中调用，5kHz）
void Motor_Update(void);

// 获取当前缓存的机械角度（供控制模块使用）
float Motor_GetCachedAngle(void);
float Motor_GetCachedMechanicalAngle(void);

// 获取当前Q轴电压（调试用）
float Motor_GetQAxisVoltage(void);

// 设置直接的Q轴电压（用于测试，绕过速度环）
void Motor_SetQAxisVoltageDirect(float voltage);

void Motor_SetControlMode(uint8_t control_mode);
uint8_t Motor_GetControlMode(void);
void Motor_SetSuspended(BOOL suspended);
BOOL Motor_IsSuspended(void);

#endif
