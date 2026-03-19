#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "config.h"
#include "encoder.h"
#include "svpwm.h"

#define MOTOR_CALIB_DEFAULT_POLE_PAIRS 7

typedef struct
{
    int pole_pairs;
    int dir;                          /* 编码器方向，1 或 -1 */
    float electrical_offset_rad;      /* 电角度零位偏置，单位 rad，可为负 */

    /* 谐波补偿参数 */
    float harmonic_amp_1;
    float harmonic_phase_1;
    float harmonic_amp_2;
    float harmonic_phase_2;
} Motor_Calibration;

/* 初始化并应用校准参数，传入NULL则使用默认参数 */
void Motor_Calibration_Init(const Motor_Calibration *calib);

/* 自动校准，返回校准结果，不直接应用到内部状态 */
Motor_Calibration Motor_Calibration_AutoCalibrate(int pole_pairs);

/* 获取当前校准参数 */
const Motor_Calibration* Motor_Calibration_GetCurrent(void);

/* 计算电角度（应用当前校准参数） */
float Motor_Calibration_GetElectricalAngle(float mechanical_rad);

/* 计算机械角度（应用当前校准参数） */
float Motor_Calibration_GetMechanicalAngle(float mechanical_rad);

#endif