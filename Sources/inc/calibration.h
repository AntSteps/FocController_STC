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

    /* 谐波补偿参数，当前版本固定为0 */
    float harmonic_amp_1;
    float harmonic_phase_1;
    float harmonic_amp_2;
    float harmonic_phase_2;
} Motor_Calibration;

void Motor_Calibration_Init(int pole_pairs);
void Motor_Calibration_AutoCalibrate(void);
float Motor_Calibration_GetElectricalAngle(float mechanical_rad);
float Motor_Calibration_GetMechanicalAngle(float mechanical_rad);
float Motor_Calibration_GetElectricalOffset(void);
float Motor_Calibration_GetHarmonicAmp1(void);
float Motor_Calibration_GetHarmonicPhase1(void);
float Motor_Calibration_GetHarmonicAmp2(void);
float Motor_Calibration_GetHarmonicPhase2(void);
int Motor_Calibration_GetDirection(void);
int Motor_Calibration_GetPolePairs(void);

#endif
