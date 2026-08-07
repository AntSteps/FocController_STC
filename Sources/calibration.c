#include "calibration.h"

#define CALIB_TWO_PI          6.28318530718f
#define CALIB_INV_TWO_PI      0.15915494309f

static Motor_Calibration g_motor_cal;
static float data s_electrical_scale = 0.0f;
static float data s_electrical_offset_rad = 0.0f;
static float data s_inv_pole_pairs = 0.0f;

static void Calibration_UpdateRuntimeCache(void)
{
    float inv_pole_pairs;

    s_electrical_scale = (float)g_motor_cal.dir * (float)g_motor_cal.pole_pairs;
    s_electrical_offset_rad = g_motor_cal.electrical_offset_rad;

    if (g_motor_cal.pole_pairs > 0)
    {
        inv_pole_pairs = 1.0f / (float)g_motor_cal.pole_pairs;
    }
    else
    {
        inv_pole_pairs = 0.0f;
    }

    s_inv_pole_pairs = inv_pole_pairs;
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

void Motor_Calibration_GetAngles(float raw_mechanical_rad,
                                 float *mechanical_rad,
                                 float *electrical_rad)
{
    float electrical_unwrapped;
    float harmonic_1;
    float harmonic_2;

    harmonic_1 = g_motor_cal.harmonic_amp_1
               * (float)sin(raw_mechanical_rad + g_motor_cal.harmonic_phase_1);
    harmonic_2 = g_motor_cal.harmonic_amp_2
               * (float)sin(2.0f * raw_mechanical_rad + g_motor_cal.harmonic_phase_2);

    electrical_unwrapped = raw_mechanical_rad * s_electrical_scale
                         - s_electrical_offset_rad
                         - harmonic_1
                         - harmonic_2;

    if (electrical_rad != NULL)
    {
        *electrical_rad = Calibration_Normalize_0_2pi_Fast(electrical_unwrapped);
    }

    if (mechanical_rad != NULL)
    {
        *mechanical_rad = Calibration_Normalize_MechanicalFast(
            electrical_unwrapped * s_inv_pole_pairs);
    }
}

float Motor_Calibration_GetElectricalAngle(float mechanical_rad)
{
    float elec;

    Motor_Calibration_GetAngles(mechanical_rad, NULL, &elec);
    return elec;
}

float Motor_Calibration_GetMechanicalAngle(float mechanical_rad)
{
    float mech;

    Motor_Calibration_GetAngles(mechanical_rad, &mech, NULL);
    return mech;
}
