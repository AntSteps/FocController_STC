#include "calibration.h"

#define CALIB_TWO_PI          6.28318530718f
#define CALIB_INV_TWO_PI      0.15915494309f

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
