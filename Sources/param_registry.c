#include "param_registry.h"
#include "motor.h"
#include <string.h>

#define PARAM_OFFSET(member) ((uint16_t)(&(((ParamStorage_Data_t *)0)->member)))

typedef enum
{
    PARAM_VALUE_FLOAT = 0,
    PARAM_VALUE_INT16 = 1,
    PARAM_VALUE_BOOL8 = 2
} ParamValueType;

typedef enum
{
    PARAM_RULE_NONE = 0,
    PARAM_RULE_BOOL = 1,
    PARAM_RULE_CALIB_DIR = 2,
    PARAM_RULE_SPEED_OUT_MAX = 3,
    PARAM_RULE_SPEED_OUT_MIN = 4
} ParamValidationRule;

typedef enum
{
    PARAM_APPLY_NONE = 0,
    PARAM_APPLY_SPEED_CTRL = 1,
    PARAM_APPLY_CALIB = 2,
    PARAM_APPLY_LIMITS = 3
} ParamApplyAction;

typedef struct
{
    uint8_t param_id;
    uint16_t offset;
    uint8_t value_type;
    float min_value;
    float max_value;
    uint8_t validation_rule;
    uint8_t apply_action;
} ParamRegistryEntry;

static const ParamRegistryEntry s_param_table[] =
{
    { PROTOCOL_PARAM_ID_SPEED_KP,             PARAM_OFFSET(speed_ctrl_params.kp),                         PARAM_VALUE_FLOAT, 0.0f,      100.0f,   PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_SPEED_KI,             PARAM_OFFSET(speed_ctrl_params.ki),                         PARAM_VALUE_FLOAT, 0.0f,      100.0f,   PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_VELOCITY_LIMIT,       PARAM_OFFSET(velocity_limit),                               PARAM_VALUE_FLOAT, 0.0f,      5000.0f,  PARAM_RULE_NONE,          PARAM_APPLY_LIMITS },

    { PROTOCOL_PARAM_ID_MOTOR_POLE_PAIRS,     PARAM_OFFSET(motor_pole_pairs),                             PARAM_VALUE_INT16, 1.0f,      64.0f,    PARAM_RULE_NONE,          PARAM_APPLY_NONE },
    { PROTOCOL_PARAM_ID_MOTOR_PHASE_R,        PARAM_OFFSET(motor_phase_resistance),                       PARAM_VALUE_FLOAT, 0.0f,      100.0f,   PARAM_RULE_NONE,          PARAM_APPLY_NONE },
    { PROTOCOL_PARAM_ID_MOTOR_PHASE_L,        PARAM_OFFSET(motor_phase_inductance),                       PARAM_VALUE_FLOAT, 0.0f,      10.0f,    PARAM_RULE_NONE,          PARAM_APPLY_NONE },

    { PROTOCOL_PARAM_ID_SPEED_OUT_MAX,        PARAM_OFFSET(speed_ctrl_params.out_max),                    PARAM_VALUE_FLOAT, 0.0f,      1.0f,     PARAM_RULE_SPEED_OUT_MAX, PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_SPEED_OUT_MIN,        PARAM_OFFSET(speed_ctrl_params.out_min),                    PARAM_VALUE_FLOAT, -1.0f,     0.0f,     PARAM_RULE_SPEED_OUT_MIN, PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_SPEED_LPF_MS,         PARAM_OFFSET(speed_ctrl_params.lpf_time_constant_ms),       PARAM_VALUE_FLOAT, 0.0f,      1000.0f,  PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_FF_ACCEL_GAIN,        PARAM_OFFSET(speed_ctrl_params.ff.acceleration_gain),       PARAM_VALUE_FLOAT, -100.0f,   100.0f,   PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_FF_FRICTION,          PARAM_OFFSET(speed_ctrl_params.ff.friction_compensation),   PARAM_VALUE_FLOAT, -100.0f,   100.0f,   PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_SPEED_BETA,           PARAM_OFFSET(speed_ctrl_params.beta),                       PARAM_VALUE_FLOAT, 0.0f,      1.0f,     PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_SPEED_CMD_LPF_MS,     PARAM_OFFSET(speed_ctrl_params.cmd_lpf_time_constant_ms),   PARAM_VALUE_FLOAT, 0.0f,      10000.0f, PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_PLL_BANDWIDTH_HZ,     PARAM_OFFSET(speed_ctrl_params.pll_bandwidth_hz),           PARAM_VALUE_FLOAT, 1.0f,      10000.0f, PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_PLL_ZETA,             PARAM_OFFSET(speed_ctrl_params.pll_zeta),                   PARAM_VALUE_FLOAT, 0.1f,      2.0f,     PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_SPEED_R,              PARAM_OFFSET(speed_ctrl_params.ff.R),                       PARAM_VALUE_FLOAT, 0.0f,      100.0f,   PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_SPEED_KE,             PARAM_OFFSET(speed_ctrl_params.ff.Ke),                      PARAM_VALUE_FLOAT, 0.0f,      100.0f,   PARAM_RULE_NONE,          PARAM_APPLY_SPEED_CTRL },
    { PROTOCOL_PARAM_ID_ENCODER_OFFSET,       PARAM_OFFSET(encoder_offset),                               PARAM_VALUE_INT16, -32768.0f, 32767.0f, PARAM_RULE_NONE,          PARAM_APPLY_NONE },
    { PROTOCOL_PARAM_ID_CALIB_POLE_PAIRS,     PARAM_OFFSET(motor_calib.pole_pairs),                       PARAM_VALUE_INT16, 1.0f,      64.0f,    PARAM_RULE_NONE,          PARAM_APPLY_CALIB },
    { PROTOCOL_PARAM_ID_CALIB_DIR,            PARAM_OFFSET(motor_calib.dir),                              PARAM_VALUE_INT16, -1.0f,     1.0f,     PARAM_RULE_CALIB_DIR,     PARAM_APPLY_CALIB },
    { PROTOCOL_PARAM_ID_CALIB_ELEC_OFFSET,    PARAM_OFFSET(motor_calib.electrical_offset_rad),            PARAM_VALUE_FLOAT, -10.0f,    10.0f,    PARAM_RULE_NONE,          PARAM_APPLY_CALIB },
    { PROTOCOL_PARAM_ID_CALIB_HARMONIC_AMP_1, PARAM_OFFSET(motor_calib.harmonic_amp_1),                   PARAM_VALUE_FLOAT, -10.0f,    10.0f,    PARAM_RULE_NONE,          PARAM_APPLY_CALIB },
    { PROTOCOL_PARAM_ID_CALIB_HARMONIC_PHS_1, PARAM_OFFSET(motor_calib.harmonic_phase_1),                 PARAM_VALUE_FLOAT, -10.0f,    10.0f,    PARAM_RULE_NONE,          PARAM_APPLY_CALIB },
    { PROTOCOL_PARAM_ID_CALIB_HARMONIC_AMP_2, PARAM_OFFSET(motor_calib.harmonic_amp_2),                   PARAM_VALUE_FLOAT, -10.0f,    10.0f,    PARAM_RULE_NONE,          PARAM_APPLY_CALIB },
    { PROTOCOL_PARAM_ID_CALIB_HARMONIC_PHS_2, PARAM_OFFSET(motor_calib.harmonic_phase_2),                 PARAM_VALUE_FLOAT, -10.0f,    10.0f,    PARAM_RULE_NONE,          PARAM_APPLY_CALIB }
};
static ParamStorage_Data_t s_param_backup;

static const ParamRegistryEntry *ParamRegistry_FindEntry(uint8_t param_id)
{
    uint8_t index;

    for (index = 0u; index < (uint8_t)(sizeof(s_param_table) / sizeof(s_param_table[0])); index++)
    {
        if (s_param_table[index].param_id == param_id)
        {
            return &s_param_table[index];
        }
    }

    return (const ParamRegistryEntry *)0;
}

static float ParamRegistry_ReadValue(const ParamRegistryEntry *entry, const ParamStorage_Data_t *params)
{
    const uint8_t *base;
    float value;

    value = 0.0f;
    base = (const uint8_t *)params;

    switch (entry->value_type)
    {
    case PARAM_VALUE_FLOAT:
        memcpy(&value, &base[entry->offset], sizeof(float));
        break;

    case PARAM_VALUE_INT16:
        {
            int16_t raw;

            memcpy(&raw, &base[entry->offset], sizeof(int16_t));
            value = (float)raw;
        }
        break;

    case PARAM_VALUE_BOOL8:
        value = (float)(base[entry->offset] != 0u);
        break;

    default:
        break;
    }

    return value;
}

static void ParamRegistry_WriteValue(const ParamRegistryEntry *entry, ParamStorage_Data_t *params, float value)
{
    uint8_t *base;

    base = (uint8_t *)params;

    switch (entry->value_type)
    {
    case PARAM_VALUE_FLOAT:
        memcpy(&base[entry->offset], &value, sizeof(float));
        break;

    case PARAM_VALUE_INT16:
        {
            int16_t raw_value;

            raw_value = (int16_t)value;
            memcpy(&base[entry->offset], &raw_value, sizeof(int16_t));
        }
        break;

    case PARAM_VALUE_BOOL8:
        base[entry->offset] = (uint8_t)(value != 0.0f);
        break;

    default:
        break;
    }
}

static ProtocolResultCode ParamRegistry_Validate(const ParamRegistryEntry *entry,
                                                 const ParamStorage_Data_t *params,
                                                 float value)
{
    if ((value < entry->min_value) || (value > entry->max_value))
    {
        return PROTOCOL_RESULT_INVALID_VALUE;
    }

    switch (entry->validation_rule)
    {
    case PARAM_RULE_BOOL:
        if (!((value == 0.0f) || (value == 1.0f)))
        {
            return PROTOCOL_RESULT_INVALID_VALUE;
        }
        break;

    case PARAM_RULE_CALIB_DIR:
        if (!((value == -1.0f) || (value == 1.0f)))
        {
            return PROTOCOL_RESULT_INVALID_VALUE;
        }
        break;

    case PARAM_RULE_SPEED_OUT_MAX:
        if (value <= params->speed_ctrl_params.out_min)
        {
            return PROTOCOL_RESULT_INVALID_VALUE;
        }
        break;

    case PARAM_RULE_SPEED_OUT_MIN:
        if (value >= params->speed_ctrl_params.out_max)
        {
            return PROTOCOL_RESULT_INVALID_VALUE;
        }
        break;

    default:
        break;
    }

    return PROTOCOL_RESULT_OK;
}

static void ParamRegistry_ApplyAction(ParamStorage_Data_t *params, uint8_t action)
{
    float target_speed;

    switch (action)
    {
    case PARAM_APPLY_SPEED_CTRL:
        target_speed = SpeedController_GetTarget();
        SpeedController_RefreshParams(&params->speed_ctrl_params);
        SpeedController_SetTarget(target_speed);
        break;

    case PARAM_APPLY_CALIB:
        Motor_Calibration_Init(&params->motor_calib);
        break;

    case PARAM_APPLY_LIMITS:
        target_speed = SpeedController_GetTarget();
        if (target_speed > params->velocity_limit)
        {
            target_speed = params->velocity_limit;
        }
        else if (target_speed < -params->velocity_limit)
        {
            target_speed = -params->velocity_limit;
        }
        SpeedController_SetTarget(target_speed);
        break;

    default:
        break;
    }
}

void ParamRegistry_ApplyAllRuntime(void)
{
    ParamStorage_Data_t *params;

    params = ParamStorage_Get();
    Motor_Calibration_Init(&params->motor_calib);
    SpeedController_Init(&params->speed_ctrl_params);

}

ProtocolResultCode ParamRegistry_ReadParam(uint8_t param_id, float *value)
{
    const ParamRegistryEntry *entry;

    if (value == (float *)0)
    {
        return PROTOCOL_RESULT_INTERNAL_ERROR;
    }

    entry = ParamRegistry_FindEntry(param_id);
    if (entry == (const ParamRegistryEntry *)0)
    {
        return PROTOCOL_RESULT_INVALID_PARAM;
    }

    *value = ParamRegistry_ReadValue(entry, ParamStorage_Get());
    return PROTOCOL_RESULT_OK;
}

ProtocolResultCode ParamRegistry_WriteParam(uint8_t param_id, float value)
{
    const ParamRegistryEntry *entry;
    ParamStorage_Data_t *params;
    ProtocolResultCode validate_result;

    entry = ParamRegistry_FindEntry(param_id);
    if (entry == (const ParamRegistryEntry *)0)
    {
        return PROTOCOL_RESULT_INVALID_PARAM;
    }

    params = ParamStorage_Get();
    validate_result = ParamRegistry_Validate(entry, params, value);
    if (validate_result != PROTOCOL_RESULT_OK)
    {
        return validate_result;
    }

    s_param_backup = *params;
    ParamRegistry_WriteValue(entry, params, value);
    ParamRegistry_ApplyAction(params, entry->apply_action);

    if (!ParamStorage_Save())
    {
        *params = s_param_backup;
        ParamRegistry_ApplyAllRuntime();
        return PROTOCOL_RESULT_EEPROM_ERROR;
    }

    return PROTOCOL_RESULT_OK;
}

void ParamRegistry_FillSnapshot(ParamSnapshotPayload *payload)
{
    ParamStorage_Data_t *params;

    if (payload == (ParamSnapshotPayload *)0)
    {
        return;
    }

    params = ParamStorage_Get();
    memset(payload, 0, sizeof(ParamSnapshotPayload));
    payload->motor_pole_pairs = params->motor_pole_pairs;
    payload->motor_phase_resistance = params->motor_phase_resistance;
    payload->motor_phase_inductance = params->motor_phase_inductance;

    payload->speed_kp = params->speed_ctrl_params.kp;
    payload->speed_ki = params->speed_ctrl_params.ki;
    payload->speed_out_max = params->speed_ctrl_params.out_max;
    payload->speed_out_min = params->speed_ctrl_params.out_min;
    payload->speed_lpf_time_constant_ms = params->speed_ctrl_params.lpf_time_constant_ms;
    payload->ff_acceleration_gain = params->speed_ctrl_params.ff.acceleration_gain;
    payload->ff_friction_compensation = params->speed_ctrl_params.ff.friction_compensation;
    payload->speed_beta = params->speed_ctrl_params.beta;
    payload->speed_cmd_lpf_ms = params->speed_ctrl_params.cmd_lpf_time_constant_ms;
    payload->pll_bandwidth_hz = params->speed_ctrl_params.pll_bandwidth_hz;
    payload->pll_zeta = params->speed_ctrl_params.pll_zeta;
    payload->speed_R = params->speed_ctrl_params.ff.R;
    payload->speed_Ke = params->speed_ctrl_params.ff.Ke;

    payload->velocity_limit = params->velocity_limit;
    payload->encoder_offset = params->encoder_offset;
    payload->calib_pole_pairs = (int16_t)params->motor_calib.pole_pairs;
    payload->calib_dir = (int16_t)params->motor_calib.dir;
    payload->calib_electrical_offset_rad = params->motor_calib.electrical_offset_rad;
    payload->calib_harmonic_amp_1 = params->motor_calib.harmonic_amp_1;
    payload->calib_harmonic_phase_1 = params->motor_calib.harmonic_phase_1;
    payload->calib_harmonic_amp_2 = params->motor_calib.harmonic_amp_2;
    payload->calib_harmonic_phase_2 = params->motor_calib.harmonic_phase_2;
}
