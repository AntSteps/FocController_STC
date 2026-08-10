#include "telemetry_service.h"
#include "encoder.h"
#include "param_storage.h"
#include "motor.h"
#include "control.h"
#include "adc.h"
#include "calibration.h"
#include "param_registry.h"

#define FAST_POSITION_SCALE  10430.37835f

static int16_t TelemetryService_EncodeI16(float value, float scale)
{
    float scaled;

    scaled = value * scale;
    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    if (scaled >= 0.0f) scaled += 0.5f;
    else scaled -= 0.5f;
    return (int16_t)scaled;
}

static uint16_t TelemetryService_EncodeU16(float value, float scale)
{
    float scaled;

    scaled = value * scale;
    if (scaled <= 0.0f) return 0u;
    if (scaled >= 65535.0f) return 65535u;
    return (uint16_t)(scaled + 0.5f);
}

uint8_t TelemetryService_BuildHealthStatus(void)
{
    uint8_t status;
    uint8_t data_good;
    uint8_t mag_field_status;
    uint8_t speed_status;

    status = 0u;
    data_good = Encoder_Get_DataGood();
    mag_field_status = Encoder_Get_MagFieldStatus();
    speed_status = Encoder_Get_SpeedStatus();

    if (data_good)
    {
        status |= PROTOCOL_STATUS_ENC_DATA_GOOD;
    }

    status |= (uint8_t)((mag_field_status & 0x03u) << PROTOCOL_STATUS_ENC_MAG_SHIFT);

    if (PORT_IsLowVoltageReset())
    {
        status |= PROTOCOL_STATUS_RESET_LOW_VOLTAGE;
    }

    if (PORT_IsDriverNFaultActive())
    {
        status |= PROTOCOL_STATUS_DRIVER_NFAULT;
    }

    if ((!data_good) ||
        (mag_field_status != ENCODER_MAG_FIELD_NORMAL) ||
        speed_status)
    {
        status |= PROTOCOL_STATUS_ENCODER_FAULT;
    }

    if (ParamStorage_WasLoadedFromEEPROM())
    {
        status |= PROTOCOL_STATUS_PARAM_FROM_EEPROM;
    }

    return status;
}

void TelemetryService_BuildAckReply(ProtocolReply *reply, uint8_t cmd_type, uint8_t result_code, uint8_t seq)
{
    reply->kind = PROTOCOL_REPLY_ACK;
    reply->seq = seq;
    reply->cmd_type = cmd_type;
    reply->result_code = result_code;
}

void TelemetryService_BuildStateReply(ProtocolReply *reply, uint8_t seq)
{
    reply->kind = PROTOCOL_REPLY_STATE;
    reply->seq = seq;
    reply->control_mode = Motor_GetControlMode();
    reply->target = SpeedController_GetTarget();
    reply->velocity = SpeedController_GetFilteredSpeed();
    reply->output = Motor_GetQAxisVoltage();
    reply->position = Motor_GetCachedMechanicalAngle();
    reply->voltage = ADC_GetBusVoltage();
    reply->health_status = TelemetryService_BuildHealthStatus();
}

void TelemetryService_BuildParamValueReply(ProtocolReply *reply, uint8_t seq, uint8_t param_id, float value)
{
    reply->kind = PROTOCOL_REPLY_PARAM_VALUE;
    reply->seq = seq;
    reply->param_id = param_id;
    reply->value = value;
}

void TelemetryService_BuildParamSnapshotReply(ProtocolReply *reply, uint8_t seq)
{
    reply->kind = PROTOCOL_REPLY_PARAM_SNAPSHOT;
    reply->seq = seq;
    reply->snapshot_version = PROTOCOL_PARAM_SNAPSHOT_VERSION;
    ParamRegistry_FillSnapshot(&reply->payload);
}

void TelemetryService_BuildCalibStatusReply(ProtocolReply *reply, uint8_t seq, uint8_t calib_state, uint8_t progress)
{
    reply->kind = PROTOCOL_REPLY_CALIB_STATUS;
    reply->seq = seq;
    reply->calib_state = calib_state;
    reply->progress = progress;
}

void TelemetryService_BuildCalibSampleReply(ProtocolReply *reply,
                                            uint16_t sample_index,
                                            int8_t direction,
                                            float command_theta,
                                            float mechanical_angle,
                                            float electrical_error)
{
    reply->kind = PROTOCOL_REPLY_CALIB_SAMPLE;
    reply->calib_sample_index = sample_index;
    reply->calib_sample_direction = direction;
    reply->calib_sample_theta = command_theta;
    reply->calib_sample_mechanical_angle = mechanical_angle;
    reply->calib_sample_error = electrical_error;
}

void TelemetryService_BuildFastStatusReply(ProtocolReply *reply, BOOL command_clamped)
{
    uint8_t health;
    uint8_t status;
    float bus_voltage;

    health = TelemetryService_BuildHealthStatus();
    bus_voltage = ADC_GetBusVoltage();
    status = 0u;

    if (Motor_GetControlMode() == MOTOR_CONTROL_MODE_SPEED)
    {
        status |= PROTOCOL_FAST_STATUS_MOTOR_ENABLED;
    }
    if (command_clamped)
    {
        status |= PROTOCOL_FAST_STATUS_COMMAND_CLAMPED;
    }
    if (SpeedController_IsOutputSaturated())
    {
        status |= PROTOCOL_FAST_STATUS_OUTPUT_SATURATED;
    }
    if ((health & PROTOCOL_STATUS_ENCODER_FAULT) != 0u)
    {
        status |= PROTOCOL_FAST_STATUS_ENCODER_FAULT;
    }
    if ((health & PROTOCOL_STATUS_DRIVER_NFAULT) != 0u)
    {
        status |= PROTOCOL_FAST_STATUS_DRIVER_FAULT;
    }
    if (bus_voltage <= MOTOR_MIN_BUS_VOLTAGE)
    {
        status |= PROTOCOL_FAST_STATUS_BUS_UNDERVOLTAGE;
    }
    if ((health & PROTOCOL_STATUS_MOTOR_FAULT) != 0u)
    {
        status |= PROTOCOL_FAST_STATUS_MOTOR_FAULT;
    }

    reply->kind = PROTOCOL_REPLY_FAST_STATUS;
    reply->fast_status = status;
    reply->fast_velocity = TelemetryService_EncodeI16(
        SpeedController_GetFilteredSpeed(), PROTOCOL_FAST_SPEED_SCALE);
    reply->fast_iq_estimate = TelemetryService_EncodeI16(
        SpeedController_GetEstimatedCurrent(), 1000.0f);
    reply->fast_position = TelemetryService_EncodeU16(
        Motor_GetCachedMechanicalAngle(), FAST_POSITION_SCALE);
    reply->fast_bus_voltage = TelemetryService_EncodeU16(bus_voltage, 1000.0f);
}
