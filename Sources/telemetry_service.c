#include "telemetry_service.h"
#include "encoder.h"
#include "param_storage.h"
#include "motor.h"
#include "control.h"
#include "adc.h"
#include "calibration.h"
#include "param_registry.h"
#include <string.h>

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
    memset(reply, 0, sizeof(ProtocolReply));
    reply->kind = PROTOCOL_REPLY_ACK;
    reply->seq = seq;
    reply->cmd_type = cmd_type;
    reply->result_code = result_code;
}

void TelemetryService_BuildStateReply(ProtocolReply *reply, uint8_t seq)
{
    memset(reply, 0, sizeof(ProtocolReply));
    reply->kind = PROTOCOL_REPLY_STATE;
    reply->seq = seq;
    reply->control_mode = Motor_GetControlMode();
    reply->target = SpeedController_GetTarget();
    reply->velocity = SpeedController_GetFilteredSpeed();
    reply->output = Motor_GetQAxisVoltage();
    reply->position = Motor_Calibration_GetMechanicalAngle(Encoder_Get_Angle());
    reply->voltage = ADC_GetBusVoltage();
    reply->health_status = TelemetryService_BuildHealthStatus();
}

void TelemetryService_BuildParamValueReply(ProtocolReply *reply, uint8_t seq, uint8_t param_id, float value)
{
    memset(reply, 0, sizeof(ProtocolReply));
    reply->kind = PROTOCOL_REPLY_PARAM_VALUE;
    reply->seq = seq;
    reply->param_id = param_id;
    reply->value = value;
}

void TelemetryService_BuildParamSnapshotReply(ProtocolReply *reply, uint8_t seq)
{
    memset(reply, 0, sizeof(ProtocolReply));
    reply->kind = PROTOCOL_REPLY_PARAM_SNAPSHOT;
    reply->seq = seq;
    reply->snapshot_version = PROTOCOL_PARAM_SNAPSHOT_VERSION;
    ParamRegistry_FillSnapshot(&reply->payload);
}

void TelemetryService_BuildCalibStatusReply(ProtocolReply *reply, uint8_t seq, uint8_t calib_state, uint8_t progress)
{
    memset(reply, 0, sizeof(ProtocolReply));
    reply->kind = PROTOCOL_REPLY_CALIB_STATUS;
    reply->seq = seq;
    reply->calib_state = calib_state;
    reply->progress = progress;
}
