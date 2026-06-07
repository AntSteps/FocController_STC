#include "command.h"
#include "uart_controller.h"
#include "param_registry.h"
#include "telemetry_service.h"
#include "calibration_service.h"
#include "param_storage.h"
#include "motor.h"
#include "control.h"

typedef struct
{
    uint8_t mode_id;
    uint8_t requires_target;
} ControlModeOps;

static const ControlModeOps s_control_mode_table[] =
{
    { PROTOCOL_CTRL_MODE_STOP, 0u },
    { PROTOCOL_CTRL_MODE_SPEED, 1u }
};
static ProtocolReply s_command_reply;

static const ControlModeOps *Command_FindControlMode(uint8_t mode_id)
{
    uint8_t index;

    for (index = 0u; index < (uint8_t)(sizeof(s_control_mode_table) / sizeof(s_control_mode_table[0])); index++)
    {
        if (s_control_mode_table[index].mode_id == mode_id)
        {
            return &s_control_mode_table[index];
        }
    }

    return (const ControlModeOps *)0;
}

static void Command_SendReply(const ProtocolReply *reply)
{
    (void)ProtocolController_SendReply(reply);
}

static ProtocolResultCode Command_HandleControl(const ProtocolRequest *request)
{
    const ControlModeOps *mode_ops;
    ParamStorage_Data_t *params;
    float target;

    mode_ops = Command_FindControlMode(request->control_mode);
    if (mode_ops == (const ControlModeOps *)0)
    {
        return PROTOCOL_RESULT_UNSUPPORTED;
    }

    params = ParamStorage_Get();
    target = request->target;

    if (mode_ops->requires_target)
    {
        if ((target > params->velocity_limit) || (target < -params->velocity_limit))
        {
            return PROTOCOL_RESULT_INVALID_VALUE;
        }
    }

    switch (mode_ops->mode_id)
    {
    case PROTOCOL_CTRL_MODE_STOP:
        SpeedController_SetTarget(0.0f);
        Motor_SetControlMode(MOTOR_CONTROL_MODE_STOP);
        break;

    case PROTOCOL_CTRL_MODE_SPEED:
        SpeedController_SetTarget(target);
        SpeedController_SetLoadFF(request->load_ff);
        Motor_SetControlMode(MOTOR_CONTROL_MODE_SPEED);
        break;

    default:
        return PROTOCOL_RESULT_UNSUPPORTED;
    }

    return PROTOCOL_RESULT_OK;
}

static void Command_HandleQuery(const ProtocolRequest *request)
{
    ProtocolResultCode result;
    float value;

    if (CalibrationService_IsBusy())
    {
        if (!((request->query_type == PROTOCOL_QUERY_GET_STATE) ||
              (request->query_type == PROTOCOL_QUERY_GET_CALIB_STATUS)))
        {
            TelemetryService_BuildAckReply(&s_command_reply,
                                           PROTOCOL_REQUEST_QUERY,
                                           PROTOCOL_RESULT_BUSY,
                                           request->seq);
            Command_SendReply(&s_command_reply);
            return;
        }
    }

    switch (request->query_type)
    {
    case PROTOCOL_QUERY_GET_STATE:
        TelemetryService_BuildStateReply(&s_command_reply, request->seq);
        Command_SendReply(&s_command_reply);
        break;

    case PROTOCOL_QUERY_GET_PARAM:
        result = ParamRegistry_ReadParam(request->arg0, &value);
        if (result != PROTOCOL_RESULT_OK)
        {
            TelemetryService_BuildAckReply(&s_command_reply,
                                           PROTOCOL_REQUEST_QUERY,
                                           (uint8_t)result,
                                           request->seq);
        }
        else
        {
            TelemetryService_BuildParamValueReply(&s_command_reply,
                                                  request->seq,
                                                  request->arg0,
                                                  value);
        }
        Command_SendReply(&s_command_reply);
        break;

    case PROTOCOL_QUERY_GET_PARAM_SNAPSHOT:
        TelemetryService_BuildParamSnapshotReply(&s_command_reply, request->seq);
        Command_SendReply(&s_command_reply);
        break;

    case PROTOCOL_QUERY_GET_CALIB_STATUS:
        TelemetryService_BuildCalibStatusReply(&s_command_reply,
                                               request->seq,
                                               CalibrationService_GetState(),
                                               CalibrationService_GetProgress());
        Command_SendReply(&s_command_reply);
        break;

    default:
        TelemetryService_BuildAckReply(&s_command_reply,
                                       PROTOCOL_REQUEST_QUERY,
                                       PROTOCOL_RESULT_UNSUPPORTED,
                                       request->seq);
        Command_SendReply(&s_command_reply);
        break;
    }
}
void ProcessCommand(const ProtocolRequest *request)
{
    ProtocolResultCode result;

    if (request == (const ProtocolRequest *)0)
    {
        return;
    }

    switch (request->kind)
    {
    case PROTOCOL_REQUEST_CTRL:
        if (CalibrationService_IsBusy())
        {
            result = PROTOCOL_RESULT_BUSY;
        }
        else
        {
            result = Command_HandleControl(request);
        }
        TelemetryService_BuildAckReply(&s_command_reply,
                                       PROTOCOL_REQUEST_CTRL,
                                       (uint8_t)result,
                                       request->seq);
        Command_SendReply(&s_command_reply);
        break;

    case PROTOCOL_REQUEST_PARAM_WRITE:
        if (CalibrationService_IsBusy())
        {
            result = PROTOCOL_RESULT_BUSY;
        }
        else
        {
            result = ParamRegistry_WriteParam(request->param_id,
                                              request->value);
        }
        TelemetryService_BuildAckReply(&s_command_reply,
                                       PROTOCOL_REQUEST_PARAM_WRITE,
                                       (uint8_t)result,
                                       request->seq);
        Command_SendReply(&s_command_reply);
        break;

    case PROTOCOL_REQUEST_QUERY:
        Command_HandleQuery(request);
        break;

    case PROTOCOL_REQUEST_CALIB:
        if (request->calib_cmd != PROTOCOL_CALIB_CMD_START_AUTO)
        {
            result = PROTOCOL_RESULT_UNSUPPORTED;
        }
        else
        {
            result = CalibrationService_Start();
        }
        TelemetryService_BuildAckReply(&s_command_reply,
                                       PROTOCOL_REQUEST_CALIB,
                                       (uint8_t)result,
                                       request->seq);
        Command_SendReply(&s_command_reply);
        break;

    default:
        TelemetryService_BuildAckReply(&s_command_reply,
                                       0u,
                                       PROTOCOL_RESULT_UNSUPPORTED,
                                       request->seq);
        Command_SendReply(&s_command_reply);
        break;
    }
}
