#include "uart_controller.h"
#include "uart_send_def.h"
#include "param_storage.h"
#include "calibration.h"
#include "encoder.h"
#include "adc.h"
#include "motor.h"

#define REPLY_STATUS_ENCODER_FAULT      0x01u
#define REPLY_STATUS_MOTOR_FAULT        0x02u
#define REPLY_STATUS_ENC_MAG_SHIFT      2u
#define REPLY_STATUS_ENC_PRESS          0x10u
#define REPLY_STATUS_ENC_OVERSPEED      0x20u
#define REPLY_STATUS_ENC_DATA_GOOD      0x40u
#define REPLY_STATUS_PARAM_FROM_EEPROM  0x80u

static void Command_DebugPulse(uint8_t count)
{
    while (count--)
    {
        P36 = 1;
        delay_us(20);
        P36 = 0;
        delay_us(20);
    }
}

static void ApplyControlCommand(CtrlCmdFrame *frame);
static void ApplyParamCommand(ParamCmdFrame *frame);
static uint8_t Command_BuildReplyStatus(void);
void UartReply(uint8_t seq);
void ProcessCommand(UartCommand *command)
{
    Command_DebugPulse(10);

    switch (command->type)
    {
    case UART_COMMAND_CTRL:
        ApplyControlCommand(&command->ctrl);
        UartReply(command->ctrl.seq); // 立即回复当前状态
        break;

    case UART_COMMAND_PARAM:
        ApplyParamCommand(&command->param);
        break;

    default:
        break;
    }
}

static void ApplyControlCommand(CtrlCmdFrame *frame)
{
    ParamStorage_Data_t *params;
    float target;

    params = ParamStorage_Get();
    switch (frame->ctrl)
    {
    case 0:
        SpeedController_SetTarget(0.0f);
        break;

    case 1:
        target = frame->target;
        if (target > params->velocity_limit)
        {
            target = params->velocity_limit;
        }
        else if (target < -params->velocity_limit)
        {
            target = -params->velocity_limit;
        }
        SpeedController_SetTarget(target);
        break;

    default:
        break;
    }
}

static void ApplyParamCommand(ParamCmdFrame *frame)
{
    ParamStorage_Data_t *params;

    params = ParamStorage_Get();
    switch (frame->param)
    {
    case PARAM_CMD_ID_VEL_KP:
        params->speed_ctrl_params.kp = frame->value;
        if (g_speed_ctrl.params != NULL)
        {
            PI_Init(&g_speed_ctrl.pi,
                    params->speed_ctrl_params.kp,
                    params->speed_ctrl_params.ki,
                    params->speed_ctrl_params.out_max,
                    params->speed_ctrl_params.out_min);
        }
        break;

    case PARAM_CMD_ID_VEL_KI:
        params->speed_ctrl_params.ki = frame->value;
        if (g_speed_ctrl.params != NULL)
        {
            PI_Init(&g_speed_ctrl.pi,
                    params->speed_ctrl_params.kp,
                    params->speed_ctrl_params.ki,
                    params->speed_ctrl_params.out_max,
                    params->speed_ctrl_params.out_min);
        }
        break;

    case PARAM_CMD_ID_VEL_LIMIT:
        params->velocity_limit = frame->value;
        break;

    case PARAM_CMD_ID_CUR_LIMIT:
        params->current_limit = frame->value;
        break;

    case PARAM_CMD_ID_FF_GAIN:
        params->speed_ctrl_params.ff.velocity_gain = frame->value;
        break;

    case PARAM_CMD_ID_FF_ENABLE:
        params->speed_ctrl_params.enable_feedforward = (uint8_t)(frame->value != 0.0f);
        break;

    default:
        return;
    }

    ParamStorage_Save();
}

static uint8_t Command_BuildReplyStatus(void)
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
        status |= REPLY_STATUS_ENC_DATA_GOOD;
    }

    status |= (uint8_t)((mag_field_status & 0x03u) << REPLY_STATUS_ENC_MAG_SHIFT);

    if (Encoder_Get_PressStatus())
    {
        status |= REPLY_STATUS_ENC_PRESS;
    }

    if (speed_status)
    {
        status |= REPLY_STATUS_ENC_OVERSPEED;
    }

    if ((!data_good) ||
        (mag_field_status != ENCODER_MAG_FIELD_NORMAL) ||
        speed_status)
    {
        status |= REPLY_STATUS_ENCODER_FAULT;
    }

    if (ParamStorage_WasLoadedFromEEPROM())
    {
        status |= REPLY_STATUS_PARAM_FROM_EEPROM;
    }

    return status;
}

void UartReply(uint8_t seq)
{
    uint8_t send_ok;
    ReplyFrame replyframe;


    replyframe.head = REPLY_FRAME_HEAD;  
    replyframe.id = 0x01;
    replyframe.velocity = 3.14159f;
    replyframe.output = Motor_GetQAxisVoltage();
    replyframe.position = Motor_Calibration_GetMechanicalAngle(Encoder_Get_Angle());
    replyframe.voltage = ADC_GetBusVoltage();
    replyframe.status = Command_BuildReplyStatus();
    replyframe.seq = seq;
    send_ok = send_messages(&replyframe);

}
