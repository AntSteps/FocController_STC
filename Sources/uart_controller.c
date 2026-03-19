#include "uart_controller.h"
#include "uart.h"
#include "uart_send_def.h"
#include "crc8.h"

CmdFrame cmdframe = {0};
ParamCmdFrame paramcmdframe = {0};
ReplyFrame replyframe = {0};

static uint8_t xdata *s_rx_buffer = (uint8_t xdata *)0;
static uint8_t s_rx_length = 0;
static uint8_t s_rx_offset = 0;

static uint8_t UARTController_MatchHead(const uint8_t xdata *buf, uint8_t head0, uint8_t head1)
{
    return (uint8_t)((buf[0] == head0) && (buf[1] == head1));
}

static uint8_t UARTController_CheckFrame(const uint8_t xdata *buf, uint8_t size)
{
    return (uint8_t)(Mem_CRC8_Check((uint8_t *)buf, size, 1) == 0);
}

static void UARTController_ReleaseCurrentBuffer(void)
{
    UART_ReleaseRxBuffer();
    s_rx_buffer = (uint8_t xdata *)0;
    s_rx_length = 0;
    s_rx_offset = 0;
}

static void UARTController_WriteU16BE(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)value;
}

static void UARTController_WriteFloatBE(uint8_t *buf, float value)
{
    // uint8_t raw[4];
    // memcpy(raw, &value, 4); 
    memcpy(buf, &value, 4);
    // buf[0] = raw[3];
    // buf[1] = raw[2];
    // buf[2] = raw[1];
    // buf[3] = raw[0];
}

void send_messages(ReplyFrame *frame)
{
    uint8_t tx_frame[sizeof(ReplyFrame)];

    UARTController_WriteU16BE(&tx_frame[0], frame->head);
    tx_frame[2] = frame->id;
    UARTController_WriteFloatBE(&tx_frame[3], frame->velocity);
    UARTController_WriteFloatBE(&tx_frame[7], frame->output);
    UARTController_WriteFloatBE(&tx_frame[11], frame->position);
    UARTController_WriteFloatBE(&tx_frame[15], frame->voltage);
    tx_frame[19] = frame->status;
    tx_frame[20] = frame->seq;
    tx_frame[21] = Mem_CRC8_Check(tx_frame, sizeof(ReplyFrame) - 1, 0);

    frame->crc8 = tx_frame[21];
    UART_DMA_Send(tx_frame, sizeof(ReplyFrame));
}

uint8_t UARTController_GetCommand(UartCommand *command)
{
    uint8_t remain;
    uint8_t xdata *frame_ptr;

    if (s_rx_buffer == (uint8_t xdata *)0)
    {
        s_rx_buffer = UART_GetRxBuffer(&s_rx_length);
        if (s_rx_buffer == (uint8_t xdata *)0)
        {
            return 1;
        }
        s_rx_offset = 0;
    }

    while (s_rx_offset < s_rx_length)
    {
        remain = (uint8_t)(s_rx_length - s_rx_offset);
        frame_ptr = s_rx_buffer + s_rx_offset;

        if ((remain >= sizeof(CmdFrame)) &&
            UARTController_MatchHead(frame_ptr, CTRL_CMD_HEAD_BYTE0, CTRL_CMD_HEAD_BYTE1) &&
            UARTController_CheckFrame(frame_ptr, sizeof(CmdFrame)))
        {
            cmdframe = *(CmdFrame *)frame_ptr;
            command->type = UART_COMMAND_CTRL;
            command->ctrl = cmdframe;
            s_rx_offset = (uint8_t)(s_rx_offset + sizeof(CmdFrame));
            if (s_rx_offset >= s_rx_length)
            {
                UARTController_ReleaseCurrentBuffer();
            }
            return 0;
        }

        if ((remain >= sizeof(ParamCmdFrame)) &&
            UARTController_MatchHead(frame_ptr, PARAM_CMD_HEAD_BYTE0, PARAM_CMD_HEAD_BYTE1) &&
            UARTController_CheckFrame(frame_ptr, sizeof(ParamCmdFrame)))
        {
            paramcmdframe = *(ParamCmdFrame *)frame_ptr;
            command->type = UART_COMMAND_PARAM;
            command->param = paramcmdframe;
            s_rx_offset = (uint8_t)(s_rx_offset + sizeof(ParamCmdFrame));
            if (s_rx_offset >= s_rx_length)
            {
                UARTController_ReleaseCurrentBuffer();
            }
            return 0;
        }

        s_rx_offset++;
    }

    UARTController_ReleaseCurrentBuffer();
    return 1;
}
