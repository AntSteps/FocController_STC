#include "uart_controller.h"
#include "uart.h"
#include "uart_send_def.h"
#include "crc8.h"



static uint8_t UARTController_MatchHead(const uint8_t xdata *buf, uint8_t head0, uint8_t head1)
{
    return (uint8_t)((buf[0] == head0) && (buf[1] == head1));
}

static void UARTController_CopyFrameFromXdata(uint8_t *dst, const uint8_t xdata *src, uint8_t size)
{
    uint8_t index;

    for (index = 0; index < size; index++)
    {
        dst[index] = src[index];
    }
}

static uint8_t UARTController_CheckFrame(const uint8_t *buf, uint8_t size)
{
    return (uint8_t)(Mem_CRC8_Check((uint8_t *)buf, size, 1) == 0);
}


static void UARTController_ReleaseCurrentBuffer(void)
{
    UART_ReleaseRxBuffer();
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

uint8_t send_messages(ReplyFrame *frame)
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
    return UART_DMA_Send(tx_frame, sizeof(ReplyFrame));
}

uint8_t UARTController_GetCommand(UartCommand *command)
{
    uint8_t remain;

    uint8_t rx_len;
    uint8_t xdata *rx_buffer = UART_GetRxBuffer(&rx_len);

    UARTController_ReleaseCurrentBuffer();
    if (rx_len == 0 || rx_buffer == (uint8_t xdata *)0)
    {
        return 0;
    }

    return 1;
}
