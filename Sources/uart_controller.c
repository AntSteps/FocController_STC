#include "uart_controller.h"
#include "uart.h"
#include "protocol_parser.h"

#define PROTOCOL_REQUEST_QUEUE_SIZE  4u

static ProtocolParser s_protocol_parser;
static ProtocolRequest s_request_queue[PROTOCOL_REQUEST_QUEUE_SIZE];
static uint8_t s_request_read_index = 0u;
static uint8_t s_request_write_index = 0u;
static uint8_t s_request_count = 0u;
static uint8_t s_protocol_tx_frame[PROTOCOL_MAX_FRAME_SIZE];

static BOOL ProtocolController_EnqueueRequest(const ProtocolRequest *request)
{
    if ((request == (const ProtocolRequest *)0) || (s_request_count >= PROTOCOL_REQUEST_QUEUE_SIZE))
    {
        return FALSE;
    }

    s_request_queue[s_request_write_index] = *request;
    s_request_write_index = (uint8_t)((s_request_write_index + 1u) % PROTOCOL_REQUEST_QUEUE_SIZE);
    s_request_count++;
    return TRUE;
}

void ProtocolController_Init(void)
{
    ProtocolParser_Init(&s_protocol_parser);
    s_request_read_index = 0u;
    s_request_write_index = 0u;
    s_request_count = 0u;
}

void ProtocolController_PumpRx(void)
{
    uint8_t rx_len;
    uint8_t xdata *rx_buffer;
    ProtocolRequest request;

    rx_buffer = UART_GetRxBuffer(&rx_len);
    if ((rx_buffer == (uint8_t xdata *)0) || (rx_len == 0u))
    {
        return;
    }

    ProtocolParser_Feed(&s_protocol_parser, rx_buffer, rx_len);
    UART_ReleaseRxBuffer();

    while (ProtocolParser_Next(&s_protocol_parser, &request))
    {
        if (!ProtocolController_EnqueueRequest(&request))
        {
            break;
        }
    }
}

BOOL ProtocolController_PopRequest(ProtocolRequest *request)
{
    if ((request == (ProtocolRequest *)0) || (s_request_count == 0u))
    {
        return FALSE;
    }

    *request = s_request_queue[s_request_read_index];
    s_request_read_index = (uint8_t)((s_request_read_index + 1u) % PROTOCOL_REQUEST_QUEUE_SIZE);
    s_request_count--;
    return TRUE;
}

BOOL ProtocolController_SendReply(const ProtocolReply *reply)
{
    uint8_t frame_len;

    if (!ProtocolFrame_EncodeReply(reply, s_protocol_tx_frame, &frame_len))
    {
        return FALSE;
    }

    return (BOOL)UART_DMA_Send(s_protocol_tx_frame, frame_len);
}
