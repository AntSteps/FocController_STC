#include "uart_controller.h"
#include "uart.h"
#include <string.h>

static uint8_t s_protocol_rx_frame[UART3_RX_DMA_BUFFER_SIZE];
static uint8_t s_protocol_rx_stream[PROTOCOL_MAX_FRAME_SIZE];
static uint8_t s_protocol_rx_stream_len = 0u;
static uint8_t s_protocol_tx_frame[PROTOCOL_MAX_FRAME_SIZE];

static void ProtocolController_AppendChunk(const uint8_t *chunk, uint8_t chunk_len)
{
    if ((chunk == (const uint8_t *)0) || (chunk_len == 0u))
    {
        return;
    }

    if ((uint16_t)s_protocol_rx_stream_len + (uint16_t)chunk_len > (uint16_t)sizeof(s_protocol_rx_stream))
    {
        s_protocol_rx_stream_len = 0u;
    }

    memcpy(&s_protocol_rx_stream[s_protocol_rx_stream_len], chunk, chunk_len);
    s_protocol_rx_stream_len = (uint8_t)(s_protocol_rx_stream_len + chunk_len);
}

static BOOL ProtocolController_ExtractRequest(ProtocolRequest *request)
{
    uint8_t offset;
    uint8_t frame_len;
    uint8_t found_head;
    uint8_t remaining_len;

    if (request == (ProtocolRequest *)0)
    {
        return FALSE;
    }

    while (s_protocol_rx_stream_len >= 2u)
    {
        found_head = FALSE;
        frame_len = 0u;
        for (offset = 0u; (uint8_t)(offset + 1u) < s_protocol_rx_stream_len; offset++)
        {
            if (ProtocolFrame_FindRequestLength(s_protocol_rx_stream[offset],
                                                s_protocol_rx_stream[offset + 1u],
                                                &frame_len))
            {
                found_head = TRUE;
                break;
            }
        }

        if (!found_head)
        {
            s_protocol_rx_stream[0] = s_protocol_rx_stream[s_protocol_rx_stream_len - 1u];
            s_protocol_rx_stream_len = 1u;
            return FALSE;
        }

        if (offset > 0u)
        {
            remaining_len = (uint8_t)(s_protocol_rx_stream_len - offset);
            memmove(s_protocol_rx_stream, &s_protocol_rx_stream[offset], remaining_len);
            s_protocol_rx_stream_len = remaining_len;
        }

        if (s_protocol_rx_stream_len < frame_len)
        {
            return FALSE;
        }

        if (!ProtocolFrame_CheckCrc(s_protocol_rx_stream, frame_len))
        {
            memmove(s_protocol_rx_stream,
                    &s_protocol_rx_stream[1],
                    (uint8_t)(s_protocol_rx_stream_len - 1u));
            s_protocol_rx_stream_len = (uint8_t)(s_protocol_rx_stream_len - 1u);
            continue;
        }

        if (!ProtocolFrame_DecodeRequest(s_protocol_rx_stream, frame_len, request))
        {
            memmove(s_protocol_rx_stream,
                    &s_protocol_rx_stream[1],
                    (uint8_t)(s_protocol_rx_stream_len - 1u));
            s_protocol_rx_stream_len = (uint8_t)(s_protocol_rx_stream_len - 1u);
            continue;
        }

        remaining_len = (uint8_t)(s_protocol_rx_stream_len - frame_len);
        if (remaining_len > 0u)
        {
            memmove(s_protocol_rx_stream, &s_protocol_rx_stream[frame_len], remaining_len);
        }
        s_protocol_rx_stream_len = remaining_len;

        if (request->motor_id != DEVICE_ID)
        {
            continue;
        }

        return TRUE;
    }

    return FALSE;
}

void ProtocolController_Init(void)
{
    s_protocol_rx_stream_len = 0u;
}

BOOL ProtocolController_PumpRx(ProtocolRequest *request)
{
    uint8_t rx_len;
    uint8_t xdata *rx_buffer;

    if (request == (ProtocolRequest *)0)
    {
        return FALSE;
    }

    while (1)
    {
        rx_buffer = UART_GetRxBuffer(&rx_len);
        if ((rx_buffer == (uint8_t xdata *)0) || (rx_len == 0u))
        {
            break;
        }

        if (rx_len <= UART3_RX_DMA_BUFFER_SIZE)
        {
            memcpy(s_protocol_rx_frame, rx_buffer, rx_len);
            ProtocolController_AppendChunk(s_protocol_rx_frame, rx_len);
        }

        UART_ReleaseRxBuffer();
    }

    return ProtocolController_ExtractRequest(request);
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
