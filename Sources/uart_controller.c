#include "uart_controller.h"
#include "uart.h"
#include <string.h>

static uint8_t s_protocol_rx_stream[PROTOCOL_MAX_FRAME_SIZE];
static uint8_t s_protocol_rx_stream_len = 0u;

static BOOL ProtocolController_TryPublishCompleteRx(void)
{
    uint8_t xdata *active_buffer;
    uint8_t active_slot;
    uint8_t active_len;
    uint8_t frame_len;
    uint8_t required_len;
    uint8_t head0;
    uint8_t head1;

    active_buffer = UART_PeekActiveRxBuffer(&active_slot, &active_len);
    if (active_buffer == (uint8_t xdata *)0)
    {
        return FALSE;
    }

    if (s_protocol_rx_stream_len >= 2u)
    {
        head0 = s_protocol_rx_stream[0];
        head1 = s_protocol_rx_stream[1];
    }
    else if (s_protocol_rx_stream_len == 1u)
    {
        head0 = s_protocol_rx_stream[0];
        head1 = active_buffer[0];
    }
    else
    {
        if (active_len < 2u)
        {
            return FALSE;
        }
        head0 = active_buffer[0];
        head1 = active_buffer[1];
    }

    if (!ProtocolFrame_FindRequestLength(head0, head1, &frame_len) ||
        (s_protocol_rx_stream_len >= frame_len))
    {
        return FALSE;
    }

    required_len = (uint8_t)(frame_len - s_protocol_rx_stream_len);
    if (active_len < required_len)
    {
        return FALSE;
    }

    return UART_PublishActiveRxBuffer(active_slot, required_len);
}

static BOOL ProtocolController_DecodeExactFrame(const uint8_t xdata *frame,
                                                uint8_t frame_len,
                                                ProtocolRequest *request)
{
    if ((frame == (const uint8_t xdata *)0) ||
        (request == (ProtocolRequest *)0) ||
        (frame_len < 2u) ||
        !ProtocolFrame_CheckCrcXData(frame, frame_len))
    {
        return FALSE;
    }

    return ProtocolFrame_DecodeRequest(frame, frame_len, request);
}

static void ProtocolController_AppendChunk(const uint8_t *chunk, uint8_t chunk_len)
{
    if ((chunk == (const uint8_t *)0) || (chunk_len == 0u))
    {
        return;
    }

    if ((uint16_t)s_protocol_rx_stream_len + (uint16_t)chunk_len > (uint16_t)sizeof(s_protocol_rx_stream))
    {
        /* 保护流缓冲不越界：丢弃旧的残片，从最新 chunk 重新同步。 */
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
    BOOL found_request;

    if (request == (ProtocolRequest *)0)
    {
        return FALSE;
    }

    found_request = FALSE;
    while (1)
    {
        rx_buffer = UART_GetRxBuffer(&rx_len);
        if ((rx_buffer == (uint8_t xdata *)0) || (rx_len == 0u))
        {
            if (ProtocolController_TryPublishCompleteRx())
            {
                continue;
            }
            break;
        }

        if (rx_len <= UART3_RX_DMA_BUFFER_SIZE)
        {
            if ((s_protocol_rx_stream_len == 0u) &&
                ProtocolController_DecodeExactFrame(rx_buffer, rx_len, request))
            {
                if (request->motor_id == DEVICE_ID)
                {
                    found_request = TRUE;
                }
            }
            else
            {
                ProtocolController_AppendChunk(rx_buffer, rx_len);
            }
        }

        UART_ReleaseRxBuffer();
        if (found_request)
        {
            break;
        }
    }

    if (!found_request && ProtocolController_ExtractRequest(request))
    {
        found_request = TRUE;
    }

    return found_request;
}

BOOL ProtocolController_SendReply(const ProtocolReply *reply)
{
    uint8_t frame_len;
    uint8_t xdata *tx_buffer;

    tx_buffer = UART3_WaitLockTxBuffer();
    if (tx_buffer == (uint8_t xdata *)0)
    {
        return FALSE;
    }

    if (!ProtocolFrame_EncodeReply(reply, tx_buffer, &frame_len))
    {
        UART3_UnlockTx();
        return FALSE;
    }

    return (BOOL)UART3_StartLockedTx(frame_len);
}
