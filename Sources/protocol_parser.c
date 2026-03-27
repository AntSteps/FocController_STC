#include "protocol_parser.h"
#include <string.h>

static BOOL ProtocolParser_IsHeadStartByte(uint8_t value)
{
    return (BOOL)((value == 0x11u) ||
                  (value == 0x12u) ||
                  (value == 0x13u) ||
                  (value == 0x14u));
}

static void ProtocolParser_RemovePrefix(ProtocolParser *parser, uint8_t count)
{
    uint8_t index;

    if ((parser == (ProtocolParser *)0) || (count == 0u))
    {
        return;
    }

    if (count >= parser->stream_len)
    {
        parser->stream_len = 0u;
        return;
    }

    for (index = 0u; index < (uint8_t)(parser->stream_len - count); index++)
    {
        parser->stream_buf[index] = parser->stream_buf[index + count];
    }

    parser->stream_len = (uint8_t)(parser->stream_len - count);
}

static void ProtocolParser_CompactOnOverflow(ProtocolParser *parser)
{
    int latest_head_index;
    uint8_t index;

    latest_head_index = -1;
    if (parser->stream_len >= 2u)
    {
        for (index = 0u; index < (uint8_t)(parser->stream_len - 1u); index++)
        {
            uint8_t dummy_len;

            if (ProtocolFrame_FindRequestLength(parser->stream_buf[index], parser->stream_buf[index + 1u], &dummy_len))
            {
                latest_head_index = (int)index;
            }
        }
    }

    if (latest_head_index > 0)
    {
        ProtocolParser_RemovePrefix(parser, (uint8_t)latest_head_index);
    }
    else if ((parser->stream_len > 0u) && ProtocolParser_IsHeadStartByte(parser->stream_buf[parser->stream_len - 1u]))
    {
        parser->stream_buf[0] = parser->stream_buf[parser->stream_len - 1u];
        parser->stream_len = 1u;
    }
    else
    {
        parser->stream_len = 0u;
    }

    parser->overflow_count++;
}

void ProtocolParser_Init(ProtocolParser *parser)
{
    if (parser == (ProtocolParser *)0)
    {
        return;
    }

    memset(parser, 0, sizeof(ProtocolParser));
}

void ProtocolParser_Feed(ProtocolParser *parser, const uint8_t xdata *chunk, uint8_t chunk_len)
{
    uint8_t index;

    if ((parser == (ProtocolParser *)0) || (chunk == (const uint8_t xdata *)0) || (chunk_len == 0u))
    {
        return;
    }

    for (index = 0u; index < chunk_len; index++)
    {
        if (parser->stream_len >= PROTOCOL_STREAM_BUFFER_SIZE)
        {
            ProtocolParser_CompactOnOverflow(parser);
        }

        if (parser->stream_len < PROTOCOL_STREAM_BUFFER_SIZE)
        {
            parser->stream_buf[parser->stream_len] = chunk[index];
            parser->stream_len++;
        }
    }
}

BOOL ProtocolParser_Next(ProtocolParser *parser, ProtocolRequest *request)
{
    uint8_t head_pos;

    if ((parser == (ProtocolParser *)0) || (request == (ProtocolRequest *)0))
    {
        return FALSE;
    }

    while (parser->stream_len >= 2u)
    {
        BOOL found_head;
        uint8_t frame_len;

        found_head = FALSE;
        frame_len = 0u;
        for (head_pos = 0u; head_pos < (uint8_t)(parser->stream_len - 1u); head_pos++)
        {
            if (ProtocolFrame_FindRequestLength(parser->stream_buf[head_pos], parser->stream_buf[head_pos + 1u], &frame_len))
            {
                found_head = TRUE;
                break;
            }
        }

        if (!found_head)
        {
            if (ProtocolParser_IsHeadStartByte(parser->stream_buf[parser->stream_len - 1u]))
            {
                parser->stream_buf[0] = parser->stream_buf[parser->stream_len - 1u];
                parser->stream_len = 1u;
            }
            else
            {
                parser->noise_drop_count = (uint16_t)(parser->noise_drop_count + parser->stream_len);
                parser->stream_len = 0u;
            }
            return FALSE;
        }

        if (head_pos > 0u)
        {
            parser->noise_drop_count = (uint16_t)(parser->noise_drop_count + head_pos);
            ProtocolParser_RemovePrefix(parser, head_pos);
        }

        if (parser->stream_len < frame_len)
        {
            return FALSE;
        }

        if (!ProtocolFrame_CheckCrc(parser->stream_buf, frame_len))
        {
            parser->crc_error_count++;
            ProtocolParser_RemovePrefix(parser, 1u);
            continue;
        }

        if (!ProtocolFrame_DecodeRequest(parser->stream_buf, frame_len, request))
        {
            ProtocolParser_RemovePrefix(parser, frame_len);
            continue;
        }

        ProtocolParser_RemovePrefix(parser, frame_len);

        if (request->motor_id != DEVICE_ID)
        {
            continue;
        }

        return TRUE;
    }

    return FALSE;
}
