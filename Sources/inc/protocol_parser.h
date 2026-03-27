#ifndef __PROTOCOL_PARSER_H__
#define __PROTOCOL_PARSER_H__

#include "config.h"
#include "protocol_frame.h"

typedef struct
{
    uint8_t stream_buf[PROTOCOL_STREAM_BUFFER_SIZE];
    uint8_t stream_len;
    uint16_t overflow_count;
    uint16_t noise_drop_count;
    uint16_t crc_error_count;
} ProtocolParser;

void ProtocolParser_Init(ProtocolParser *parser);
void ProtocolParser_Feed(ProtocolParser *parser, const uint8_t xdata *chunk, uint8_t chunk_len);
BOOL ProtocolParser_Next(ProtocolParser *parser, ProtocolRequest *request);

#endif
