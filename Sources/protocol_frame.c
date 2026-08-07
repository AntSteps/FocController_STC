#include "protocol_frame.h"
#include "crc8.h"
#include <string.h>

typedef struct
{
    uint16_t head;
    uint8_t frame_len;
    ProtocolRequestKind kind;
} ProtocolRequestDescriptor;

static const ProtocolRequestDescriptor s_request_descriptors[] =
{
    { PROTOCOL_HEAD_CTRL_CMD, sizeof(CtrlCmdFrame), PROTOCOL_REQUEST_CTRL },
    { PROTOCOL_HEAD_PARAM_WRITE_CMD, sizeof(ParamWriteCmdFrame), PROTOCOL_REQUEST_PARAM_WRITE },
    { PROTOCOL_HEAD_QUERY_CMD, sizeof(QueryCmdFrame), PROTOCOL_REQUEST_QUERY },
    { PROTOCOL_HEAD_CALIB_CMD, sizeof(CalibCmdFrame), PROTOCOL_REQUEST_CALIB }
};

static void ProtocolFrame_WriteU16BE(uint8_t xdata *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)value;
}

static uint16_t ProtocolFrame_ReadU16BE(const uint8_t *buf)
{
    return (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

static void ProtocolFrame_WriteFloatRaw(uint8_t xdata *buf, float value)
{
    memcpy(buf, &value, sizeof(float));
}

static float ProtocolFrame_ReadFloatRaw(const uint8_t *buf)
{
    float value;

    memcpy(&value, buf, sizeof(float));
    return value;
}

static const ProtocolRequestDescriptor *ProtocolFrame_FindRequestDescriptorByHead(uint16_t head)
{
    uint8_t index;

    for (index = 0; index < (uint8_t)(sizeof(s_request_descriptors) / sizeof(s_request_descriptors[0])); index++)
    {
        if (s_request_descriptors[index].head == head)
        {
            return &s_request_descriptors[index];
        }
    }

    return (const ProtocolRequestDescriptor *)0;
}

BOOL ProtocolFrame_FindRequestLength(uint8_t head_byte0, uint8_t head_byte1, uint8_t *frame_len)
{
    const ProtocolRequestDescriptor *descriptor;
    uint16_t head;

    if (frame_len == (uint8_t *)0)
    {
        return FALSE;
    }

    head = (uint16_t)(((uint16_t)head_byte0 << 8) | (uint16_t)head_byte1);
    descriptor = ProtocolFrame_FindRequestDescriptorByHead(head);
    if (descriptor == (const ProtocolRequestDescriptor *)0)
    {
        return FALSE;
    }

    *frame_len = descriptor->frame_len;
    return TRUE;
}

uint8_t ProtocolFrame_CalcCrc(const uint8_t *frame, uint8_t frame_len_without_crc)
{
    return Mem_CRC8_Check((uint8_t *)frame, frame_len_without_crc, 0);
}

BOOL ProtocolFrame_CheckCrc(const uint8_t *frame, uint8_t frame_len)
{
    return (BOOL)(Mem_CRC8_Check((uint8_t *)frame, frame_len, 1) == 0);
}

BOOL ProtocolFrame_CheckCrcXData(const uint8_t xdata *frame, uint8_t frame_len)
{
    return (BOOL)(Mem_CRC8_CheckXData(frame, frame_len, 1u) == 0u);
}

static uint8_t ProtocolFrame_CalcCrcXData(const uint8_t xdata *frame,
                                          uint8_t frame_len_without_crc)
{
    return Mem_CRC8_CheckXData(frame, frame_len_without_crc, 0u);
}

BOOL ProtocolFrame_DecodeRequest(const uint8_t *frame, uint8_t frame_len, ProtocolRequest *request)
{
    const ProtocolRequestDescriptor *descriptor;
    uint16_t head;

    if ((frame == (const uint8_t *)0) || (request == (ProtocolRequest *)0))
    {
        return FALSE;
    }

    head = ProtocolFrame_ReadU16BE(frame);
    descriptor = ProtocolFrame_FindRequestDescriptorByHead(head);
    if (descriptor == (const ProtocolRequestDescriptor *)0)
    {
        return FALSE;
    }

    if (descriptor->frame_len != frame_len)
    {
        return FALSE;
    }

    request->kind = descriptor->kind;
    request->motor_id = frame[2];
    request->seq = frame[frame_len - 2];

    switch (descriptor->kind)
    {
    case PROTOCOL_REQUEST_CTRL:
        request->control_mode = frame[3];
        request->target = ProtocolFrame_ReadFloatRaw(&frame[4]);
        request->load_ff = ProtocolFrame_ReadFloatRaw(&frame[8]);
        break;

    case PROTOCOL_REQUEST_PARAM_WRITE:
        request->param_id = frame[3];
        request->value = ProtocolFrame_ReadFloatRaw(&frame[4]);
        break;

    case PROTOCOL_REQUEST_QUERY:
        request->query_type = frame[3];
        request->arg0 = frame[4];
        break;

    case PROTOCOL_REQUEST_CALIB:
        request->calib_cmd = frame[3];
        request->arg0 = frame[4];
        break;

    default:
        return FALSE;
    }

    return TRUE;
}
// 对于协议请求构造响应
BOOL ProtocolFrame_EncodeReply(const ProtocolReply *reply, uint8_t xdata *frame, uint8_t *frame_len)
{
    uint8_t len;

    if ((reply == (const ProtocolReply *)0) ||
        (frame == (uint8_t xdata *)0) ||
        (frame_len == (uint8_t *)0))
    {
        return FALSE;
    }

    len = 0;
    switch (reply->kind)
    {
    case PROTOCOL_REPLY_ACK:
        len = sizeof(AckFrame);
        ProtocolFrame_WriteU16BE(&frame[0], PROTOCOL_HEAD_ACK_REPLY);
        frame[2] = DEVICE_ID;
        frame[3] = reply->cmd_type;
        frame[4] = reply->result_code;
        frame[5] = reply->seq;
        frame[6] = ProtocolFrame_CalcCrcXData(frame, (uint8_t)(len - 1));
        break;

    case PROTOCOL_REPLY_STATE:
        len = sizeof(StateReplyFrame);
        ProtocolFrame_WriteU16BE(&frame[0], PROTOCOL_HEAD_STATE_REPLY);
        frame[2] = DEVICE_ID;
        frame[3] = reply->control_mode;
        ProtocolFrame_WriteFloatRaw(&frame[4], reply->target);
        ProtocolFrame_WriteFloatRaw(&frame[8], reply->velocity);
        ProtocolFrame_WriteFloatRaw(&frame[12], reply->output);
        ProtocolFrame_WriteFloatRaw(&frame[16], reply->position);
        ProtocolFrame_WriteFloatRaw(&frame[20], reply->voltage);
        frame[24] = reply->health_status;
        frame[25] = reply->seq;
        frame[26] = ProtocolFrame_CalcCrcXData(frame, (uint8_t)(len - 1));
        break;

    case PROTOCOL_REPLY_PARAM_VALUE:
        len = sizeof(ParamValueReplyFrame);
        ProtocolFrame_WriteU16BE(&frame[0], PROTOCOL_HEAD_PARAM_VALUE_REPLY);
        frame[2] = DEVICE_ID;
        frame[3] = reply->param_id;
        ProtocolFrame_WriteFloatRaw(&frame[4], reply->value);
        frame[8] = reply->seq;
        frame[9] = ProtocolFrame_CalcCrcXData(frame, (uint8_t)(len - 1));
        break;

    case PROTOCOL_REPLY_PARAM_SNAPSHOT:
        len = sizeof(ParamSnapshotReplyFrame);
        ProtocolFrame_WriteU16BE(&frame[0], PROTOCOL_HEAD_PARAM_SNAPSHOT_REPLY);
        frame[2] = DEVICE_ID;
        frame[3] = reply->snapshot_version;
        memcpy(&frame[4], &reply->payload, sizeof(ParamSnapshotPayload));
        frame[4 + sizeof(ParamSnapshotPayload)] = reply->seq;
        frame[5 + sizeof(ParamSnapshotPayload)] = ProtocolFrame_CalcCrcXData(frame, (uint8_t)(len - 1));
        break;

    case PROTOCOL_REPLY_CALIB_STATUS:
        len = sizeof(CalibStatusReplyFrame);
        ProtocolFrame_WriteU16BE(&frame[0], PROTOCOL_HEAD_CALIB_STATUS_REPLY);
        frame[2] = DEVICE_ID;
        frame[3] = reply->calib_state;
        frame[4] = reply->progress;
        frame[5] = reply->seq;
        frame[6] = ProtocolFrame_CalcCrcXData(frame, (uint8_t)(len - 1));
        break;

    case PROTOCOL_REPLY_CALIB_SAMPLE:
        len = sizeof(CalibSampleReplyFrame);
        ProtocolFrame_WriteU16BE(&frame[0], PROTOCOL_HEAD_CALIB_SAMPLE_REPLY);
        frame[2] = DEVICE_ID;
        ProtocolFrame_WriteU16BE(&frame[3], reply->calib_sample_index);
        frame[5] = (uint8_t)reply->calib_sample_direction;
        ProtocolFrame_WriteFloatRaw(&frame[6], reply->calib_sample_theta);
        ProtocolFrame_WriteFloatRaw(&frame[10], reply->calib_sample_mechanical_angle);
        ProtocolFrame_WriteFloatRaw(&frame[14], reply->calib_sample_error);
        frame[18] = 0u;
        frame[19] = ProtocolFrame_CalcCrcXData(frame, (uint8_t)(len - 1));
        break;

    default:
        return FALSE;
    }

    *frame_len = len;
    return TRUE;
}
