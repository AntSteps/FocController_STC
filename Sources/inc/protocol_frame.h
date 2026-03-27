#ifndef __PROTOCOL_FRAME_H__
#define __PROTOCOL_FRAME_H__

#include "def.h"

#ifndef DEVICE_ID
#define DEVICE_ID                               0x01u
#endif

#define PROTOCOL_STREAM_BUFFER_SIZE             64u
#define PROTOCOL_MAX_FRAME_SIZE                 128u
#define PROTOCOL_PARAM_SNAPSHOT_VERSION         0x01u

#define PROTOCOL_HEAD_ACK_REPLY                 0x0621u
#define PROTOCOL_HEAD_STATE_REPLY               0x0721u
#define PROTOCOL_HEAD_PARAM_VALUE_REPLY         0x0821u
#define PROTOCOL_HEAD_PARAM_SNAPSHOT_REPLY      0x0921u
#define PROTOCOL_HEAD_CALIB_STATUS_REPLY        0x0A21u

#define PROTOCOL_HEAD_CTRL_CMD                  0x1145u
#define PROTOCOL_HEAD_PARAM_WRITE_CMD           0x1245u
#define PROTOCOL_HEAD_QUERY_CMD                 0x1345u
#define PROTOCOL_HEAD_CALIB_CMD                 0x1445u

#define PROTOCOL_STATUS_ENCODER_FAULT           0x01u
#define PROTOCOL_STATUS_MOTOR_FAULT             0x02u
#define PROTOCOL_STATUS_ENC_MAG_SHIFT           2u
#define PROTOCOL_STATUS_ENC_PRESS               0x10u
#define PROTOCOL_STATUS_ENC_OVERSPEED           0x20u
#define PROTOCOL_STATUS_ENC_DATA_GOOD           0x40u
#define PROTOCOL_STATUS_PARAM_FROM_EEPROM       0x80u

#define PROTOCOL_CTRL_MODE_STOP                 0x00u
#define PROTOCOL_CTRL_MODE_SPEED                0x01u

#define PROTOCOL_PARAM_ID_SPEED_KP              0x01u
#define PROTOCOL_PARAM_ID_SPEED_KI              0x02u
#define PROTOCOL_PARAM_ID_VELOCITY_LIMIT        0x03u
#define PROTOCOL_PARAM_ID_CURRENT_LIMIT         0x04u
#define PROTOCOL_PARAM_ID_FF_VELOCITY_GAIN      0x05u
#define PROTOCOL_PARAM_ID_FF_ENABLE             0x06u
#define PROTOCOL_PARAM_ID_MOTOR_POLE_PAIRS      0x10u
#define PROTOCOL_PARAM_ID_MOTOR_PHASE_R         0x11u
#define PROTOCOL_PARAM_ID_MOTOR_PHASE_L         0x12u
#define PROTOCOL_PARAM_ID_CUR_KP                0x13u
#define PROTOCOL_PARAM_ID_CUR_KI                0x14u
#define PROTOCOL_PARAM_ID_SPEED_OUT_MAX         0x15u
#define PROTOCOL_PARAM_ID_SPEED_OUT_MIN         0x16u
#define PROTOCOL_PARAM_ID_SPEED_LPF_MS          0x17u
#define PROTOCOL_PARAM_ID_FF_ACCEL_GAIN         0x18u
#define PROTOCOL_PARAM_ID_FF_FRICTION           0x19u
#define PROTOCOL_PARAM_ID_SPEED_SAMPLE_FREQ     0x1Au
#define PROTOCOL_PARAM_ID_ENCODER_OFFSET        0x20u
#define PROTOCOL_PARAM_ID_CALIB_POLE_PAIRS      0x30u
#define PROTOCOL_PARAM_ID_CALIB_DIR             0x31u
#define PROTOCOL_PARAM_ID_CALIB_ELEC_OFFSET     0x32u
#define PROTOCOL_PARAM_ID_CALIB_HARMONIC_AMP_1  0x33u
#define PROTOCOL_PARAM_ID_CALIB_HARMONIC_PHS_1  0x34u
#define PROTOCOL_PARAM_ID_CALIB_HARMONIC_AMP_2  0x35u
#define PROTOCOL_PARAM_ID_CALIB_HARMONIC_PHS_2  0x36u

typedef enum
{
    PROTOCOL_REQUEST_NONE = 0,
    PROTOCOL_REQUEST_CTRL = 1,
    PROTOCOL_REQUEST_PARAM_WRITE = 2,
    PROTOCOL_REQUEST_QUERY = 3,
    PROTOCOL_REQUEST_CALIB = 4
} ProtocolRequestKind;

typedef enum
{
    PROTOCOL_REPLY_NONE = 0,
    PROTOCOL_REPLY_ACK = 1,
    PROTOCOL_REPLY_STATE = 2,
    PROTOCOL_REPLY_PARAM_VALUE = 3,
    PROTOCOL_REPLY_PARAM_SNAPSHOT = 4,
    PROTOCOL_REPLY_CALIB_STATUS = 5
} ProtocolReplyKind;

typedef enum
{
    PROTOCOL_RESULT_OK = 0,
    PROTOCOL_RESULT_BUSY = 1,
    PROTOCOL_RESULT_UNSUPPORTED = 2,
    PROTOCOL_RESULT_INVALID_PARAM = 3,
    PROTOCOL_RESULT_INVALID_VALUE = 4,
    PROTOCOL_RESULT_EEPROM_ERROR = 5,
    PROTOCOL_RESULT_CALIB_FAILED = 6,
    PROTOCOL_RESULT_INTERNAL_ERROR = 7
} ProtocolResultCode;

typedef enum
{
    PROTOCOL_QUERY_GET_STATE = 0x01u,
    PROTOCOL_QUERY_GET_PARAM = 0x02u,
    PROTOCOL_QUERY_GET_PARAM_SNAPSHOT = 0x03u,
    PROTOCOL_QUERY_GET_CALIB_STATUS = 0x04u
} ProtocolQueryType;

typedef enum
{
    PROTOCOL_CALIB_CMD_START_AUTO = 0x01u
} ProtocolCalibCommand;

typedef enum
{
    PROTOCOL_CALIB_STATE_IDLE = 0x00u,
    PROTOCOL_CALIB_STATE_RUNNING = 0x01u,
    PROTOCOL_CALIB_STATE_SUCCESS = 0x02u,
    PROTOCOL_CALIB_STATE_FAIL_ENCODER = 0x03u,
    PROTOCOL_CALIB_STATE_FAIL_BUSY = 0x04u,
    PROTOCOL_CALIB_STATE_FAIL_INTERNAL = 0x05u
} ProtocolCalibState;

#pragma pack(1)
typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t ctrl_mode;
    float target;
    float ff0;
    float ff1;
    uint8_t seq;
    uint8_t crc8;
} CtrlCmdFrame;

typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t param_id;
    float value;
    uint8_t seq;
    uint8_t crc8;
} ParamWriteCmdFrame;

typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t query_type;
    uint8_t arg0;
    uint8_t seq;
    uint8_t crc8;
} QueryCmdFrame;

typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t calib_cmd;
    uint8_t arg0;
    uint8_t seq;
    uint8_t crc8;
} CalibCmdFrame;

typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t cmd_type;
    uint8_t result_code;
    uint8_t seq;
    uint8_t crc8;
} AckFrame;

typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t control_mode;
    float target;
    float velocity;
    float output;
    float position;
    float voltage;
    uint8_t health_status;
    uint8_t seq;
    uint8_t crc8;
} StateReplyFrame;

typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t param_id;
    float value;
    uint8_t seq;
    uint8_t crc8;
} ParamValueReplyFrame;

typedef struct
{
    int16_t motor_pole_pairs;
    float motor_phase_resistance;
    float motor_phase_inductance;
    float cur_kp;
    float cur_ki;
    float speed_kp;
    float speed_ki;
    float speed_out_max;
    float speed_out_min;
    float speed_lpf_time_constant_ms;
    float ff_velocity_gain;
    float ff_acceleration_gain;
    float ff_friction_compensation;
    uint8_t ff_enable;
    float speed_sample_freq_hz;
    float current_limit;
    float velocity_limit;
    int16_t encoder_offset;
    int16_t calib_pole_pairs;
    int16_t calib_dir;
    float calib_electrical_offset_rad;
    float calib_harmonic_amp_1;
    float calib_harmonic_phase_1;
    float calib_harmonic_amp_2;
    float calib_harmonic_phase_2;
} ParamSnapshotPayload;

typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t snapshot_version;
    ParamSnapshotPayload payload;
    uint8_t seq;
    uint8_t crc8;
} ParamSnapshotReplyFrame;

typedef struct
{
    uint16_t head;
    uint8_t id;
    uint8_t calib_state;
    uint8_t progress;
    uint8_t seq;
    uint8_t crc8;
} CalibStatusReplyFrame;
#pragma pack()

typedef struct
{
    ProtocolRequestKind kind;
    uint8_t motor_id;
    uint8_t seq;
    uint8_t control_mode;
    float target;
    float ff0;
    float ff1;
    uint8_t param_id;
    float value;
    uint8_t query_type;
    uint8_t arg0;
    uint8_t calib_cmd;
} ProtocolRequest;

typedef struct
{
    ProtocolReplyKind kind;
    uint8_t seq;
    uint8_t cmd_type;
    uint8_t result_code;
    uint8_t control_mode;
    float target;
    float velocity;
    float output;
    float position;
    float voltage;
    uint8_t health_status;
    uint8_t param_id;
    float value;
    uint8_t snapshot_version;
    ParamSnapshotPayload payload;
    uint8_t calib_state;
    uint8_t progress;
} ProtocolReply;

BOOL ProtocolFrame_FindRequestLength(uint8_t head_byte0, uint8_t head_byte1, uint8_t *frame_len);
BOOL ProtocolFrame_DecodeRequest(const uint8_t *frame, uint8_t frame_len, ProtocolRequest *request);
BOOL ProtocolFrame_EncodeReply(const ProtocolReply *reply, uint8_t *frame, uint8_t *frame_len);
uint8_t ProtocolFrame_CalcCrc(const uint8_t *frame, uint8_t frame_len_without_crc);
BOOL ProtocolFrame_CheckCrc(const uint8_t *frame, uint8_t frame_len);

#endif
