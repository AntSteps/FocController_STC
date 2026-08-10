#ifndef __TELEMETRY_SERVICE_H__
#define __TELEMETRY_SERVICE_H__

#include "protocol_frame.h"

uint8_t TelemetryService_BuildHealthStatus(void);
void TelemetryService_BuildAckReply(ProtocolReply *reply, uint8_t cmd_type, uint8_t result_code, uint8_t seq);
void TelemetryService_BuildStateReply(ProtocolReply *reply, uint8_t seq);
void TelemetryService_BuildParamValueReply(ProtocolReply *reply, uint8_t seq, uint8_t param_id, float value);
void TelemetryService_BuildParamSnapshotReply(ProtocolReply *reply, uint8_t seq);
void TelemetryService_BuildCalibStatusReply(ProtocolReply *reply, uint8_t seq, uint8_t calib_state, uint8_t progress);
void TelemetryService_BuildCalibSampleReply(ProtocolReply *reply,
                                            uint16_t sample_index,
                                            int8_t direction,
                                            float command_theta,
                                            float mechanical_angle,
                                            float electrical_error);
void TelemetryService_BuildFastStatusReply(ProtocolReply *reply, BOOL command_clamped);

#endif
