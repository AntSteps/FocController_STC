#ifndef __PARAM_REGISTRY_H__
#define __PARAM_REGISTRY_H__

#include "protocol_frame.h"
#include "param_storage.h"

typedef struct
{
    uint8_t param_id;
    uint16_t offset;
} ParamDescriptor;

void ParamRegistry_ApplyAllRuntime(void);
ProtocolResultCode ParamRegistry_ReadParam(uint8_t param_id, float *value);
ProtocolResultCode ParamRegistry_WriteParam(uint8_t param_id, float value);
void ParamRegistry_FillSnapshot(ParamSnapshotPayload *payload);

#endif
