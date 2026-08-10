#ifndef __CALIBRATION_SERVICE_H__
#define __CALIBRATION_SERVICE_H__

#include "protocol_frame.h"

void CalibrationService_Init(void);
ProtocolResultCode CalibrationService_Start(BOOL stream_samples);
void CalibrationService_Poll(void);
BOOL CalibrationService_IsBusy(void);
uint8_t CalibrationService_GetState(void);
uint8_t CalibrationService_GetProgress(void);

#endif
