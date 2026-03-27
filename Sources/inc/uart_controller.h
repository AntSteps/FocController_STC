#ifndef __UART_CONTROLLER_H__
#define __UART_CONTROLLER_H__

#include "config.h"
#include "protocol_frame.h"

void ProtocolController_Init(void);
BOOL ProtocolController_PumpRx(ProtocolRequest *request);
BOOL ProtocolController_SendReply(const ProtocolReply *reply);

#endif // __UART_CONTROLLER_H__
