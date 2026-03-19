#ifndef __COMMAND_H__
#define __COMMAND_H__

#include "uart_controller.h"

void ProcessCommand(UartCommand *command);
void UartReply(uint8_t seq);

#endif