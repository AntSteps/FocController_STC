#ifndef __UART_CONTROLLER_H__
#define __UART_CONTROLLER_H__

#include "config.h"

void send_messages(ReplyFrame *frame);
uint8_t UARTController_GetCommand(UartCommand *command);

extern CmdFrame cmdframe;
extern ParamCmdFrame paramcmdframe;
extern ReplyFrame replyframe;

#endif // __UART_CONTROLLER_H__