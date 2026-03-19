#ifndef __UART_CONTROLLER_H__
#define __UART_CONTROLLER_H__

#include "config.h"

uint8_t send_messages(ReplyFrame *frame);
uint8_t UARTController_GetCommand(UartCommand *command);

#endif // __UART_CONTROLLER_H__