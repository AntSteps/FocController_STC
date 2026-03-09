#ifndef __UART_CONTROLLER_H__
#define __UART_CONTROLLER_H__

#include "config.h"

void send_messages(ReplyFrame *frame);
uint8_t receive_messages(CmdFrame *frame);

extern CmdFrame cmdframe;
extern ReplyFrame replyframe;

#endif // __UART_CONTROLLER_H__