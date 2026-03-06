#ifndef __UART_CONTROLLER_H__
#define __UART_CONTROLLER_H__

#include "def.h"
#include "crc8.h"
#include "uart_send_def.h"
#include "uart.h"

void send_messages(ReplyFrame *frame);
uint8_t receive_messages(CmdFrame *frame);

extern CmdFrame cmdframe;
extern ReplyFrame replyframe;

#endif // __UART_CONTROLLER_H__