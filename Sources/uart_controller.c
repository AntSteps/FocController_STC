#include "uart_controller.h"
#include "uart.h"
#include "uart_send_def.h"
#include "crc8.h"

CmdFrame cmdframe = {0};
ReplyFrame replyframe = {0};

void send_messages(ReplyFrame *frame)
{
    frame->crc8 = Mem_CRC8_Check((uint8_t *)frame, sizeof(ReplyFrame) - 1, 0);
    UART_DMA_Send((uint8_t *)frame, sizeof(ReplyFrame));
}

// 返回0表示更新
uint8_t receive_messages(CmdFrame *frame)
{
    // 这里rxd_buf已经被DMA填充了接收到的数据
    if (read_ptr == write_ptr)
        return 1; // null_frame.

    while (read_ptr != write_ptr)
    {
        if (rxd_buf[read_ptr] == 0x11 && rxd_buf[(read_ptr + 1) % (18 * 5)] == 0x45) // 检测帧头0x1145
        {
            if ((write_ptr + 18 * 5 - read_ptr) % (18 * 5) >= sizeof(CmdFrame))
            {
                uint8_t temp_buf[sizeof(CmdFrame)];
                memcpy(temp_buf, &rxd_buf[read_ptr], sizeof(CmdFrame));

                if (!Mem_CRC8_Check(temp_buf, sizeof(CmdFrame), 1))
                {
                    *frame = *(CmdFrame *)temp_buf;
                    read_ptr = (read_ptr + sizeof(CmdFrame)) % (18 * 5);
                    return 0;
                }
                else
                {
                    read_ptr = (read_ptr + sizeof(CmdFrame)) % (18 * 5);
                    // debug here.
                    continue;
                }
            }
        }
        read_ptr = (read_ptr + 1) % (18 * 5);
    }
    return 1;
}
