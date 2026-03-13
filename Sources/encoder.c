#include "encoder.h"

void Encoder_Init(void)
{
    ENCODER_CS = 1;     // CS 默认高电平
}

void Encoder_Update(Encoder* enc)
{
    uint8_t b1, b2, b3;
    uint32_t data24;

    ENCODER_CS = 0;     // 拉低 CS，开始通信

    b1 = SPI_ReadByte();
    b2 = SPI_ReadByte();
    b3 = SPI_ReadByte();

    ENCODER_CS = 1;

    data24 = ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
    enc->raw_angle = (uint16_t)((data24 >> 10) & 0x3FFF); // FUCKING FUCKING FUCKING FUCKING FUCKING.
    enc->degree = ((float)enc->raw_angle / 16384.0f) * 360.0f;
}