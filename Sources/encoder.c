#include "encoder.h"
#include "crc6.h"

#define ENCODER_ANGLE_SCALE 0.00038349519697f

static float data angle = 0.0f;
static uint8_t data datagood = 0;

void Encoder_Init(void)
{
    ENCODER_CS = 1;     // CS 默认高电平
}

void Encoder_Update()
{
    (void)Encoder_UpdateAndGetAngle();
}

float Encoder_UpdateAndGetAngle(void)
{
    uint8_t b1;
    uint8_t b2;
    uint8_t b3;
    uint16_t raw_angle;

    ENCODER_CS = 0;     // 拉低 CS，开始通信
    // NOP(10);            // 稍微等待芯片准备好数据输出

    b1 = SPI_ReadByte();
    b2 = SPI_ReadByte();
    b3 = SPI_ReadByte();

    ENCODER_CS = 1;

    datagood = (Mem_CRC6_CheckBytes(b1, b2, b3) == 0);

    if (datagood)
    {
        raw_angle = ((uint16_t)b1 << 6) | ((uint16_t)b2 >> 2);
        angle = (float)raw_angle * ENCODER_ANGLE_SCALE;
    }

    return angle;
}

uint8_t Encoder_Get_DataGood(void)
{
    return datagood;
}

float Encoder_Get_Angle(void)
{
    return angle;
}