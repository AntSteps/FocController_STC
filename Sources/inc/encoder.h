#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "config.h"
#include "spi.h"

sbit ENCODER_CS = P3^5;

typedef struct {
    uint16_t raw_angle;     // 读取到的原始值 (0-16383, 14bit)
    float degree;           // 解算后的角度 (0-360度)
} Encoder;

void Encoder_Init(void);
void Encoder_Update(Encoder* enc);

#endif // __ENCODER_H__