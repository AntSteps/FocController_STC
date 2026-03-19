#ifndef __CRC6_H__
#define __CRC6_H__

#include "def.h"

uint8_t Mem_CRC6_CheckBytes(uint8_t b1, uint8_t b2, uint8_t b3);
uint8_t Mem_CRC6_Check(uint32_t data24);

#endif // __CRC6_H__