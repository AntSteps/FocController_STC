#ifndef __CRC8_H__
#define __CRC8_H__
// #include <stdint.h>
#include "def.h"

uint8_t Mem_CRC8_Check(uint8_t *dat, int size, uint8_t has_crc);
uint8_t Mem_CRC8_CheckXData(const uint8_t xdata *dat, uint8_t size, uint8_t has_crc);

#endif
