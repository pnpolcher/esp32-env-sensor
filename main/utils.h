#ifndef __UTILS_H
#define __UTILS_H

#include <stdlib.h>

#define INT16_FROM_BYTES(msb, lsb)      (int16_t)((msb << 8) | lsb)
#define UINT16_FROM_BYTES(msb, lsb)     (uint16_t)((msb << 8) | lsb)

uint32_t bytes_to_uint32(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
float bytes_to_float(uint8_t a, uint8_t b, uint8_t c, uint8_t d);

#endif
