#include <stdlib.h>

#include "utils.h"

uint32_t bytes_to_uint32(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    return (uint32_t)a << 24 | (uint32_t)b << 16 | (uint32_t)c << 8 | (uint32_t)d;
}

float bytes_to_float(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    union
    {
        uint32_t u32_value;
        float f_value;
    } tmp;

    tmp.u32_value = bytes_to_uint32(a, b, c, d);
    return tmp.f_value;
}
