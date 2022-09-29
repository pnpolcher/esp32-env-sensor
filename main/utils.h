#ifndef __UTILS_H
#define __UTILS_H

#include <stdlib.h>

#define INT16_FROM_BYTES(msb, lsb)      (int16_t)((msb << 8) | lsb)
#define UINT16_FROM_BYTES(msb, lsb)     (uint16_t)((msb << 8) | lsb)


#endif
