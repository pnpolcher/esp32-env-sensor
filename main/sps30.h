#ifndef __SPS30_H
#define __SPS30_H

#include <stdint.h>

#include "i2c.h"


void sps30_start_measurement();
void sps30_stop_measurement();
void sps30_get_data_ready_flag(uint8_t *result);
void sps30_read_data_ready_flag(uint8_t *result);
void sps30_read_measured_values();

#endif
