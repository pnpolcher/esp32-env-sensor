#ifndef __BME68X_WRAPPER
#define __BME68X_WRAPPER

#include <stdint.h>

void bme68x_wrapper_delay(uint32_t period, void *intf_ptr);
int8_t bme68x_wrapper_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bme68x_wrapper_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
void bme68x_wrapper_init_device(struct bme68x_dev *dev);

#endif
