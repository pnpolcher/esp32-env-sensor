#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bme68x_defs.h"
#include "i2c.h"

#define BME680_I2C_ADDR 0x76


void bme68x_wrapper_delay(uint32_t period, void *intf_ptr)
{
    vTaskDelay((period / 1000) / portTICK_PERIOD_MS);
}

int8_t bme68x_wrapper_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    printf("Read gets called with reg addr = 0x%X\n", reg_addr);
    return i2c_read_register(BME680_I2C_ADDR, reg_addr, reg_data, length) == ESP_OK ? 0 : -1;
}

int8_t bme68x_wrapper_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    printf("Write gets called with reg addr = 0x%X\n", reg_addr);
    return i2c_write_register(BME680_I2C_ADDR, reg_addr, reg_data, length) == ESP_OK ? 0 : -1;
}

void bme68x_wrapper_init_device(struct bme68x_dev *dev)
{
    dev->delay_us = &bme68x_wrapper_delay;
    dev->read = &bme68x_wrapper_read;
    dev->write = &bme68x_wrapper_write;
    dev->amb_temp = 25;
    dev->chip_id = BME680_I2C_ADDR;
    dev->intf = BME68X_I2C_INTF;
    dev->intf_ptr = NULL;
}