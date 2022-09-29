#ifndef __I2C_H
#define __I2C_H

#include "driver/i2c.h"

#define CONFIG_I2C_MASTER_SDA       21
#define CONFIG_I2C_MASTER_SCL       22

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_PORT_NUM         I2C_NUM_0                  /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

esp_err_t i2c_master_init(void);
esp_err_t i2c_write_many(uint8_t *buffer, size_t buflen);
esp_err_t i2c_read_many(uint8_t addr, uint8_t *buffer, size_t buflen);
esp_err_t i2c_read_register(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
esp_err_t i2c_write_register(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

#endif
