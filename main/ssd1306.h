#ifndef __SSD1306_H
#define __SSD1306_H

// #include "esp_log.h"

#define SSD1306_MEMORY_MODE             0x20
#define SSD1306_COLUMN_ADDRESS          0x21
#define SSD1306_PAGE_ADDRESS            0x22
#define SSD1306_DEACTIVATE_SCROLL       0x2E
#define SSD1306_ACTIVATE_SCROLL         0x2F
#define SSD1306_SET_START_LINE          0x40
#define SSD1306_SET_CONTRAST            0x81
#define SSD1306_CHARGE_PUMP             0x8D
#define SSD1306_SEGMENT_REMAP           0xA0
#define SSD1306_DISPLAY_ALLON_RESUME    0xA4
#define SSD1306_NORMAL_DISPLAY          0xA6
#define SSD1306_SET_MULTIPLEX           0xA8
#define SSD1306_DISPLAY_OFF             0xAE
#define SSD1306_DISPLAY_ON              0xAF
#define SSD1306_COM_SCAN_INC            0xC0
#define SSD1306_COM_SCAN_DESC           0xC8
#define SSD1306_SET_DISPLAY_OFFSET      0xD3
#define SSD1306_SET_DISPLAY_CLKDIV      0xD5
#define SSD1306_SET_PRECHARGE           0xD9
#define SSD1306_SET_COM_PINS            0xDA
#define SSD1306_SET_VCOM_DETECT         0xDB

#define SSD1306_CHARGE_PUMP_OFF         0x10
#define SSD1306_CHARGE_PUMP_ON          0x14


#define CONFIG_SSD1306_DISPLAY_WIDTH    128
#define CONFIG_SSD1306_DISPLAY_HEIGHT   32
#define CONFIG_SSD1306_EXTERNAL_VCC     0
#define CONFIG_SSD1306_I2C_ADDRESS      0x3C

#define DISPLAY_BUFFER_SIZE             (CONFIG_SSD1306_DISPLAY_WIDTH * CONFIG_SSD1306_DISPLAY_HEIGHT) / 8

esp_err_t ssd1306_init();
void ssd1306_clear_display();
esp_err_t ssd1306_display();

#endif
