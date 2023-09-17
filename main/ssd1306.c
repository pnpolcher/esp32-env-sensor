#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "i2c.h"
#include "ssd1306.h"


static const char *TAG = "ssd1306";


static esp_err_t ssd1306_reg_write_many(uint8_t reg, uint8_t *buffer, size_t buflen)
{
    i2c_cmd_handle_t cmd;
    esp_err_t err;
   
    cmd = i2c_cmd_link_create();

    err = i2c_master_start(cmd);
    if (err != ESP_OK)
    {
        printf("SM-F: i2c_master_start\n");
        goto i2c_error;
    }

    err = i2c_master_write_byte(cmd, (CONFIG_SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    if (err != ESP_OK)
    {
        printf("SM-F: i2c_master_write\n");
        goto i2c_error;
    }

    err = i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    if (err != ESP_OK)
    {
        printf("SM-F: i2c_master_write\n");
        goto i2c_error;
    }

    err = i2c_master_write(cmd, buffer, buflen, I2C_MASTER_ACK);
    if (err != ESP_OK)
    {
        printf("SM-F: i2c_master_write\n");
        goto i2c_error;
    }
    
    err = i2c_master_stop(cmd);
    if (err != ESP_OK)
    {
        printf("SM-F: i2c_master_stop1\n");
        goto i2c_error;
    }

    err = i2c_master_cmd_begin(I2C_MASTER_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        printf("SM-F: i2c_master_cmd_begin\n");
        goto i2c_error;
    }

i2c_error:
    i2c_cmd_link_delete(cmd);
    return err;
}

void ssd1306_draw_hline(uint8_t x0, uint8_t x1, uint8_t y, uint8_t *buffer)
{
    uint8_t *ptr = &buffer[(y / 8) * CONFIG_SSD1306_DISPLAY_WIDTH + x0];
    uint8_t mask = 1 << (y & 0x07);

    for (uint8_t x = x0; x < x1; x++)
    {
        *ptr++ |= mask;
    }
}

void ssd1306_draw_vline(uint8_t x, uint8_t y0, uint8_t y1, uint8_t *buffer)
{
    uint8_t *ptr = &buffer[(y0 / 8) * CONFIG_SSD1306_DISPLAY_WIDTH + x];
    uint8_t y = y0, h = y1 - y0;

    uint8_t mod = y0 & 0x07;
    uint8_t mask;
    if (mod > 0)
    {
        uint8_t mask_lut[] = {
            0x00, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80
            // 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE
        };

        mask = mask_lut[mod];

        if (mod > h) {
            mask &= (0xFF >> (mod - h));
        }

        *ptr |= mask;
        ptr += CONFIG_SSD1306_DISPLAY_WIDTH;
        h -= mod;
    }

    if (h >= 8)
    {
        do {
            *ptr = 0xff;
            ptr += CONFIG_SSD1306_DISPLAY_WIDTH;
            h -= 8;
        } while (h >= 8);
    }

    if (h > 0)
    {
        mod = h & 7;

        uint8_t mask_lut[] = {
            0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F
        };

        mask = mask_lut[mod];
        *ptr |= mask;
    }
}

void ssd1306_clear_display(uint8_t *buffer)
{
    memset(buffer, 0, DISPLAY_BUFFER_SIZE);
}

esp_err_t ssd1306_init()
{
    uint8_t bComPins, bContrast;

    if (CONFIG_SSD1306_DISPLAY_WIDTH == 128 && CONFIG_SSD1306_DISPLAY_HEIGHT == 32)
    {
        bComPins = 0x02;
        bContrast = 0x8F;
    }
    else if (CONFIG_SSD1306_DISPLAY_WIDTH == 128 && CONFIG_SSD1306_DISPLAY_HEIGHT == 64)
    {
        bComPins = 0x12;
        bContrast = CONFIG_SSD1306_EXTERNAL_VCC == 0 ? 0xCF : 0x9F;
    }
    else if (CONFIG_SSD1306_DISPLAY_WIDTH == 96 && CONFIG_SSD1306_DISPLAY_HEIGHT == 16)
    {
        bComPins = 0x02;
        bContrast = CONFIG_SSD1306_EXTERNAL_VCC == 0 ? 0xAF : 0x10;
    }
    else
    {
        // TODO: Not supported
        // goto i2c_error;
    }

    uint8_t bCommandList[] = {
        0x00,
        SSD1306_DISPLAY_OFF,
        SSD1306_SET_DISPLAY_CLKDIV,
        0x80,  // RESET values from datasheet.
        SSD1306_SET_MULTIPLEX,
        CONFIG_SSD1306_DISPLAY_HEIGHT - 1,
        SSD1306_SET_DISPLAY_OFFSET,
        0x00,  // No offset.
        SSD1306_SET_START_LINE | 0x00,  // Start at line zero.
        SSD1306_CHARGE_PUMP,
        CONFIG_SSD1306_EXTERNAL_VCC == 0 ? SSD1306_CHARGE_PUMP_ON : SSD1306_CHARGE_PUMP_OFF,
        SSD1306_MEMORY_MODE,
        0x00,  // Act like KS0108.
        SSD1306_SEGMENT_REMAP | 0x01,
        SSD1306_COM_SCAN_DESC,
        SSD1306_SET_COM_PINS,
        bComPins,
        SSD1306_SET_CONTRAST,
        bContrast,
        SSD1306_SET_PRECHARGE,
        CONFIG_SSD1306_EXTERNAL_VCC == 0 ? 0xF1 : 0x22,
        SSD1306_SET_VCOM_DETECT,
        0x40,
        SSD1306_DISPLAY_ALLON_RESUME,
        SSD1306_NORMAL_DISPLAY,
        SSD1306_DEACTIVATE_SCROLL,
        SSD1306_DISPLAY_ON,
    };

    return i2c_write_many(CONFIG_SSD1306_I2C_ADDRESS, bCommandList, sizeof(bCommandList));
}

esp_err_t ssd1306_display(uint8_t *buffer)
{
    esp_err_t err;

    uint8_t bCommandList[] = {
        SSD1306_COLUMN_ADDRESS,
        0,
        CONFIG_SSD1306_DISPLAY_WIDTH - 1,
        SSD1306_PAGE_ADDRESS,
        0,
        (CONFIG_SSD1306_DISPLAY_HEIGHT >> 3) - 1  // Total number of pages.
    };

    err = i2c_write_many(CONFIG_SSD1306_I2C_ADDRESS, bCommandList, sizeof(bCommandList));
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    uint8_t *ptr = buffer;
    for (size_t len = 0; len < DISPLAY_BUFFER_SIZE; len += 16, ptr += 16) {
        ssd1306_reg_write_many(0x40, ptr, 16);
    }

i2c_error:
    return err;
}
