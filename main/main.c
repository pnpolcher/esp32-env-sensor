#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "i2c.h"
#include "sps30.h"
#include "ssd1306.h"

static const char *TAG = "main";


void vMeasureTask(void *pvParameters)
{
    uint8_t ready_flag;

    ESP_LOGI(TAG, "Start measurement");
    sps30_start_measurement();

    for (int i = 0; i < 10; i++)
    {
        ESP_LOGI(TAG, "Get ready");
        sps30_get_data_ready_flag(&ready_flag);
        while(!ready_flag) {
            ESP_LOGI(TAG, "Data ready loop");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            sps30_read_data_ready_flag(&ready_flag);
        }
        ESP_LOGI(TAG, "Read measured values.");
        sps30_read_measured_values();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    sps30_stop_measurement();
    // for(;;)
    // {
    //     test();
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized OK!");

    // ESP_ERROR_CHECK(ssd1306_init());
    // ESP_LOGI(TAG, "SSD1306 display OK!");

    BaseType_t xReturned;
    TaskHandle_t xMeasureHandle = NULL;

    xReturned = xTaskCreate(
        vMeasureTask,
        "Measure",
        2048, // stack size in words
        (void *)1, // parameters
        10, // priority,
        &xMeasureHandle
    );

    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_PORT_NUM));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");
}
