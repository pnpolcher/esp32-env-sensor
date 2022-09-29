#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "bme68x.h"
#include "bme68x_defs.h"
#include "bme68x_wrapper.h"
#include "i2c.h"
#include "sps30.h"
#include "ssd1306.h"

static const char *TAG = "main";


void vMeasureTask(void *pvParameters)
{
    uint8_t ready_flag;
    
    struct bme68x_dev dev;
    printf("Wrapper init.\n");
    bme68x_wrapper_init_device(&dev);
    printf("Device init.\n");
    int8_t err = bme68x_init(&dev);


    struct bme68x_conf conf;
    struct bme68x_heatr_conf hconf;

    conf.filter = BME68X_FILTER_SIZE_3;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_1X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;
    bme68x_set_conf(&conf, &dev);
    bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);

    hconf.enable = BME68X_ENABLE;
    hconf.heatr_temp = 320;
    hconf.heatr_dur = 150;

    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &hconf, &dev);

    uint8_t n_fields;
    struct bme68x_data data;
    for(size_t i = 0; i < 10; i++)
    {
        bme68x_set_op_mode(BME68X_FORCED_MODE, &dev);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &dev);
        
        printf("Humidity: %.2f\n", data.humidity);
        printf("Pressure: %.2f\n", data.pressure);
        printf("Temperature: %.2f\n", data.temperature);
        printf("Gas resistance: %.2f\n", data.gas_resistance);
    }

    hconf.enable = BME68X_DISABLE;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &hconf, &dev);

    for(;;)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

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
        4096, // stack size in words
        (void *)1, // parameters
        10, // priority,
        &xMeasureHandle
    );

    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_PORT_NUM));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");
}
