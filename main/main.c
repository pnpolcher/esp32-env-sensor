#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_sleep.h"

#include "bme68x.h"
#include "bme68x_defs.h"
#include "bme68x_wrapper.h"
#include "i2c.h"
#include "sps30.h"
#include "ssd1306.h"

static const char *TAG = "main";


static void bme680_init(struct bme68x_dev *dev)
{
    struct bme68x_conf conf;

    bme68x_wrapper_init_device(dev);
    int8_t err = bme68x_init(dev);

    conf.filter = BME68X_FILTER_SIZE_3;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_1X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;
    bme68x_set_conf(&conf, dev);
    bme68x_set_op_mode(BME68X_FORCED_MODE, dev);
}

static void bme680_start_measurement(struct bme68x_dev *dev)
{
    struct bme68x_heatr_conf hconf;

    hconf.enable = BME68X_ENABLE;
    hconf.heatr_temp = 320;
    hconf.heatr_dur = 150;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &hconf, dev);
    bme68x_set_op_mode(BME68X_FORCED_MODE, dev);
}

static void bme680_suspend(struct bme68x_dev *dev)
{
    struct bme68x_heatr_conf hconf;

    hconf.enable = BME68X_DISABLE;
    hconf.heatr_dur = 0;
    hconf.heatr_temp = 0;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &hconf, dev);
}

void vMeasureTask(void *pvParameters)
{
    uint8_t n_fields;
    uint8_t ready_flag;
    struct bme68x_dev dev;
    struct bme68x_data data;
    struct sps30_result sps30_result;
   
    bme680_init(&dev);

    for(;;)
    {
        bme680_start_measurement(&dev);
        sps30_start_measurement();
        esp_sleep_enable_timer_wakeup(1000 * 500); // 500 microseconds.
        esp_light_sleep_start();

        sps30_get_data_ready_flag(&ready_flag);
        if (ready_flag)
        {
            sps30_read_measured_values(&sps30_result);
            printf("PM1.0 µg/m³ = %f\n", sps30_result.pm1p0);
            printf("PM2.5 µg/m³ = %f\n", sps30_result.pm2p5);
            printf("PM4.0 µg/m³ = %f\n", sps30_result.pm4p0);
            printf("PM10 µg/m³ = %f\n", sps30_result.pm10);
        }

        bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &dev);
        printf("Humidity: %.2f\n", data.humidity);
        printf("Pressure: %.2f\n", data.pressure);
        printf("Temperature: %.2f\n", data.temperature);
        printf("Gas resistance: %.2f\n", data.gas_resistance);
        fflush(stdin);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        bme680_suspend(&dev);
        sps30_stop_measurement();
        esp_sleep_enable_timer_wakeup(1000 * 5000); // 3 seconds.
        esp_light_sleep_start();
    }
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
