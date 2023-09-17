#include <sys/cdefs.h>
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#include <driver/uart.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_sleep.h"

#include "bme68x.h"
#include "bme68x_defs.h"
#include "bme68x_wrapper.h"
#include "i2c.h"
#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "sps30.h"
#include "ssd1306.h"


static const char *TAG = "main";


static void bme680_init(struct bme68x_dev *dev)
{
    struct bme68x_conf conf;

    ESP_LOGI(TAG, "Wrapper init");
    bme68x_wrapper_init_device(dev);
    ESP_LOGI(TAG, "Init");
    int8_t err = bme68x_init(dev);

    conf.filter = BME68X_FILTER_SIZE_3;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_1X;
    conf.os_pres = BME68X_OS_4X;
    conf.os_temp = BME68X_OS_8X;
    ESP_LOGI(TAG, "Set conf");
    bme68x_set_conf(&conf, dev);
    ESP_LOGI(TAG, "Set op mode");
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

_Noreturn void vMeasureTask(void *pvParameters)
{
    uint8_t n_fields;
    uint8_t ready_flag;
    struct bme68x_dev dev;
    struct bme68x_data data;
    struct sps30_result sps30_result;
    int16_t scd4x_error;
    bool scd4x_data_ready_flag;
    uint16_t scd4x_co2;
    int32_t scd4x_temperature;
    int32_t scd4x_humidity;
   
    ESP_LOGI(TAG, "Got here");
    bme680_init(&dev);
    ESP_LOGI(TAG, "And here");

    sensirion_i2c_hal_init();

    // Clean up potential SCD40 states
    scd4x_wake_up();
    ESP_LOGI(TAG, "Woken up");
    scd4x_stop_periodic_measurement();
    ESP_LOGI(TAG, "Stopped");
    scd4x_reinit();
    ESP_LOGI(TAG, "Reinit");

    uint16_t serial_0;
    uint16_t serial_1;
    uint16_t serial_2;
    scd4x_error = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
    if (scd4x_error) {
        printf("Error executing scd4x_get_serial_number(): %i\n", scd4x_error);
    } else {
        printf("serial: 0x%04x%04x%04x\n", serial_0, serial_1, serial_2);
    }

    scd4x_error = scd4x_start_periodic_measurement();
    if (scd4x_error) {
        printf("Error executing scd4x_start_periodic_measurement(): %i\n", scd4x_error);
    }

    for(;;)
    {
        bme680_start_measurement(&dev);
        sps30_start_measurement();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // esp_sleep_enable_timer_wakeup(1000 * 500); // 500 microseconds.
        // esp_light_sleep_start();

        scd4x_data_ready_flag = false;
        scd4x_error = scd4x_get_data_ready_flag(&scd4x_data_ready_flag);
        if (scd4x_error) {
            printf("Error executing scd4x_get_data_ready_flag(): %i\n", scd4x_error);
            continue;
        }
        if (scd4x_data_ready_flag) {
            scd4x_error = scd4x_read_measurement(&scd4x_co2, &scd4x_temperature, &scd4x_humidity);
            if (scd4x_error) {
                printf("Error executing scd4x_read_measurement(): %i\n", scd4x_error);
            } else if (scd4x_co2 == 0) {
                printf("Invalid sample detected, skipping.\n");
            } else {
                printf("CO2: %u\n", scd4x_co2);
                printf("Temperature: %d m°C\n", scd4x_temperature);
                printf("Humidity: %d mRH\n", scd4x_humidity);
            }
        }

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
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        bme680_suspend(&dev);
        sps30_stop_measurement();
        // esp_sleep_enable_timer_wakeup(1000 * 5000); // 3 seconds.
        // esp_light_sleep_start();
    }
}

void vNopTask()
{
    for(;;) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
     ESP_ERROR_CHECK(i2c_master_init());
     ESP_LOGI(TAG, "I2C initialized OK!");

    // gpio_sleep_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT);
    // gpio_sleep_set_pull_mode(GPIO_NUM_20, GPIO_PULLUP_ONLY);

    gpio_set_direction(GPIO_NUM_8, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_8, GPIO_PULLUP_ONLY);

    // uart_set_wakeup_threshold(UART_NUM_0, 3);   // 3 edges on U0RXD to wakeup
    // esp_sleep_enable_uart_wakeup(UART_NUM_0);   // Enable UART 0 as wakeup source

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

     ESP_LOGI(TAG, "Got to the end of app_main()");

//    xReturned = xTaskCreate(
//        vNopTask,
//        "NOP",
//        2048,
//        0,
//        1,
//        &xMeasureHandle
//    );

    // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_PORT_NUM));
    // ESP_LOGI(TAG, "I2C de-initialized successfully");
}
