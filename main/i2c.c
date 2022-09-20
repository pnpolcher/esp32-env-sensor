#include "esp_log.h"

#include "i2c.h"


static const char *TAG = "i2c";


esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_PORT_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        // .sda_pullup_en = GPIO_PULLUP_ENABLE,
        // .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    printf("SDA, SCL = %d, %d\n", conf.sda_io_num, conf.scl_io_num);

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_write_many(uint8_t *buffer, size_t buflen)
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

esp_err_t i2c_read_many(uint8_t addr, uint8_t *buffer, size_t buflen)
{
    i2c_cmd_handle_t cmd;
    esp_err_t err;

    // Start + read
    cmd = i2c_cmd_link_create();

    err = i2c_master_start(cmd);
    if (err != ESP_OK)
    {
        printf("F: i2c_master_start 2\n");
        goto i2c_error;
    }

    err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK)
    {
        printf("F: i2c_master_write_byte\n");
        goto i2c_error;
    }

    err = i2c_master_read(cmd, buffer, buflen, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK)
    {
        printf("F: i2c_master_read\n");
        goto i2c_error;
    }

    err = i2c_master_stop(cmd);
    if (err != ESP_OK)
    {
        printf("F: i2c_master_stop3\n");
        goto i2c_error;
    }

    err = i2c_master_cmd_begin(I2C_MASTER_PORT_NUM, cmd, 3000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        printf("F: i2c_master_cmd_begin2\n");
        goto i2c_error;
    }

i2c_error:
    i2c_cmd_link_delete(cmd);
    return err;    
}


esp_err_t test()
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

    err = i2c_master_write_byte(cmd, (0x30 << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    if (err != ESP_OK)
    {
        printf("SM-F: i2c_master_write\n");
        goto i2c_error;
    }
    err = i2c_master_write_byte(cmd, 0xc7, I2C_MASTER_ACK);
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
