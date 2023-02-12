#include <stdint.h>

#include "i2c.h"
#include "sps30.h"
#include "utils.h"


#define SPS30_I2C_ADDR 0x69

#define SPS30_START_MEASUREMENT     0x0010
#define SPS30_STOP_MEASUREMENT      0x0014
#define SPS30_READ_DATA_READY_FLAG  0x0202
#define SPS30_READ_MEASURED_VALUES  0x0300

#define SPS30_CMD_MSB(x)    (uint8_t)((x & 0xFF00) >> 8)
#define SPS30_CMD_LSB(x)    (uint8_t)(x & 0x00FF)


static uint8_t calculate_crc(uint8_t data[2])
{
    uint8_t crc = 0xff;

    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31u;
            }
            else
            {
                crc = (crc << 1);
            }
            
        }
    }

    return crc;
}

sps30_err_t sps30_start_measurement()
{
    uint8_t start_measurement_cmd[] = {
        (SPS30_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        SPS30_CMD_MSB(SPS30_START_MEASUREMENT),
        SPS30_CMD_LSB(SPS30_START_MEASUREMENT),
        0x03, 0x00, 0xAC
    };

    return i2c_write_many(
        start_measurement_cmd, sizeof(start_measurement_cmd)) == ESP_OK ? SPS30_OK : SPS30_ERR_I2C;
}

sps30_err_t sps30_stop_measurement()
{
    uint8_t stop_measurement_cmd[] = {
        (SPS30_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        SPS30_CMD_MSB(SPS30_STOP_MEASUREMENT),
        SPS30_CMD_LSB(SPS30_STOP_MEASUREMENT),
    };
    return i2c_write_many(
        stop_measurement_cmd, sizeof(stop_measurement_cmd)) == ESP_OK ? SPS30_OK : SPS30_ERR_I2C;
}

sps30_err_t sps30_get_data_ready_flag(uint8_t *result)
{
    esp_err_t err;
    uint8_t read_buffer[3];

    uint8_t get_data_ready_flag_cmd[] = {
        (SPS30_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        SPS30_CMD_MSB(SPS30_READ_DATA_READY_FLAG),
        SPS30_CMD_LSB(SPS30_READ_DATA_READY_FLAG)
    };

    err = i2c_write_many(get_data_ready_flag_cmd, sizeof(get_data_ready_flag_cmd));
    if (err != ESP_OK)
    {
        printf("Error here!");
        return SPS30_ERR_I2C;
    }

    err = i2c_read_many(SPS30_I2C_ADDR, read_buffer, sizeof(read_buffer));
    if (err != ESP_OK)
    {
        return SPS30_ERR_I2C;
    }

    if (calculate_crc(read_buffer) != read_buffer[2])
    {
        return SPS30_ERR_CRC;
    }

    *result = read_buffer[1];
    return SPS30_OK;
}

sps30_err_t sps30_read_data_ready_flag(uint8_t *result)
{
    uint8_t read_buffer[3];

    if (i2c_read_many(SPS30_I2C_ADDR, read_buffer, 3) != ESP_OK)
    {
        return SPS30_ERR_I2C;
    }

    if (calculate_crc(read_buffer) != read_buffer[2])
    {
        return SPS30_ERR_CRC;
    }

    *result = read_buffer[1];
    return SPS30_OK;
}

sps30_err_t sps30_read_measured_values(struct sps30_result *result)
{
    uint8_t read_buffer[60];

    uint8_t read_measured_values_cmd[] = {
        (SPS30_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        SPS30_CMD_MSB(SPS30_READ_MEASURED_VALUES),
        SPS30_CMD_LSB(SPS30_READ_MEASURED_VALUES)
    };

    if (i2c_write_many(read_measured_values_cmd, sizeof(read_measured_values_cmd)) != ESP_OK)
    {
        return SPS30_ERR_I2C;
    }

    if (i2c_read_many(SPS30_I2C_ADDR, read_buffer, sizeof(read_buffer)) != ESP_OK)
    {
        return SPS30_ERR_I2C;
    }

    if (calculate_crc(read_buffer) != read_buffer[2] ||
        calculate_crc(&read_buffer[3]) != read_buffer[5] ||
        calculate_crc(&read_buffer[6]) != read_buffer[8] ||
        calculate_crc(&read_buffer[9]) != read_buffer[11] ||
        calculate_crc(&read_buffer[12]) != read_buffer[14] ||
        calculate_crc(&read_buffer[15]) != read_buffer[17] ||
        calculate_crc(&read_buffer[18]) != read_buffer[20] ||
        calculate_crc(&read_buffer[21]) != read_buffer[23] ||
        calculate_crc(&read_buffer[24]) != read_buffer[26] ||
        calculate_crc(&read_buffer[27]) != read_buffer[29] ||
        calculate_crc(&read_buffer[30]) != read_buffer[32] ||
        calculate_crc(&read_buffer[33]) != read_buffer[35] ||
        calculate_crc(&read_buffer[36]) != read_buffer[38] ||
        calculate_crc(&read_buffer[39]) != read_buffer[41] ||
        calculate_crc(&read_buffer[42]) != read_buffer[44] ||
        calculate_crc(&read_buffer[45]) != read_buffer[47] ||
        calculate_crc(&read_buffer[48]) != read_buffer[50] ||
        calculate_crc(&read_buffer[51]) != read_buffer[53] ||
        calculate_crc(&read_buffer[54]) != read_buffer[56] ||
        calculate_crc(&read_buffer[57]) != read_buffer[59])
    {
        return SPS30_ERR_CRC;
    }

    result->pm1p0 = bytes_to_float(read_buffer[0], read_buffer[1], read_buffer[3], read_buffer[4]);
    result->pm2p5 = bytes_to_float(read_buffer[6], read_buffer[7], read_buffer[9], read_buffer[10]);
    result->pm4p0 = bytes_to_float(read_buffer[12], read_buffer[13], read_buffer[15], read_buffer[16]);
    result->pm10 = bytes_to_float(read_buffer[18], read_buffer[19], read_buffer[21], read_buffer[22]);
    result->pm0p5cm3 = bytes_to_float(read_buffer[24], read_buffer[25], read_buffer[27], read_buffer[28]);
    result->pm1p0cm3 = bytes_to_float(read_buffer[30], read_buffer[31], read_buffer[33], read_buffer[34]);
    result->pm2p5cm3 = bytes_to_float(read_buffer[36], read_buffer[37], read_buffer[39], read_buffer[40]);
    result->pm4p0cm3 = bytes_to_float(read_buffer[42], read_buffer[43], read_buffer[45], read_buffer[46]);
    result->pm10cm3 = bytes_to_float(read_buffer[48], read_buffer[49], read_buffer[51], read_buffer[52]);
    result->typ_particle_size = bytes_to_float(read_buffer[54], read_buffer[55], read_buffer[57], read_buffer[58]);

    return SPS30_OK;
}
