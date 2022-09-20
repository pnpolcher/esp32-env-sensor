#include <stdint.h>

#include "i2c.h"


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

void sps30_start_measurement()
{
    uint8_t start_measurement_cmd[] = {
        (SPS30_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        SPS30_CMD_MSB(SPS30_START_MEASUREMENT),
        SPS30_CMD_LSB(SPS30_START_MEASUREMENT),
        0x03, 0x00, 0xAC
    };

    i2c_write_many(start_measurement_cmd, sizeof(start_measurement_cmd));
}

void sps30_stop_measurement()
{
    uint8_t stop_measurement_cmd[] = {
        (SPS30_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        SPS30_CMD_MSB(SPS30_STOP_MEASUREMENT),
        SPS30_CMD_LSB(SPS30_STOP_MEASUREMENT),
    };
    i2c_write_many(stop_measurement_cmd, sizeof(stop_measurement_cmd));
}

void sps30_get_data_ready_flag(uint8_t *result)
{
    uint8_t read_buffer[3];

    uint8_t get_data_ready_flag_cmd[] = {
        (SPS30_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        SPS30_CMD_MSB(SPS30_READ_DATA_READY_FLAG),
        SPS30_CMD_LSB(SPS30_READ_DATA_READY_FLAG)
    };

    i2c_write_many(get_data_ready_flag_cmd, sizeof(get_data_ready_flag_cmd));
    i2c_read_many(SPS30_I2C_ADDR, read_buffer, sizeof(read_buffer));

    *result = read_buffer[1];
}

void sps30_read_data_ready_flag(uint8_t *result)
{
    uint8_t read_buffer[3];

    i2c_read_many(SPS30_I2C_ADDR, read_buffer, 3);

    *result = read_buffer[1];
}

static uint32_t bytes_to_uint32(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    return (uint32_t)a << 24 | (uint32_t)b << 16 | (uint32_t)c << 8 | (uint32_t)d;
}

static float bytes_to_float(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    union
    {
        uint32_t u32_value;
        float f_value
    } tmp;

    tmp.u32_value = bytes_to_uint32(a, b, c, d);
    return tmp.f_value;
}

void sps30_read_measured_values()
{
    uint8_t read_buffer[60];

    uint8_t read_measured_values_cmd[] = {
        (SPS30_I2C_ADDR << 1) | I2C_MASTER_WRITE,
        SPS30_CMD_MSB(SPS30_READ_MEASURED_VALUES),
        SPS30_CMD_LSB(SPS30_READ_MEASURED_VALUES)
    };

    i2c_write_many(read_measured_values_cmd, sizeof(read_measured_values_cmd));
    i2c_read_many(SPS30_I2C_ADDR, read_buffer, sizeof(read_buffer));

    float pm1 = bytes_to_float(read_buffer[0], read_buffer[1], read_buffer[3], read_buffer[4]);
    printf("PM1.0 µg/m³ = %f\n", pm1);
    float pm25 = bytes_to_float(read_buffer[6], read_buffer[7], read_buffer[9], read_buffer[10]);
    printf("PM2.5 µg/m³ = %f\n", pm25);
    float pm4 = bytes_to_float(read_buffer[12], read_buffer[13], read_buffer[15], read_buffer[16]);
    printf("PM4.0 µg/m³ = %f\n", pm4);
    float pm10 = bytes_to_float(read_buffer[18], read_buffer[19], read_buffer[21], read_buffer[22]);
    printf("PM10 µg/m³ = %f\n", pm10);
}
