#include <stdlib.h>

#include "driver/i2c.h"
#include "freertos/task.h"

#include "bme680.h"
#include "i2c.h"
#include "utils.h"


#define BME680_I2C_ADDR     0x76

#define BME680_CHIP_ID      0x61
#define BME680_PERIOD_RESET 10000
#define BME680_PERIOD_POLL  10          // In milliseconds

#define BME680_CMD_SOFT_RESET   0xb6

/* Information - only available via BME680_dev.info_msg */
#define BME680_I_PARAM_CORR                       UINT8_C(1)

#define BME680_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     ((data << bitname##_POS) & bitname##_MASK))

#define BME680_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     (data & bitname##_MASK))

#define BME680_GET_BITS(reg_data, bitname)        ((reg_data & (bitname##_MASK)) >> \
                                                   (bitname##_POS))

#define BME680_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MASK))


static esp_err_t bme680_get_calibration_data(struct bme680_cal_data *cal);
esp_err_t bme680_read_all_field_data(struct bme680_data * const data[], struct bme680_cal_data *cal, uint8_t variant_id);
static esp_err_t bme680_read_field_data(uint8_t index, struct bme680_data *data, struct bme680_cal_data *cal, uint8_t variant_id);
static float bme680_calc_gas_resistance_high(uint16_t gas_res_adc, uint8_t gas_range);
static float bme680_calc_gas_resistance_low(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_cal_data *cal);
static float bme680_calc_humidity(uint16_t hum_adc, const struct bme680_cal_data *cal);
static float bme680_calc_pressure(uint32_t pres_adc, const struct bme680_cal_data *cal);
static float bme680_calc_temperature(uint32_t temp_adc, struct bme680_cal_data *cal);
static void sort_sensor_data(uint8_t low_index, uint8_t high_index, struct bme680_data *field[]);


esp_err_t bme680_soft_reset()
{
    esp_err_t err;

    uint8_t cmd[] = {
        BME680_I2C_ADDR | I2C_MASTER_WRITE,
        BME680_REG_SOFT_RESET,
        BME680_CMD_SOFT_RESET,
    };

    err = i2c_write_many(cmd, sizeof(cmd));
    return err;
}

esp_err_t bme680_init(struct bme680_cal_data *cal)
{
    esp_err_t err;
    uint8_t chip_id;
    uint8_t variant_id;

    err = bme680_soft_reset();
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    err = i2c_read_register(BME680_I2C_ADDR, BME680_REG_CHIP_ID, &chip_id, sizeof(chip_id));
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    err = i2c_read_register(BME680_I2C_ADDR, BME680_REG_VARIANT_ID, &variant_id, sizeof(variant_id));
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    err = bme680_get_calibration_data(cal);

    printf("Found BMP680 with ID 0x%x and variant ID 0x%x\n", chip_id, variant_id);

i2c_error:
    return err;
}

esp_err_t bme680_get_op_mode(uint8_t *op_mode)
{
    esp_err_t err;

    err = i2c_read_register(BME680_I2C_ADDR, BME680_REG_CTRL_MEAS, op_mode, 1);
    if (err == ESP_OK)
    {
        *op_mode = *op_mode & BME680_MODE_MASK;
    }

    return err;
}

esp_err_t bme680_set_op_mode(uint8_t op_mode)
{
    esp_err_t err;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;

    do
    {
        err = i2c_read_register(BME680_I2C_ADDR, BME680_REG_CTRL_MEAS, &tmp_pow_mode, sizeof(tmp_pow_mode));
        if (err != ESP_OK)
        {
            break;
        }

        pow_mode = (tmp_pow_mode & BME680_MODE_MASK);
        if (pow_mode != BME680_SLEEP_MODE)
        {
            tmp_pow_mode &= ~BME680_MODE_MASK;
            uint8_t cmd[] = {
                BME680_I2C_ADDR | I2C_MASTER_WRITE,
                BME680_REG_CTRL_MEAS,
                tmp_pow_mode
            };
            err = i2c_write_many(cmd, sizeof(cmd));
            vTaskDelay(BME680_PERIOD_POLL / portTICK_PERIOD_MS);
        }
    } while (pow_mode != BME680_SLEEP_MODE && err == ESP_OK);

    if (op_mode != BME680_SLEEP_MODE && err == ESP_OK)
    {
        tmp_pow_mode = (tmp_pow_mode & ~BME680_MODE_MASK) | (op_mode & BME680_MODE_MASK);

        uint8_t cmd[] = {
            BME680_I2C_ADDR | I2C_MASTER_WRITE,
            BME680_REG_CTRL_MEAS,
            tmp_pow_mode
        };
        err = i2c_write_many(cmd, sizeof(cmd));
    }

    return err;
}

esp_err_t bme680_set_config(struct bme680_config *config)
{
    esp_err_t err;
    uint8_t op_mode;
    uint8_t odr20 = 0, odr3 = 1;
    uint8_t data[BME680_LEN_CONFIG] = { 0 };

    err = bme680_get_op_mode(&op_mode);
    if (err != ESP_OK)
    {
        err = bme680_set_op_mode(BME680_SLEEP_MODE);
    }

    err = i2c_read_register(BME680_I2C_ADDR, 0x71, data, BME680_LEN_CONFIG);
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    // err = boundary_check(config->filter, BME680_FILTER_SIZE_127);
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    // err = boundary_check(config->os_temp, BME680_OS_16X);
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    // err = boundary_check(config->os_pres, BME680_OS_16X);
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    // err = boundary_check(config->os_hum, BME680_OS_16X);
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    // err = boundary_check(config->os_odr, BME680_ODR_NONE);
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    data[4] = BME680_SET_BITS(data[4], BME680_FILTER, config->filter);
    data[3] = BME680_SET_BITS(data[3], BME680_OST, config->os_temp);
    data[3] = BME680_SET_BITS(data[3], BME680_OSP, config->os_pres);
    data[1] = BME680_SET_BITS_POS_0(data[3], BME680_OSP, config->os_hum);
    if (config->odr != BME680_ODR_NONE)
    {
        odr20 = config->odr;
        odr3 = 0;
    }
    data[4] = BME680_SET_BITS(data[4], BME680_ODR20, odr20);
    data[0] = BME680_SET_BITS(data[0], BME680_ODR3, odr3);

    uint8_t cmd[] = {
        BME680_I2C_ADDR | I2C_MASTER_WRITE,
        0x71,
        data[0],
        0x72,
        data[1],
        0x73,
        data[2],
        0x74,
        data[3],
        0x75,
        data[4]
    };

    err = i2c_write_many(cmd, sizeof(cmd));
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    err = bme680_set_op_mode(op_mode);


i2c_error:
    return err;
}

esp_err_t bme680_get_config(struct bme680_config *config)
{
    esp_err_t err;

    uint8_t data[BME680_LEN_CONFIG];
    err = i2c_read_register(BME680_I2C_ADDR, BME680_REG_CTRL_GAS_1, data, BME680_LEN_CONFIG);
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    config->os_hum = BME680_GET_BITS_POS_0(data[1], BME680_OSH);
    config->filter = BME680_GET_BITS(data[4], BME680_FILTER);
    config->os_temp = BME680_GET_BITS(data[3], BME680_OST);
    config->os_pres = BME680_GET_BITS(data[3], BME680_OSP);
    if (BME680_GET_BITS(data[0], BME680_ODR3))
    {
        config->odr = BME680_ODR_NONE;
    }
    else
    {
        config->odr = BME680_GET_BITS(data[4], BME680_ODR20);
    }


i2c_error:
    return err;
}

esp_err_t bme680_get_data(uint8_t op_mode, struct bme680_data *data, uint8_t *n_data, struct bme680_cal_data *cal, uint8_t variant_id)
{
    esp_err_t err = ESP_OK;
    uint8_t i = 0, j = 0, new_fields = 0;
    struct bme680_data *field_ptr[3] = { 0 };
    struct bme680_data field_data[3] = { { 0 } };

    field_ptr[0] = &field_data[0];
    field_ptr[1] = &field_data[1];
    field_ptr[2] = &field_data[2];

    if (op_mode == BME680_FORCED_MODE)
    {
        err = bme680_read_field_data(0, data, cal, variant_id);
        if (err != ESP_OK)
        {
            goto i2c_error;
        }

        if (data->status & BME680_NEW_DATA_MASK)
        {
            new_fields = 1;
        }
        else
        {
            new_fields = 0;
        }
    }
    else if (op_mode == BME680_PARALLEL_MODE || op_mode == BME680_SEQUENTIAL_MODE)
    {
        err = bme680_read_all_field_data(field_ptr, cal, variant_id);
        if (err != ESP_OK)
        {
            goto i2c_error;
        }

        new_fields = 0;
        for (i = 0; i < 3; i++)
        {
            if (field_ptr[i]->status & BME680_NEW_DATA_MASK)
            {
                new_fields++;
            }
        }

        for (i = 0; i < 2; i++)
        {
            for (j = i + 1; j < 3; j++)
            {
                sort_sensor_data(i, j, field_ptr);
            }
        }

        for (i = 0; i < 3; i++)
        {
            data[i] = *field_ptr[i];
        }
    }

    *n_data = new_fields;

i2c_error:
    return err;
}

static esp_err_t bme680_get_calibration_data(struct bme680_cal_data *cal)
{
    esp_err_t err;

    uint8_t cal_data_array[BME680_LEN_COEFF_ALL];

    err = i2c_read_register(
        BME680_I2C_ADDR,
        BME680_REG_COEFF1,
        cal_data_array,
        BME680_LEN_COEFF1
    );
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    err = i2c_read_register(
        BME680_I2C_ADDR,
        BME680_REG_COEFF2,
        &cal_data_array[BME680_LEN_COEFF1],
        BME680_LEN_COEFF2
    );
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    err = i2c_read_register(
        BME680_I2C_ADDR,
        BME680_REG_COEFF3,
        &cal_data_array[BME680_LEN_COEFF1 + BME680_LEN_COEFF2],
        BME680_LEN_COEFF3
    );
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    cal->par_t1 =
        UINT16_FROM_BYTES(cal_data_array[BME680_IDX_T1_MSB], cal_data_array[BME680_IDX_T1_LSB]);
    cal->par_t2 =
        INT16_FROM_BYTES(cal_data_array[BME680_IDX_T2_MSB], cal_data_array[BME680_IDX_T2_LSB]);
    cal->par_t3 = (int8_t)(cal_data_array[BME680_IDX_T3]);
    

    cal->par_p1 =
        UINT16_FROM_BYTES(cal_data_array[BME680_IDX_P1_MSB], cal_data_array[BME680_IDX_P1_LSB]);
    cal->par_p2 =
        INT16_FROM_BYTES(cal_data_array[BME680_IDX_P2_MSB], cal_data_array[BME680_IDX_P2_LSB]);
    cal->par_p3 = (int8_t)(cal_data_array[BME680_IDX_P3]);
    cal->par_p4 =
        INT16_FROM_BYTES(cal_data_array[BME680_IDX_P4_MSB], cal_data_array[BME680_IDX_P4_LSB]);
    cal->par_p5 =
        INT16_FROM_BYTES(cal_data_array[BME680_IDX_P5_MSB], cal_data_array[BME680_IDX_P5_LSB]);
    cal->par_p6 = (int8_t)(cal_data_array[BME680_IDX_P6]);
    cal->par_p7 = (int8_t)(cal_data_array[BME680_IDX_P7]);
    cal->par_p8 =
        INT16_FROM_BYTES(cal_data_array[BME680_IDX_P8_MSB], cal_data_array[BME680_IDX_P8_LSB]);
    cal->par_p9 =
        INT16_FROM_BYTES(cal_data_array[BME680_IDX_P9_MSB], cal_data_array[BME680_IDX_P9_LSB]);
    cal->par_p10 = (uint8_t)cal_data_array[BME680_IDX_P10];

    cal->par_h1 =
        (uint16_t)(((uint16_t)cal_data_array[BME680_IDX_H1_MSB] << 4) |
                    (cal_data_array[BME680_IDX_H1_LSB] & BME680_BIT_H1_DATA_MASK));
                    
    cal->par_h2 =
        (uint16_t)(((uint16_t)cal_data_array[BME680_IDX_H2_MSB] << 4) | (cal_data_array[BME680_IDX_H2_LSB] >> 4));
    cal->par_h3 = (int8_t)cal_data_array[BME680_IDX_H3];
    cal->par_h4 = (int8_t)cal_data_array[BME680_IDX_H4];
    cal->par_h5 = (int8_t)cal_data_array[BME680_IDX_H5];
    cal->par_h6 = (int8_t)cal_data_array[BME680_IDX_H6];
    cal->par_h7 = (int8_t)cal_data_array[BME680_IDX_H7];

    cal->par_gh1 = (int8_t)cal_data_array[BME680_IDX_GH1];
    cal->par_gh2 = INT16_FROM_BYTES(cal_data_array[BME680_IDX_GH2_MSB], cal_data_array[BME680_IDX_GH2_LSB]);
    cal->par_gh3 = (int8_t)cal_data_array[BME680_IDX_GH3];

    cal->res_heat_range = ((cal_data_array[BME680_IDX_RES_HEAT_RANGE] & BME680_RHRANGE_MASK) / 16);
    cal->res_heat_val = (int8_t)cal_data_array[BME680_IDX_RES_HEAT_VAL];
    cal->range_sw_err = ((int8_t)(cal_data_array[BME680_IDX_RANGE_SW_ERR] & BME680_RSERROR_MASK)) / 16;

i2c_error:
    return err;
}

static esp_err_t bme680_read_field_data(uint8_t index, struct bme680_data *data, struct bme680_cal_data *cal, uint8_t variant_id)
{
    esp_err_t err = ESP_OK;
    uint8_t tries = 5;
    uint8_t buffer[BME680_LEN_FIELD] = { 0 };
    uint8_t gas_range_l, gas_range_h;
    uint32_t adc_temp;
    uint32_t adc_pres;
    uint16_t adc_hum;
    uint16_t adc_gas_res_low, adc_gas_res_high;

    while (tries && err == ESP_OK)
    {
        err = i2c_read_register(
            BME680_I2C_ADDR,
            (uint8_t)(BME680_REG_FIELD0 + (index * BME680_LEN_FIELD_OFFSET)),
            buffer,
            (uint16_t)BME680_LEN_FIELD
        );

        data->status = buffer[0] & BME680_NEW_DATA_MASK;
        data->gas_index = buffer[0] & BME680_GAS_INDEX_MASK;
        data->meas_index = buffer[1];

        adc_pres = (uint32_t)(((uint32_t)buffer[2] * 4096) | ((uint32_t)buffer[3] * 16) | ((uint32_t)buffer[4] / 16));
        adc_temp = (uint32_t)(((uint32_t)buffer[5] * 4096) | ((uint32_t)buffer[6] * 16) | ((uint32_t)buffer[7] / 16));
        adc_hum = (uint16_t)(((uint32_t)buffer[8] * 256) | (uint32_t)buffer[9]);
        adc_gas_res_low = (uint16_t)((uint32_t)buffer[13] * 4 | (((uint32_t)buffer[14]) / 64));
        adc_gas_res_high = (uint16_t)((uint32_t)buffer[15] * 4 | (((uint32_t)buffer[16]) / 64));
        gas_range_l = buffer[14] & BME680_GAS_RANGE_MASK;
        gas_range_h = buffer[16] & BME680_GAS_RANGE_MASK;

        if (variant_id == BME680_VARIANT_GAS_HIGH)
        {
            data->status |= buffer[16] & BME680_GASM_VALID_MASK;
            data->status |= buffer[16] & BME680_HEAT_STAB_MASK;
        }
        else
        {
            data->status |= buffer[14] & BME680_GASM_VALID_MASK;
            data->status |= buffer[14] & BME680_HEAT_STAB_MASK;
        }

        if ((data->status & BME680_NEW_DATA_MASK) && err == ESP_OK)
        {
            err = i2c_read_register(BME680_I2C_ADDR, BME680_REG_RES_HEAT0 + data->gas_index, &data->res_heat, 1);
            if (err != ESP_OK)
            {
                break;
            }

            err = i2c_read_register(BME680_I2C_ADDR, BME680_REG_IDAC_HEAT0 + data->gas_index, &data->idac, 1);
            if (err != ESP_OK)
            {
                break;
            }

            err = i2c_read_register(BME680_I2C_ADDR, BME680_REG_GAS_WAIT0 + data->gas_index, &data->gas_wait, 1);
            if (err != ESP_OK)
            {
                break;
            }

            data->temperature = bme680_calc_temperature(adc_temp, cal);
            data->pressure = bme680_calc_pressure(adc_pres, cal);
            data->humidity = bme680_calc_humidity(adc_hum, cal);
            if (variant_id == BME680_VARIANT_GAS_HIGH)
            {
                data->gas_resistance = bme680_calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
            }
            else
            {
                data->gas_resistance = bme680_calc_gas_resistance_low(adc_gas_res_low, gas_range_l, cal);
            }

            break;
        }

        if (err == ESP_OK)
        {
            vTaskDelay(BME680_PERIOD_POLL / portTICK_PERIOD_MS);
        }

        tries--;
    }

    return err;
}

esp_err_t bme680_read_all_field_data(struct bme680_data * const data[], struct bme680_cal_data *cal, uint8_t variant_id)
{
    esp_err_t err = ESP_OK;
    uint8_t buffer[BME680_LEN_FIELD * 3] = { 0 };
    uint8_t gas_range_l, gas_range_h;
    uint32_t adc_temp;
    uint32_t adc_pres;
    uint16_t adc_hum;
    uint16_t adc_gas_res_low, adc_gas_res_high;
    uint8_t offset;
    uint8_t set_val[30] = { 0 }; /* idac, res_heat, gas_wait */
    uint8_t i;

    err = i2c_read_register(
        BME680_I2C_ADDR, BME680_REG_FIELD0, buffer, (uint32_t)BME680_LEN_FIELD * 3);
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    err = i2c_read_register(
        BME680_I2C_ADDR, BME680_REG_IDAC_HEAT0, set_val, sizeof(set_val));
    if (err != ESP_OK)
    {
        goto i2c_error;
    }

    for (i = 0; ((i < 3) && (err == ESP_OK)); i++)
    {
        offset = (uint8_t)(i * BME680_LEN_FIELD);
        data[i]->status = buffer[offset] & BME680_NEW_DATA_MASK;
        data[i]->gas_index = buffer[offset] & BME680_GAS_INDEX_MASK;
        data[i]->meas_index = buffer[offset + 1];

        /* read the raw data from the sensor */
        adc_pres =
            (uint32_t) (((uint32_t) buffer[offset + 2] * 4096) | ((uint32_t) buffer[offset + 3] * 16) |
                        ((uint32_t) buffer[offset + 4] / 16));
        adc_temp =
            (uint32_t) (((uint32_t) buffer[offset + 5] * 4096) | ((uint32_t) buffer[offset + 6] * 16) |
                        ((uint32_t) buffer[offset + 7] / 16));
        adc_hum = (uint16_t) (((uint32_t) buffer[offset + 8] * 256) | (uint32_t) buffer[offset + 9]);
        adc_gas_res_low = (uint16_t) ((uint32_t) buffer[offset + 13] * 4 | (((uint32_t) buffer[offset + 14]) / 64));
        adc_gas_res_high = (uint16_t) ((uint32_t) buffer[offset + 15] * 4 | (((uint32_t) buffer[offset + 16]) / 64));
        gas_range_l = buffer[offset + 14] & BME680_GAS_RANGE_MASK;
        gas_range_h = buffer[offset + 16] & BME680_GAS_RANGE_MASK;
        if (variant_id == BME680_VARIANT_GAS_HIGH)
        {
            data[i]->status |= buffer[offset + 16] & BME680_GASM_VALID_MASK;
            data[i]->status |= buffer[offset + 16] & BME680_HEAT_STAB_MASK;
        }
        else
        {
            data[i]->status |= buffer[offset + 14] & BME680_GASM_VALID_MASK;
            data[i]->status |= buffer[offset + 14] & BME680_HEAT_STAB_MASK;
        }

        data[i]->idac = set_val[data[i]->gas_index];
        data[i]->res_heat = set_val[10 + data[i]->gas_index];
        data[i]->gas_wait = set_val[20 + data[i]->gas_index];
        data[i]->temperature = bme680_calc_temperature(adc_temp, cal);
        data[i]->pressure = bme680_calc_pressure(adc_pres, cal);
        data[i]->humidity = bme680_calc_humidity(adc_hum, cal);
        if (variant_id == BME680_VARIANT_GAS_HIGH)
        {
            data[i]->gas_resistance = bme680_calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
        }
        else
        {
            data[i]->gas_resistance = bme680_calc_gas_resistance_low(adc_gas_res_low, gas_range_l, cal);
        }
    }

i2c_error:
    return err;

}

/* @brief This internal API is used to calculate the temperature value. */
static float bme680_calc_temperature(uint32_t temp_adc, struct bme680_cal_data *cal)
{
    float var1;
    float var2;
    float calc_temp;

    /* calculate var1 data */
    var1 = ((((float)temp_adc / 16384.0f) - ((float)cal->par_t1 / 1024.0f)) * ((float)cal->par_t2));

    /* calculate var2 data */
    var2 =
        (((((float)temp_adc / 131072.0f) - ((float)cal->par_t1 / 8192.0f)) *
          (((float)temp_adc / 131072.0f) - ((float)cal->par_t1 / 8192.0f))) * ((float)cal->par_t3 * 16.0f));

    /* t_fine value*/
    cal->t_fine = (var1 + var2);

    /* compensated temperature data*/
    calc_temp = ((cal->t_fine) / 5120.0f);

    return calc_temp;
}

/* @brief This internal API is used to calculate the pressure value. */
static float bme680_calc_pressure(uint32_t pres_adc, const struct bme680_cal_data *cal)
{
    float var1;
    float var2;
    float var3;
    float calc_pres;

    var1 = (((float)cal->t_fine / 2.0f) - 64000.0f);
    var2 = var1 * var1 * (((float)cal->par_p6) / (131072.0f));
    var2 = var2 + (var1 * ((float)cal->par_p5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)cal->par_p4) * 65536.0f);
    var1 = (((((float)cal->par_p3 * var1 * var1) / 16384.0f) + ((float)cal->par_p2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)cal->par_p1));
    calc_pres = (1048576.0f - ((float)pres_adc));

    /* Avoid exception caused by division by zero */
    if ((int)var1 != 0)
    {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
        var1 = (((float)cal->par_p9) * calc_pres * calc_pres) / 2147483648.0f;
        var2 = calc_pres * (((float)cal->par_p8) / 32768.0f);
        var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) * (cal->par_p10 / 131072.0f));
        calc_pres = (calc_pres + (var1 + var2 + var3 + ((float)cal->par_p7 * 128.0f)) / 16.0f);
    }
    else
    {
        calc_pres = 0;
    }

    return calc_pres;
}

/* This internal API is used to calculate the humidity in integer */
static float bme680_calc_humidity(uint16_t hum_adc, const struct bme680_cal_data *cal)
{
    float calc_hum;
    float var1;
    float var2;
    float var3;
    float var4;
    float temp_comp;

    /* compensated temperature data*/
    temp_comp = ((cal->t_fine) / 5120.0f);
    var1 = (float)((float)hum_adc) -
           (((float)cal->par_h1 * 16.0f) + (((float)cal->par_h3 / 2.0f) * temp_comp));
    var2 = var1 *
           ((float)(((float)cal->par_h2 / 262144.0f) *
                    (1.0f + (((float)cal->par_h4 / 16384.0f) * temp_comp) +
                     (((float)cal->par_h5 / 1048576.0f) * temp_comp * temp_comp))));
    var3 = (float)cal->par_h6 / 16384.0f;
    var4 = (float)cal->par_h7 / 2097152.0f;
    calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
    if (calc_hum > 100.0f)
    {
        calc_hum = 100.0f;
    }
    else if (calc_hum < 0.0f)
    {
        calc_hum = 0.0f;
    }

    return calc_hum;
}

/* This internal API is used to calculate the gas resistance low value in float */
static float bme680_calc_gas_resistance_low(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_cal_data *cal)
{
    float calc_gas_res;
    float var1;
    float var2;
    float var3;
    float gas_res_f = gas_res_adc;
    float gas_range_f = (1U << gas_range); /*lint !e790 / Suspicious truncation, integral to float */
    const float lookup_k1_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f, 0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
    };
    const float lookup_k2_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f, -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    var1 = (1340.0f + (5.0f * cal->range_sw_err));
    var2 = (var1) * (1.0f + lookup_k1_range[gas_range] / 100.0f);
    var3 = 1.0f + (lookup_k2_range[gas_range] / 100.0f);
    calc_gas_res = 1.0f / (float)(var3 * (0.000000125f) * gas_range_f * (((gas_res_f - 512.0f) / var2) + 1.0f));

    return calc_gas_res;
}

/* This internal API is used to calculate the gas resistance value in float */
static float bme680_calc_gas_resistance_high(uint16_t gas_res_adc, uint8_t gas_range)
{
    float calc_gas_res;
    uint32_t var1 = 262144u >> gas_range;
    int32_t var2 = (int32_t)gas_res_adc - 512;

    var2 *= 3u;
    var2 = 4096 + var2;

    calc_gas_res = 1000000.0f * (float)var1 / (float)var2;

    return calc_gas_res;
}

/* This internal API is used to calculate the heater resistance value */
static uint8_t calc_res_heat(uint16_t temp, int8_t amb_temp, const struct bme680_cal_data *cal)
{
    float var1;
    float var2;
    float var3;
    float var4;
    float var5;
    uint8_t res_heat;

    if (temp > 400) /* Cap temperature */
    {
        temp = 400;
    }

    var1 = (((float)cal->par_gh1 / (16.0f)) + 49.0f);
    var2 = ((((float)cal->par_gh2 / (32768.0f)) * (0.0005f)) + 0.00235f);
    var3 = ((float)cal->par_gh3 / (1024.0f));
    var4 = (var1 * (1.0f + (var2 * (float)temp)));
    var5 = (var4 + (var3 * (float)amb_temp));
    res_heat =
        (uint8_t)(3.4f *
                  ((var5 * (4 / (4 + (float)cal->res_heat_range)) *
                    (1 / (1 + ((float)cal->res_heat_val * 0.002f)))) -
                   25));

    return res_heat;
}

/* This internal API is used sort the sensor data */
static void swap_fields(uint8_t index1, uint8_t index2, struct bme680_data *field[])
{
    struct bme680_data *temp;

    temp = field[index1];
    field[index1] = field[index2];
    field[index2] = temp;
}

/* This internal API is used sort the sensor data */
static void sort_sensor_data(uint8_t low_index, uint8_t high_index, struct bme680_data *field[])
{
    int16_t meas_index1;
    int16_t meas_index2;

    meas_index1 = (int16_t)field[low_index]->meas_index;
    meas_index2 = (int16_t)field[high_index]->meas_index;
    if ((field[low_index]->status & BME680_NEW_DATA_MASK) && (field[high_index]->status & BME680_NEW_DATA_MASK))
    {
        int16_t diff = meas_index2 - meas_index1;
        if (((diff > -3) && (diff < 0)) || (diff > 2))
        {
            swap_fields(low_index, high_index, field);
        }
    }
    else if (field[high_index]->status & BME680_NEW_DATA_MASK)
    {
        swap_fields(low_index, high_index, field);
    }
}
