#include <stdlib.h>

#include "i2c.h"


#define BME680_I2C_ADDR     0x76

#define BME680_CHIP_ID      0x61
#define BME680_PERIOD_RESET 10000

#define BME680_CMD_SOFT_RESET   0xb6


/* Registers */
#define BME680_REG_COEFF3                         0x00
#define BME680_REG_FIELD0                         0x1d
#define BME680_REG_IDAC_HEAT0                     0x50
#define BME680_REG_RES_HEAT0                      0x5a
#define BME680_REG_GAS_WAIT0                      0x64
#define BME680_REG_SHD_HEATR_DUR                  0x6e
#define BME680_REG_CTRL_GAS_0                     0x70
#define BME680_REG_CTRL_GAS_1                     0x71
#define BME680_REG_CTRL_HUM                       0x72
#define BME680_REG_CTRL_MEAS                      0x74
#define BME680_REG_CONFIG                         0x75
#define BME680_REG_MEM_PAGE                       0xf3
#define BME680_REG_UNIQUE_ID                      0x83
#define BME680_REG_COEFF1                         0x8a
#define BME680_REG_CHIP_ID                        0xd0
#define BME680_REG_SOFT_RESET                     0xe0
#define BME680_REG_COEFF2                         0xe1
#define BME680_REG_VARIANT_ID                     0xf0


/* Information - only available via BME680_dev.info_msg */
#define BME680_I_PARAM_CORR                       UINT8_C(1)

#define BME680_ENABLE                             UINT8_C(0x01)
#define BME680_DISABLE                            UINT8_C(0x00)

/* Variant ID macros */

/* Low Gas variant */
#define BME680_VARIANT_GAS_LOW                    UINT8_C(0x00)

/* High Gas variant */
#define BME680_VARIANT_GAS_HIGH                   UINT8_C(0x01)

/* Oversampling setting macros */

/* Switch off measurement */
#define BME680_OS_NONE                            UINT8_C(0)

/* Perform 1 measurement */
#define BME680_OS_1X                              UINT8_C(1)

/* Perform 2 measurements */
#define BME680_OS_2X                              UINT8_C(2)

/* Perform 4 measurements */
#define BME680_OS_4X                              UINT8_C(3)

/* Perform 8 measurements */
#define BME680_OS_8X                              UINT8_C(4)

/* Perform 16 measurements */
#define BME680_OS_16X                             UINT8_C(5)

/* IIR Filter settings */

/* Switch off the filter */
#define BME680_FILTER_OFF                         UINT8_C(0)

/* Filter coefficient of 2 */
#define BME680_FILTER_SIZE_1                      UINT8_C(1)

/* Filter coefficient of 4 */
#define BME680_FILTER_SIZE_3                      UINT8_C(2)

/* Filter coefficient of 8 */
#define BME680_FILTER_SIZE_7                      UINT8_C(3)

/* Filter coefficient of 16 */
#define BME680_FILTER_SIZE_15                     UINT8_C(4)

/* Filter coefficient of 32 */
#define BME680_FILTER_SIZE_31                     UINT8_C(5)

/* Filter coefficient of 64 */
#define BME680_FILTER_SIZE_63                     UINT8_C(6)

/* Filter coefficient of 128 */
#define BME680_FILTER_SIZE_127                    UINT8_C(7)

/* ODR/Standby time macros */

/* Standby time of 0.59ms */
#define BME680_ODR_0_59_MS                        UINT8_C(0)

/* Standby time of 62.5ms */
#define BME680_ODR_62_5_MS                        UINT8_C(1)

/* Standby time of 125ms */
#define BME680_ODR_125_MS                         UINT8_C(2)

/* Standby time of 250ms */
#define BME680_ODR_250_MS                         UINT8_C(3)

/* Standby time of 500ms */
#define BME680_ODR_500_MS                         UINT8_C(4)

/* Standby time of 1s */
#define BME680_ODR_1000_MS                        UINT8_C(5)

/* Standby time of 10ms */
#define BME680_ODR_10_MS                          UINT8_C(6)

/* Standby time of 20ms */
#define BME680_ODR_20_MS                          UINT8_C(7)

/* No standby time */
#define BME680_ODR_NONE                           UINT8_C(8)

/* Operating mode macros */

/* Sleep operation mode */
#define BME680_SLEEP_MODE                         UINT8_C(0)

/* Forced operation mode */
#define BME680_FORCED_MODE                        UINT8_C(1)

/* Parallel operation mode */
#define BME680_PARALLEL_MODE                      UINT8_C(2)

/* Sequential operation mode */
#define BME680_SEQUENTIAL_MODE                    UINT8_C(3)

/* SPI page macros */

/* SPI memory page 0 */
#define BME680_MEM_PAGE0                          UINT8_C(0x10)

/* SPI memory page 1 */
#define BME680_MEM_PAGE1                          UINT8_C(0x00)

/* Coefficient index macros */

/* Length for all coefficients */
#define BME680_LEN_COEFF_ALL                      UINT8_C(42)

/* Length for 1st group of coefficients */
#define BME680_LEN_COEFF1                         UINT8_C(23)

/* Length for 2nd group of coefficients */
#define BME680_LEN_COEFF2                         UINT8_C(14)

/* Length for 3rd group of coefficients */
#define BME680_LEN_COEFF3                         UINT8_C(5)

/* Length of the field */
#define BME680_LEN_FIELD                          UINT8_C(17)

/* Length between two fields */
#define BME680_LEN_FIELD_OFFSET                   UINT8_C(17)

/* Length of the configuration register */
#define BME680_LEN_CONFIG                         UINT8_C(5)

/* Length of the interleaved buffer */
#define BME680_LEN_INTERLEAVE_BUFF                UINT8_C(20)

/* Coefficient index macros */

/* Coefficient T2 LSB position */
#define BME680_IDX_T2_LSB                         (0)

/* Coefficient T2 MSB position */
#define BME680_IDX_T2_MSB                         (1)

/* Coefficient T3 position */
#define BME680_IDX_T3                             (2)

/* Coefficient P1 LSB position */
#define BME680_IDX_P1_LSB                         (4)

/* Coefficient P1 MSB position */
#define BME680_IDX_P1_MSB                         (5)

/* Coefficient P2 LSB position */
#define BME680_IDX_P2_LSB                         (6)

/* Coefficient P2 MSB position */
#define BME680_IDX_P2_MSB                         (7)

/* Coefficient P3 position */
#define BME680_IDX_P3                             (8)

/* Coefficient P4 LSB position */
#define BME680_IDX_P4_LSB                         (10)

/* Coefficient P4 MSB position */
#define BME680_IDX_P4_MSB                         (11)

/* Coefficient P5 LSB position */
#define BME680_IDX_P5_LSB                         (12)

/* Coefficient P5 MSB position */
#define BME680_IDX_P5_MSB                         (13)

/* Coefficient P7 position */
#define BME680_IDX_P7                             (14)

/* Coefficient P6 position */
#define BME680_IDX_P6                             (15)

/* Coefficient P8 LSB position */
#define BME680_IDX_P8_LSB                         (18)

/* Coefficient P8 MSB position */
#define BME680_IDX_P8_MSB                         (19)

/* Coefficient P9 LSB position */
#define BME680_IDX_P9_LSB                         (20)

/* Coefficient P9 MSB position */
#define BME680_IDX_P9_MSB                         (21)

/* Coefficient P10 position */
#define BME680_IDX_P10                            (22)

/* Coefficient H2 MSB position */
#define BME680_IDX_H2_MSB                         (23)

/* Coefficient H2 LSB position */
#define BME680_IDX_H2_LSB                         (24)

/* Coefficient H1 LSB position */
#define BME680_IDX_H1_LSB                         (24)

/* Coefficient H1 MSB position */
#define BME680_IDX_H1_MSB                         (25)

/* Coefficient H3 position */
#define BME680_IDX_H3                             (26)

/* Coefficient H4 position */
#define BME680_IDX_H4                             (27)

/* Coefficient H5 position */
#define BME680_IDX_H5                             (28)

/* Coefficient H6 position */
#define BME680_IDX_H6                             (29)

/* Coefficient H7 position */
#define BME680_IDX_H7                             (30)

/* Coefficient T1 LSB position */
#define BME680_IDX_T1_LSB                         (31)

/* Coefficient T1 MSB position */
#define BME680_IDX_T1_MSB                         (32)

/* Coefficient GH2 LSB position */
#define BME680_IDX_GH2_LSB                        (33)

/* Coefficient GH2 MSB position */
#define BME680_IDX_GH2_MSB                        (34)

/* Coefficient GH1 position */
#define BME680_IDX_GH1                            (35)

/* Coefficient GH3 position */
#define BME680_IDX_GH3                            (36)

/* Coefficient res heat value position */
#define BME680_IDX_RES_HEAT_VAL                   (37)

/* Coefficient res heat range position */
#define BME680_IDX_RES_HEAT_RANGE                 (39)

/* Coefficient range switching error position */
#define BME680_IDX_RANGE_SW_ERR                   (41)

/* Gas measurement macros */

/* Disable gas measurement */
#define BME680_DISABLE_GAS_MEAS                   UINT8_C(0x00)

/* Enable gas measurement low */
#define BME680_ENABLE_GAS_MEAS_L                  UINT8_C(0x01)

/* Enable gas measurement high */
#define BME680_ENABLE_GAS_MEAS_H                  UINT8_C(0x02)

/* Heater control macros */

/* Enable heater */
#define BME680_ENABLE_HEATER                      UINT8_C(0x00)

/* Disable heater */
#define BME680_DISABLE_HEATER                     UINT8_C(0x01)

#ifdef BME680_USE_FPU

/* 0 degree Celsius */
#define BME680_MIN_TEMPERATURE                    INT16_C(0)

/* 60 degree Celsius */
#define BME680_MAX_TEMPERATURE                    INT16_C(60)

/* 900 hecto Pascals */
#define BME680_MIN_PRESSURE                       UINT32_C(90000)

/* 1100 hecto Pascals */
#define BME680_MAX_PRESSURE                       UINT32_C(110000)

/* 20% relative humidity */
#define BME680_MIN_HUMIDITY                       UINT32_C(20)

/* 80% relative humidity*/
#define BME680_MAX_HUMIDITY                       UINT32_C(80)
#else

/* 0 degree Celsius */
#define BME680_MIN_TEMPERATURE                    INT16_C(0)

/* 60 degree Celsius */
#define BME680_MAX_TEMPERATURE                    INT16_C(6000)

/* 900 hecto Pascals */
#define BME680_MIN_PRESSURE                       UINT32_C(90000)

/* 1100 hecto Pascals */
#define BME680_MAX_PRESSURE                       UINT32_C(110000)

/* 20% relative humidity */
#define BME680_MIN_HUMIDITY                       UINT32_C(20000)

/* 80% relative humidity*/
#define BME680_MAX_HUMIDITY                       UINT32_C(80000)

#endif

#define BME680_HEATR_DUR1                         UINT16_C(1000)
#define BME680_HEATR_DUR2                         UINT16_C(2000)
#define BME680_HEATR_DUR1_DELAY                   UINT32_C(1000000)
#define BME680_HEATR_DUR2_DELAY                   UINT32_C(2000000)
#define BME680_N_MEAS                             UINT8_C(6)
#define BME680_LOW_TEMP                           UINT8_C(150)
#define BME680_HIGH_TEMP                          UINT16_C(350)

/* Mask macros */
/* Mask for number of conversions */
#define BME680_NBCONV_MSK                         UINT8_C(0X0f)

/* Mask for IIR filter */
#define BME680_FILTER_MSK                         UINT8_C(0X1c)

/* Mask for ODR[3] */
#define BME680_ODR3_MSK                           UINT8_C(0x80)

/* Mask for ODR[2:0] */
#define BME680_ODR20_MSK                          UINT8_C(0xe0)

/* Mask for temperature oversampling */
#define BME680_OST_MSK                            UINT8_C(0Xe0)

/* Mask for pressure oversampling */
#define BME680_OSP_MSK                            UINT8_C(0X1c)

/* Mask for humidity oversampling */
#define BME680_OSH_MSK                            UINT8_C(0X07)

/* Mask for heater control */
#define BME680_HCTRL_MSK                          UINT8_C(0x08)

/* Mask for run gas */
#define BME680_RUN_GAS_MSK                        UINT8_C(0x30)

/* Mask for operation mode */
#define BME680_MODE_MSK                           UINT8_C(0x03)

/* Mask for res heat range */
#define BME680_RHRANGE_MSK                        UINT8_C(0x30)

/* Mask for range switching error */
#define BME680_RSERROR_MSK                        UINT8_C(0xf0)

/* Mask for new data */
#define BME680_NEW_DATA_MSK                       UINT8_C(0x80)

/* Mask for gas index */
#define BME680_GAS_INDEX_MSK                      UINT8_C(0x0f)

/* Mask for gas range */
#define BME680_GAS_RANGE_MSK                      UINT8_C(0x0f)

/* Mask for gas measurement valid */
#define BME680_GASM_VALID_MSK                     UINT8_C(0x20)

/* Mask for heater stability */
#define BME680_HEAT_STAB_MSK                      UINT8_C(0x10)

/* Mask for SPI memory page */
#define BME680_MEM_PAGE_MSK                       UINT8_C(0x10)

/* Mask for reading a register in SPI */
#define BME680_SPI_RD_MSK                         UINT8_C(0x80)

/* Mask for writing a register in SPI */
#define BME680_SPI_WR_MSK                         UINT8_C(0x7f)

/* Mask for the H1 calibration coefficient */
#define BME680_BIT_H1_DATA_MSK                    UINT8_C(0x0f)

/* Position macros */

/* Filter bit position */
#define BME680_FILTER_POS                         UINT8_C(2)

/* Temperature oversampling bit position */
#define BME680_OST_POS                            UINT8_C(5)

/* Pressure oversampling bit position */
#define BME680_OSP_POS                            UINT8_C(2)

/* ODR[3] bit position */
#define BME680_ODR3_POS                           UINT8_C(7)

/* ODR[2:0] bit position */
#define BME680_ODR20_POS                          UINT8_C(5)

/* Run gas bit position */
#define BME680_RUN_GAS_POS                        UINT8_C(4)

/* Heater control bit position */
#define BME680_HCTRL_POS                          UINT8_C(3)



esp_err_t bme680_sw_reset()
{
    esp_err_t err;

    uint8_t cmd[] = {
        BME680_REG_SOFT_RESET,
        BME680_CMD_SOFT_RESET,
    };

    err = i2c_write_many(cmd, sizeof(cmd));
    return err;
}

esp_err_t bme680_init()
{
    esp_err_t err;
    uint8_t chip_id;
    uint8_t variant_id;

    err = bme680_sw_reset();
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

    printf("Found BMP680 with ID 0x%x and variant ID 0x%x\n", chip_id, variant_id);

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

    // /* Temperature related coefficients */
    // dev->calib.par_t1 =
    //     (uint16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T1_MSB], coeff_array[BME68X_IDX_T1_LSB]));
    // dev->calib.par_t2 =
    //     (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T2_MSB], coeff_array[BME68X_IDX_T2_LSB]));
    // dev->calib.par_t3 = (int8_t)(coeff_array[BME68X_IDX_T3]);

    // /* Pressure related coefficients */
    // dev->calib.par_p1 =
    //     (uint16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P1_MSB], coeff_array[BME68X_IDX_P1_LSB]));
    // dev->calib.par_p2 =
    //     (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P2_MSB], coeff_array[BME68X_IDX_P2_LSB]));
    // dev->calib.par_p3 = (int8_t)coeff_array[BME68X_IDX_P3];
    // dev->calib.par_p4 =
    //     (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P4_MSB], coeff_array[BME68X_IDX_P4_LSB]));
    // dev->calib.par_p5 =
    //     (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P5_MSB], coeff_array[BME68X_IDX_P5_LSB]));
    // dev->calib.par_p6 = (int8_t)(coeff_array[BME68X_IDX_P6]);
    // dev->calib.par_p7 = (int8_t)(coeff_array[BME68X_IDX_P7]);
    // dev->calib.par_p8 =
    //     (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P8_MSB], coeff_array[BME68X_IDX_P8_LSB]));
    // dev->calib.par_p9 =
    //     (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_P9_MSB], coeff_array[BME68X_IDX_P9_LSB]));
    // dev->calib.par_p10 = (uint8_t)(coeff_array[BME68X_IDX_P10]);

    // /* Humidity related coefficients */
    // dev->calib.par_h1 =
    //     (uint16_t)(((uint16_t)coeff_array[BME68X_IDX_H1_MSB] << 4) |
    //                 (coeff_array[BME68X_IDX_H1_LSB] & BME68X_BIT_H1_DATA_MSK));
    // dev->calib.par_h2 =
    //     (uint16_t)(((uint16_t)coeff_array[BME68X_IDX_H2_MSB] << 4) | ((coeff_array[BME68X_IDX_H2_LSB]) >> 4));
    // dev->calib.par_h3 = (int8_t)coeff_array[BME68X_IDX_H3];
    // dev->calib.par_h4 = (int8_t)coeff_array[BME68X_IDX_H4];
    // dev->calib.par_h5 = (int8_t)coeff_array[BME68X_IDX_H5];
    // dev->calib.par_h6 = (uint8_t)coeff_array[BME68X_IDX_H6];
    // dev->calib.par_h7 = (int8_t)coeff_array[BME68X_IDX_H7];

    // /* Gas heater related coefficients */
    // dev->calib.par_gh1 = (int8_t)coeff_array[BME68X_IDX_GH1];
    // dev->calib.par_gh2 =
    //     (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_GH2_MSB], coeff_array[BME68X_IDX_GH2_LSB]));
    // dev->calib.par_gh3 = (int8_t)coeff_array[BME68X_IDX_GH3];

    // /* Other coefficients */
    // dev->calib.res_heat_range = ((coeff_array[BME68X_IDX_RES_HEAT_RANGE] & BME68X_RHRANGE_MSK) / 16);
    // dev->calib.res_heat_val = (int8_t)coeff_array[BME68X_IDX_RES_HEAT_VAL];
    // dev->calib.range_sw_err = ((int8_t)(coeff_array[BME68X_IDX_RANGE_SW_ERR] & BME68X_RSERROR_MSK)) / 16;

i2c_error:
    return err;
}
