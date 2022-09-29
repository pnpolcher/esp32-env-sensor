#ifndef __BME680_H
#define __BME680_H

#include <stdint.h>

#include "esp_err.h"

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

#define BME680_ENABLE                             UINT8_C(0x01)
#define BME680_DISABLE                            UINT8_C(0x00)

/* Variant ID macros */

#define BME680_VARIANT_GAS_LOW                    UINT8_C(0x00)
#define BME680_VARIANT_GAS_HIGH                   UINT8_C(0x01)

/* Oversampling setting macros */

#define BME680_OS_NONE                            UINT8_C(0)
#define BME680_OS_1X                              UINT8_C(1)
#define BME680_OS_2X                              UINT8_C(2)
#define BME680_OS_4X                              UINT8_C(3)
#define BME680_OS_8X                              UINT8_C(4)
#define BME680_OS_16X                             UINT8_C(5)

#define BME680_FILTER_OFF                         UINT8_C(0)
#define BME680_FILTER_SIZE_1                      UINT8_C(1)
#define BME680_FILTER_SIZE_3                      UINT8_C(2)
#define BME680_FILTER_SIZE_7                      UINT8_C(3)
#define BME680_FILTER_SIZE_15                     UINT8_C(4)
#define BME680_FILTER_SIZE_31                     UINT8_C(5)
#define BME680_FILTER_SIZE_63                     UINT8_C(6)
#define BME680_FILTER_SIZE_127                    UINT8_C(7)

/* ODR/Standby time macros */

#define BME680_ODR_0_59_MS                        UINT8_C(0)
#define BME680_ODR_62_5_MS                        UINT8_C(1)
#define BME680_ODR_125_MS                         UINT8_C(2)
#define BME680_ODR_250_MS                         UINT8_C(3)
#define BME680_ODR_500_MS                         UINT8_C(4)
#define BME680_ODR_1000_MS                        UINT8_C(5)
#define BME680_ODR_10_MS                          UINT8_C(6)
#define BME680_ODR_20_MS                          UINT8_C(7)

#define BME680_ODR_NONE                           UINT8_C(8)

/* Operating mode macros */

/* Sleep operation mode */
#define BME680_SLEEP_MODE                         UINT8_C(0)
#define BME680_FORCED_MODE                        UINT8_C(1)
#define BME680_PARALLEL_MODE                      UINT8_C(2)
#define BME680_SEQUENTIAL_MODE                    UINT8_C(3)

/* SPI page macros */

#define BME680_MEM_PAGE0                          UINT8_C(0x10)
#define BME680_MEM_PAGE1                          UINT8_C(0x00)

/* Coefficient index macros */

#define BME680_LEN_COEFF_ALL                      UINT8_C(42)
#define BME680_LEN_COEFF1                         UINT8_C(23)
#define BME680_LEN_COEFF2                         UINT8_C(14)
#define BME680_LEN_COEFF3                         UINT8_C(5)
#define BME680_LEN_FIELD                          UINT8_C(17)
#define BME680_LEN_FIELD_OFFSET                   UINT8_C(17)
#define BME680_LEN_CONFIG                         UINT8_C(5)
#define BME680_LEN_INTERLEAVE_BUFF                UINT8_C(20)

/* Coefficient index macros */

#define BME680_IDX_T2_LSB                         (0)
#define BME680_IDX_T2_MSB                         (1)
#define BME680_IDX_T3                             (2)
#define BME680_IDX_P1_LSB                         (4)
#define BME680_IDX_P1_MSB                         (5)
#define BME680_IDX_P2_LSB                         (6)
#define BME680_IDX_P2_MSB                         (7)
#define BME680_IDX_P3                             (8)
#define BME680_IDX_P4_LSB                         (10)
#define BME680_IDX_P4_MSB                         (11)
#define BME680_IDX_P5_LSB                         (12)
#define BME680_IDX_P5_MSB                         (13)
#define BME680_IDX_P7                             (14)
#define BME680_IDX_P6                             (15)
#define BME680_IDX_P8_LSB                         (18)
#define BME680_IDX_P8_MSB                         (19)
#define BME680_IDX_P9_LSB                         (20)
#define BME680_IDX_P9_MSB                         (21)
#define BME680_IDX_P10                            (22)
#define BME680_IDX_H2_MSB                         (23)
#define BME680_IDX_H2_LSB                         (24)
#define BME680_IDX_H1_LSB                         (24)
#define BME680_IDX_H1_MSB                         (25)
#define BME680_IDX_H3                             (26)
#define BME680_IDX_H4                             (27)
#define BME680_IDX_H5                             (28)
#define BME680_IDX_H6                             (29)
#define BME680_IDX_H7                             (30)
#define BME680_IDX_T1_LSB                         (31)
#define BME680_IDX_T1_MSB                         (32)
#define BME680_IDX_GH2_LSB                        (33)
#define BME680_IDX_GH2_MSB                        (34)
#define BME680_IDX_GH1                            (35)
#define BME680_IDX_GH3                            (36)
#define BME680_IDX_RES_HEAT_VAL                   (37)
#define BME680_IDX_RES_HEAT_RANGE                 (39)
#define BME680_IDX_RANGE_SW_ERR                   (41)

/* Gas measurement macros */

#define BME680_DISABLE_GAS_MEAS                   UINT8_C(0x00)
#define BME680_ENABLE_GAS_MEAS_L                  UINT8_C(0x01)
#define BME680_ENABLE_GAS_MEAS_H                  UINT8_C(0x02)

/* Heater control macros */
#define BME680_ENABLE_HEATER                      UINT8_C(0x00)
#define BME680_DISABLE_HEATER                     UINT8_C(0x01)

#define BME680_MIN_TEMPERATURE                    INT16_C(0)
#define BME680_MAX_TEMPERATURE                    INT16_C(60)
#define BME680_MIN_PRESSURE                       UINT32_C(90000)
#define BME680_MAX_PRESSURE                       UINT32_C(110000)
#define BME680_MIN_HUMIDITY                       UINT32_C(20)
#define BME680_MAX_HUMIDITY                       UINT32_C(80)
#define BME680_HEATR_DUR1                         UINT16_C(1000)
#define BME680_HEATR_DUR2                         UINT16_C(2000)
#define BME680_HEATR_DUR1_DELAY                   UINT32_C(1000000)
#define BME680_HEATR_DUR2_DELAY                   UINT32_C(2000000)
#define BME680_N_MEAS                             UINT8_C(6)
#define BME680_LOW_TEMP                           UINT8_C(150)
#define BME680_HIGH_TEMP                          UINT16_C(350)

/* Mask macros */
#define BME680_NBCONV_MASK                         UINT8_C(0X0f)
#define BME680_FILTER_MASK                         UINT8_C(0X1c)
#define BME680_ODR3_MASK                           UINT8_C(0x80)
#define BME680_ODR20_MASK                          UINT8_C(0xe0)
#define BME680_OST_MASK                            UINT8_C(0Xe0)
#define BME680_OSP_MASK                            UINT8_C(0X1c)
#define BME680_OSH_MASK                            UINT8_C(0X07)
#define BME680_HCTRL_MASK                          UINT8_C(0x08)
#define BME680_RUN_GAS_MASK                        UINT8_C(0x30)
#define BME680_MODE_MASK                           UINT8_C(0x03)
#define BME680_RHRANGE_MASK                        UINT8_C(0x30)
#define BME680_RSERROR_MASK                        UINT8_C(0xf0)
#define BME680_NEW_DATA_MASK                       UINT8_C(0x80)
#define BME680_GAS_INDEX_MASK                      UINT8_C(0x0f)
#define BME680_GAS_RANGE_MASK                      UINT8_C(0x0f)
#define BME680_GASM_VALID_MASK                     UINT8_C(0x20)
#define BME680_HEAT_STAB_MASK                      UINT8_C(0x10)
#define BME680_MEM_PAGE_MASK                       UINT8_C(0x10)
#define BME680_SPI_RD_MASK                         UINT8_C(0x80)
#define BME680_SPI_WR_MASK                         UINT8_C(0x7f)
#define BME680_BIT_H1_DATA_MASK                    UINT8_C(0x0f)

/* Position macros */

#define BME680_FILTER_POS                         UINT8_C(2)
#define BME680_OST_POS                            UINT8_C(5)
#define BME680_OSP_POS                            UINT8_C(2)
#define BME680_ODR3_POS                           UINT8_C(7)
#define BME680_ODR20_POS                          UINT8_C(5)
#define BME680_RUN_GAS_POS                        UINT8_C(4)
#define BME680_HCTRL_POS                          UINT8_C(3)

struct bme680_cal_data
{
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t par_h3;
    int8_t par_h4;
    int8_t par_h5;
    uint8_t par_h6;
    int8_t par_h7;
    int8_t par_gh1;
    int16_t par_gh2;
    int8_t par_gh3;
    uint16_t par_t1;
    int16_t par_t2;
    int8_t par_t3;
    uint16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int16_t par_p4;
    int16_t par_p5;
    int8_t par_p6;
    int8_t par_p7;
    int16_t par_p8;
    int16_t par_p9;
    uint8_t par_p10;

    float t_fine;

    uint8_t res_heat_range;
    int8_t res_heat_val;
    int8_t range_sw_err;
};

struct bme680_config
{
    uint8_t os_hum;
    uint8_t os_temp;
    uint8_t os_pres;
    uint8_t filter;
    uint8_t odr;
};

struct bme680_data
{
    uint8_t status;
    uint8_t gas_index;
    uint8_t meas_index;
    uint8_t res_heat;
    uint8_t idac;
    uint8_t gas_wait;

    float temperature;
    float pressure;
    float humidity;
    float gas_resistance;
};

esp_err_t bme680_init();

#endif
