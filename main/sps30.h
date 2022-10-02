#ifndef __SPS30_H
#define __SPS30_H

#include <stdint.h>

#include "i2c.h"

#define SPS30_OK        0
#define SPS30_ERR_I2C   -1
#define SPS30_ERR_CRC   -2

typedef int sps30_err_t;

struct sps30_result {
    // PM1.0 in µg/m³.
    float pm1p0;
    // PM2.5 in µg/m³.
    float pm2p5;
    // PM4.0 in µg/m³.
    float pm4p0;
    // PM10 in µg/m³.
    float pm10;

    // PM0.5 in #/cm³.
    float pm0p5cm3;

    // PM1.0 in #/cm³.
    float pm1p0cm3;

    // PM2.5 in #/cm³.
    float pm2p5cm3;

    // PM4 in #/cm³.
    float pm4p0cm3;

    // PM10 in #/cm³.
    float pm10cm3;

    // Typical particle size.
    float typ_particle_size;
};

sps30_err_t sps30_start_measurement();
sps30_err_t sps30_stop_measurement();
sps30_err_t sps30_get_data_ready_flag(uint8_t *result);
sps30_err_t sps30_read_data_ready_flag(uint8_t *result);
sps30_err_t sps30_read_measured_values(struct sps30_result *result);

#endif
