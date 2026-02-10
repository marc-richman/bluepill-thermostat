#ifndef I2C_SHT40_H
#define I2C_SHT40_H

#include <stdint.h>

/* Error codes: 0 == OK, negative = error */
enum {
    SHT40_OK = 0,
    SHT40_ERR_I2C = -1,
    SHT40_ERR_NACK = -2,
    SHT40_ERR_TIMEOUT = -3,
    SHT40_ERR_CRC = -4,
    SHT40_ERR_BUS = -5,
};

void sht40_i2c_init(void);
void sht40_i2c_reinit(void);

/* Read temperature in centi-°C. Returns SHT40_OK on success. */
int sht40_read_temperature_centi(int32_t *temp_centi);

#endif /* I2C_SHT40_H */