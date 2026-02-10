/*
 * src/i2c_sht40.c
 *
 * STM32F103 low-level I2C + SHT40 support.
 */

#include "i2c_sht40.h"
#include "utils.h" /* provides system_millis */

#include <string.h>
#include <stdio.h>

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

/* Ensure SHT40/I2C macros are present when this file is compiled standalone. */
#ifndef SHT40_I2C_MAX_RETRIES
#define SHT40_I2C_MAX_RETRIES           3U
#endif
#ifndef SHT40_I2C_MAX_BUS_REINITS
#define SHT40_I2C_MAX_BUS_REINITS       4U
#endif
#ifndef SHT40_MEAS_MAX_TIME_MS
#define SHT40_MEAS_MAX_TIME_MS          9U
#endif
#ifndef SHT40_I2C_RETRY_DELAY_MS
#define SHT40_I2C_RETRY_DELAY_MS        2U
#endif

/* Local definitions tuned for STM32F103 */
#define SHT40_I2C               I2C1
#define SHT40_ADDR              0x44
#define SHT40_CMD_MEAS_HIGH_PREC 0xFD

#define I2C_PCLK1_FREQ_MHZ      36U
#define I2C_CCR_SM_100KHZ_36MHZ 180U
#define I2C_TRISE_SM_36MHZ      (I2C_PCLK1_FREQ_MHZ + 1U)

#define I2C_FLAG_TIMEOUT_MS     5U
#define I2C_BYTE_TIMEOUT_MS     5U

/* SR1 masks */
#define I2C_SR1_SB_MASK      (1U << 0)
#define I2C_SR1_ADDR_MASK    (1U << 1)
#define I2C_SR1_BTF_MASK     (1U << 2)
#define I2C_SR1_RXNE_MASK    (1U << 6)
#define I2C_SR1_TXE_MASK     (1U << 7)
#define I2C_SR1_BERR_MASK    (1U << 8)
#define I2C_SR1_ARLO_MASK    (1U << 9)
#define I2C_SR1_AF_MASK      (1U << 10)

/* Use the shared system_millis from utils.c (declared in utils.h) */
static uint32_t local_millis(void) { return system_millis; }

/* Wait for SR1 mask to be set; detect basic errors */
static bool i2c_wait_flag(uint32_t i2c, uint32_t sr1_mask, uint32_t timeout_ms)
{
    uint32_t start = local_millis();
    while (!(I2C_SR1(i2c) & sr1_mask)) {
        uint32_t sr1 = I2C_SR1(i2c);
        if (sr1 & (I2C_SR1_BERR_MASK | I2C_SR1_ARLO_MASK | I2C_SR1_AF_MASK)) {
            return false;
        }
        if ((local_millis() - start) >= timeout_ms) return false;
    }
    return true;
}

static int i2c_write_bytes_internal(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t len,
                                    uint32_t timeout_ms, char *err, size_t errlen)
{
    i2c_send_start(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB_MASK, timeout_ms)) {
        if (err) snprintf(err, errlen, "START_TO");
        goto err_no_stop;
    }

    i2c_send_7bit_address(i2c, addr, I2C_WRITE);
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR_MASK, timeout_ms)) {
        if (I2C_SR1(i2c) & I2C_SR1_AF_MASK) {
            if (err) snprintf(err, errlen, "ADDR_NACK");
            i2c_send_stop(i2c);
            return -2;
        }
        if (err) snprintf(err, errlen, "ADDR_TO");
        i2c_send_stop(i2c);
        return -1;
    }
    (void)I2C_SR1(i2c);
    (void)I2C_SR2(i2c);

    for (size_t i = 0; i < len; ++i) {
        I2C_DR(i2c) = data[i];
        if (!i2c_wait_flag(i2c, I2C_SR1_TXE_MASK, timeout_ms)) {
            if (I2C_SR1(i2c) & I2C_SR1_AF_MASK) {
                if (err) snprintf(err, errlen, "DATA_NACK");
                i2c_send_stop(i2c);
                return -2;
            }
            if (err) snprintf(err, errlen, "DATA_TO");
            i2c_send_stop(i2c);
            return -1;
        }
    }

    (void)i2c_wait_flag(i2c, I2C_SR1_BTF_MASK, timeout_ms);

    i2c_send_stop(i2c);
    return 0;

err_no_stop:
    i2c_send_stop(i2c);
    return -1;
}

static int i2c_read_bytes_internal(uint32_t i2c, uint8_t addr, uint8_t *buf, size_t len,
                                   uint32_t timeout_ms, char *err, size_t errlen)
{
    if (len == 0) return 0;

    /* Ensure ACK enabled */
    I2C_CR1(i2c) |= I2C_CR1_ACK;

    i2c_send_start(i2c);
    if (!i2c_wait_flag(i2c, I2C_SR1_SB_MASK, timeout_ms)) {
        if (err) snprintf(err, errlen, "START_TO");
        goto rd_err_no_stop;
    }

    i2c_send_7bit_address(i2c, addr, I2C_READ);
    if (!i2c_wait_flag(i2c, I2C_SR1_ADDR_MASK, timeout_ms)) {
        if (I2C_SR1(i2c) & I2C_SR1_AF_MASK) {
            if (err) snprintf(err, errlen, "ADDR_NACK");
            i2c_send_stop(i2c);
            return -2;
        }
        if (err) snprintf(err, errlen, "ADDR_TO");
        i2c_send_stop(i2c);
        return -1;
    }

    (void)I2C_SR1(i2c);
    (void)I2C_SR2(i2c);

    for (size_t i = 0; i < len; ++i) {
        if (i == (len - 1)) {
            I2C_CR1(i2c) &= ~I2C_CR1_ACK;
            i2c_send_stop(i2c);
        }
        if (!i2c_wait_flag(i2c, I2C_SR1_RXNE_MASK, timeout_ms)) {
            uint32_t sr1 = I2C_SR1(i2c);
            if (sr1 & (I2C_SR1_BERR_MASK | I2C_SR1_ARLO_MASK)) {
                if (err) snprintf(err, errlen, "BUS_ERR");
                return -3;
            }
            if (err) snprintf(err, errlen, "RXNE_TO");
            return -1;
        }
        buf[i] = (uint8_t)I2C_DR(i2c);
    }

    /* Re-enable ACK for future ops */
    I2C_CR1(i2c) |= I2C_CR1_ACK;
    return 0;

rd_err_no_stop:
    i2c_send_stop(i2c);
    return -1;
}

void sht40_i2c_init(void)
{
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);

    i2c_peripheral_disable(SHT40_I2C);
    I2C_CR1(SHT40_I2C) |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 100; ++i) __asm__("nop");
    I2C_CR1(SHT40_I2C) &= ~I2C_CR1_SWRST;

    i2c_set_clock_frequency(SHT40_I2C, I2C_PCLK1_FREQ_MHZ);
    i2c_set_standard_mode(SHT40_I2C);
    i2c_set_ccr(SHT40_I2C, I2C_CCR_SM_100KHZ_36MHZ);
    i2c_set_trise(SHT40_I2C, I2C_TRISE_SM_36MHZ);

    I2C_CR1(SHT40_I2C) |= I2C_CR1_ACK;
    i2c_peripheral_enable(SHT40_I2C);
}

void sht40_i2c_reinit(void)
{
    i2c_peripheral_disable(SHT40_I2C);
    I2C_CR1(SHT40_I2C) |= I2C_CR1_SWRST;
    for (volatile int i = 0; i < 100; ++i) __asm__("nop");
    I2C_CR1(SHT40_I2C) &= ~I2C_CR1_SWRST;

    i2c_set_clock_frequency(SHT40_I2C, I2C_PCLK1_FREQ_MHZ);
    i2c_set_standard_mode(SHT40_I2C);
    i2c_set_ccr(SHT40_I2C, I2C_CCR_SM_100KHZ_36MHZ);
    i2c_set_trise(SHT40_I2C, I2C_TRISE_SM_36MHZ);

    I2C_CR1(SHT40_I2C) |= I2C_CR1_ACK;
    i2c_peripheral_enable(SHT40_I2C);
}

static uint8_t sht40_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* Local busy delay (used only inside this module) */
static void i2c_delay_ms(uint32_t ms)
{
    uint32_t start = local_millis();
    while ((local_millis() - start) < ms) {
        /* spin; USB poll not available in this module */
    }
}

int sht40_read_temperature_centi(int32_t *temp_centi)
{
    uint8_t cmd = SHT40_CMD_MEAS_HIGH_PREC;
    uint8_t buf[6];
    unsigned int bus_reinit_count = 0;

    for (uint8_t attempt = 0; attempt < SHT40_I2C_MAX_RETRIES; ++attempt) {
        if (bus_reinit_count > SHT40_I2C_MAX_BUS_REINITS) break;

        char i2c_err[64] = {0};
        int wres = i2c_write_bytes_internal(SHT40_I2C, SHT40_ADDR, &cmd, 1,
                                            I2C_FLAG_TIMEOUT_MS, i2c_err, sizeof(i2c_err));
        if (wres != 0) {
            bus_reinit_count++;
            sht40_i2c_reinit();
            continue;
        }

        uint32_t tstart = local_millis();
        uint32_t read_attempts = 0;
        uint32_t crc_fail_count = 0;

        while ((local_millis() - tstart) < (SHT40_MEAS_MAX_TIME_MS + 11U)) {
            memset(buf, 0, sizeof(buf));
            int rres = i2c_read_bytes_internal(SHT40_I2C, SHT40_ADDR, buf, sizeof(buf),
                                               I2C_BYTE_TIMEOUT_MS, i2c_err, sizeof(i2c_err));
            ++read_attempts;
            if (rres == 0) {
                if ((sht40_crc8(&buf[0], 2) == buf[2]) &&
                    (sht40_crc8(&buf[3], 2) == buf[5])) {
                    uint16_t t_raw = (uint16_t)((buf[0] << 8) | buf[1]);
                    int32_t t_milli = ((21875 * (int32_t)t_raw) >> 13) - 45000;
                    if (t_milli >= 0) *temp_centi = (t_milli + 5) / 10;
                    else              *temp_centi = (t_milli - 5) / 10;
                    return SHT40_OK;
                }
                ++crc_fail_count;
            } else {
                ++crc_fail_count;
            }
            i2c_delay_ms(SHT40_I2C_RETRY_DELAY_MS);
        }

        bus_reinit_count++;
        sht40_i2c_reinit();
        i2c_delay_ms(2);
    }

    return SHT40_ERR_I2C;
}