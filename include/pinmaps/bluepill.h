#pragma once

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

/* -------- Heater output -------- */
#define HEATER_PORT   GPIOB
#define HEATER_PIN    GPIO0

/* -------- SHT40 / I2C -------- */
#define SHT40_I2C     I2C1
#define SHT40_ADDR    0x44

/* STM32F103 I2C1 default pins */
#define SHT40_SCL_PORT GPIOB
#define SHT40_SCL_PIN  GPIO6
#define SHT40_SDA_PORT GPIOB
#define SHT40_SDA_PIN  GPIO7
#define SHT40_GPIO_RCC RCC_GPIOB
#define SHT40_I2C_RCC  RCC_I2C1

