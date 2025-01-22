#pragma once

#include "driver/spi_common.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

// SPI 3 - sdcard
#define JC8048W550_SPI3_PIN_SCLK GPIO_NUM_12
#define JC8048W550_SPI3_PIN_MOSI GPIO_NUM_11
#define JC8048W550_SPI3_PIN_MISO GPIO_NUM_13
#define JC8048W550_SPI3_TRANSACTION_LIMIT 8192 // TODO: Determine proper limit
