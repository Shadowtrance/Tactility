#pragma once

#include "driver/spi_common.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

// SPI 3 - sdcard
#define CYD8048S043_SPI3_PIN_SCLK GPIO_NUM_12
#define CYD8048S043_SPI3_PIN_MOSI GPIO_NUM_11
#define CYD8048S043_SPI3_PIN_MISO GPIO_NUM_13
#define CYD8048S043_SPI3_TRANSACTION_LIMIT 8192 // TODO: Determine proper limit
