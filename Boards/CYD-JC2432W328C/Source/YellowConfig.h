#pragma once

#include "driver/spi_common.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "hal/YellowDisplayConstants.h"

// SPI 2 - display
#define JC2432W328C_SPI2_PIN_SCLK GPIO_NUM_14
#define JC2432W328C_SPI2_PIN_MOSI GPIO_NUM_13
#define JC2432W328C_SPI2_TRANSACTION_LIMIT JC2432W328C_LCD_DRAW_BUFFER_SIZE

// SPI 3 - sdcard
#define JC2432W328C_SPI3_PIN_SCLK GPIO_NUM_18
#define JC2432W328C_SPI3_PIN_MOSI GPIO_NUM_23
#define JC2432W328C_SPI3_PIN_MISO GPIO_NUM_19
#define JC2432W328C_SPI3_TRANSACTION_LIMIT 8192 // TODO: Determine proper limit
