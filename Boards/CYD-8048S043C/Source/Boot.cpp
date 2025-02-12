#include "YellowConfig.h"
#include "TactilityCore.h"
#include "hal/YellowTouchConstants.h"
#include <driver/spi_common.h>

#define TAG "cyd8048s043_bootstrap"

static bool init_i2c() {
    TT_LOG_I(TAG, LOG_MESSAGE_I2C_INIT_START);

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_19,
        .scl_io_num = GPIO_NUM_20,
        .sda_pullup_en = false,
        .scl_pullup_en = false,
        .master = {
            .clk_speed = 400000
        }
    };

    if (i2c_param_config(CYD8048S043_TOUCH_I2C_PORT, &i2c_conf) != ESP_OK) {
        TT_LOG_E(TAG, LOG_MESSAGE_I2C_INIT_CONFIG_FAILED );
        return false;
    }

    if (i2c_driver_install(CYD8048S043_TOUCH_I2C_PORT, i2c_conf.mode, 0, 0, 0) != ESP_OK) {
        TT_LOG_E(TAG, LOG_MESSAGE_I2C_INIT_DRIVER_INSTALL_FAILED);
        return false;
    }

    return true;
}

static bool init_spi() {
    TT_LOG_I(TAG, LOG_MESSAGE_SPI_INIT_START_FMT, SPI2_HOST);

    const spi_bus_config_t bus_config = {
        .mosi_io_num = CYD8048S043_SPI3_PIN_MOSI,
        .miso_io_num = CYD8048S043_SPI3_PIN_MISO,
        .sclk_io_num = CYD8048S043_SPI3_PIN_SCLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .data4_io_num = 0,
        .data5_io_num = 0,
        .data6_io_num = 0,
        .data7_io_num = 0,
        .max_transfer_sz = CYD8048S043_SPI3_TRANSACTION_LIMIT,
        .flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0
    };

    if (spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO) != ESP_OK) {
        TT_LOG_E(TAG, LOG_MESSAGE_SPI_INIT_FAILED_FMT, SPI2_HOST);
        return false;
    }

    return true;
}

bool cyd8048s043c_boot() {
    return init_i2c() && init_spi();
}
