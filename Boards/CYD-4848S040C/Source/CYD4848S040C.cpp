#include "CYD4848S040C.h"
#include "hal/CydDisplay.h"
#include "hal/CydSdCard.h"

#include <PwmBacklight.h>

using namespace tt::hal;

bool initBoot() {
    return driver::pwmbacklight::init(GPIO_NUM_38, 1000);
}

const Configuration cyd_4848s040c_config = {
    .initBoot = initBoot,
    .createDisplay = createDisplay,
    .sdcard = createSdCard(),
    .power = nullptr,
    .i2c = {
        //Touch
        i2c::Configuration {
            .name = "Internal",
            .port = I2C_NUM_0,
            .initMode = i2c::InitMode::ByTactility,
            .isMutable = true,
            .config = (i2c_config_t) {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = GPIO_NUM_19,
                .scl_io_num = GPIO_NUM_45,
                .sda_pullup_en = true,
                .scl_pullup_en = true,
                .master = {
                    .clk_speed = 400000
                },
                .clk_flags = 0
            }
        },
        //H1 header, 
        i2c::Configuration {
            .name = "External",
            .port = I2C_NUM_1,
            .initMode = i2c::InitMode::Disabled,
            .isMutable = true,
            .config = (i2c_config_t) {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = GPIO_NUM_NC,
                .scl_io_num = GPIO_NUM_NC,
                .sda_pullup_en = false,
                .scl_pullup_en = false,
                .master = {
                    .clk_speed = 400000
                },
                .clk_flags = 0
            }
        }
    },
    .spi {
        //SD Card
        spi::Configuration {
            .device = SPI2_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .mosi_io_num = GPIO_NUM_47,
                .miso_io_num = GPIO_NUM_41,
                .sclk_io_num = GPIO_NUM_48,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
                .data4_io_num = -1,
                .data5_io_num = -1,
                .data6_io_num = -1,
                .data7_io_num = -1,
                .data_io_default_level = false,
                .max_transfer_sz = 8192,
                .flags = 0,
                .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                .intr_flags = 0
            },
            .initMode = spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = nullptr
        }
    }
};
