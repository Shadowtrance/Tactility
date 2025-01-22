#include "JC8048W550C.h"
#include "hal/YellowDisplay.h"
#include "hal/YellowSdCard.h"

bool jc8048w550c_lvgl_init();
bool jc8048w550c_boot();

const tt::hal::Configuration cyd_jc8048w550c_config = {
    .initBoot = &jc8048w550c_boot,
    .initLvgl = &jc8048w550c_lvgl_init,
    .createDisplay = createDisplay,
    .sdcard = createYellowSdCard(),
    .power = nullptr,
    .i2c = {
        tt::hal::i2c::Configuration {
            .name = "First",
            .port = I2C_NUM_0,
            .initMode = tt::hal::i2c::InitMode::Disabled,
            .canReinit = true,
            .hasMutableConfiguration = true,
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
        },
        tt::hal::i2c::Configuration {
            .name = "Second",
            .port = I2C_NUM_1,
            .initMode = tt::hal::i2c::InitMode::Disabled,
            .canReinit = true,
            .hasMutableConfiguration = true,
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
    }
};
