#include "devices/Display.h"

#include <Tactility/hal/Configuration.h>
#include <PwmBacklight.h>

static bool initBoot() {
    return driver::pwmbacklight::init(LCD_BACKLIGHT_PIN);
}

static tt::hal::DeviceVector createDevices() {
    return {
        createDisplay(),
    };
}

extern const tt::hal::Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices
};
