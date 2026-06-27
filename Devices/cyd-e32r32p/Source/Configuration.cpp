#include "devices/Display.h"
#include "devices/Power.h"

#include <Tactility/hal/Configuration.h>
#include <PwmBacklight.h>

using namespace tt::hal;

static bool initBoot() {
    return driver::pwmbacklight::init(DISPLAY_BACKLIGHT_PIN);
}

static tt::hal::DeviceVector createDevices() {
    return {
        createPower(),
        createDisplay(),
    };
}

extern const Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices
};