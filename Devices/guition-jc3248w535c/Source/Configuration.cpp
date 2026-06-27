#include "devices/Display.h"

#include <Tactility/hal/Configuration.h>
#include <PwmBacklight.h>

using namespace tt::hal;

static DeviceVector createDevices() {
    return {
        createDisplay(),
    };
}

static bool initBoot() {
    return driver::pwmbacklight::init(GPIO_NUM_1);
}

extern const Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices
};
