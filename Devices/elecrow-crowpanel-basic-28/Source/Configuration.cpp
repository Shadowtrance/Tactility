#include "PwmBacklight.h"
#include "devices/Display.h"

#include <Tactility/hal/Configuration.h>

using namespace tt::hal;

static bool initBoot() {
    return driver::pwmbacklight::init(GPIO_NUM_27);
}

static DeviceVector createDevices() {
    return {
        createDisplay(),
    };
}

extern const Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices
};
