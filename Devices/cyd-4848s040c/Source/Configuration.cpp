#include "devices/St7701Display.h"

#include <Tactility/hal/Configuration.h>
#include <PwmBacklight.h>

using namespace tt::hal;

static bool initBoot() {
    return driver::pwmbacklight::init(GPIO_NUM_38, 1000);
}

static DeviceVector createDevices() {
    return {
        std::make_shared<St7701Display>(),
    };
}

extern const Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices
};
