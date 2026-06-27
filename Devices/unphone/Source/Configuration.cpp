#include "UnPhoneFeatures.h"
#include "devices/Hx8357Display.h"

#include <Tactility/hal/Configuration.h>

bool initBoot();

static tt::hal::DeviceVector createDevices() {
    return {
        createDisplay(),
    };
}

extern const tt::hal::Configuration hardwareConfiguration = {
    .initBoot = initBoot,
    .createDevices = createDevices
};
