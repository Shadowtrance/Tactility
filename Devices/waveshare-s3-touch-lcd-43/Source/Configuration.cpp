#include "devices/Display.h"

#include <Tactility/hal/Configuration.h>

using namespace tt::hal;

static DeviceVector createDevices() {
    return {
        createDisplay(),
    };
}

extern const Configuration hardwareConfiguration = {
    .createDevices = createDevices
};
