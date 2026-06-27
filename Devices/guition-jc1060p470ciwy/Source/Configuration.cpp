#include "devices/Display.h"
#include "devices/Power.h"

#include <Tactility/hal/Configuration.h>

using namespace tt::hal;

static DeviceVector createDevices() {
    return {
        createDisplay(),
        createPower()
    };
}

extern const Configuration hardwareConfiguration = {
    .createDevices = createDevices,
};
