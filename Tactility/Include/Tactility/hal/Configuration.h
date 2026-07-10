#pragma once

#include <memory>
#include <tactility/hal/Device.h>
#include <vector>

namespace tt::hal {

typedef bool (*InitBoot)();

typedef std::vector<std::shared_ptr<Device>> DeviceVector;

typedef std::shared_ptr<Device> (*CreateDevice)();

struct Configuration {
    /**
     * Runs before kernel_init(), i.e. before devicetree devices are constructed/started.
     * Only raw hardware access (e.g. direct gpio_config/gpio_set_level) is valid here --
     * devicetree devices (device_find_by_name() etc.) do not exist yet. Use this for
     * board-level power-on sequencing that devicetree devices depend on (e.g. a GPIO that
     * gates the rail powering an I2C-attached codec).
     */
    const InitBoot earlyInit = nullptr;

    /**
     * Runs after kernel_init(), i.e. after devicetree devices are constructed/started.
     * Used for powering on peripherals that depend on devicetree devices (e.g. GPIO
     * expanders) being available via device_find_by_name().
     */
    const InitBoot initBoot = nullptr;

    const std::function<DeviceVector()> createDevices = [] { return DeviceVector(); };
};

} // namespace
