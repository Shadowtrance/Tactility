#include "drivers/sdl_display.h"

#include <tactility/device.h>
#include <tactility/device_listener.h>
#include <tactility/driver.h>
#include <tactility/error.h>
#include <tactility/log.h>
#include <tactility/module.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

constexpr auto* TAG = "Simulator";

extern "C" {

extern Driver sdl_display_driver;
extern Driver sdl_pointer_driver;
extern Driver sdl_keyboard_driver;

static Driver* const simulator_drivers[] = {
    &sdl_display_driver,
    &sdl_pointer_driver,
    &sdl_keyboard_driver,
    nullptr
};

}

/**
 * Named presets for the resolutions real devices use, so a UI can be previewed at the size
 * it will actually run at.
 *
 * Fonts are not part of this: they are selected at compile time from `lvgl.fontSize` in
 * device.properties (see device.py), so a preset changes the canvas but not the type size.
 * Good enough for layout work, not a pixel-exact reproduction of a given board.
 */
struct DisplayPreset {
    const char* name;
    uint16_t width;
    uint16_t height;
};

static constexpr DisplayPreset DISPLAY_PRESETS[] = {
    { "cardputer", 240, 135 },  // M5Stack Cardputer
    { "cyd", 240, 320 },        // CYD boards, Elecrow CrowPanel - portrait
    { "tdeck", 320, 240 },      // LilyGO T-Deck, M5Stack CoreS3
    { "tab5", 1280, 720 },      // M5Stack Tab5
};

static SdlDisplayConfig sdl_display_config = { 320, 240 };

/**
 * Applies TACTILITY_DISPLAY, which is either a preset name or an explicit "<width>x<height>".
 *
 * An unusable value logs and falls back to the default rather than failing to boot: a typo
 * in an environment variable should not stop the simulator from starting.
 */
static void apply_display_size_from_environment() {
    const char* requested = getenv("TACTILITY_DISPLAY");
    if (requested == nullptr || requested[0] == '\0') {
        return;
    }

    for (const auto& preset : DISPLAY_PRESETS) {
        if (std::strcmp(requested, preset.name) == 0) {
            sdl_display_config = { preset.width, preset.height };
            LOG_I(TAG, "Display preset '%s': %ux%u", preset.name, preset.width, preset.height);
            return;
        }
    }

    unsigned width = 0;
    unsigned height = 0;
    if (std::sscanf(requested, "%ux%u", &width, &height) == 2 &&
        width >= 64 && height >= 64 && width <= 4096 && height <= 4096) {
        sdl_display_config = { (uint16_t)width, (uint16_t)height };
        LOG_I(TAG, "Display size %ux%u", width, height);
        return;
    }

    LOG_W(TAG, "Unusable TACTILITY_DISPLAY '%s' - using %ux%u", requested,
          sdl_display_config.horizontal_resolution, sdl_display_config.vertical_resolution);
    LOG_W(TAG, "Expected <width>x<height> or one of: cardputer, cyd, tdeck, tab5");
}

// These devices have no real bus to attach to (SDL has no notion of one), but every non-root
// device is still expected to have a parent (see Device::parent) - they're parented to root once
// it's available below.
static Device sdl_display_device {};
static Device sdl_pointer_device {};
static Device sdl_keyboard_device {};

static bool construct_add_start(Device* device, Device* parent, const char* name, const void* config, const char* compatible) {
    device->address = 0;
    device->name = name;
    device->config = config;
    device->parent = nullptr;
    device->internal = nullptr;

    error_t error = device_construct(device);
    if (error != ERROR_NONE) {
        LOG_E(TAG, "Failed to construct %s: %s", name, error_to_string(error));
        return false;
    }

    device_set_parent(device, parent);

    Driver* driver = driver_find_compatible(compatible);
    if (driver == nullptr) {
        LOG_E(TAG, "No driver registered for %s", compatible);
        device_destruct(device);
        return false;
    }
    device_set_driver(device, driver);

    if (device_add(device) != ERROR_NONE) {
        LOG_E(TAG, "Failed to add %s", name);
        device_destruct(device);
        return false;
    }

    if (device_start(device) != ERROR_NONE) {
        LOG_E(TAG, "Failed to start %s", name);
        device_remove(device);
        device_destruct(device);
        return false;
    }

    return true;
}

// Root is only constructed/added/started after all dts_modules (including this one) have already
// started (see kernel_init()), so it can't be looked up by name from this module's own start() -
// wait for its DEVICE_EVENT_STARTED instead, same as e.g. m5stack-tab5's display/keyboard detection.
static void on_root_started(Device* device, DeviceEvent event, void* context) {
    if (event != DEVICE_EVENT_STARTED || strcmp(device->name, "/") != 0) {
        return;
    }

    construct_add_start(&sdl_display_device, device, "display0", &sdl_display_config, "tactility,sdl-display");
    construct_add_start(&sdl_pointer_device, device, "pointer0", nullptr, "tactility,sdl-pointer");
    construct_add_start(&sdl_keyboard_device, device, "keyboard0", nullptr, "tactility,sdl-keyboard");
}

extern "C" {

static error_t start() {
    // Before the display device is constructed below, since it takes its size from the config
    apply_display_size_from_environment();
    device_listener_add(on_root_started, nullptr);
    return ERROR_NONE;
}

static error_t stop() {
    device_listener_remove(on_root_started);
    return ERROR_NONE;
}

Module simulator_module = {
    .name = "simulator",
    .start = start,
    .stop = stop,
    .drivers = simulator_drivers
};

}
