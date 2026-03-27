#include <Tactility/service/bluetooth/BluetoothSettings.h>

#include <Tactility/file/File.h>
#include <Tactility/file/PropertiesFile.h>
#include <Tactility/Logger.h>
#include <Tactility/Mutex.h>
#include <Tactility/service/ServicePaths.h>
#include <Tactility/service/bluetooth/BluetoothPrivate.h>

namespace tt::service::bluetooth::settings {

static const auto LOGGER = Logger("BluetoothSettings");
constexpr auto* SETTINGS_KEY_ENABLE_ON_BOOT  = "enableOnBoot";
constexpr auto* SETTINGS_KEY_SPP_AUTO_START  = "sppAutoStart";
constexpr auto* SETTINGS_KEY_MIDI_AUTO_START = "midiAutoStart";

struct BluetoothSettings {
    bool enableOnBoot  = false;
    bool sppAutoStart  = false;
    bool midiAutoStart = false;
};

static Mutex settings_mutex;
static BluetoothSettings cachedSettings;
static bool cached = false;

static bool load(BluetoothSettings& settings) {
    auto service_context = findServiceContext();
    if (service_context == nullptr) {
        return false;
    }

    std::map<std::string, std::string> map;
    std::string settings_path = service_context->getPaths()->getUserDataPath("settings.properties");
    if (!file::loadPropertiesFile(settings_path, map)) {
        return false;
    }

    auto it = map.find(SETTINGS_KEY_ENABLE_ON_BOOT);
    if (it == map.end()) {
        return false;
    }

    settings.enableOnBoot = (it->second == "true");

    auto spp_it = map.find(SETTINGS_KEY_SPP_AUTO_START);
    settings.sppAutoStart = (spp_it != map.end() && spp_it->second == "true");

    auto midi_it = map.find(SETTINGS_KEY_MIDI_AUTO_START);
    settings.midiAutoStart = (midi_it != map.end() && midi_it->second == "true");

    return true;
}

static bool save(const BluetoothSettings& settings) {
    auto service_context = findServiceContext();
    if (service_context == nullptr) {
        return false;
    }

    std::string settings_path = service_context->getPaths()->getUserDataPath("settings.properties");

    // Load existing properties first so we don't clobber unrelated keys.
    std::map<std::string, std::string> map;
    file::loadPropertiesFile(settings_path, map); // Ignore failure — file may not exist yet.
    map[SETTINGS_KEY_ENABLE_ON_BOOT]  = settings.enableOnBoot  ? "true" : "false";
    map[SETTINGS_KEY_SPP_AUTO_START]  = settings.sppAutoStart  ? "true" : "false";
    map[SETTINGS_KEY_MIDI_AUTO_START] = settings.midiAutoStart ? "true" : "false";

    return file::savePropertiesFile(settings_path, map);
}

static BluetoothSettings getCachedOrLoad() {
    settings_mutex.lock();
    if (!cached) {
        if (!load(cachedSettings)) {
            // File missing or key absent: use default. Mark cached so we don't
            // retry and spam the log on every call.
            cachedSettings = BluetoothSettings{};
        }
        cached = true;
    }
    auto result = cachedSettings;
    settings_mutex.unlock();
    return result;
}

void setEnableOnBoot(bool enable) {
    settings_mutex.lock();
    cachedSettings.enableOnBoot = enable;
    cached = true;
    settings_mutex.unlock();

    if (!save(cachedSettings)) {
        LOGGER.error("Failed to save");
    }
}

bool shouldEnableOnBoot() {
    return getCachedOrLoad().enableOnBoot;
}

void setSppAutoStart(bool enable) {
    settings_mutex.lock();
    cachedSettings.sppAutoStart = enable;
    cached = true;
    settings_mutex.unlock();
    if (!save(cachedSettings)) {
        LOGGER.error("Failed to save (setSppAutoStart)");
    }
}

bool shouldSppAutoStart() {
    return getCachedOrLoad().sppAutoStart;
}

void setMidiAutoStart(bool enable) {
    settings_mutex.lock();
    cachedSettings.midiAutoStart = enable;
    cached = true;
    settings_mutex.unlock();
    if (!save(cachedSettings)) {
        LOGGER.error("Failed to save (setMidiAutoStart)");
    }
}

bool shouldMidiAutoStart() {
    return getCachedOrLoad().midiAutoStart;
}

} // namespace tt::service::bluetooth::settings
