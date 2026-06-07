#include <Tactility/settings/AudioSettings.h>

#include <Tactility/file/PropertiesFile.h>

#include <cstdlib>
#include <map>
#include <string>

namespace tt::settings::audio {

constexpr auto* SETTINGS_FILE = "/data/settings/audio.properties";
constexpr auto* SETTINGS_KEY_INPUT_ENABLED = "inputEnabled";
constexpr auto* SETTINGS_KEY_OUTPUT_ENABLED = "outputEnabled";
constexpr auto* SETTINGS_KEY_INPUT_MUTED = "inputMuted";
constexpr auto* SETTINGS_KEY_OUTPUT_MUTED = "outputMuted";
constexpr auto* SETTINGS_KEY_INPUT_VOLUME = "inputVolume";
constexpr auto* SETTINGS_KEY_OUTPUT_VOLUME = "outputVolume";

static bool toBool(const std::map<std::string, std::string>& map, const char* key, bool defaultValue) {
    auto entry = map.find(key);
    if (entry == map.end()) {
        return defaultValue;
    }
    return (entry->second == "1" || entry->second == "true" || entry->second == "True");
}

static float toFloat(const std::map<std::string, std::string>& map, const char* key, float defaultValue) {
    auto entry = map.find(key);
    if (entry == map.end()) {
        return defaultValue;
    }
    return std::strtof(entry->second.c_str(), nullptr);
}

static std::string toString(bool value) {
    return value ? "1" : "0";
}

static std::string toString(float value) {
    return std::to_string(value);
}

bool load(AudioSettings& settings) {
    std::map<std::string, std::string> map;
    if (!file::loadPropertiesFile(SETTINGS_FILE, map)) {
        return false;
    }

    settings.inputEnabled = toBool(map, SETTINGS_KEY_INPUT_ENABLED, true);
    settings.outputEnabled = toBool(map, SETTINGS_KEY_OUTPUT_ENABLED, true);
    settings.inputMuted = toBool(map, SETTINGS_KEY_INPUT_MUTED, false);
    settings.outputMuted = toBool(map, SETTINGS_KEY_OUTPUT_MUTED, false);
    settings.inputVolume = toFloat(map, SETTINGS_KEY_INPUT_VOLUME, 20.0f);
    settings.outputVolume = toFloat(map, SETTINGS_KEY_OUTPUT_VOLUME, 20.0f);

    return true;
}

AudioSettings getDefault() {
    return AudioSettings {
        .inputEnabled = true,
        .outputEnabled = true,
        .inputMuted = false,
        .outputMuted = false,
        .inputVolume = 20.0f,
        .outputVolume = 20.0f
    };
}

AudioSettings loadOrGetDefault() {
    AudioSettings settings;
    if (!load(settings)) {
        settings = getDefault();
    }
    return settings;
}

bool save(const AudioSettings& settings) {
    std::map<std::string, std::string> map;
    map[SETTINGS_KEY_INPUT_ENABLED] = toString(settings.inputEnabled);
    map[SETTINGS_KEY_OUTPUT_ENABLED] = toString(settings.outputEnabled);
    map[SETTINGS_KEY_INPUT_MUTED] = toString(settings.inputMuted);
    map[SETTINGS_KEY_OUTPUT_MUTED] = toString(settings.outputMuted);
    map[SETTINGS_KEY_INPUT_VOLUME] = toString(settings.inputVolume);
    map[SETTINGS_KEY_OUTPUT_VOLUME] = toString(settings.outputVolume);
    return file::savePropertiesFile(SETTINGS_FILE, map);
}

} // namespace tt::settings::audio
