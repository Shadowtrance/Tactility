#include <Tactility/settings/QuickPanelSettings.h>

#include <Tactility/file/File.h>
#include <Tactility/file/PropertiesFile.h>
#include <Tactility/Paths.h>

#include <map>
#include <sstream>
#include <string>

namespace tt::settings::quickpanel {

static std::string getSettingsFilePath() {
    return getUserDataPath() + "/settings/quickpanel.properties";
}

constexpr auto* SETTINGS_KEY_TILE_ORDER = "tileOrder";
constexpr auto* SETTINGS_KEY_HIDDEN_TILE_IDS = "hiddenTileIds";
constexpr auto DELIMITER = ',';

static std::string toString(const std::vector<std::string>& ids) {
    std::string result;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i > 0) {
            result += DELIMITER;
        }
        result += ids[i];
    }
    return result;
}

static std::vector<std::string> fromString(const std::string& value) {
    std::vector<std::string> ids;
    std::stringstream stream(value);
    std::string token;
    while (std::getline(stream, token, DELIMITER)) {
        if (!token.empty()) {
            ids.push_back(token);
        }
    }
    return ids;
}

bool load(QuickPanelSettings& settings) {
    auto settings_path = getSettingsFilePath();
    if (!file::isFile(settings_path)) {
        return false;
    }

    std::map<std::string, std::string> map;
    if (!file::loadPropertiesFile(settings_path, map)) {
        return false;
    }

    auto orderEntry = map.find(SETTINGS_KEY_TILE_ORDER);
    settings.tileOrder = (orderEntry != map.end()) ? fromString(orderEntry->second) : std::vector<std::string>();

    auto hiddenEntry = map.find(SETTINGS_KEY_HIDDEN_TILE_IDS);
    settings.hiddenTileIds = (hiddenEntry != map.end()) ? fromString(hiddenEntry->second) : std::vector<std::string>();

    return true;
}

QuickPanelSettings getDefault() {
    return QuickPanelSettings {
        .tileOrder = {},
        .hiddenTileIds = {}
    };
}

QuickPanelSettings loadOrGetDefault() {
    QuickPanelSettings settings;
    if (!load(settings)) {
        settings = getDefault();
    }
    return settings;
}

bool save(const QuickPanelSettings& settings) {
    std::map<std::string, std::string> map;
    map[SETTINGS_KEY_TILE_ORDER] = toString(settings.tileOrder);
    map[SETTINGS_KEY_HIDDEN_TILE_IDS] = toString(settings.hiddenTileIds);
    auto settings_path = getSettingsFilePath();
    if (!file::findOrCreateParentDirectory(settings_path, 0755)) {
        return false;
    }
    return file::savePropertiesFile(settings_path, map);
}

} // namespace tt::settings::quickpanel
