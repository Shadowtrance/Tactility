#include <Tactility/app/AppManifestParsing.h>
#include <Tactility/app/AppManifestParsingInternal.h>

#include <tactility/log.h>

namespace tt::app {

constexpr auto* TAG = "AppManifestV2";

bool parseManifestV2(const std::map<std::string, std::string>& map, AppManifest& manifest) {
    // manifest

    std::string manifest_version;
    if (!getValueFromManifest(map, "manifest.version", manifest_version)) {
        return false;
    }

    if (!isValidManifestVersion(manifest_version)) {
        LOG_E(TAG, "Invalid version");
        return false;
    }

    // app

    if (!getValueFromManifest(map, "app.id", manifest.appId)) {
        return false;
    }

    if (!isValidId(manifest.appId)) {
        LOG_E(TAG, "Invalid app id");
        return false;
    }

    if (!getValueFromManifest(map, "app.name", manifest.appName)) {
        return false;
    }

    if (!isValidName(manifest.appName)) {
        LOG_E(TAG, "Invalid app name");
        return false;
    }

    if (!getValueFromManifest(map, "app.version.name", manifest.appVersionName)) {
        return false;
    }

    if (!isValidAppVersionName(manifest.appVersionName)) {
        LOG_E(TAG, "Invalid app version name");
        return false;
    }

    std::string version_code_string;
    if (!getValueFromManifest(map, "app.version.code", version_code_string)) {
        return false;
    }

    if (!isValidAppVersionCode(version_code_string)) {
        LOG_E(TAG, "Invalid app version code");
        return false;
    }

    manifest.appVersionCode = std::stoull(version_code_string);

    // Optional: absent means ELF, so manifests written before this key existed keep working.
    // Looked up directly rather than via getValueFromManifest(), which logs an error when a
    // key is missing - correct for required keys, but every ELF manifest would trip it.
    //
    // An unrecognised value fails rather than falling back: a typo would otherwise start a
    // Lua app as an ELF one and report a missing binary, far from the cause.
    if (const auto runtime = map.find("app.runtime"); runtime != map.end()) {
        if (runtime->second == "elf") {
            manifest.appRuntime = Runtime::Elf;
        } else if (runtime->second == "lua") {
            manifest.appRuntime = Runtime::Lua;
        } else {
            LOG_E(TAG, "Invalid app runtime '%s' - expected 'elf' or 'lua'", runtime->second.c_str());
            return false;
        }
    }

    // target

    if (!getValueFromManifest(map, "target.sdk", manifest.targetSdk)) {
        return false;
    }

    if (!getValueFromManifest(map, "target.platforms", manifest.targetPlatforms)) {
        return false;
    }

    return true;
}

}
