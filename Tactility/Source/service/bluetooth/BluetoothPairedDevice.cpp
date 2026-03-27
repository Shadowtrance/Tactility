#include <Tactility/service/bluetooth/BluetoothPairedDevice.h>
#include <Tactility/service/bluetooth/BluetoothPrivate.h>

#include <Tactility/file/File.h>
#include <Tactility/file/PropertiesFile.h>
#include <Tactility/Logger.h>
#include <Tactility/service/ServicePaths.h>

#include <dirent.h>
#include <format>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstdio>

namespace tt::service::bluetooth::settings {

static const auto LOGGER = Logger("BluetoothPairedDevice");

constexpr auto* DEVICE_SETTINGS_FORMAT = "{}/{}.device.properties";

constexpr auto* DEVICE_KEY_NAME = "name";
constexpr auto* DEVICE_KEY_ADDR = "addr";
constexpr auto* DEVICE_KEY_AUTO_CONNECT = "autoConnect";
constexpr auto* DEVICE_KEY_PROFILE_ID = "profileId";

std::string addrToHex(const std::array<uint8_t, 6>& addr) {
    std::stringstream stream;
    stream << std::hex;
    for (int i = 0; i < 6; ++i) {
        stream << std::setw(2) << std::setfill('0') << static_cast<int>(addr[i]);
    }
    return stream.str();
}

static bool hexToAddr(const std::string& hex, std::array<uint8_t, 6>& addr) {
    if (hex.size() != 12) {
        LOGGER.error("hexToAddr() length mismatch: expected 12, got {}", hex.size());
        return false;
    }
    char buf[3] = { 0 };
    for (int i = 0; i < 6; ++i) {
        buf[0] = hex[i * 2];
        buf[1] = hex[i * 2 + 1];
        char* endptr = nullptr;
        addr[i] = static_cast<uint8_t>(strtoul(buf, &endptr, 16));
        if (endptr != buf + 2) {
            LOGGER.error("hexToAddr() invalid hex character at byte {}: '{}{}'", i, buf[0], buf[1]);
            return false;
        }
    }
    return true;
}

static std::string getDeviceFilePath(std::shared_ptr<ServicePaths> paths, const std::string& addr_hex) {
    return std::format(DEVICE_SETTINGS_FORMAT, paths->getUserDataDirectory(), addr_hex);
}

bool contains(const std::string& addr_hex) {
    auto service_context = findServiceContext();
    if (service_context == nullptr) {
        return false;
    }
    const auto file_path = getDeviceFilePath(service_context->getPaths(), addr_hex);
    return file::isFile(file_path);
}

bool load(const std::string& addr_hex, PairedDevice& device) {
    auto service_context = findServiceContext();
    if (service_context == nullptr) {
        return false;
    }

    const auto file_path = getDeviceFilePath(service_context->getPaths(), addr_hex);
    std::map<std::string, std::string> map;
    if (!file::loadPropertiesFile(file_path, map)) {
        return false;
    }

    if (!map.contains(DEVICE_KEY_ADDR)) {
        return false;
    }

    if (!hexToAddr(map[DEVICE_KEY_ADDR], device.addr)) {
        return false;
    }

    device.name = map.contains(DEVICE_KEY_NAME) ? map[DEVICE_KEY_NAME] : "";

    if (map.contains(DEVICE_KEY_AUTO_CONNECT)) {
        device.autoConnect = (map[DEVICE_KEY_AUTO_CONNECT] == "true");
    } else {
        device.autoConnect = true;
    }

    if (map.contains(DEVICE_KEY_PROFILE_ID)) {
        device.profileId = std::stoi(map[DEVICE_KEY_PROFILE_ID]);
    }

    return true;
}

bool save(const PairedDevice& device) {
    auto service_context = findServiceContext();
    if (service_context == nullptr) {
        return false;
    }

    const auto addr_hex = addrToHex(device.addr);
    const auto file_path = getDeviceFilePath(service_context->getPaths(), addr_hex);

    std::map<std::string, std::string> map;
    map[DEVICE_KEY_NAME] = device.name;
    map[DEVICE_KEY_ADDR] = addr_hex;
    map[DEVICE_KEY_AUTO_CONNECT] = device.autoConnect ? "true" : "false";
    map[DEVICE_KEY_PROFILE_ID] = std::to_string(device.profileId);

    return file::savePropertiesFile(file_path, map);
}

bool remove(const std::string& addr_hex) {
    auto service_context = findServiceContext();
    if (service_context == nullptr) {
        return false;
    }

    const auto file_path = getDeviceFilePath(service_context->getPaths(), addr_hex);
    if (!file::isFile(file_path)) {
        return false;
    }
    return ::remove(file_path.c_str()) == 0;
}

std::vector<PairedDevice> loadAll() {
    auto service_context = findServiceContext();
    if (service_context == nullptr) {
        return {};
    }

    const auto dir = service_context->getPaths()->getUserDataDirectory();
    std::vector<dirent> entries;
    file::scandir(dir, entries, [](const dirent* entry) -> int {
        if (entry->d_type != file::TT_DT_REG && entry->d_type != file::TT_DT_UNKNOWN) {
            return -1;
        }
        std::string name = entry->d_name;
        return name.ends_with(".device.properties") ? 0 : -1;
    }, nullptr);

    std::vector<PairedDevice> result;
    result.reserve(entries.size());
    for (const auto& entry : entries) {
        std::string filename = entry.d_name;
        // Strip ".device.properties" suffix to get addr_hex
        constexpr std::string_view suffix = ".device.properties";
        if (filename.size() <= suffix.size()) continue;
        const std::string addr_hex = filename.substr(0, filename.size() - suffix.size());
        PairedDevice device;
        if (load(addr_hex, device)) {
            result.push_back(std::move(device));
        }
    }
    return result;
}

} // namespace tt::service::bluetooth::settings
