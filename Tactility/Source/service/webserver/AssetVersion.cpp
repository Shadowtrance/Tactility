#include <Tactility/service/webserver/AssetVersion.h>

#include <Tactility/file/File.h>
#include <Tactility/Log.h>

#include <cJSON.h>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace tt::service::webserver {

constexpr auto* TAG = "AssetVersion";
constexpr auto* DATA_VERSION_FILE = "/data/webserver/version.json";
constexpr auto* SD_VERSION_FILE = "/sdcard/.tactility/webserver/version.json";
constexpr auto* DATA_ASSETS_DIR = "/data/webserver";
constexpr auto* SD_ASSETS_DIR = "/sdcard/.tactility/webserver";

static bool loadVersionFromFile(const char* path, AssetVersion& version) {
    if (!file::isFile(path)) {
        TT_LOG_W(TAG, "Version file not found: %s", path);
        return false;
    }
    
    // Read file content
    std::string content;
    {
        auto lock = file::getLock(path);
        lock->lock(portMAX_DELAY);
        
        FILE* fp = fopen(path, "r");
        if (!fp) {
            TT_LOG_E(TAG, "Failed to open version file: %s", path);
            lock->unlock();
            return false;
        }
        
        char buffer[256];
        size_t bytesRead = fread(buffer, 1, sizeof(buffer) - 1, fp);
        fclose(fp);
        lock->unlock();
        
        if (bytesRead == 0) {
            TT_LOG_E(TAG, "Failed to read version file: %s", path);
            return false;
        }
        buffer[bytesRead] = '\0';
        content = buffer;
    }
    
    // Parse JSON
    cJSON* json = cJSON_Parse(content.c_str());
    if (json == nullptr) {
        TT_LOG_E(TAG, "Failed to parse version JSON: %s", path);
        return false;
    }
    
    cJSON* versionItem = cJSON_GetObjectItem(json, "version");
    if (versionItem == nullptr || !cJSON_IsNumber(versionItem)) {
        TT_LOG_E(TAG, "Invalid version JSON format: %s", path);
        cJSON_Delete(json);
        return false;
    }
    
    version.version = static_cast<uint32_t>(versionItem->valueint);
    cJSON_Delete(json);
    
    TT_LOG_I(TAG, "Loaded version %u from %s", version.version, path);
    return true;
}

static bool saveVersionToFile(const char* path, const AssetVersion& version) {
    // Create directory if it doesn't exist
    std::string dirPath(path);
    size_t lastSlash = dirPath.find_last_of('/');
    if (lastSlash != std::string::npos) {
        dirPath = dirPath.substr(0, lastSlash);
        if (!file::isDirectory(dirPath.c_str())) {
            if (!file::findOrCreateDirectory(dirPath.c_str(), 0755)) {
                TT_LOG_E(TAG, "Failed to create directory: %s", dirPath.c_str());
                return false;
            }
        }
    }
    
    // Create JSON
    cJSON* json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "version", version.version);
    
    char* jsonString = cJSON_Print(json);
    if (jsonString == nullptr) {
        TT_LOG_E(TAG, "Failed to serialize version JSON");
        cJSON_Delete(json);
        return false;
    }
    
    // Write to file
    bool success = false;
    {
        auto lock = file::getLock(path);
        lock->lock(portMAX_DELAY);
        
        FILE* fp = fopen(path, "w");
        if (fp) {
            size_t written = fwrite(jsonString, 1, strlen(jsonString), fp);
            success = (written == strlen(jsonString));
            fclose(fp);
        }
        lock->unlock();
    }
    
    cJSON_free(jsonString);
    cJSON_Delete(json);
    
    if (success) {
        TT_LOG_I(TAG, "Saved version %u to %s", version.version, path);
    } else {
        TT_LOG_E(TAG, "Failed to write version file: %s", path);
    }
    
    return success;
}

bool loadDataVersion(AssetVersion& version) {
    return loadVersionFromFile(DATA_VERSION_FILE, version);
}

bool loadSdVersion(AssetVersion& version) {
    return loadVersionFromFile(SD_VERSION_FILE, version);
}

bool saveDataVersion(const AssetVersion& version) {
    return saveVersionToFile(DATA_VERSION_FILE, version);
}

bool saveSdVersion(const AssetVersion& version) {
    return saveVersionToFile(SD_VERSION_FILE, version);
}

bool hasDataAssets() {
    return file::isDirectory(DATA_ASSETS_DIR);
}

bool hasSdAssets() {
    return file::isDirectory(SD_ASSETS_DIR);
}

static bool copyDirectory(const char* src, const char* dst) {
    TT_LOG_I(TAG, "Copying directory: %s -> %s", src, dst);
    
    // Create destination directory
    if (!file::isDirectory(dst)) {
        if (!file::findOrCreateDirectory(dst, 0755)) {
            TT_LOG_E(TAG, "Failed to create destination directory: %s", dst);
            return false;
        }
    }
    
    // List source directory and copy each entry
    bool copySuccess = true;
    bool listSuccess = file::listDirectory(src, [&](const dirent& entry) {
        // Skip "." and ".." entries (though listDirectory should already filter these)
        if (strcmp(entry.d_name, ".") == 0 || strcmp(entry.d_name, "..") == 0) {
            return;
        }
        
        std::string srcPath = std::string(src) + "/" + entry.d_name;
        std::string dstPath = std::string(dst) + "/" + entry.d_name;
        
        if (entry.d_type == file::TT_DT_DIR) {
            // Recursively copy subdirectory
            if (!copyDirectory(srcPath.c_str(), dstPath.c_str())) {
                copySuccess = false;
            }
        } else if (entry.d_type == file::TT_DT_REG) {
            // Copy file using standard C FILE* operations
            auto lock = file::getLock(srcPath);
            lock->lock(portMAX_DELAY);
            
            FILE* srcFile = fopen(srcPath.c_str(), "rb");
            if (!srcFile) {
                TT_LOG_E(TAG, "Failed to open source file: %s", srcPath.c_str());
                lock->unlock();
                copySuccess = false;
                return;
            }
            
            FILE* dstFile = fopen(dstPath.c_str(), "wb");
            if (!dstFile) {
                TT_LOG_E(TAG, "Failed to create destination file: %s", dstPath.c_str());
                fclose(srcFile);
                lock->unlock();
                copySuccess = false;
                return;
            }
            
            // Copy in chunks
            char buffer[512];
            size_t bytesRead;
            while ((bytesRead = fread(buffer, 1, sizeof(buffer), srcFile)) > 0) {
                size_t bytesWritten = fwrite(buffer, 1, bytesRead, dstFile);
                if (bytesWritten != bytesRead) {
                    TT_LOG_E(TAG, "Failed to write to destination file: %s", dstPath.c_str());
                    copySuccess = false;
                    break;
                }
            }
            
            fclose(srcFile);
            fclose(dstFile);
            lock->unlock();
            
            if (copySuccess) {
                TT_LOG_I(TAG, "Copied file: %s", entry.d_name);
            }
        }
    });
    
    if (!listSuccess) {
        TT_LOG_E(TAG, "Failed to list source directory: %s", src);
        return false;
    }
    
    return copySuccess;
}

bool syncAssets() {
    TT_LOG_I(TAG, "Starting asset synchronization...");

    // Check if Data partition and SD card exist
    bool dataExists = hasDataAssets();
    bool sdExists = hasSdAssets();

    // FIRST BOOT SCENARIO: Data has version 0, SD card is missing
    if (dataExists && !sdExists) {
        TT_LOG_I(TAG, "First boot - Data exists but SD card backup missing");
        TT_LOG_W(TAG, "Skipping SD backup during boot - will be created on first settings save");
        TT_LOG_W(TAG, "This avoids watchdog timeout if SD card is slow or corrupted");
        return true;  // Don't block boot - defer copy to runtime
    }

    // NO SD CARD: Just ensure Data has default structure
    if (!sdExists) {
        TT_LOG_W(TAG, "No SD card available - creating default Data structure if needed");
        if (!dataExists) {
            if (!file::findOrCreateDirectory(DATA_ASSETS_DIR, 0755)) {
                TT_LOG_E(TAG, "Failed to create Data assets directory");
                return false;
            }
            AssetVersion defaultVersion(0);  // Start at version 0 - SD card updates will be version 1+
            if (!saveDataVersion(defaultVersion)) {
                TT_LOG_E(TAG, "Failed to save default Data version");
                return false;
            }
            TT_LOG_I(TAG, "Created default Data assets structure (version 0)");
        }
        return true;
    }

    // POST-FLASH RECOVERY: Data empty but SD card exists
    if (!dataExists) {
        TT_LOG_I(TAG, "Data partition empty - copying from SD card (recovery mode)");
        if (!copyDirectory(SD_ASSETS_DIR, DATA_ASSETS_DIR)) {
            TT_LOG_E(TAG, "Failed to copy assets from SD card to Data");
            return false;
        }
        TT_LOG_I(TAG, "Recovery complete - assets restored from SD card");
        return true;
    }

    // NORMAL OPERATION: Both exist - compare versions
    AssetVersion dataVersion, sdVersion;
    bool hasDataVer = loadDataVersion(dataVersion);
    bool hasSdVer = loadSdVersion(sdVersion);

    if (!hasDataVer) {
        TT_LOG_W(TAG, "No Data version.json - assuming version 0");
        dataVersion.version = 0;
        saveDataVersion(dataVersion);  // Save version 0 to Data
    }

    if (!hasSdVer) {
        TT_LOG_W(TAG, "No SD version.json - assuming version 0");
        sdVersion.version = 0;
        // DON'T save to SD during boot - defer to runtime
        TT_LOG_W(TAG, "Skipping SD version.json creation during boot - will be created on first settings save");
    }

    TT_LOG_I(TAG, "Version comparison - Data: %u, SD: %u", dataVersion.version, sdVersion.version);

    if (sdVersion.version > dataVersion.version) {
        // Firmware update - copy SD -> Data
        TT_LOG_I(TAG, "SD card newer (v%u > v%u) - copying assets SD -> Data (firmware update)",
                 sdVersion.version, dataVersion.version);
        if (!copyDirectory(SD_ASSETS_DIR, DATA_ASSETS_DIR)) {
            TT_LOG_E(TAG, "Failed to copy assets from SD to Data");
            return false;
        }
        TT_LOG_I(TAG, "Firmware update complete - assets updated from SD card");
    } else if (dataVersion.version > sdVersion.version) {
        // User customization - backup Data -> SD
        TT_LOG_W(TAG, "Data newer (v%u > v%u) - deferring SD backup to avoid boot watchdog",
                 dataVersion.version, sdVersion.version);
        TT_LOG_W(TAG, "SD backup will occur on first WebServer settings save");
        return true;  // Don't block boot - defer copy to runtime
    } else {
        TT_LOG_I(TAG, "Versions match (v%u) - no sync needed", dataVersion.version);
    }

    return true;
}

} // namespace
