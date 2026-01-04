#pragma once
#ifdef ESP_PLATFORM

#include <Tactility/service/Service.h>
#include <Tactility/network/HttpServer.h>
#include <Tactility/RecursiveMutex.h>
#include <Tactility/kernel/SystemEvents.h>

#include <esp_http_server.h>
#include <string>

namespace tt::service::webserver {

/**
 * @brief Web server service with resilient asset architecture
 *
 * Provides:
 * - Core HTML endpoints (hardcoded in firmware)
 * - Dynamic asset serving from Data partition
 * - SD card fallback
 * - Asset synchronization
 */
class WebServerService final : public Service {
private:
    mutable RecursiveMutex mutex;
    std::unique_ptr<network::HttpServer> httpServer;
    std::string deviceResponse;
    kernel::SystemEventSubscription settingsEventSubscription = kernel::NoSystemEventSubscription;
    int8_t statusbarIconId = -1;  // Statusbar icon for WebServer state
    
    // Core HTML endpoints (hardcoded in firmware)
    static esp_err_t handleRoot(httpd_req_t* request);
    static esp_err_t handleUpload(httpd_req_t* request);
    static esp_err_t handleSync(httpd_req_t* request);
    static esp_err_t handleReboot(httpd_req_t* request);
    
    // File browser endpoints
    static esp_err_t handleFileBrowser(httpd_req_t* request);
    static esp_err_t handleFsList(httpd_req_t* request);
    static esp_err_t handleFsTree(httpd_req_t* request);
    static esp_err_t handleFsDownload(httpd_req_t* request);
    static esp_err_t handleFsMkdir(httpd_req_t* request);
    static esp_err_t handleFsDelete(httpd_req_t* request);
    static esp_err_t handleFsRename(httpd_req_t* request);
    static esp_err_t handleFsUpload(httpd_req_t* request);
    // Consolidated dispatch handlers to reduce URI handler table usage
    static esp_err_t handleFsGenericGet(httpd_req_t* request);
    static esp_err_t handleFsGenericPost(httpd_req_t* request);
    // Admin dispatcher to consolidate small POST endpoints (sync/reboot)
    static esp_err_t handleAdminPost(httpd_req_t* request);
    
    // Dynamic asset serving
    static esp_err_t handleAssets(httpd_req_t* request);
    
    bool startServer();
    void stopServer();
    
public:
    bool onStart(ServiceContext& service) override;
    void onStop(ServiceContext& service) override;
    
    void setEnabled(bool enabled);
    bool isEnabled() const;
};

// Global accessor for controlling the WebServer service
void setWebServerEnabled(bool enabled);

} // namespace

#endif
