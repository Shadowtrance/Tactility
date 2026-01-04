#include <Tactility/TactilityCore.h>
#include <Tactility/service/ServiceRegistration.h>
#include <Tactility/service/ServiceManifest.h>
#include <Tactility/service/ServiceContext.h>
#include <Tactility/settings/WebServerSettings.h>
#include <Tactility/service/wifi/Wifi.h>
#include <Tactility/service/wifi/WifiApSettings.h>
#include <Tactility/service/webserver/WebServerService.h>
#include <Tactility/Log.h>

#ifdef ESP_PLATFORM
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/ip4_addr.h>
#endif

namespace tt::service::webserver {

constexpr auto* TAG = "WebServerInit";

/**
 * @brief Service that initializes WiFi on boot
 * 
 * This service connects WiFi based on saved credentials.
 * The actual WebServer is handled by WebServerService.
 */
class WebServerInitService final : public Service {
private:
    esp_netif_t* ap_netif = nullptr;

public:
    bool onStart(TT_UNUSED ServiceContext& service) override {
        TT_LOG_I(TAG, "Loading web server settings...");
        
        auto settings = settings::webserver::loadOrGetDefault();
        
        TT_LOG_I(TAG, "WiFi Mode: %s",
            settings.wifiMode == settings::webserver::WiFiMode::Station ? "Station" : "AccessPoint");

        if (settings.wifiMode == settings::webserver::WiFiMode::AccessPoint) {
            TT_LOG_I(TAG, "AP SSID: %s", settings.apSsid.c_str());
        }
        
        TT_LOG_I(TAG, "WiFi Enabled: %s", settings.wifiEnabled ? "true" : "false");
        TT_LOG_I(TAG, "Web Server Enabled: %s", settings.webServerEnabled ? "true" : "false");
        TT_LOG_I(TAG, "Web Server Port: %d", settings.webServerPort);
        TT_LOG_I(TAG, "Web Server Auth: %s", settings.webServerAuthEnabled ? "enabled" : "disabled");

        // Only start WiFi if enabled in settings
        if (!settings.wifiEnabled) {
            TT_LOG_I(TAG, "WiFi disabled in settings - skipping WiFi initialization");
            return true;
        }

        // Connect WiFi based on mode
        // WebServer is handled by WebServerService which starts automatically if webServerEnabled=true
        if (settings.wifiMode == settings::webserver::WiFiMode::Station) {
            // Station mode - WiFi credentials are managed via WiFi menu (/data/settings/*.ap.properties)
            // Auto-connect happens via WiFi service, not here
            TT_LOG_I(TAG, "WiFi Station mode - auto-connect will use saved network credentials");
            TT_LOG_I(TAG, "Use WiFi menu to connect to networks (credentials saved to /data/settings/*.ap.properties)");
        } else {
            // Access Point mode
            TT_LOG_I(TAG, "Starting WiFi in Access Point mode...");
            
#ifdef ESP_PLATFORM
            // Initialize WiFi in AP mode
            wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
            if (esp_wifi_init(&cfg) != ESP_OK) {
                TT_LOG_E(TAG, "esp_wifi_init() failed");
                return false;
            }

            ap_netif = esp_netif_create_default_wifi_ap();
            if (ap_netif == nullptr) {
                TT_LOG_E(TAG, "esp_netif_create_default_wifi_ap() failed");
                return false;
            }

            // Configure static IP for AP: 192.168.4.1/24
            if (esp_netif_dhcps_stop(ap_netif) != ESP_OK) {
                TT_LOG_E(TAG, "esp_netif_dhcps_stop() failed");
                return false;
            }

            esp_netif_ip_info_t ip_info;
            IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
            IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
            IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

            if (esp_netif_set_ip_info(ap_netif, &ip_info) != ESP_OK) {
                TT_LOG_E(TAG, "esp_netif_set_ip_info() failed");
                return false;
            }

            if (esp_netif_dhcps_start(ap_netif) != ESP_OK) {
                TT_LOG_E(TAG, "esp_netif_dhcps_start() failed");
                return false;
            }

            // Configure WiFi AP settings
            wifi_config_t wifi_config = {};
            strncpy(reinterpret_cast<char*>(wifi_config.ap.ssid), settings.apSsid.c_str(), sizeof(wifi_config.ap.ssid) - 1);
            wifi_config.ap.ssid[sizeof(wifi_config.ap.ssid) - 1] = '\0';
            if (settings.apPassword.length() >= 8 && settings.apPassword.length() <= 63) {
                wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
                strncpy(reinterpret_cast<char*>(wifi_config.ap.password), settings.apPassword.c_str(), sizeof(wifi_config.ap.password) - 1);
                wifi_config.ap.password[sizeof(wifi_config.ap.password) - 1] = '\0';
            } else {
                if (!settings.apPassword.empty()) {
                    TT_LOG_W(TAG, "AP password invalid (must be 8-63 chars) - using OPEN mode");
                }
                wifi_config.ap.authmode = WIFI_AUTH_OPEN;
            }
            wifi_config.ap.max_connection = 4;
            wifi_config.ap.channel = settings.apChannel;

            if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK) {
                TT_LOG_E(TAG, "esp_wifi_set_mode() failed");
                return false;
            }

            if (esp_wifi_set_config(WIFI_IF_AP, &wifi_config) != ESP_OK) {
                TT_LOG_E(TAG, "esp_wifi_set_config() failed");
                return false;
            }

            if (esp_wifi_start() != ESP_OK) {
                TT_LOG_E(TAG, "esp_wifi_start() failed");
                return false;
            }

            TT_LOG_I(TAG, "WiFi AP started - SSID: %s, IP: 192.168.4.1", settings.apSsid.c_str());
#else
            TT_LOG_W(TAG, "WiFi AP mode not available on simulator");
#endif
        }
        
        return true;
    }

    void onStop(TT_UNUSED ServiceContext& service) override {
        if (ap_netif != nullptr) {
            esp_netif_destroy(ap_netif);
            ap_netif = nullptr;
        }
        esp_wifi_deinit();
    }
};

extern const ServiceManifest webServerInitManifest = {
    .id = "WebServerInit",
    .createService = create<WebServerInitService>
};

}
