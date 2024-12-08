#ifdef ESP_TARGET

#include "Wifi.h"

#include "TactilityHeadless.h"
#include "Timer.h"
#include "service/ServiceContext.h"
#include "WifiSettings.h"

#include "freertos/FreeRTOS.h"

#include <atomic>
#include <cstring>
#include <sys/cdefs.h>

namespace tt::service::wifi {

#define TAG "wifi_service"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define AUTO_SCAN_INTERVAL 10000 // ms

// Forward declarations
class Wifi;
static void scan_list_free_safely(std::shared_ptr<Wifi> wifi);
// Methods for main thread dispatcher
static void dispatchAutoConnect(std::shared_ptr<void> context);
static void dispatchEnable(std::shared_ptr<void> context);
static void dispatchDisable(std::shared_ptr<void> context);
static void dispatchScan(std::shared_ptr<void> context);
static void dispatchConnect(std::shared_ptr<void> context);
static void dispatchDisconnectButKeepActive(std::shared_ptr<void> context);

class Wifi {

private:

    std::atomic<WifiRadioState> radio_state = WIFI_RADIO_OFF;
    bool scan_active = false;
    bool secure_connection = false;

public:

    /** @brief Locking mechanism for modifying the Wifi instance */
    Mutex radioMutex = Mutex(Mutex::TypeRecursive);
    Mutex dataMutex = Mutex(Mutex::TypeRecursive);
    std::unique_ptr<Timer> autoConnectTimer;
    /** @brief The public event bus */
    std::shared_ptr<PubSub> pubsub = std::make_shared<PubSub>();
    // TODO: Deal with messages that come in while an action is ongoing
    // for example: when scanning and you turn off the radio, the scan should probably stop or turning off
    // the radio should disable the on/off button in the app as it is pending.
    /** @brief The network interface when wifi is started */
    esp_netif_t* _Nullable netif = nullptr;
    /** @brief Scanning results */
    wifi_ap_record_t* _Nullable scan_list = nullptr;
    /** @brief The current item count in scan_list (-1 when scan_list is NULL) */
    uint16_t scan_list_count = 0;
    /** @brief Maximum amount of records to scan (value > 0) */
    uint16_t scan_list_limit = TT_WIFI_SCAN_RECORD_LIMIT;
    /** @brief when we last requested a scan. Loops around every 50 days. */
    TickType_t last_scan_time = portMAX_DELAY;
    esp_event_handler_instance_t event_handler_any_id = nullptr;
    esp_event_handler_instance_t event_handler_got_ip = nullptr;
    EventFlag connection_wait_flags;
    settings::WifiApSettings connection_target = {
        .ssid = { 0 },
        .password = { 0 },
        .auto_connect = false
    };
    bool pause_auto_connect = false; // Pause when manually disconnecting until manually connecting again
    bool connection_target_remember = false; // Whether to store the connection_target on successful connection or not

    WifiRadioState getRadioState() const {
        auto lock = dataMutex.scoped();
        lock->acquire(TtWaitForever);
        return radio_state;
    }

    void setRadioState(WifiRadioState newState) {
        auto lock = dataMutex.scoped();
        lock->acquire(TtWaitForever);
        radio_state = newState;
    }

    bool isScanning() const {
        auto lock = dataMutex.scoped();
        lock->acquire(TtWaitForever);
        return radio_state;
    }

    void setScanning(bool newState) {
        auto lock = dataMutex.scoped();
        lock->acquire(TtWaitForever);
        scan_active = newState;
    }

    bool isScanActive() const {
        auto lock = dataMutex.scoped();
        lock->acquire(TtWaitForever);
        return scan_active;
    }

    void setScanActive(bool newState) {
        auto lock = dataMutex.scoped();
        lock->acquire(TtWaitForever);
        scan_active = newState;
    }

    bool isSecureConnection() const {
        auto lock = dataMutex.scoped();
        lock->acquire(TtWaitForever);
        return secure_connection;
    }

    void setSecureConnection(bool newState) {
        auto lock = dataMutex.scoped();
        lock->acquire(TtWaitForever);
        secure_connection = newState;
    }
};

static std::shared_ptr<Wifi> wifi_singleton;


// region Public functions

std::shared_ptr<PubSub> getPubsub() {
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        tt_crash("Service not running");
    }

    return wifi->pubsub;
}

WifiRadioState getRadioState() {
    auto wifi = wifi_singleton;
    if (wifi != nullptr) {
        return wifi->getRadioState();
    } else {
        return WIFI_RADIO_OFF;
    }
}

std::string getConnectionTarget() {
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        return "";
    }

    WifiRadioState state = wifi->getRadioState();
    if (
        state != WIFI_RADIO_CONNECTION_PENDING &&
        state != WIFI_RADIO_CONNECTION_ACTIVE
    ) {
        return "";
    }

    return wifi->connection_target.ssid;
}

void scan() {
    TT_LOG_I(TAG, "scan()");
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        return;
    }

    getMainDispatcher().dispatch(dispatchScan, wifi);
}

bool isScanning() {
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        return false;
    } else {
        return wifi->isScanActive();
    }
}

void connect(const settings::WifiApSettings* ap, bool remember) {
    TT_LOG_I(TAG, "connect(%s, %d)", ap->ssid, remember);
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        return;
    }

    auto lock = wifi->dataMutex.scoped();
    if (!lock->acquire(10 / portTICK_PERIOD_MS)) {
        return;
    }

    // Manual connect (e.g. via app) should stop auto-connecting until the connection is established
    wifi->pause_auto_connect = true;
    memcpy(&wifi->connection_target, ap, sizeof(settings::WifiApSettings));
    wifi->connection_target_remember = remember;
    getMainDispatcher().dispatch(dispatchConnect, wifi);
}

void disconnect() {
    TT_LOG_I(TAG, "disconnect()");
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        return;
    }

    auto lock = wifi->dataMutex.scoped();
    if (!lock->acquire(10 / portTICK_PERIOD_MS)) {
        return;
    }

    wifi->connection_target = (settings::WifiApSettings) {
        .ssid = { 0 },
        .password = { 0 },
        .auto_connect = false
    };
    // Manual disconnect (e.g. via app) should stop auto-connecting until a new connection is established
    wifi->pause_auto_connect = true;
    getMainDispatcher().dispatch(dispatchDisconnectButKeepActive, wifi);
}

void setScanRecords(uint16_t records) {
    TT_LOG_I(TAG, "setScanRecords(%d)", records);
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        return;
    }

    auto lock = wifi->dataMutex.scoped();
    if (!lock->acquire(10 / portTICK_PERIOD_MS)) {
        return;
    }

    if (records != wifi->scan_list_limit) {
        scan_list_free_safely(wifi);
        wifi->scan_list_limit = records;
    }
}

std::vector<WifiApRecord> getScanResults() {
    TT_LOG_I(TAG, "getScanResults()");
    auto wifi = wifi_singleton;

    std::vector<WifiApRecord> records;

    if (wifi == nullptr) {
        return records;
    }

    auto lock = wifi->dataMutex.scoped();
    if (!lock->acquire(10 / portTICK_PERIOD_MS)) {
        return records;
    }

    if (wifi->scan_list_count > 0) {
        uint16_t i = 0;
        for (; i < wifi->scan_list_count; ++i) {
            records.push_back((WifiApRecord) {
                .ssid = (const char*)wifi->scan_list[i].ssid,
                .rssi = wifi->scan_list[i].rssi,
                .auth_mode = wifi->scan_list[i].authmode
            });
        }
    }

    return records;
}

void setEnabled(bool enabled) {
    TT_LOG_I(TAG, "setEnabled(%d)", enabled);
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        return;
    }

    auto lock = wifi->dataMutex.scoped();
    if (!lock->acquire(10 / portTICK_PERIOD_MS)) {
        return;
    }

    if (enabled) {
        getMainDispatcher().dispatch(dispatchEnable, wifi);
    } else {
        getMainDispatcher().dispatch(dispatchDisable, wifi);
    }
    wifi->pause_auto_connect = false;
    wifi->last_scan_time = 0;
}

bool isConnectionSecure() {
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        return false;
    }

    auto lock = wifi->dataMutex.scoped();
    if (!lock->acquire(10 / portTICK_PERIOD_MS)) {
        return false;
    }

    return wifi->isSecureConnection();
}

int getRssi() {
    tt_assert(wifi_singleton);
    static int rssi = 0;
    if (esp_wifi_sta_get_rssi(&rssi) == ESP_OK) {
        return rssi;
    } else {
        return 1;
    }
}

// endregion Public functions

static void scan_list_alloc(std::shared_ptr<Wifi> wifi) {
    auto lock = wifi->dataMutex.scoped();
    if (lock->acquire(TtWaitForever)) {
        tt_assert(wifi->scan_list == nullptr);
        wifi->scan_list = static_cast<wifi_ap_record_t*>(malloc(sizeof(wifi_ap_record_t) * wifi->scan_list_limit));
        wifi->scan_list_count = 0;
    }
}

static void scan_list_alloc_safely(std::shared_ptr<Wifi> wifi) {
    auto lock = wifi->dataMutex.scoped();
    if (lock->acquire(TtWaitForever)) {
        if (wifi->scan_list == nullptr) {
            scan_list_alloc(wifi);
        }
    }
}

static void scan_list_free(std::shared_ptr<Wifi> wifi) {
    auto lock = wifi->dataMutex.scoped();
    if (lock->acquire(TtWaitForever)) {
        tt_assert(wifi->scan_list != nullptr);
        free(wifi->scan_list);
        wifi->scan_list = nullptr;
        wifi->scan_list_count = 0;
    }
}

static void scan_list_free_safely(std::shared_ptr<Wifi> wifi) {
    auto lock = wifi->dataMutex.scoped();
    if (lock->acquire(TtWaitForever)) {
        if (wifi->scan_list != nullptr) {
            scan_list_free(wifi);
        }
    }
}

static void publish_event_simple(std::shared_ptr<Wifi> wifi, WifiEventType type) {
    auto lock = wifi->dataMutex.scoped();
    if (lock->acquire(TtWaitForever)) {
        WifiEvent turning_on_event = {.type = type};
        tt_pubsub_publish(wifi->pubsub, &turning_on_event);
    }
}

static bool copy_scan_list(std::shared_ptr<Wifi> wifi) {
    auto state = wifi->getRadioState();
    bool can_fetch_results = (state == WIFI_RADIO_ON || state == WIFI_RADIO_CONNECTION_ACTIVE) &&
        wifi->isScanActive();

    if (!can_fetch_results) {
        TT_LOG_I(TAG, "Skip scan result fetching");
        return false;
    }

    auto lock = wifi->dataMutex.scoped();
    if (!lock->acquire(TtWaitForever)) {
        return false;
    }

    // Create scan list if it does not exist
    scan_list_alloc_safely(wifi);
    wifi->scan_list_count = 0;
    uint16_t record_count = wifi->scan_list_limit;
    esp_err_t scan_result = esp_wifi_scan_get_ap_records(&record_count, wifi->scan_list);
    if (scan_result == ESP_OK) {
        uint16_t safe_record_count = TT_MIN(wifi->scan_list_limit, record_count);
        wifi->scan_list_count = safe_record_count;
        TT_LOG_I(TAG, "Scanned %u APs. Showing %u:", record_count, safe_record_count);
        for (uint16_t i = 0; i < safe_record_count; i++) {
            wifi_ap_record_t* record = &wifi->scan_list[i];
            TT_LOG_I(TAG, " - SSID %s (RSSI %d, channel %d)", record->ssid, record->rssi, record->primary);
        }
        return true;
    } else {
        TT_LOG_I(TAG, "Failed to get scanned records: %s", esp_err_to_name(scan_result));
        return false;
    }
}

static bool find_auto_connect_ap(std::shared_ptr<void> context, settings::WifiApSettings& settings) {
    auto wifi = std::static_pointer_cast<Wifi>(context);
    auto lock = wifi->dataMutex.scoped();

    if (lock->acquire(10 / portTICK_PERIOD_MS)) {
        TT_LOG_I(TAG, "auto_connect()");
        for (int i = 0; i < wifi->scan_list_count; ++i) {
            auto ssid = reinterpret_cast<const char*>(wifi->scan_list[i].ssid);
            if (settings::contains(ssid)) {
                static_assert(sizeof(wifi->scan_list[i].ssid) == (TT_WIFI_SSID_LIMIT + 1), "SSID size mismatch");
                if (settings::load(ssid, &settings)) {
                    if (settings.auto_connect) {
                        return true;
                    }
                } else {
                    TT_LOG_E(TAG, "Failed to load credentials for ssid %s", ssid);
                }
                break;
            }
        }
    }

    return false;
}

static void dispatchAutoConnect(std::shared_ptr<void> context) {
    TT_LOG_I(TAG, "dispatchAutoConnect()");
    auto wifi = std::static_pointer_cast<Wifi>(context);

    settings::WifiApSettings settings;
    if (find_auto_connect_ap(context, settings)) {
        TT_LOG_I(TAG, "Auto-connecting to %s", settings.ssid);
        connect(&settings, false);
    }
}

static void eventHandler(TT_UNUSED void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    auto wifi = wifi_singleton;
    if (wifi == nullptr) {
        TT_LOG_E(TAG, "eventHandler: no wifi instance");
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        TT_LOG_I(TAG, "eventHandler: sta start");
        if (wifi->getRadioState() == WIFI_RADIO_CONNECTION_PENDING) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        TT_LOG_I(TAG, "eventHandler: disconnected");
        if (wifi->getRadioState() == WIFI_RADIO_CONNECTION_PENDING) {
            wifi->connection_wait_flags.set(WIFI_FAIL_BIT);
        }
        wifi->setRadioState(WIFI_RADIO_ON);
        publish_event_simple(wifi, WifiEventTypeDisconnected);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(event_data);
        TT_LOG_I(TAG, "eventHandler: got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        if (wifi->getRadioState() == WIFI_RADIO_CONNECTION_PENDING) {
            wifi->connection_wait_flags.set(WIFI_CONNECTED_BIT);
            // We resume auto-connecting only when there was an explicit request by the user for the connection
            // TODO: Make thread-safe
            wifi->pause_auto_connect = false; // Resume auto-connection
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        auto* event = static_cast<wifi_event_sta_scan_done_t*>(event_data);
        TT_LOG_I(TAG, "eventHandler: wifi scanning done (scan id %u)", event->scan_id);
        bool copied_list = copy_scan_list(wifi);

        auto state = wifi->getRadioState();
        if (
            state != WIFI_RADIO_OFF &&
            state != WIFI_RADIO_OFF_PENDING
        ) {
            wifi->setScanActive(false);
            esp_wifi_scan_stop();
        }

        publish_event_simple(wifi_singleton, WifiEventTypeScanFinished);
        TT_LOG_I(TAG, "eventHandler: Finished scan");

        if (copied_list && wifi_singleton->getRadioState() == WIFI_RADIO_ON && !wifi->pause_auto_connect) {
            getMainDispatcher().dispatch(dispatchAutoConnect, wifi);
        }
    }
}

static void dispatchEnable(std::shared_ptr<void> context) {
    TT_LOG_I(TAG, "dispatchEnable()");
    auto wifi = std::static_pointer_cast<Wifi>(context);

    WifiRadioState state = wifi->getRadioState();
    if (
        state == WIFI_RADIO_ON ||
        state == WIFI_RADIO_ON_PENDING ||
        state == WIFI_RADIO_OFF_PENDING
        ) {
        TT_LOG_W(TAG, "Can't enable from current state");
        return;
    }

    auto lock = std::make_unique<ScopedMutexUsage>(wifi->radioMutex);

    if (lock->acquire(50 / portTICK_PERIOD_MS)) {
        TT_LOG_I(TAG, "Enabling");
        wifi->setRadioState(WIFI_RADIO_ON_PENDING);
        publish_event_simple(wifi, WifiEventTypeRadioStateOnPending);

        if (wifi->netif != nullptr) {
            esp_netif_destroy(wifi->netif);
        }
        wifi->netif = esp_netif_create_default_wifi_sta();

        // Warning: this is the memory-intensive operation
        // It uses over 117kB of RAM with default settings for S3 on IDF v5.1.2
        wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t init_result = esp_wifi_init(&config);
        if (init_result != ESP_OK) {
            TT_LOG_E(TAG, "Wifi init failed");
            if (init_result == ESP_ERR_NO_MEM) {
                TT_LOG_E(TAG, "Insufficient memory");
            }
            wifi->setRadioState(WIFI_RADIO_OFF);
            publish_event_simple(wifi, WifiEventTypeRadioStateOff);
            return;
        }

        esp_wifi_set_storage(WIFI_STORAGE_RAM);

        // TODO: don't crash on check failure
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &eventHandler,
            nullptr,
            &wifi->event_handler_any_id
        ));

        // TODO: don't crash on check failure
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &eventHandler,
            nullptr,
            &wifi->event_handler_got_ip
        ));

        if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) {
            TT_LOG_E(TAG, "Wifi mode setting failed");
            wifi->setRadioState(WIFI_RADIO_OFF);
            esp_wifi_deinit();
            publish_event_simple(wifi, WifiEventTypeRadioStateOff);
            return;
        }

        esp_err_t start_result = esp_wifi_start();
        if (start_result != ESP_OK) {
            TT_LOG_E(TAG, "Wifi start failed");
            if (start_result == ESP_ERR_NO_MEM) {
                TT_LOG_E(TAG, "Insufficient memory");
            }
            wifi->setRadioState(WIFI_RADIO_OFF);
            esp_wifi_set_mode(WIFI_MODE_NULL);
            esp_wifi_deinit();
            publish_event_simple(wifi, WifiEventTypeRadioStateOff);
            return;
        }

        wifi->setRadioState(WIFI_RADIO_ON);
        publish_event_simple(wifi, WifiEventTypeRadioStateOn);
        TT_LOG_I(TAG, "Enabled");
    } else {
        TT_LOG_E(TAG, "enable() mutex timeout");
    }
}

static void dispatchDisable(std::shared_ptr<void> context) {
    TT_LOG_I(TAG, "dispatchDisable()");
    auto wifi = std::static_pointer_cast<Wifi>(context);
    auto lock = wifi->radioMutex.scoped();

    if (!lock->acquire(50 / portTICK_PERIOD_MS)) {
        TT_LOG_E(TAG, "disable() mutex timeout");
        return;
    }

    WifiRadioState state = wifi->getRadioState();
    if (
        state == WIFI_RADIO_OFF ||
        state == WIFI_RADIO_OFF_PENDING ||
        state == WIFI_RADIO_ON_PENDING
        ) {
        TT_LOG_W(TAG, "Can't disable from current state");
        return;
    }

    TT_LOG_I(TAG, "Disabling");
    wifi->setRadioState(WIFI_RADIO_OFF_PENDING);
    publish_event_simple(wifi, WifiEventTypeRadioStateOffPending);

    // Free up scan list memory
    scan_list_free_safely(wifi_singleton);

    if (esp_wifi_stop() != ESP_OK) {
        TT_LOG_E(TAG, "Failed to stop radio");
        wifi->setRadioState(WIFI_RADIO_ON);
        publish_event_simple(wifi, WifiEventTypeRadioStateOn);
        return;
    }

    if (esp_wifi_set_mode(WIFI_MODE_NULL) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to unset mode");
    }

    if (esp_event_handler_instance_unregister(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi->event_handler_any_id
    ) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to unregister id event handler");
    }

    if (esp_event_handler_instance_unregister(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi->event_handler_got_ip
    ) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to unregister ip event handler");
    }

    if (esp_wifi_deinit() != ESP_OK) {
        TT_LOG_E(TAG, "Failed to deinit");
    }

    tt_assert(wifi->netif != nullptr);
    esp_netif_destroy(wifi->netif);
    wifi->netif = nullptr;
    wifi->setScanActive(false);
    wifi->setRadioState(WIFI_RADIO_OFF);
    publish_event_simple(wifi, WifiEventTypeRadioStateOff);
    TT_LOG_I(TAG, "Disabled");
}

static void dispatchScan(std::shared_ptr<void> context) {
    TT_LOG_I(TAG, "dispatchScan()");
    auto wifi = std::static_pointer_cast<Wifi>(context);
    auto lock = wifi->radioMutex.scoped();

    if (!lock->acquire(10 / portTICK_PERIOD_MS)) {
        TT_LOG_E(TAG, "dispatchScan() mutex timeout");
        return;
    }

    WifiRadioState state = wifi->getRadioState();
    if (state != WIFI_RADIO_ON && state != WIFI_RADIO_CONNECTION_ACTIVE && state != WIFI_RADIO_CONNECTION_PENDING) {
        TT_LOG_W(TAG, "Scan unavailable: wifi not enabled");
        return;
    }

    if (wifi->isScanActive()) {
        TT_LOG_W(TAG, "Scan already pending");
        return;
    }

    // TODO: Thread safety
    wifi->last_scan_time = tt::kernel::getTicks();

    if (esp_wifi_scan_start(nullptr, false) != ESP_OK) {
        TT_LOG_I(TAG, "Can't start scan");
        return;
    }

    TT_LOG_I(TAG, "Starting scan");
    wifi->setScanActive(true);
    publish_event_simple(wifi, WifiEventTypeScanStarted);
}

static void dispatchConnect(std::shared_ptr<void> context) {
    TT_LOG_I(TAG, "dispatchConnect()");
    auto wifi = std::static_pointer_cast<Wifi>(context);
    auto lock = wifi->radioMutex.scoped();

    if (!lock->acquire(50 / portTICK_PERIOD_MS)) {
        TT_LOG_E(TAG, "dispatchConnect() mutex timeout");
        return;
    }

    TT_LOG_I(TAG, "Connecting to %s", wifi->connection_target.ssid);

    // Stop radio first, if needed
    WifiRadioState radio_state = wifi->getRadioState();
    if (
        radio_state == WIFI_RADIO_ON ||
        radio_state == WIFI_RADIO_CONNECTION_ACTIVE ||
        radio_state == WIFI_RADIO_CONNECTION_PENDING
        ) {
        TT_LOG_I(TAG, "Connecting: Stopping radio first");
        esp_err_t stop_result = esp_wifi_stop();
        wifi->setScanActive(false);
        if (stop_result != ESP_OK) {
            TT_LOG_E(TAG, "Connecting: Failed to disconnect (%s)", esp_err_to_name(stop_result));
            return;
        }
    }

    wifi->setRadioState(WIFI_RADIO_CONNECTION_PENDING);

    publish_event_simple(wifi, WifiEventTypeConnectionPending);

    wifi_config_t wifi_config = {
        .sta = {
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .ssid = {0},
            .password = {0},
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .bssid_set = false,
            .bssid = { 0 },
            .channel = 0,
            .listen_interval = 0,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold = {
                .rssi = 0,
                .authmode = WIFI_AUTH_WPA2_WPA3_PSK,
            },
            .pmf_cfg = {
                .capable = false,
                .required = false
            },
            .rm_enabled = 0,
            .btm_enabled = 0,
            .mbo_enabled = 0,
            .ft_enabled = 0,
            .owe_enabled = 0,
            .transition_disable = 0,
            .reserved = 0,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .sae_pk_mode = WPA3_SAE_PK_MODE_AUTOMATIC,
            .failure_retry_cnt = 1,
            .he_dcm_set = 0,
            .he_dcm_max_constellation_tx = 0,
            .he_dcm_max_constellation_rx = 0,
            .he_mcs9_enabled = 0,
            .he_su_beamformee_disabled = 0,
            .he_trig_su_bmforming_feedback_disabled = 0,
            .he_trig_mu_bmforming_partial_feedback_disabled = 0,
            .he_trig_cqi_feedback_disabled = 0,
            .he_reserved = 0,
            .sae_h2e_identifier = {0},
        }
    };

    static_assert(sizeof(wifi_config.sta.ssid) == (sizeof(wifi_singleton->connection_target.ssid)-1), "SSID size mismatch");
    memcpy(wifi_config.sta.ssid, wifi_singleton->connection_target.ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, wifi_singleton->connection_target.password, sizeof(wifi_config.sta.password));

    esp_err_t set_config_result = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (set_config_result != ESP_OK) {
        wifi->setRadioState(WIFI_RADIO_ON);
        TT_LOG_E(TAG, "Failed to set wifi config (%s)", esp_err_to_name(set_config_result));
        publish_event_simple(wifi, WifiEventTypeConnectionFailed);
        return;
    }

    esp_err_t wifi_start_result = esp_wifi_start();
    if (wifi_start_result != ESP_OK) {
        wifi->setRadioState(WIFI_RADIO_ON);
        TT_LOG_E(TAG, "Failed to start wifi to begin connecting (%s)", esp_err_to_name(wifi_start_result));
        publish_event_simple(wifi, WifiEventTypeConnectionFailed);
        return;
    }

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT)
     * or connection failed for the maximum number of re-tries (WIFI_FAIL_BIT).
     * The bits are set by wifi_event_handler() */
    uint32_t bits = wifi_singleton->connection_wait_flags.wait(WIFI_FAIL_BIT | WIFI_CONNECTED_BIT);
    TT_LOG_I(TAG, "Waiting for EventFlag by event_handler()");

    if (bits & WIFI_CONNECTED_BIT) {
        wifi->setSecureConnection(wifi_config.sta.password[0] != 0x00);
        wifi->setRadioState(WIFI_RADIO_CONNECTION_ACTIVE);
        publish_event_simple(wifi, WifiEventTypeConnectionSuccess);
        TT_LOG_I(TAG, "Connected to %s", wifi->connection_target.ssid);
        if (wifi->connection_target_remember) {
            if (!settings::save(&wifi->connection_target)) {
                TT_LOG_E(TAG, "Failed to store credentials");
            } else {
                TT_LOG_I(TAG, "Stored credentials");
            }
        }
    } else if (bits & WIFI_FAIL_BIT) {
        wifi->setRadioState(WIFI_RADIO_ON);
        publish_event_simple(wifi, WifiEventTypeConnectionFailed);
        TT_LOG_I(TAG, "Failed to connect to %s", wifi->connection_target.ssid);
    } else {
        wifi->setRadioState(WIFI_RADIO_ON);
        publish_event_simple(wifi, WifiEventTypeConnectionFailed);
        TT_LOG_E(TAG, "UNEXPECTED EVENT");
    }

    wifi_singleton->connection_wait_flags.clear(WIFI_FAIL_BIT | WIFI_CONNECTED_BIT);
}

static void dispatchDisconnectButKeepActive(std::shared_ptr<void> context) {
    TT_LOG_I(TAG, "dispatchDisconnectButKeepActive()");
    auto wifi = std::static_pointer_cast<Wifi>(context);
    auto lock = wifi->radioMutex.scoped();

    if (!lock->acquire(50 / portTICK_PERIOD_MS)) {
        TT_LOG_E(TAG, "disconnect_internal_but_keep_active() mutex timeout");
        return;
    }

    esp_err_t stop_result = esp_wifi_stop();
    if (stop_result != ESP_OK) {
        TT_LOG_E(TAG, "Failed to disconnect (%s)", esp_err_to_name(stop_result));
        return;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},
            .password = {0},
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .bssid_set = false,
            .bssid = { 0 },
            .channel = 0,
            .listen_interval = 0,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
            .threshold = {
                .rssi = 0,
                .authmode = WIFI_AUTH_OPEN,
            },
            .pmf_cfg = {
                .capable = false,
                .required = false,
            },
            .rm_enabled = false,
            .btm_enabled = false,
            .mbo_enabled = false,
            .ft_enabled = false,
            .owe_enabled = false,
            .transition_disable = false,
            .reserved = 0,
            .sae_pwe_h2e = WPA3_SAE_PWE_UNSPECIFIED,
            .sae_pk_mode = WPA3_SAE_PK_MODE_AUTOMATIC,
            .failure_retry_cnt = 0,
            .he_dcm_set = false,
            .he_dcm_max_constellation_tx = false,
            .he_dcm_max_constellation_rx = false,
            .he_mcs9_enabled = false,
            .he_su_beamformee_disabled = false,
            .he_trig_su_bmforming_feedback_disabled = false,
            .he_trig_mu_bmforming_partial_feedback_disabled = false,
            .he_trig_cqi_feedback_disabled = false,
            .he_reserved = 0,
            .sae_h2e_identifier = {0},
        },
    };

    esp_err_t set_config_result = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (set_config_result != ESP_OK) {
        // TODO: disable radio, because radio state is in limbo between off and on
        wifi->setRadioState(WIFI_RADIO_OFF);
        TT_LOG_E(TAG, "failed to set wifi config (%s)", esp_err_to_name(set_config_result));
        publish_event_simple(wifi, WifiEventTypeRadioStateOff);
        return;
    }

    esp_err_t wifi_start_result = esp_wifi_start();
    if (wifi_start_result != ESP_OK) {
        // TODO: disable radio, because radio state is in limbo between off and on
        wifi->setRadioState(WIFI_RADIO_OFF);
        TT_LOG_E(TAG, "failed to start wifi to begin connecting (%s)", esp_err_to_name(wifi_start_result));
        publish_event_simple(wifi, WifiEventTypeRadioStateOff);
        return;
    }

    wifi->setRadioState(WIFI_RADIO_ON);
    publish_event_simple(wifi, WifiEventTypeDisconnected);
    TT_LOG_I(TAG, "Disconnected");
}

static bool shouldScanForAutoConnect(std::shared_ptr<Wifi> wifi) {
    auto lock = wifi->dataMutex.scoped();

    if (!lock->acquire(100)) {
        return false;
    }

    bool is_radio_in_scannable_state = wifi->getRadioState() == WIFI_RADIO_ON &&
       !wifi->isScanActive() &&
       !wifi->pause_auto_connect;

    if (!is_radio_in_scannable_state) {
        return false;
    }

    TickType_t current_time = tt::kernel::getTicks();
    bool scan_time_has_looped = (current_time < wifi->last_scan_time);
    bool no_recent_scan = (current_time - wifi->last_scan_time) > (AUTO_SCAN_INTERVAL / portTICK_PERIOD_MS);

    return scan_time_has_looped || no_recent_scan;
}

void onAutoConnectTimer(std::shared_ptr<void> context) {
    auto wifi = std::static_pointer_cast<Wifi>(wifi_singleton);
    // Automatic scanning is done so we can automatically connect to access points
    bool should_auto_scan = shouldScanForAutoConnect(wifi);
    if (should_auto_scan) {
        getMainDispatcher().dispatch(dispatchScan, wifi);
    }
}

static void onStart(ServiceContext& service) {
    tt_assert(wifi_singleton == nullptr);
    wifi_singleton = std::make_shared<Wifi>();

    service.setData(wifi_singleton);

    wifi_singleton->autoConnectTimer = std::make_unique<Timer>(Timer::TypePeriodic, onAutoConnectTimer, wifi_singleton);
    // We want to try and scan more often in case of startup or scan lock failure
    wifi_singleton->autoConnectTimer->start(TT_MIN(2000, AUTO_SCAN_INTERVAL));

    if (settings::shouldEnableOnBoot()) {
        TT_LOG_I(TAG, "Auto-enabling due to setting");
        getMainDispatcher().dispatch(dispatchEnable, wifi_singleton);
    }
}

static void onStop(ServiceContext& service) {
    auto wifi = wifi_singleton;
    tt_assert(wifi != nullptr);

    WifiRadioState state = wifi->getRadioState();
    if (state != WIFI_RADIO_OFF) {
        dispatchDisable(wifi);
    }

    wifi->autoConnectTimer->stop();
    wifi->autoConnectTimer = nullptr; // Must release as it holds a reference to this Wifi instance

    // Acquire all mutexes
    wifi->dataMutex.acquire(TtWaitForever);
    wifi->radioMutex.acquire(TtWaitForever);

    // Detach
    wifi_singleton = nullptr;

    // Release mutexes
    wifi->dataMutex.release();
    wifi->radioMutex.release();

    // Release (hopefully) last Wifi instance by scope
}

extern const ServiceManifest manifest = {
    .id = "Wifi",
    .onStart = onStart,
    .onStop = onStop
};

} // namespace

#endif // ESP_TARGET
