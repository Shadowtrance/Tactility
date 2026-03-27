#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/service/bluetooth/BluetoothNimBLEInternal.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

namespace tt::service::bluetooth {

static const auto LOGGER = Logger("BtService");

// ---- NUS (Nordic UART Service) UUIDs ----

// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
const ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
);

// 6E400002 RX char (write from client → device)
static const ble_uuid128_t NUS_RX_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
);

// 6E400003 TX char (notify device → client)
static const ble_uuid128_t NUS_TX_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
);

uint16_t nus_tx_handle;

static int nusChrAccess(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        LOGGER.info("NUS RX {} bytes", len);
        auto bt = bt_singleton;
        if (bt != nullptr && len > 0) {
            std::vector<uint8_t> packet(len);
            os_mbuf_copydata(ctxt->om, 0, len, packet.data());
            {
                auto lock = bt->dataMutex.asScopedLock();
                lock.lock();
                bt->sppRxQueue.push_back(std::move(packet));
                // Cap at 16 packets to bound memory use on a slow consumer
                while (bt->sppRxQueue.size() > 16) {
                    bt->sppRxQueue.pop_front();
                }
            }
            publishEvent(bt, BtEvent::SppDataReceived);
        }
    }
    return 0;
}

const struct ble_gatt_chr_def nus_chars_with_handle[] = {
    {
        .uuid = &NUS_RX_UUID.u,
        .access_cb = nusChrAccess,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &NUS_TX_UUID.u,
        .access_cb = nusChrAccess,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &nus_tx_handle,
    },
    { 0 }
};

error_t sppStartInternal() {
    return sppStart(nullptr);
}

void sppInitGattHandles() {
    // nus_tx_handle is populated by NimBLE when ble_gatts_add_svcs is called
    // (the val_handle pointer in nus_chars_with_handle[] is written by NimBLE).
    // Sync back to bt->nusTxHandle for any code that reads that field.
    auto bt = bt_singleton;
    if (bt != nullptr) {
        bt->nusTxHandle = nus_tx_handle;
    }
}

// ---- Profile sub-API implementations ----

// Serial (BLE SPP)
static error_t sppStart(struct Device* device) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_INVALID_STATE;
    bt->sppActive = true;
    startAdvertising(&NUS_SVC_UUID);
    return ERROR_NONE;
}

static error_t sppStop(struct Device* device) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_NONE;
    bt->sppActive = false;
    if (bt->sppConnHandle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(bt->sppConnHandle, BLE_ERR_REM_USER_CONN_TERM);
        bt->sppConnHandle = BLE_HS_CONN_HANDLE_NONE;
    }
    // Do NOT restart advertising after a user-initiated stop.
    // Restarting name-only advertising causes bonded Windows hosts to auto-reconnect
    // in a tight loop (connect → discover → disconnect → repeat).
    // Advertising will resume when the user starts a new profile server.
    if (!bt->midiActive && !bt->hidActive) {
        ble_gap_adv_stop();
    }
    return ERROR_NONE;
}

static error_t sppWrite(struct Device* device, const uint8_t* data, size_t len, size_t* written) {
    auto bt = bt_singleton;
    if (bt == nullptr || bt->sppConnHandle == BLE_HS_CONN_HANDLE_NONE) {
        *written = 0;
        return ERROR_INVALID_STATE;
    }
    struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
    if (om == nullptr) {
        *written = 0;
        return ERROR_INVALID_STATE;
    }
    int rc = ble_gatts_notify_custom(bt->sppConnHandle, nus_tx_handle, om);
    if (rc != 0) {
        *written = 0;
        return ERROR_INVALID_STATE;
    }
    *written = len;
    return ERROR_NONE;
}

static error_t sppRead(struct Device* device, uint8_t* data, size_t max_len, size_t* read_out) {
    auto bt = bt_singleton;
    if (bt == nullptr || data == nullptr || max_len == 0) {
        if (read_out) *read_out = 0;
        return ERROR_NONE;
    }
    auto lock = bt->dataMutex.asScopedLock();
    lock.lock();
    if (bt->sppRxQueue.empty()) {
        *read_out = 0;
        return ERROR_NONE;
    }
    auto& front = bt->sppRxQueue.front();
    size_t copy_len = std::min(front.size(), max_len);
    std::memcpy(data, front.data(), copy_len);
    bt->sppRxQueue.pop_front();
    *read_out = copy_len;
    return ERROR_NONE;
}

static bool sppIsConnected(struct Device* device) {
    auto bt = bt_singleton;
    return bt != nullptr && bt->sppConnHandle != BLE_HS_CONN_HANDLE_NONE;
}

const BtSerialApi nimble_serial_api = {
    .start = sppStart,
    .stop = sppStop,
    .write = sppWrite,
    .read = sppRead,
    .is_connected = sppIsConnected,
};

// ---- Service-level SPP wrappers (called by C API and external apps) ----

bool sppStart() {
    bool ok = sppStart(nullptr) == ERROR_NONE;
    if (ok) settings::setSppAutoStart(true);
    return ok;
}
void sppStop() {
    sppStop(nullptr);
    settings::setSppAutoStart(false);
}
bool sppWrite(const uint8_t* data, size_t len) {
    size_t written = 0;
    return sppWrite(nullptr, data, len, &written) == ERROR_NONE;
}
size_t sppRead(uint8_t* data, size_t max_len) {
    size_t read_out = 0;
    sppRead(nullptr, data, max_len, &read_out);
    return read_out;
}
bool sppIsConnected() { return sppIsConnected(nullptr); }

} // namespace tt::service::bluetooth

#pragma GCC diagnostic pop

#endif // CONFIG_BT_NIMBLE_ENABLED
