#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <Tactility/service/bluetooth/BluetoothNimBLEInternal.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

namespace tt::service::bluetooth {

static const auto LOGGER = Logger("BtService");

// ---- BLE MIDI UUIDs ----

// 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
const ble_uuid128_t MIDI_SVC_UUID = BLE_UUID128_INIT(
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
);

// 7772E5DB-3868-4112-A1A9-F2669D106BF3
static const ble_uuid128_t MIDI_IO_UUID = BLE_UUID128_INIT(
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
);

uint16_t midi_io_handle;

static int midiChrAccess(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        LOGGER.info("MIDI RX {} bytes", len);
        auto bt = bt_singleton;
        if (bt != nullptr && len > 0) {
            std::vector<uint8_t> packet(len);
            os_mbuf_copydata(ctxt->om, 0, len, packet.data());
            {
                auto lock = bt->getDataMutex().asScopedLock();
                lock.lock();
                bt->getMidiRxQueue().push_back(std::move(packet));
                while (bt->getMidiRxQueue().size() > 16) bt->getMidiRxQueue().pop_front();
            }
            publishEvent(bt, BtEvent::MidiDataReceived);
        }
    }
    return 0;
}

const struct ble_gatt_chr_def midi_chars[] = {
    {
        .uuid = &MIDI_IO_UUID.u,
        .access_cb = midiChrAccess,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
        .val_handle = &midi_io_handle,
    },
    { 0 }
};

// Forward declaration so midiStartInternal() can call the Device* overload below
// without resolving to the public bool midiStart() declared in Bluetooth.h.
static error_t midiStart(struct Device* device);

error_t midiStartInternal() {
    return midiStart(nullptr);
}

void midiInitGattHandles() {
    // midi_io_handle is populated by NimBLE when ble_gatts_add_svcs is called
    // (the val_handle pointer in midi_chars[] is written by NimBLE).
    // Sync back to bt->getMidiIoHandle() for any code that reads that field.
    auto bt = bt_singleton;
    if (bt != nullptr) {
        bt->setMidiIoHandle(midi_io_handle);
    }
}

// ---- MIDI Active Sensing keepalive ----

static void midiKeepaliveCallback(void* /*arg*/) {
    auto bt = bt_singleton;
    if (bt == nullptr || bt->getMidiConnHandle() == BLE_HS_CONN_HANDLE_NONE) return;
    static const uint8_t as_pkt[3] = { 0x80, 0x80, 0xFE };
    struct os_mbuf* om = ble_hs_mbuf_from_flat(as_pkt, 3);
    if (om == nullptr) return;
    int rc = bt->getMidiUseIndicate()
        ? ble_gatts_indicate_custom(bt->getMidiConnHandle(), midi_io_handle, om)
        : ble_gatts_notify_custom(bt->getMidiConnHandle(), midi_io_handle, om);
    if (rc != 0) os_mbuf_free_chain(om);
}

// MIDI (BLE MIDI)
static error_t midiStart(struct Device* device) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_INVALID_STATE;
    bt->setMidiActive(true);
    // Create a 2-second periodic Active Sensing timer to prevent Windows BLE
    // MIDI driver from declaring the connection idle and disconnecting.
    if (bt->getMidiKeepaliveTimer() == nullptr) {
        esp_timer_create_args_t args = {};
        args.callback = midiKeepaliveCallback;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "midi_as";
        int crc = esp_timer_create(&args, &bt->getMidiKeepaliveTimer());
        if (crc != ESP_OK) {
            LOGGER.error("midiStart: keepalive timer create failed (rc={})", crc);
            return ERROR_INVALID_STATE;
        }
    }
    int src = esp_timer_start_periodic(bt->getMidiKeepaliveTimer(), 2'000'000); // 2 seconds
    if (src != ESP_OK) {
        LOGGER.error("midiStart: keepalive timer start failed (rc={})", src);
    }
    startAdvertising(&MIDI_SVC_UUID);
    return ERROR_NONE;
}

static error_t midiStop(struct Device* device) {
    auto bt = bt_singleton;
    if (bt == nullptr) return ERROR_NONE;
    bt->setMidiActive(false);
    if (bt->getMidiKeepaliveTimer() != nullptr) {
        esp_timer_stop(bt->getMidiKeepaliveTimer());
    }
    if (bt->getMidiConnHandle() != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(bt->getMidiConnHandle(), BLE_ERR_REM_USER_CONN_TERM);
        bt->setMidiConnHandle(BLE_HS_CONN_HANDLE_NONE);
    }
    // Do NOT restart advertising — see sppStop for rationale.
    if (!bt->getSppActive() && !bt->getHidActive()) {
        ble_gap_adv_stop();
    }
    return ERROR_NONE;
}

static error_t midiSend(struct Device* device, const uint8_t* msg, size_t len) {
    auto bt = bt_singleton;
    if (bt == nullptr || bt->getMidiConnHandle() == BLE_HS_CONN_HANDLE_NONE) {
        return ERROR_INVALID_STATE;
    }
    // BLE MIDI wraps each message with a 2-byte header (timestamp)
    // Header byte 1: 0x80 | (timestamp_high & 0x3F)
    // Header byte 2: 0x80 | (timestamp_low & 0x7F)
    uint8_t timestamp = 0;
    uint8_t header[2] = { static_cast<uint8_t>(0x80 | (timestamp >> 7)), static_cast<uint8_t>(0x80 | (timestamp & 0x7F)) };
    struct os_mbuf* om = ble_hs_mbuf_from_flat(header, 2);
    if (om == nullptr) return ERROR_INVALID_STATE;
    if (os_mbuf_append(om, msg, len) != 0) {
        os_mbuf_free_chain(om);
        LOGGER.error("midiSend: mbuf append failed");
        return ERROR_INVALID_STATE;
    }
    LOGGER.info("midiSend {} bytes (indicate={})", len, (bool)bt->getMidiUseIndicate());
    int rc = bt->getMidiUseIndicate()
        ? ble_gatts_indicate_custom(bt->getMidiConnHandle(), midi_io_handle, om)
        : ble_gatts_notify_custom(bt->getMidiConnHandle(), midi_io_handle, om);
    if (rc != 0) {
        os_mbuf_free_chain(om);
        LOGGER.error("midiSend failed rc={}", rc);
    }
    return (rc == 0) ? ERROR_NONE : ERROR_INVALID_STATE;
}

static bool midiIsConnected(struct Device* device) {
    auto bt = bt_singleton;
    return bt != nullptr && bt->getMidiConnHandle() != BLE_HS_CONN_HANDLE_NONE;
}

const BtMidiApi nimble_midi_api = {
    .start = midiStart,
    .stop = midiStop,
    .send = midiSend,
    .is_connected = midiIsConnected,
};

// ---- Service-level MIDI wrappers (called by C API and external apps) ----

bool midiStart() {
    bool ok = midiStart(nullptr) == ERROR_NONE;
    if (ok) settings::setMidiAutoStart(true);
    return ok;
}
void midiStop() {
    midiStop(nullptr);
    settings::setMidiAutoStart(false);
}
bool midiSend(const uint8_t* msg, size_t len) {
    return midiSend(nullptr, msg, len) == ERROR_NONE;
}
bool midiIsConnected() { return midiIsConnected(nullptr); }

size_t midiRead(uint8_t* data, size_t max_len) {
    auto bt = bt_singleton;
    if (bt == nullptr || data == nullptr || max_len == 0) return 0;
    auto lock = bt->getDataMutex().asScopedLock();
    lock.lock();
    if (bt->getMidiRxQueue().empty()) return 0;
    auto& front = bt->getMidiRxQueue().front();
    size_t copy_len = std::min(front.size(), max_len);
    std::memcpy(data, front.data(), copy_len);
    bt->getMidiRxQueue().pop_front();
    return copy_len;
}

} // namespace tt::service::bluetooth

#pragma GCC diagnostic pop

#endif // CONFIG_BT_NIMBLE_ENABLED
