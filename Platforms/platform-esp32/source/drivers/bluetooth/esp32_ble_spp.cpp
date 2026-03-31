#ifdef ESP_PLATFORM
#include <sdkconfig.h>
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)

#include <bluetooth/esp32_ble_internal.h>

#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs_mbuf.h>
#include <algorithm>
#include <cstring>

#define TAG "esp32_ble_spp"
#include <tactility/log.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

// ---- NUS (Nordic UART Service) UUIDs ----

// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
const ble_uuid128_t NUS_SVC_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
);

// 6E400002 RX (write from client → device)
static const ble_uuid128_t NUS_RX_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
);

// 6E400003 TX (notify device → client)
static const ble_uuid128_t NUS_TX_UUID = BLE_UUID128_INIT(
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
);

uint16_t nus_tx_handle;

static int nus_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt* ctxt, void* arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        LOG_I(TAG, "NUS RX %u bytes", (unsigned)len);
        BleCtx* ctx = g_ctx;
        if (ctx != nullptr && len > 0) {
            std::vector<uint8_t> packet(len);
            os_mbuf_copydata(ctxt->om, 0, len, packet.data());
            {
                xSemaphoreTake(ctx->data_mutex, portMAX_DELAY);
                ctx->spp_rx_queue.push_back(std::move(packet));
                while (ctx->spp_rx_queue.size() > 16) ctx->spp_rx_queue.pop_front();
                xSemaphoreGive(ctx->data_mutex);
            }
            struct BtEvent e = {};
            e.type = BT_EVENT_SPP_DATA_RECEIVED;
            ble_publish_event(ctx, e);
        }
    }
    return 0;
}

const struct ble_gatt_chr_def nus_chars_with_handle[] = {
    {
        .uuid      = &NUS_RX_UUID.u,
        .access_cb = nus_chr_access,
        .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid       = &NUS_TX_UUID.u,
        .access_cb  = nus_chr_access,
        .flags      = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &nus_tx_handle,
    },
    { 0 }
};

void ble_spp_init_gatt_handles(BleCtx* /*ctx*/) {
    // nus_tx_handle is written by NimBLE via the val_handle pointer above.
    // Nothing else needed; the extern variable is accessed directly by esp32_ble.cpp.
}

// ---- SPP sub-API implementations ----

static error_t spp_start(struct Device* device) {
    BleCtx* ctx = g_ctx;
    if (ctx == nullptr) return ERROR_INVALID_STATE;
    ctx->spp_active.store(true);
    ble_start_advertising(&NUS_SVC_UUID);
    return ERROR_NONE;
}

error_t ble_spp_start_internal(BleCtx* ctx) {
    return spp_start(nullptr);
}

static error_t spp_stop(struct Device* device) {
    BleCtx* ctx = g_ctx;
    if (ctx == nullptr) return ERROR_NONE;
    ctx->spp_active.store(false);
    if (ctx->spp_conn_handle.load() != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(ctx->spp_conn_handle.load(), BLE_ERR_REM_USER_CONN_TERM);
        ctx->spp_conn_handle.store(BLE_HS_CONN_HANDLE_NONE);
    }
    // Do NOT restart advertising after user-initiated stop — restarting name-only
    // advertising causes bonded Windows hosts to auto-reconnect in a tight loop.
    if (!ctx->midi_active.load() && !ctx->hid_active.load()) {
        ble_gap_adv_stop();
    }
    return ERROR_NONE;
}

static error_t spp_write(struct Device* device, const uint8_t* data, size_t len, size_t* written) {
    BleCtx* ctx = g_ctx;
    if (ctx == nullptr || ctx->spp_conn_handle.load() == BLE_HS_CONN_HANDLE_NONE) {
        if (written) *written = 0;
        return ERROR_INVALID_STATE;
    }
    struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
    if (om == nullptr) {
        if (written) *written = 0;
        return ERROR_INVALID_STATE;
    }
    int rc = ble_gatts_notify_custom(ctx->spp_conn_handle.load(), nus_tx_handle, om);
    if (rc != 0) {
        os_mbuf_free_chain(om);
        if (written) *written = 0;
        return ERROR_INVALID_STATE;
    }
    if (written) *written = len;
    return ERROR_NONE;
}

static error_t spp_read(struct Device* device, uint8_t* data, size_t max_len, size_t* read_out) {
    BleCtx* ctx = g_ctx;
    if (ctx == nullptr || data == nullptr || max_len == 0) {
        if (read_out) *read_out = 0;
        return ERROR_NONE;
    }
    xSemaphoreTake(ctx->data_mutex, portMAX_DELAY);
    if (ctx->spp_rx_queue.empty()) {
        xSemaphoreGive(ctx->data_mutex);
        if (read_out) *read_out = 0;
        return ERROR_NONE;
    }
    auto& front = ctx->spp_rx_queue.front();
    size_t copy_len = std::min(front.size(), max_len);
    memcpy(data, front.data(), copy_len);
    ctx->spp_rx_queue.pop_front();
    xSemaphoreGive(ctx->data_mutex);
    if (read_out) *read_out = copy_len;
    return ERROR_NONE;
}

static bool spp_is_connected(struct Device* device) {
    BleCtx* ctx = g_ctx;
    return ctx != nullptr && ctx->spp_conn_handle.load() != BLE_HS_CONN_HANDLE_NONE;
}

const BtSerialApi nimble_serial_api = {
    .start        = spp_start,
    .stop         = spp_stop,
    .write        = spp_write,
    .read         = spp_read,
    .is_connected = spp_is_connected,
};

#pragma GCC diagnostic pop

#endif // CONFIG_BT_NIMBLE_ENABLED
