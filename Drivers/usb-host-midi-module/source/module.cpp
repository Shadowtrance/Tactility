#include "usb_host_midi_module.h"

#include <tactility/module.h>
#include <tactility/log.h>

#include <atomic>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <usb/usb_host.h>
#include <usb/usb_helpers.h>
#include <usb/usb_types_ch9.h>
#include <usb/usb_types_stack.h>

#define TAG "UsbMidi"

constexpr auto MIDI_TASK_STACK        = 4096;
constexpr auto MIDI_TASK_PRIORITY     = 5;
constexpr auto MIDI_STOP_TIMEOUT_MS   = 2000;
// Buffer must be a multiple of 4 (USB MIDI packet size) and at least one MPS.
// 512 bytes handles a full HS bulk MPS; FS MIDI devices use 64-byte MPS.
constexpr auto MIDI_TRANSFER_BUF_SIZE = 512;

// USB Audio Class / MIDI Streaming subclass values
constexpr uint8_t MIDI_INTF_CLASS        = 0x01;
constexpr uint8_t MIDI_INTF_SUBCLASS     = 0x03;

// ---- State ----
static usb_host_client_handle_t midi_client_hdl   = nullptr;
static usb_device_handle_t      midi_dev_hdl       = nullptr;
static usb_transfer_t*          midi_transfer       = nullptr;
static uint8_t                  midi_ep_addr        = 0;
static uint8_t                  midi_intf_num       = 0;
static std::atomic<bool>        midi_connected{false};
static std::atomic<bool>        midi_running{false};
static TaskHandle_t             midi_task_handle    = nullptr;
static SemaphoreHandle_t        midi_task_done      = nullptr;

static usb_midi_message_cb_t    midi_callback       = nullptr;
static void*                    midi_callback_arg   = nullptr;

// ---- Public API ----

extern "C" void usb_midi_set_callback(usb_midi_message_cb_t callback, void* user_data) {
    midi_callback     = callback;
    midi_callback_arg = user_data;
}

extern "C" bool usb_midi_is_connected(void) {
    return midi_connected;
}

static const struct ModuleSymbol usb_midi_symbols[] = {
    DEFINE_MODULE_SYMBOL(usb_midi_set_callback),
    DEFINE_MODULE_SYMBOL(usb_midi_is_connected),
    MODULE_SYMBOL_TERMINATOR
};

// ---- Descriptor scanning ----

/**
 * Scan the active configuration descriptor for an Audio/MIDI Streaming interface
 * and return its interface number and the first bulk or interrupt IN endpoint.
 */
static bool find_midi_interface(const usb_config_desc_t* cfg, uint8_t* out_intf, uint8_t* out_ep) {
    int offset = 0;
    const usb_standard_desc_t* cur = reinterpret_cast<const usb_standard_desc_t*>(cfg);

    while ((cur = usb_parse_next_descriptor_of_type(
                cur, cfg->wTotalLength, USB_W_VALUE_DT_INTERFACE, &offset)) != nullptr) {
        const auto* intf = reinterpret_cast<const usb_intf_desc_t*>(cur);

        if (intf->bInterfaceClass != MIDI_INTF_CLASS || intf->bInterfaceSubClass != MIDI_INTF_SUBCLASS) {
            continue;
        }

        // Found a MIDI Streaming interface — now find its bulk/interrupt IN endpoint
        int ep_offset = offset;
        const usb_standard_desc_t* ep_cur = cur;
        for (int e = 0; e < intf->bNumEndpoints; e++) {
            ep_cur = usb_parse_next_descriptor_of_type(
                ep_cur, cfg->wTotalLength, USB_W_VALUE_DT_ENDPOINT, &ep_offset);
            if (!ep_cur) break;

            const auto* ep = reinterpret_cast<const usb_ep_desc_t*>(ep_cur);
            usb_transfer_type_t xtype = USB_EP_DESC_GET_XFERTYPE(ep);
            bool is_in = USB_EP_DESC_GET_EP_DIR(ep) == 1;

            if (is_in && (xtype == USB_TRANSFER_TYPE_BULK || xtype == USB_TRANSFER_TYPE_INTR)) {
                *out_intf = intf->bInterfaceNumber;
                *out_ep   = ep->bEndpointAddress;
                return true;
            }
        }
    }
    return false;
}

// ---- MIDI packet parsing and dispatch ----

static void dispatch_midi_packets(const uint8_t* buf, int len) {
    // Snapshot callback and arg together so both are consistent for this dispatch.
    usb_midi_message_cb_t cb = midi_callback;
    void* arg = midi_callback_arg;
    if (!cb) return;

    // USB MIDI Event Packets are exactly 4 bytes each.
    // CIN (Code Index Number, low nibble of byte 0) indicates message type and data byte count.
    // CIN 0x00 (misc) and 0x01 (cable events) are not standard channel messages — skip them.
    for (int i = 0; i + 3 < len; i += 4) {
        uint8_t cin = buf[i] & 0x0F;
        if (cin < 0x02) continue;

        usb_midi_message_t msg = {
            .cable  = static_cast<uint8_t>(buf[i] >> 4),
            .status = buf[i + 1],
            .data1  = buf[i + 2],
            .data2  = buf[i + 3],
        };
        cb(&msg, arg);
    }
}

// ---- Transfer callback ----

static void midi_transfer_cb(usb_transfer_t* transfer) {
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
        dispatch_midi_packets(transfer->data_buffer, transfer->actual_num_bytes);
    }

    // Resubmit as long as the device is still connected. Any error status (NO_DEVICE,
    // CANCELED, STALL) stops the loop naturally; cleanup is handled in client_event_cb.
    if (midi_running && midi_connected && transfer->status != USB_TRANSFER_STATUS_NO_DEVICE) {
        transfer->num_bytes = MIDI_TRANSFER_BUF_SIZE;
        if (usb_host_transfer_submit(transfer) != ESP_OK) {
            LOG_E(TAG, "Failed to resubmit MIDI transfer");
        }
    }
}

// ---- Client event callback (runs inside usb_host_client_handle_events) ----

static void client_event_cb(const usb_host_client_event_msg_t* msg, void* /*arg*/) {
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        uint8_t addr = msg->new_dev.address;

        usb_device_handle_t dev_hdl = nullptr;
        if (usb_host_device_open(midi_client_hdl, addr, &dev_hdl) != ESP_OK) {
            return;
        }

        const usb_config_desc_t* cfg = nullptr;
        if (usb_host_get_active_config_descriptor(dev_hdl, &cfg) != ESP_OK) {
            usb_host_device_close(midi_client_hdl, dev_hdl);
            return;
        }

        uint8_t intf_num = 0, ep_addr = 0;
        if (!find_midi_interface(cfg, &intf_num, &ep_addr)) {
            // Not a MIDI device — close silently
            usb_host_device_close(midi_client_hdl, dev_hdl);
            return;
        }

        if (usb_host_interface_claim(midi_client_hdl, dev_hdl, intf_num, 0) != ESP_OK) {
            LOG_E(TAG, "Failed to claim MIDI interface %d", intf_num);
            usb_host_device_close(midi_client_hdl, dev_hdl);
            return;
        }

        midi_dev_hdl  = dev_hdl;
        midi_intf_num = intf_num;
        midi_ep_addr  = ep_addr;

        midi_transfer->device_handle    = dev_hdl;
        midi_transfer->bEndpointAddress = ep_addr;
        midi_transfer->num_bytes        = MIDI_TRANSFER_BUF_SIZE;
        midi_transfer->callback         = midi_transfer_cb;
        midi_transfer->context          = nullptr;
        midi_transfer->timeout_ms       = 0;

        if (usb_host_transfer_submit(midi_transfer) == ESP_OK) {
            midi_connected = true;
            LOG_I(TAG, "MIDI device connected (intf=%d ep=0x%02x)", intf_num, ep_addr);
        } else {
            LOG_E(TAG, "Failed to submit initial MIDI transfer");
            usb_host_interface_release(midi_client_hdl, dev_hdl, intf_num);
            usb_host_device_close(midi_client_hdl, dev_hdl);
            midi_dev_hdl = nullptr;
        }

    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        if (midi_dev_hdl && msg->dev_gone.dev_hdl == midi_dev_hdl) {
            LOG_I(TAG, "MIDI device disconnected");
            midi_connected = false;
            // Pending transfer will complete with USB_TRANSFER_STATUS_NO_DEVICE and not resubmit.
            usb_host_interface_release(midi_client_hdl, midi_dev_hdl, midi_intf_num);
            usb_host_device_close(midi_client_hdl, midi_dev_hdl);
            midi_dev_hdl = nullptr;
        }
    }
}

// ---- Client task ----

static void midiClientTask(void* /*arg*/) {
    LOG_I(TAG, "MIDI client task started");

    while (midi_running) {
        usb_host_client_handle_events(midi_client_hdl, pdMS_TO_TICKS(100));
    }

    // Teardown any open device
    if (midi_dev_hdl) {
        midi_connected = false;
        usb_host_interface_release(midi_client_hdl, midi_dev_hdl, midi_intf_num);
        usb_host_device_close(midi_client_hdl, midi_dev_hdl);
        midi_dev_hdl = nullptr;
    }

    LOG_I(TAG, "MIDI client task stopped");
    xSemaphoreGive(midi_task_done);
    vTaskDelete(nullptr);
}

// ---- Module lifecycle ----

extern "C" {

static error_t start() {
    if (midi_task_handle != nullptr) {
        LOG_W(TAG, "MIDI module already running");
        return ERROR_NONE;
    }

    if (usb_host_transfer_alloc(MIDI_TRANSFER_BUF_SIZE, 0, &midi_transfer) != ESP_OK) {
        LOG_E(TAG, "Failed to allocate MIDI transfer");
        return ERROR_RESOURCE;
    }

    midi_task_done = xSemaphoreCreateBinary();
    if (!midi_task_done) {
        LOG_E(TAG, "Failed to create task done semaphore");
        usb_host_transfer_free(midi_transfer);
        midi_transfer = nullptr;
        return ERROR_RESOURCE;
    }

    const usb_host_client_config_t client_cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg          = nullptr,
        },
    };
    if (usb_host_client_register(&client_cfg, &midi_client_hdl) != ESP_OK) {
        LOG_E(TAG, "Failed to register USB host client");
        vSemaphoreDelete(midi_task_done);
        midi_task_done = nullptr;
        usb_host_transfer_free(midi_transfer);
        midi_transfer = nullptr;
        return ERROR_RESOURCE;
    }

    midi_running = true;
    BaseType_t result = xTaskCreate(
        midiClientTask, "midi_client", MIDI_TASK_STACK, nullptr, MIDI_TASK_PRIORITY, &midi_task_handle);
    if (result != pdPASS) {
        LOG_E(TAG, "Failed to create MIDI client task");
        midi_running = false;
        usb_host_client_deregister(midi_client_hdl);
        midi_client_hdl = nullptr;
        vSemaphoreDelete(midi_task_done);
        midi_task_done = nullptr;
        usb_host_transfer_free(midi_transfer);
        midi_transfer = nullptr;
        return ERROR_RESOURCE;
    }

    LOG_I(TAG, "USB MIDI started");
    return ERROR_NONE;
}

static error_t stop() {
    if (midi_task_handle == nullptr) {
        return ERROR_NONE;
    }

    midi_running = false;
    // Unblock usb_host_client_handle_events()
    usb_host_client_unblock(midi_client_hdl);

    if (xSemaphoreTake(midi_task_done, pdMS_TO_TICKS(MIDI_STOP_TIMEOUT_MS)) != pdTRUE) {
        LOG_W(TAG, "MIDI client task stop timed out, force terminating task");
        vTaskDelete(midi_task_handle);
    }
    midi_task_handle = nullptr;
    vSemaphoreDelete(midi_task_done);
    midi_task_done = nullptr;

    usb_host_client_deregister(midi_client_hdl);
    midi_client_hdl = nullptr;

    usb_host_transfer_free(midi_transfer);
    midi_transfer = nullptr;

    LOG_I(TAG, "USB MIDI stopped");
    return ERROR_NONE;
}

Module usb_host_midi_module = {
    .name     = "usb-host-midi",
    .start    = start,
    .stop     = stop,
    .symbols  = usb_midi_symbols,
    .internal = nullptr,
};

} // extern "C"
