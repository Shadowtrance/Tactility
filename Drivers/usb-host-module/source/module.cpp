#include "usb_host_module.h"

#include <tactility/module.h>
#include <tactility/log.h>

#include <atomic>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <usb/usb_host.h>
#include <esp_intr_alloc.h>

#define TAG "UsbHost"

constexpr auto USB_LIB_TASK_STACK = 4096;
constexpr auto USB_LIB_TASK_PRIORITY = 10;
constexpr auto USB_LIB_EVENT_TIMEOUT_MS = 500;
constexpr auto USB_HOST_STOP_TIMEOUT_MS = 3000;

static TaskHandle_t usb_lib_task_handle = nullptr;
static SemaphoreHandle_t usb_lib_task_done = nullptr;
static std::atomic<bool> usb_host_running{false};

static void usbLibTask(void* /*arg*/) {
    LOG_I(TAG, "USB lib task started");

    while (true) {
        uint32_t flags = 0;
        esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(USB_LIB_EVENT_TIMEOUT_MS), &flags);
        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            LOG_W(TAG, "usb_host_lib_handle_events: %s", esp_err_to_name(err));
        }

        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            LOG_I(TAG, "No more USB clients, freeing all devices");
            usb_host_device_free_all();
        }
        if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            LOG_I(TAG, "All USB devices freed");
        }

        // Check for stop notification (non-blocking)
        if (ulTaskNotifyTake(pdFALSE, 0) > 0) {
            break;
        }
    }

    LOG_I(TAG, "USB lib task stopping");
    xSemaphoreGive(usb_lib_task_done);
    vTaskDelete(nullptr);
}

extern "C" {

bool usb_host_is_running(void) {
    return usb_host_running;
}

static error_t start() {
    if (usb_lib_task_handle != nullptr) {
        LOG_W(TAG, "USB host already running");
        return ERROR_NONE;
    }

    usb_host_config_t cfg = {
        .skip_phy_setup = false,
        .root_port_unpowered = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = nullptr,
        .fifo_settings_custom = {},
        .peripheral_map = 0,
    };

    esp_err_t ret = usb_host_install(&cfg);
    if (ret != ESP_OK) {
        LOG_E(TAG, "usb_host_install failed: %d", ret);
        return ERROR_RESOURCE;
    }

    usb_lib_task_done = xSemaphoreCreateBinary();
    if (!usb_lib_task_done) {
        LOG_E(TAG, "Failed to create task done semaphore");
        usb_host_uninstall();
        return ERROR_RESOURCE;
    }

    usb_host_running = true;
    BaseType_t result = xTaskCreate(usbLibTask, "usb_lib", USB_LIB_TASK_STACK, nullptr, USB_LIB_TASK_PRIORITY, &usb_lib_task_handle);
    if (result != pdPASS) {
        LOG_E(TAG, "Failed to create usb_lib task");
        usb_host_running = false;
        vSemaphoreDelete(usb_lib_task_done);
        usb_lib_task_done = nullptr;
        usb_host_uninstall();
        return ERROR_RESOURCE;
    }

    LOG_I(TAG, "USB host started");
    return ERROR_NONE;
}

static error_t stop() {
    if (usb_lib_task_handle == nullptr) {
        return ERROR_NONE;
    }

    usb_host_running = false;
    xTaskNotifyGive(usb_lib_task_handle);

    if (xSemaphoreTake(usb_lib_task_done, pdMS_TO_TICKS(USB_HOST_STOP_TIMEOUT_MS)) != pdTRUE) {
        LOG_W(TAG, "USB host stop timed out, force terminating task");
        vTaskDelete(usb_lib_task_handle);
    }
    usb_lib_task_handle = nullptr;
    vSemaphoreDelete(usb_lib_task_done);
    usb_lib_task_done = nullptr;

    usb_host_uninstall();
    LOG_I(TAG, "USB host stopped");
    return ERROR_NONE;
}

Module usb_host_module = {
    .name = "usb-host",
    .start = start,
    .stop = stop,
    .symbols = nullptr,
    .internal = nullptr,
};

} // extern "C"
