#include "usb_host_msc_module.h"

#include <tactility/module.h>
#include <tactility/log.h>
#include <tactility/filesystem/file_system.h>

#include <atomic>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <usb/msc_host_vfs.h>
#include <esp_vfs_fat.h>

#define TAG "UsbMsc"

constexpr auto MAX_MSC_DEVICES = 2;
constexpr auto MSC_EVENT_QUEUE_SIZE = 8;
constexpr auto MSC_PROC_TASK_STACK = 4096;
constexpr auto MSC_PROC_TASK_PRIORITY = 5;
constexpr auto MSC_STOP_TIMEOUT_MS = 2000;

typedef struct {
    uint8_t usb_addr;
    msc_host_device_handle_t device;
    msc_host_vfs_handle_t vfs;
    char mount_path[16];
    struct FileSystem* fs_entry;
    bool mounted;
} msc_dev_entry_t;

// C++ workaround: anonymous enum values inside typedef struct are not injected into
// the enclosing scope. Alias them via decltype so they're usable as named constants.
using msc_event_type_t = decltype(msc_host_event_t{}.event);
static constexpr msc_event_type_t MSC_DEVICE_CONNECTED    = static_cast<msc_event_type_t>(0);
static constexpr msc_event_type_t MSC_DEVICE_DISCONNECTED = static_cast<msc_event_type_t>(1);

enum class MscMsgId : uint8_t { Connected, Disconnected };

typedef struct {
    MscMsgId id;
    union {
        uint8_t address;
        msc_host_device_handle_t handle;
    };
} msc_msg_t;

static msc_dev_entry_t* msc_devs[MAX_MSC_DEVICES] = {};
static QueueHandle_t msc_event_queue = nullptr;
static TaskHandle_t msc_proc_task_handle = nullptr;
static SemaphoreHandle_t msc_proc_task_done = nullptr;
static std::atomic<bool> msc_proc_running{false};

static error_t usb_fs_mount(void* /*data*/) { return ERROR_NONE; }
static error_t usb_fs_unmount(void* /*data*/) { return ERROR_NONE; }
static bool usb_fs_is_mounted(void* data) { return static_cast<msc_dev_entry_t*>(data)->mounted; }
static error_t usb_fs_get_path(void* data, char* out_path, size_t out_path_size) {
    auto* entry = static_cast<msc_dev_entry_t*>(data);
    snprintf(out_path, out_path_size, "%s", entry->mount_path);
    return ERROR_NONE;
}
static const FileSystemApi usb_fs_api = {
    .mount = usb_fs_mount,
    .unmount = usb_fs_unmount,
    .is_mounted = usb_fs_is_mounted,
    .get_path = usb_fs_get_path,
};

static int find_free_slot() {
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        if (msc_devs[i] == nullptr) return i;
    }
    return -1;
}

static int find_slot_by_handle(msc_host_device_handle_t handle) {
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        if (msc_devs[i] && msc_devs[i]->device == handle) return i;
    }
    return -1;
}

static void free_msc_device(int slot) {
    if (slot < 0 || slot >= MAX_MSC_DEVICES || !msc_devs[slot]) return;
    if (msc_devs[slot]->fs_entry) {
        msc_devs[slot]->mounted = false;
        file_system_remove(msc_devs[slot]->fs_entry);
        msc_devs[slot]->fs_entry = nullptr;
    }
    if (msc_devs[slot]->vfs) {
        msc_host_vfs_unregister(msc_devs[slot]->vfs);
    }
    if (msc_devs[slot]->device) {
        msc_host_uninstall_device(msc_devs[slot]->device);
    }
    free(msc_devs[slot]);
    msc_devs[slot] = nullptr;
}

static void free_all_msc_devices() {
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        free_msc_device(i);
    }
}

extern "C" bool usb_msc_eject(const char* mount_path) {
    for (int i = 0; i < MAX_MSC_DEVICES; i++) {
        if (msc_devs[i] && strcmp(msc_devs[i]->mount_path, mount_path) == 0) {
            LOG_I(TAG, "Ejecting USB drive at %s (slot %d)", mount_path, i);
            free_msc_device(i);
            LOG_I(TAG, "USB drive ejected, safe to remove");
            return true;
        }
    }
    LOG_W(TAG, "usb_msc_eject: no drive mounted at %s", mount_path);
    return false;
}

static void msc_event_cb(const msc_host_event_t* event, void* /*arg*/) {
    if (!msc_event_queue) return;
    msc_msg_t msg = {};
    if (event->event == MSC_DEVICE_CONNECTED) {
        msg.id = MscMsgId::Connected;
        msg.address = event->device.address;
        xQueueSend(msc_event_queue, &msg, 0);
    } else if (event->event == MSC_DEVICE_DISCONNECTED) {
        msg.id = MscMsgId::Disconnected;
        msg.handle = event->device.handle;
        xQueueSend(msc_event_queue, &msg, 0);
    }
}

static void mscProcTask(void* /*arg*/) {
    LOG_I(TAG, "MSC proc task started");

    while (msc_proc_running) {
        msc_msg_t msg;
        if (xQueueReceive(msc_event_queue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        if (msg.id == MscMsgId::Connected) {
            LOG_I(TAG, "USB drive connected (addr=%d)", msg.address);
            int slot = find_free_slot();
            if (slot < 0) {
                LOG_W(TAG, "No free slots for USB drive");
                continue;
            }
            msc_devs[slot] = static_cast<msc_dev_entry_t*>(calloc(1, sizeof(msc_dev_entry_t)));
            if (!msc_devs[slot]) {
                LOG_E(TAG, "Failed to allocate MSC device entry");
                continue;
            }
            if (msc_host_install_device(msg.address, &msc_devs[slot]->device) != ESP_OK) {
                LOG_E(TAG, "msc_host_install_device failed");
                free(msc_devs[slot]);
                msc_devs[slot] = nullptr;
                continue;
            }
            msc_devs[slot]->usb_addr = msg.address;
            snprintf(msc_devs[slot]->mount_path, sizeof(msc_devs[slot]->mount_path), USB_MSC_MOUNT_PATH "%d", slot);
            const char* mount_path = msc_devs[slot]->mount_path;

            const esp_vfs_fat_mount_config_t mount_cfg = {
                .format_if_mount_failed = false,
                .max_files = 4,
                .allocation_unit_size = 4096,
                .disk_status_check_enable = false,
                .use_one_fat = false,
            };
            esp_err_t vfs_err = msc_host_vfs_register(msc_devs[slot]->device, mount_path, &mount_cfg, &msc_devs[slot]->vfs);
            if (vfs_err != ESP_OK) {
                LOG_E(TAG, "msc_host_vfs_register failed for %s: %s", mount_path, esp_err_to_name(vfs_err));
                msc_host_uninstall_device(msc_devs[slot]->device);
                free(msc_devs[slot]);
                msc_devs[slot] = nullptr;
                continue;
            }
            LOG_I(TAG, "USB drive mounted at %s", mount_path);
            msc_devs[slot]->mounted = true;
            msc_devs[slot]->fs_entry = file_system_add(&usb_fs_api, msc_devs[slot]);
            if (!msc_devs[slot]->fs_entry) {
                LOG_W(TAG, "Failed to register filesystem for %s", mount_path);
            }

        } else if (msg.id == MscMsgId::Disconnected) {
            int slot = find_slot_by_handle(msg.handle);
            if (slot >= 0) {
                LOG_I(TAG, "USB drive disconnected, unmounting slot %d", slot);
                free_msc_device(slot);
            }
        }
    }

    free_all_msc_devices();
    LOG_I(TAG, "MSC proc task stopped");
    xSemaphoreGive(msc_proc_task_done);
    vTaskDelete(nullptr);
}

extern "C" {

static error_t start() {
    if (msc_proc_task_handle != nullptr) {
        LOG_W(TAG, "MSC module already running");
        return ERROR_NONE;
    }

    msc_event_queue = xQueueCreate(MSC_EVENT_QUEUE_SIZE, sizeof(msc_msg_t));
    if (!msc_event_queue) {
        LOG_E(TAG, "Failed to create MSC event queue");
        return ERROR_RESOURCE;
    }

    msc_proc_task_done = xSemaphoreCreateBinary();
    if (!msc_proc_task_done) {
        LOG_E(TAG, "Failed to create task done semaphore");
        vQueueDelete(msc_event_queue);
        msc_event_queue = nullptr;
        return ERROR_RESOURCE;
    }

    const msc_host_driver_config_t msc_cfg = {
        .create_backround_task = true,
        .task_priority = MSC_PROC_TASK_PRIORITY,
        .stack_size = MSC_PROC_TASK_STACK,
        .core_id = tskNO_AFFINITY,
        .callback = msc_event_cb,
        .callback_arg = nullptr,
    };
    if (msc_host_install(&msc_cfg) != ESP_OK) {
        LOG_E(TAG, "msc_host_install failed");
        vQueueDelete(msc_event_queue);
        vSemaphoreDelete(msc_proc_task_done);
        msc_event_queue = nullptr;
        msc_proc_task_done = nullptr;
        return ERROR_RESOURCE;
    }

    msc_proc_running = true;
    BaseType_t result = xTaskCreate(mscProcTask, "msc_proc", MSC_PROC_TASK_STACK, nullptr, MSC_PROC_TASK_PRIORITY, &msc_proc_task_handle);
    if (result != pdPASS) {
        LOG_E(TAG, "Failed to create msc_proc task");
        msc_proc_running = false;
        msc_host_uninstall();
        vQueueDelete(msc_event_queue);
        vSemaphoreDelete(msc_proc_task_done);
        msc_event_queue = nullptr;
        msc_proc_task_done = nullptr;
        return ERROR_RESOURCE;
    }

    LOG_I(TAG, "USB MSC started");
    return ERROR_NONE;
}

static error_t stop() {
    if (msc_proc_task_handle == nullptr) {
        return ERROR_NONE;
    }

    msc_proc_running = false;

    if (xSemaphoreTake(msc_proc_task_done, pdMS_TO_TICKS(MSC_STOP_TIMEOUT_MS)) != pdTRUE) {
        LOG_W(TAG, "MSC proc task stop timed out, force terminating task");
        vTaskDelete(msc_proc_task_handle);
    }
    msc_proc_task_handle = nullptr;
    vSemaphoreDelete(msc_proc_task_done);
    msc_proc_task_done = nullptr;

    msc_host_uninstall();

    if (msc_event_queue) {
        vQueueDelete(msc_event_queue);
        msc_event_queue = nullptr;
    }

    LOG_I(TAG, "USB MSC stopped");
    return ERROR_NONE;
}

static const struct ModuleSymbol usb_msc_symbols[] = {
    DEFINE_MODULE_SYMBOL(usb_msc_eject),
    MODULE_SYMBOL_TERMINATOR
};

Module usb_host_msc_module = {
    .name = "usb-host-msc",
    .start = start,
    .stop = stop,
    .symbols = usb_msc_symbols,
    .internal = nullptr,
};

} // extern "C"
