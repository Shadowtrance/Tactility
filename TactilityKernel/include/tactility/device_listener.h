#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct Device;

enum DeviceEvent {
    DEVICE_EVENT_STARTED,
    DEVICE_EVENT_STOPPING,
    DEVICE_EVENT_STOPPED,
};

typedef void (*DeviceListenerCallback)(
    struct Device *dev,
    enum DeviceEvent event,
    void* context
);

struct DeviceEventListener {
    DeviceListenerCallback callback;
    void* callback_context;
};

void device_listener_add(DeviceListenerCallback callback, void* context);

// Removes the (callback, context) pair added via device_listener_add(). Matching on context too
// (not just callback) matters when the same callback is registered for multiple instances (e.g.
// one per device of the same type): removing one instance's listener must not also remove
// another instance's, which a callback-only match would do by finding whichever was added first.
void device_listener_remove(DeviceListenerCallback callback, void* context);

#ifdef __cplusplus
}
#endif
