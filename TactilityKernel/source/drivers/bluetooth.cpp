#include <tactility/drivers/bluetooth.h>
#include <tactility/device.h>
#include <tactility/driver.h>

#define BT_API(device) ((const struct BluetoothApi*)device_get_driver(device)->api)

extern "C" {

// ---- Core radio / scan ----

error_t bluetooth_get_radio_state(struct Device* device, enum BtRadioState* state) {
    return BT_API(device)->get_radio_state(device, state);
}

error_t bluetooth_set_radio_enabled(struct Device* device, bool enabled) {
    return BT_API(device)->set_radio_enabled(device, enabled);
}

error_t bluetooth_scan_start(struct Device* device) {
    return BT_API(device)->scan_start(device);
}

error_t bluetooth_scan_stop(struct Device* device) {
    return BT_API(device)->scan_stop(device);
}

bool bluetooth_is_scanning(struct Device* device) {
    return BT_API(device)->is_scanning(device);
}

// ---- Pairing ----

error_t bluetooth_pair(struct Device* device, const BtAddr addr) {
    return BT_API(device)->pair(device, addr);
}

error_t bluetooth_unpair(struct Device* device, const BtAddr addr) {
    return BT_API(device)->unpair(device, addr);
}

// ---- Connect / disconnect ----

error_t bluetooth_connect(struct Device* device, const BtAddr addr, enum BtProfileId profile) {
    return BT_API(device)->connect(device, addr, profile);
}

error_t bluetooth_disconnect(struct Device* device, const BtAddr addr, enum BtProfileId profile) {
    return BT_API(device)->disconnect(device, addr, profile);
}

// ---- Event callbacks ----

error_t bluetooth_add_event_callback(struct Device* device, void* context, BtEventCallback callback) {
    return BT_API(device)->add_event_callback(device, context, callback);
}

error_t bluetooth_remove_event_callback(struct Device* device, BtEventCallback callback) {
    return BT_API(device)->remove_event_callback(device, callback);
}

// ---- HID host active flag ----

void bluetooth_set_hid_host_active(struct Device* device, bool active) {
    BT_API(device)->set_hid_host_active(device, active);
}

// ---- HID sub-API ----

error_t bluetooth_hid_host_connect(struct Device* device, const BtAddr addr) {
    return BT_API(device)->hid->host_connect(device, addr);
}

error_t bluetooth_hid_host_disconnect(struct Device* device, const BtAddr addr) {
    return BT_API(device)->hid->host_disconnect(device, addr);
}

error_t bluetooth_hid_device_start(struct Device* device) {
    return BT_API(device)->hid->device_start(device);
}

error_t bluetooth_hid_device_stop(struct Device* device) {
    return BT_API(device)->hid->device_stop(device);
}

// ---- Serial sub-API ----

error_t bluetooth_serial_start(struct Device* device) {
    return BT_API(device)->serial->start(device);
}

error_t bluetooth_serial_stop(struct Device* device) {
    return BT_API(device)->serial->stop(device);
}

error_t bluetooth_serial_write(struct Device* device, const uint8_t* data, size_t len, size_t* written) {
    return BT_API(device)->serial->write(device, data, len, written);
}

error_t bluetooth_serial_read(struct Device* device, uint8_t* data, size_t max_len, size_t* read_out) {
    return BT_API(device)->serial->read(device, data, max_len, read_out);
}

bool bluetooth_serial_is_connected(struct Device* device) {
    return BT_API(device)->serial->is_connected(device);
}

// ---- MIDI sub-API ----

error_t bluetooth_midi_start(struct Device* device) {
    return BT_API(device)->midi->start(device);
}

error_t bluetooth_midi_stop(struct Device* device) {
    return BT_API(device)->midi->stop(device);
}

error_t bluetooth_midi_send(struct Device* device, const uint8_t* msg, size_t len) {
    return BT_API(device)->midi->send(device, msg, len);
}

bool bluetooth_midi_is_connected(struct Device* device) {
    return BT_API(device)->midi->is_connected(device);
}

// ---- Device type ----

const struct DeviceType BLUETOOTH_TYPE = {
    .name = "bluetooth",
};

} // extern "C"
