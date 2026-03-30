#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <tactility/error.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Device;
struct DeviceType;

// ---- Address ----

#define BT_ADDR_LEN 6

typedef uint8_t BtAddr[BT_ADDR_LEN];

// ---- Radio ----

enum BtRadioState {
    BT_RADIO_STATE_OFF,
    BT_RADIO_STATE_ON_PENDING,
    BT_RADIO_STATE_ON,
    BT_RADIO_STATE_OFF_PENDING,
};

// ---- Peer record ----

#define BT_NAME_MAX 248

struct BtPeerRecord {
    BtAddr addr;
    /** BLE address type (BLE_ADDR_PUBLIC=0, BLE_ADDR_RANDOM=1, etc.) */
    uint8_t addr_type;
    char name[BT_NAME_MAX + 1];
    int8_t rssi;
    bool paired;
    bool connected;
};

// ---- Profile identifiers ----

enum BtProfileId {
    /** Connect to a BLE HID device (keyboard, mouse, gamepad) */
    BT_PROFILE_HID_HOST,
    /** Present this device as a BLE HID peripheral (keyboard, gamepad) */
    BT_PROFILE_HID_DEVICE,
    /** BLE SPP serial port (Nordic UART Service / custom GATT) */
    BT_PROFILE_SPP,
    /** BLE MIDI (GATT-based) */
    BT_PROFILE_MIDI,
};

enum BtProfileState {
    BT_PROFILE_STATE_IDLE,
    BT_PROFILE_STATE_CONNECTING,
    BT_PROFILE_STATE_CONNECTED,
    BT_PROFILE_STATE_DISCONNECTING,
};

// ---- Events ----

enum BtEventType {
    /** Radio state changed */
    BT_EVENT_RADIO_STATE_CHANGED,
    /** Started scanning for peers */
    BT_EVENT_SCAN_STARTED,
    /** Finished scanning for peers */
    BT_EVENT_SCAN_FINISHED,
    /** A new peer was found during scan */
    BT_EVENT_PEER_FOUND,
    /** Pairing requires user confirmation (passkey displayed or entry required) */
    BT_EVENT_PAIR_REQUEST,
    /** Pairing attempt completed */
    BT_EVENT_PAIR_RESULT,
    /** A peer's connection state changed */
    BT_EVENT_CONNECT_STATE_CHANGED,
    /** A profile's state changed */
    BT_EVENT_PROFILE_STATE_CHANGED,
    /** Data was received on the BLE SPP (NUS) RX characteristic */
    BT_EVENT_SPP_DATA_RECEIVED,
    /** Data was received on the BLE MIDI I/O characteristic */
    BT_EVENT_MIDI_DATA_RECEIVED,
};

enum BtPairResult {
    BT_PAIR_RESULT_SUCCESS,
    BT_PAIR_RESULT_FAILED,
    BT_PAIR_RESULT_REJECTED,
    /** Stale bond detected and removed; fresh pairing will follow */
    BT_PAIR_RESULT_BOND_LOST,
};

struct BtPairRequestData {
    BtAddr addr;
    uint32_t passkey; /**< Passkey to display (0 if not applicable) */
    bool needs_confirmation; /**< true: just confirm, false: user must enter passkey */
};

struct BtPairResultData {
    BtAddr addr;
    enum BtPairResult result;
    /** Profile active when pairing completed (BtProfileId value) */
    int profile;
};

struct BtProfileStateData {
    BtAddr addr;
    enum BtProfileId profile;
    enum BtProfileState state;
};

struct BtEvent {
    enum BtEventType type;
    union {
        enum BtRadioState radio_state;
        struct BtPeerRecord peer;
        struct BtPairRequestData pair_request;
        struct BtPairResultData pair_result;
        struct BtProfileStateData profile_state;
    };
};

typedef void (*BtEventCallback)(struct Device* device, void* context, struct BtEvent event);

// ---- HID sub-API ----

/**
 * HID host and device profile API.
 * Host: connect to remote BLE HID peripherals (keyboards, mice, gamepads).
 * Device: present this device as a BLE HID peripheral to a host.
 */
struct BtHidApi {
    /**
     * Connect to a remote HID device.
     * @param[in] device the bluetooth device
     * @param[in] addr the address of the remote HID device
     * @return ERROR_NONE on success
     */
    error_t (*host_connect)(struct Device* device, const BtAddr addr);

    /**
     * Disconnect from a remote HID device.
     * @param[in] device the bluetooth device
     * @param[in] addr the address of the remote HID device
     * @return ERROR_NONE on success
     */
    error_t (*host_disconnect)(struct Device* device, const BtAddr addr);

    /**
     * Start advertising as a BLE HID device.
     * @param[in] device the bluetooth device
     * @return ERROR_NONE on success
     */
    error_t (*device_start)(struct Device* device);

    /**
     * Stop advertising as a BLE HID device.
     * @param[in] device the bluetooth device
     * @return ERROR_NONE on success
     */
    error_t (*device_stop)(struct Device* device);

    /**
     * Send a key event when operating as a HID device.
     * @param[in] device the bluetooth device
     * @param[in] keycode the HID keycode
     * @param[in] pressed true for key down, false for key up
     * @return ERROR_NONE on success
     */
    error_t (*device_send_key)(struct Device* device, uint8_t keycode, bool pressed);
};

// ---- Serial sub-API (BLE SPP) ----

/**
 * BLE serial port profile API (Nordic UART Service or equivalent GATT-based SPP).
 */
struct BtSerialApi {
    /**
     * Start advertising the BLE serial service and accept incoming connections.
     * @param[in] device the bluetooth device
     * @return ERROR_NONE on success
     */
    error_t (*start)(struct Device* device);

    /**
     * Stop the BLE serial service and close any active connections.
     * @param[in] device the bluetooth device
     * @return ERROR_NONE on success
     */
    error_t (*stop)(struct Device* device);

    /**
     * Write data over the BLE serial connection.
     * @param[in] device the bluetooth device
     * @param[in] data the data to send
     * @param[in] len the number of bytes to send
     * @param[out] written the number of bytes actually written
     * @return ERROR_NONE on success
     */
    error_t (*write)(struct Device* device, const uint8_t* data, size_t len, size_t* written);

    /**
     * Read data from the BLE serial receive buffer.
     * @param[in] device the bluetooth device
     * @param[out] data the buffer to read into
     * @param[in] max_len the maximum number of bytes to read
     * @param[out] read_out the number of bytes actually read
     * @return ERROR_NONE on success
     */
    error_t (*read)(struct Device* device, uint8_t* data, size_t max_len, size_t* read_out);

    /**
     * @param[in] device the bluetooth device
     * @return true when a remote device is connected
     */
    bool (*is_connected)(struct Device* device);
};

// ---- MIDI sub-API (BLE MIDI) ----

/**
 * BLE MIDI profile API (MIDI over Bluetooth Low Energy specification).
 */
struct BtMidiApi {
    /**
     * Start advertising the BLE MIDI service and accept incoming connections.
     * @param[in] device the bluetooth device
     * @return ERROR_NONE on success
     */
    error_t (*start)(struct Device* device);

    /**
     * Stop the BLE MIDI service and close any active connections.
     * @param[in] device the bluetooth device
     * @return ERROR_NONE on success
     */
    error_t (*stop)(struct Device* device);

    /**
     * Send MIDI message bytes over the BLE MIDI connection.
     * @param[in] device the bluetooth device
     * @param[in] msg the raw MIDI bytes
     * @param[in] len the number of bytes
     * @return ERROR_NONE on success
     */
    error_t (*send)(struct Device* device, const uint8_t* msg, size_t len);

    /**
     * @param[in] device the bluetooth device
     * @return true when a remote device is connected
     */
    bool (*is_connected)(struct Device* device);
};

// ---- Top-level Bluetooth API ----

struct BluetoothApi {
    /**
     * Get the radio state.
     * @param[in] device the bluetooth device
     * @param[out] state the current radio state
     * @return ERROR_NONE on success
     */
    error_t (*get_radio_state)(struct Device* device, enum BtRadioState* state);

    /**
     * Enable or disable the Bluetooth radio.
     * @param[in] device the bluetooth device
     * @param[in] enabled true to enable, false to disable
     * @return ERROR_NONE on success
     */
    error_t (*set_radio_enabled)(struct Device* device, bool enabled);

    /**
     * Start scanning for nearby BLE devices.
     * @param[in] device the bluetooth device
     * @return ERROR_NONE on success
     */
    error_t (*scan_start)(struct Device* device);

    /**
     * Stop an active scan.
     * @param[in] device the bluetooth device
     * @return ERROR_NONE on success
     */
    error_t (*scan_stop)(struct Device* device);

    /**
     * @param[in] device the bluetooth device
     * @return true when a scan is in progress
     */
    bool (*is_scanning)(struct Device* device);

    /**
     * Initiate pairing with a peer.
     * @param[in] device the bluetooth device
     * @param[in] addr the peer address
     * @return ERROR_NONE on success
     */
    error_t (*pair)(struct Device* device, const BtAddr addr);

    /**
     * Remove a previously paired peer.
     * @param[in] device the bluetooth device
     * @param[in] addr the peer address
     * @return ERROR_NONE on success
     */
    error_t (*unpair)(struct Device* device, const BtAddr addr);

    /**
     * Get the list of currently paired peers.
     * @param[in] device the bluetooth device
     * @param[out] out the buffer to write records into (may be NULL to query count only)
     * @param[in, out] count in: capacity of out, out: actual number of paired peers
     * @return ERROR_NONE on success
     */
    error_t (*get_paired_peers)(struct Device* device, struct BtPeerRecord* out, size_t* count);

    /**
     * Connect to a peer using the specified profile.
     * @param[in] device the bluetooth device
     * @param[in] addr the peer address
     * @param[in] profile the profile to connect with
     * @return ERROR_NONE on success
     */
    error_t (*connect)(struct Device* device, const BtAddr addr, enum BtProfileId profile);

    /**
     * Disconnect a peer from the specified profile.
     * @param[in] device the bluetooth device
     * @param[in] addr the peer address
     * @param[in] profile the profile to disconnect from
     * @return ERROR_NONE on success
     */
    error_t (*disconnect)(struct Device* device, const BtAddr addr, enum BtProfileId profile);

    /**
     * Add an event callback.
     * @param[in] device the bluetooth device
     * @param[in] context context pointer passed to the callback
     * @param[in] callback the callback function
     * @return ERROR_NONE on success
     */
    error_t (*add_event_callback)(struct Device* device, void* context, BtEventCallback callback);

    /**
     * Remove a previously added event callback.
     * @param[in] device the bluetooth device
     * @param[in] callback the callback to remove
     * @return ERROR_NONE on success
     */
    error_t (*remove_event_callback)(struct Device* device, BtEventCallback callback);

    /**
     * Notify the driver that a HID host connection is in progress or complete.
     * Called by the Tactility HID host module to prevent name resolution from
     * initiating a simultaneous central connection (BLE_HS_EALREADY).
     * @param[in] device the bluetooth device
     * @param[in] active true when HID host is connecting/connected, false when idle
     */
    void (*set_hid_host_active)(struct Device* device, bool active);

    /** HID host/device profile API */
    const struct BtHidApi* hid;

    /** BLE serial port profile API */
    const struct BtSerialApi* serial;

    /** BLE MIDI profile API */
    const struct BtMidiApi* midi;
};

extern const struct DeviceType BLUETOOTH_TYPE;

// ---- Public C API ----
// These are the only functions external code should call.
// The BluetoothApi struct above is the internal driver interface only.

error_t bluetooth_get_radio_state(struct Device* device, enum BtRadioState* state);
error_t bluetooth_set_radio_enabled(struct Device* device, bool enabled);
error_t bluetooth_scan_start(struct Device* device);
error_t bluetooth_scan_stop(struct Device* device);
bool    bluetooth_is_scanning(struct Device* device);
error_t bluetooth_pair(struct Device* device, const BtAddr addr);
error_t bluetooth_unpair(struct Device* device, const BtAddr addr);
error_t bluetooth_connect(struct Device* device, const BtAddr addr, enum BtProfileId profile);
error_t bluetooth_disconnect(struct Device* device, const BtAddr addr, enum BtProfileId profile);
error_t bluetooth_add_event_callback(struct Device* device, void* context, BtEventCallback callback);
error_t bluetooth_remove_event_callback(struct Device* device, BtEventCallback callback);
void    bluetooth_set_hid_host_active(struct Device* device, bool active);

error_t bluetooth_hid_host_connect(struct Device* device, const BtAddr addr);
error_t bluetooth_hid_host_disconnect(struct Device* device, const BtAddr addr);
error_t bluetooth_hid_device_start(struct Device* device);
error_t bluetooth_hid_device_stop(struct Device* device);

error_t bluetooth_serial_start(struct Device* device);
error_t bluetooth_serial_stop(struct Device* device);
error_t bluetooth_serial_write(struct Device* device, const uint8_t* data, size_t len, size_t* written);
error_t bluetooth_serial_read(struct Device* device, uint8_t* data, size_t max_len, size_t* read_out);
bool    bluetooth_serial_is_connected(struct Device* device);

error_t bluetooth_midi_start(struct Device* device);
error_t bluetooth_midi_stop(struct Device* device);
error_t bluetooth_midi_send(struct Device* device, const uint8_t* msg, size_t len);
bool    bluetooth_midi_is_connected(struct Device* device);

#ifdef __cplusplus
}
#endif
