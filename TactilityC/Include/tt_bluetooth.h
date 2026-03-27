#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Important: These values must map to tt::service::bluetooth::RadioState values exactly */
typedef enum {
    BtRadioStateOff,
    BtRadioStateOnPending,
    BtRadioStateOn,
    BtRadioStateOffPending,
} TtBtRadioState;

/** Profile IDs — must match BtProfileId in bluetooth.h */
typedef enum {
    TtBtProfileHidHost = 0,
    TtBtProfileHidDevice = 1,
    TtBtProfileSpp = 2,
    TtBtProfileMidi = 3,
} TtBtProfileId;

// ---- Peer record ----

#define TT_BT_NAME_MAX 64

typedef struct {
    uint8_t  addr[6];
    char     name[TT_BT_NAME_MAX + 1];
    int8_t   rssi;
    bool     paired;
    bool     connected;
} TtBtPeer;

/** @return the current Bluetooth radio state */
TtBtRadioState tt_bt_get_radio_state();

/** @return a textual representation of the Bluetooth radio state */
const char* tt_bt_radio_state_to_string(TtBtRadioState state);

/**
 * @brief Enable/disable the Bluetooth radio.
 * @param[in] enabled true to enable, false to disable
 */
void tt_bt_set_enabled(bool enabled);

/** @brief Start scanning for nearby BLE devices. */
void tt_bt_scan_start();

/** @brief Stop an active scan. */
void tt_bt_scan_stop();

/** @return true when a scan is in progress */
bool tt_bt_is_scanning();

/**
 * @brief Connect to a peer using the specified profile.
 * @param[in] addr 6-byte BLE address
 * @param[in] profile_id one of TtBtProfileId
 */
void tt_bt_connect(const uint8_t addr[6], int profile_id);

/**
 * @brief Disconnect a peer from the specified profile.
 * @param[in] addr 6-byte BLE address
 * @param[in] profile_id one of TtBtProfileId
 */
void tt_bt_disconnect(const uint8_t addr[6], int profile_id);

/**
 * @brief Initiate pairing with a peer.
 * @param[in] addr 6-byte BLE address
 */
void tt_bt_pair(const uint8_t addr[6]);

/**
 * @brief Remove a previously paired peer.
 * @param[in] addr 6-byte BLE address
 */
void tt_bt_unpair(const uint8_t addr[6]);

/**
 * @brief Check whether a profile is supported on this build/SOC.
 * @param[in] profile_id one of TtBtProfileId
 * @return true if the profile is available
 */
bool tt_bt_is_profile_supported(int profile_id);

/**
 * @brief Copy current scan results into a caller-provided array.
 * @param[out] out   array to fill
 * @param[in]  max   maximum entries to write
 * @return number of entries written
 */
size_t tt_bt_get_scan_results(TtBtPeer* out, size_t max);

// ---- HID Host (BLE HID central — connect to external keyboard/mouse) ----

/**
 * @brief Connect to a remote BLE HID device by address.
 * Pass the 6-byte address from a scan result. Discovery and LVGL indev
 * registration happen automatically in the background.
 * @param[in] addr 6-byte BLE address
 */
void tt_bt_hid_host_connect(const uint8_t addr[6]);

/** @brief Disconnect from the currently connected BLE HID device. */
void tt_bt_hid_host_disconnect();

/** @return true when the HID host is fully connected and subscribed */
bool tt_bt_hid_host_is_connected();

// ---- HID Device (BLE HID peripheral) ----

/**
 * @brief Start advertising as a BLE HID device.
 * @param[in] appearance BLE GAP Appearance:
 *   0x03C0=Generic HID, 0x03C1=Keyboard, 0x03C2=Mouse, 0x03C4=Gamepad
 * @return true on success
 */
bool tt_bt_hid_device_start(uint16_t appearance);

/** @brief Stop the HID device server and close the active connection. */
void tt_bt_hid_device_stop();

/** @return true when a HID host is connected */
bool tt_bt_hid_device_is_connected();

/**
 * @brief Send a full 8-byte keyboard input report.
 * report[0]=modifier bits, report[1]=reserved, report[2..7]=USB HID keycodes (up to 6 simultaneous).
 * @return true if sent successfully
 */
bool tt_bt_hid_send_keyboard(const uint8_t report[8]);

/**
 * @brief Send a 2-byte consumer/media key report (16-bit HID Consumer usage, little-endian).
 * Examples: {0xE9,0x00}=Volume Up, {0xEA,0x00}=Volume Down, {0xCD,0x00}=Play/Pause.
 * @return true if sent successfully
 */
bool tt_bt_hid_send_consumer(const uint8_t report[2]);

/**
 * @brief Send a 4-byte mouse input report.
 * report[0]=button bits (5), report[1]=X delta, report[2]=Y delta, report[3]=wheel delta.
 * @return true if sent successfully
 */
bool tt_bt_hid_send_mouse(const uint8_t report[4]);

/**
 * @brief Send an 8-byte gamepad input report.
 * report[0..1]=16 button bits, report[2..3]=left stick X/Y, report[4..5]=right stick X/Y,
 * report[6]=L2 trigger, report[7]=R2 trigger. Axes are signed (-127..127).
 * @return true if sent successfully
 */
bool tt_bt_hid_send_gamepad(const uint8_t report[8]);

// ---- SPP (BLE NUS / Nordic UART Service) ----

/** @brief Start advertising as a NUS peripheral. @return true on success */
bool tt_bt_spp_start();

/** @brief Stop the SPP server and close the active connection. */
void tt_bt_spp_stop();

/**
 * @brief Send data over the active SPP connection.
 * @return true if sent successfully
 */
bool tt_bt_spp_write(const uint8_t* data, size_t len);

/**
 * @brief Read data received over the active SPP connection. Non-blocking.
 * One call returns one NUS RX packet (up to max bytes).
 * Subscribe to events or poll; returns 0 if no data available.
 * @param[out] data  buffer to fill (must not be NULL)
 * @param[in]  max   maximum bytes to read
 * @return number of bytes written into data
 */
size_t tt_bt_spp_read(uint8_t* data, size_t max);

/** @return true when a client is connected */
bool tt_bt_spp_is_connected();

// ---- MIDI (BLE MIDI) ----

/** @brief Start advertising as a BLE MIDI peripheral. @return true on success */
bool tt_bt_midi_start();

/** @brief Stop the MIDI server and close the active connection. */
void tt_bt_midi_stop();

/**
 * @brief Send raw MIDI bytes (BLE framing header added automatically).
 * @return true if sent successfully
 */
bool tt_bt_midi_send(const uint8_t* msg, size_t len);

/** @return true when a client is connected */
bool tt_bt_midi_is_connected();

/**
 * @brief Read raw BLE MIDI bytes received from the connected central.
 * Non-blocking: returns 0 immediately if no data is available.
 * The first two bytes of each packet are the BLE MIDI header (timestamp).
 * @param[out] data  buffer to fill (must not be NULL)
 * @param[in]  max   maximum bytes to read
 * @return number of bytes written into data
 */
size_t tt_bt_midi_read(uint8_t* data, size_t max);

#ifdef __cplusplus
}
#endif
