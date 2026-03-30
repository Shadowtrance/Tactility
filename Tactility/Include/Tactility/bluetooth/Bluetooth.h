#pragma once

#include <Tactility/PubSub.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace tt::bluetooth {

enum class BtEvent {
    /** Radio was turned on */
    RadioStateOn,
    /** Radio is turning on */
    RadioStateOnPending,
    /** Radio was turned off */
    RadioStateOff,
    /** Radio is turning off */
    RadioStateOffPending,
    /** Scan started */
    ScanStarted,
    /** Scan finished */
    ScanFinished,
    /** A new peer was discovered during scan */
    PeerFound,
    /** Pairing requires user action (confirm passkey or enter pin) */
    PairRequest,
    /** Pairing succeeded */
    PairSuccess,
    /** Pairing failed */
    PairFailed,
    /** A profile connection state changed */
    ProfileStateChanged,
    /** Data was received on the SPP (NUS) RX characteristic */
    SppDataReceived,
    /** Data was received on the BLE MIDI I/O characteristic */
    MidiDataReceived,
};

enum class RadioState {
    Off,
    OnPending,
    On,
    OffPending,
};

struct PeerRecord {
    std::array<uint8_t, 6> addr;
    std::string name;
    int8_t rssi;
    bool paired;
    bool connected;
    /** Profile used to pair (BtProfileId value). Only meaningful for paired peers. */
    int profileId = 0;
};

/**
 * @brief Get the bluetooth PubSub that broadcasts BtEvent values.
 * @return the PubSub instance
 */
std::shared_ptr<PubSub<BtEvent>> getPubsub();

/** @return the current radio state */
RadioState getRadioState();

/** For logging purposes */
const char* radioStateToString(RadioState state);

/**
 * @brief Enable or disable the Bluetooth radio.
 * @param[in] enabled true to enable, false to disable
 */
void setEnabled(bool enabled);

/**
 * @brief Start scanning for nearby BLE devices.
 * Returns immediately; results are delivered via pubsub.
 */
void scanStart();

/** @brief Stop an active scan. */
void scanStop();

/** @return true when a scan is in progress */
bool isScanning();

/** @return the peers found during the last scan */
std::vector<PeerRecord> getScanResults();

/** @return the list of currently paired peers */
std::vector<PeerRecord> getPairedPeers();

/**
 * @brief Initiate pairing with a peer.
 * Returns immediately; result is delivered via pubsub PairSuccess/PairFailed.
 * @param[in] addr the peer address
 */
void pair(const std::array<uint8_t, 6>& addr);

/**
 * @brief Remove a previously paired peer.
 * @param[in] addr the peer address
 */
void unpair(const std::array<uint8_t, 6>& addr);

/**
 * @brief Connect to a peer using the specified profile.
 * @param[in] addr the peer address
 * @param[in] profileId the BtProfileId value (from bluetooth.h)
 */
void connect(const std::array<uint8_t, 6>& addr, int profileId);

/**
 * @brief Disconnect a peer from the specified profile.
 * @param[in] addr the peer address
 * @param[in] profileId the BtProfileId value (from bluetooth.h)
 */
void disconnect(const std::array<uint8_t, 6>& addr, int profileId);

/**
 * @brief Check whether a given profile is supported on this build/SOC.
 * @param[in] profileId the BtProfileId value to query
 * @return true when the profile is available
 */
bool isProfileSupported(int profileId);

// ---- BLE HID Host (central role — connect to external BLE keyboard/mouse) ----

/**
 * @brief Connect to a remote BLE HID device (keyboard, mouse, etc.) as a host.
 * Discovery, CCCD subscription, and LVGL indev registration happen automatically.
 * @param[in] addr 6-byte BLE address of the HID peripheral
 */
void hidHostConnect(const std::array<uint8_t, 6>& addr);

/** @brief Disconnect from the currently connected BLE HID device. */
void hidHostDisconnect();

/** @return true when a BLE HID peripheral is fully subscribed and acting as LVGL input device */
bool hidHostIsConnected();

// ---- BLE HID Device ----

/**
 * @brief Start advertising as a BLE HID device.
 * @param[in] appearance BLE GAP Appearance value:
 *   0x03C0=Generic HID, 0x03C1=Keyboard (default), 0x03C2=Mouse, 0x03C4=Gamepad
 * @return true on success
 */
bool hidDeviceStart(uint16_t appearance = 0x03C1);

/** @brief Stop the HID device server and close any active connection. */
void hidDeviceStop();

/** @return true when a remote HID host is connected */
bool hidDeviceIsConnected();

/**
 * @brief Send a full 8-byte keyboard input report.
 * report[0]=modifier bits, report[1]=reserved, report[2..7]=USB HID keycodes.
 * @return true if sent, false if not connected or send failed
 */
bool hidSendKeyboard(const uint8_t report[8]);

/**
 * @brief Send a 2-byte consumer/media key report (16-bit HID Consumer usage, little-endian).
 * @return true if sent, false if not connected or send failed
 */
bool hidSendConsumer(const uint8_t report[2]);

/**
 * @brief Send a 4-byte mouse input report.
 * report[0]=button bits (5), report[1]=X delta, report[2]=Y delta, report[3]=wheel delta.
 * @return true if sent, false if not connected or send failed
 */
bool hidSendMouse(const uint8_t report[4]);

/**
 * @brief Send an 8-byte gamepad input report.
 * @return true if sent, false if not connected or send failed
 */
bool hidSendGamepad(const uint8_t report[8]);

// ---- BLE SPP (Nordic UART Service) ----

/** @brief Start advertising as a NUS (Nordic UART Service) server. @return true on success */
bool sppStart();

/** @brief Stop the SPP server and close any active connection. */
void sppStop();

/**
 * @brief Send data over the active SPP connection.
 * @return true if data was sent, false if not connected or send failed
 */
bool sppWrite(const uint8_t* data, size_t len);

/**
 * @brief Read data received over the active SPP connection.
 * Non-blocking: returns 0 immediately if no data is available.
 * @param[out] data    buffer to fill
 * @param[in]  max_len maximum bytes to read
 * @return number of bytes written into data (0 if queue empty)
 */
size_t sppRead(uint8_t* data, size_t max_len);

/** @return true when a remote device is connected to the SPP service */
bool sppIsConnected();

// ---- BLE MIDI ----

/** @brief Start advertising as a BLE MIDI device. @return true on success */
bool midiStart();

/** @brief Stop the MIDI server and close any active connection. */
void midiStop();

/**
 * @brief Send raw MIDI bytes. The BLE MIDI framing header is added automatically.
 * @return true if sent, false if not connected or send failed
 */
bool midiSend(const uint8_t* msg, size_t len);

/** @return true when a remote device is connected to the MIDI service */
bool midiIsConnected();

/**
 * @brief Read raw MIDI bytes received from the connected central.
 * Non-blocking: returns 0 immediately if no data is available.
 * @param[out] data    buffer to fill
 * @param[in]  max_len maximum bytes to read
 * @return number of bytes written into data (0 if queue empty)
 */
size_t midiRead(uint8_t* data, size_t max_len);

/**
 * @brief Initialize the Bluetooth bridge layer and optionally enable the radio.
 * Called once from Tactility startup (after kernel drivers are ready).
 * Reads settings and enables the radio if configured to auto-start.
 */
void systemStart();

} // namespace tt::bluetooth
