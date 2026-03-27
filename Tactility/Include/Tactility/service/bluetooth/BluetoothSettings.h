#pragma once

namespace tt::service::bluetooth::settings {

/**
 * @brief Set whether Bluetooth should be enabled automatically on boot.
 * @param[in] enable true to enable on boot
 */
void setEnableOnBoot(bool enable);

/**
 * @return true if Bluetooth should be enabled on boot
 */
bool shouldEnableOnBoot();

/**
 * @brief Persist whether the SPP (NUS) profile server should restart on boot.
 * Called automatically by sppStart() and sppStop().
 */
void setSppAutoStart(bool enable);

/** @return true if SPP server should be started automatically when the radio comes up */
bool shouldSppAutoStart();

/**
 * @brief Persist whether the BLE MIDI profile server should restart on boot.
 * Called automatically by midiStart() and midiStop().
 */
void setMidiAutoStart(bool enable);

/** @return true if MIDI server should be started automatically when the radio comes up */
bool shouldMidiAutoStart();

} // namespace tt::service::bluetooth::settings
