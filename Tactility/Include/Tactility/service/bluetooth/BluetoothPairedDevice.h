#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace tt::service::bluetooth::settings {

struct PairedDevice {
    std::string name;
    std::array<uint8_t, 6> addr;
    bool autoConnect = false;
    /** Profile used to pair (BtProfileId value). Defaults to BT_PROFILE_SPP=2. */
    int profileId = 2;
};

/**
 * @brief Convert a 6-byte BT address to a hex string (e.g. "aabbccddeeff").
 * @param[in] addr the address bytes
 * @return the hex string
 */
std::string addrToHex(const std::array<uint8_t, 6>& addr);

/**
 * @brief Check if a paired device record exists for the given address.
 * @param[in] addr_hex the hex string address (from addrToHex)
 * @return true if a record exists
 */
bool contains(const std::string& addr_hex);

/**
 * @brief Load a paired device record.
 * @param[in] addr_hex the hex string address
 * @param[out] device the loaded settings
 * @return true on success
 */
bool load(const std::string& addr_hex, PairedDevice& device);

/**
 * @brief Save a paired device record.
 * @param[in] device the settings to save
 * @return true on success
 */
bool save(const PairedDevice& device);

/**
 * @brief Remove a paired device record.
 * @param[in] addr_hex the hex string address
 * @return true if the record was found and removed
 */
bool remove(const std::string& addr_hex);

/**
 * @brief Load all stored paired device records.
 * @return vector of all PairedDevice records found in storage
 */
std::vector<PairedDevice> loadAll();

} // namespace tt::service::bluetooth::settings
