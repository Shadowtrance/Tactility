// SPDX-License-Identifier: Apache-2.0
#include "bindings_private.h"

#include <tactility/device.h>
#include <tactility/drivers/wifi.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

/** How many access points one scan_results() call will return. */
constexpr size_t MAX_SCAN_RESULTS = 32;

/** Holds a reference to the first active wifi device for the duration of a call. */
struct ScopedWifi {
    Device* device = nullptr;

    ScopedWifi() {
        if (device_get_first_active_by_type(&WIFI_TYPE, &device) != ERROR_NONE) {
            device = nullptr;
        }
    }

    ~ScopedWifi() {
        if (device != nullptr) {
            device_put(device);
        }
    }

    ScopedWifi(const ScopedWifi&) = delete;
    ScopedWifi& operator=(const ScopedWifi&) = delete;

    explicit operator bool() const { return device != nullptr; }
};

int push_no_wifi(lua_State* state) {
    lua_pushnil(state);
    lua_pushstring(state, "no wifi device");
    return 2;
}

/** Strings rather than numbers: a script comparing to "connected" reads better. */
const char* radio_state_name(WifiRadioState state) {
    switch (state) {
        case WIFI_RADIO_STATE_OFF: return "off";
        case WIFI_RADIO_STATE_ON_PENDING: return "turning_on";
        case WIFI_RADIO_STATE_ON: return "on";
        case WIFI_RADIO_STATE_OFF_PENDING: return "turning_off";
        default: return "unknown";
    }
}

const char* station_state_name(WifiStationState state) {
    switch (state) {
        case WIFI_STATION_STATE_DISCONNECTED: return "disconnected";
        case WIFI_STATION_STATE_CONNECTION_PENDING: return "connecting";
        case WIFI_STATION_STATE_CONNECTED: return "connected";
        default: return "unknown";
    }
}

const char* authentication_name(WifiAuthenticationType type) {
    switch (type) {
        case WIFI_AUTHENTICATION_TYPE_OPEN: return "open";
        case WIFI_AUTHENTICATION_TYPE_WEP: return "wep";
        case WIFI_AUTHENTICATION_TYPE_WPA_PSK: return "wpa";
        case WIFI_AUTHENTICATION_TYPE_WPA2_PSK: return "wpa2";
        case WIFI_AUTHENTICATION_TYPE_WPA_WPA2_PSK: return "wpa/wpa2";
        case WIFI_AUTHENTICATION_TYPE_WPA2_ENTERPRISE: return "wpa2-enterprise";
        case WIFI_AUTHENTICATION_TYPE_WPA3_PSK: return "wpa3";
        case WIFI_AUTHENTICATION_TYPE_WPA2_WPA3_PSK: return "wpa2/wpa3";
        case WIFI_AUTHENTICATION_TYPE_WAPI_PSK: return "wapi";
        case WIFI_AUTHENTICATION_TYPE_OWE: return "owe";
        case WIFI_AUTHENTICATION_TYPE_WPA3_ENT_192: return "wpa3-enterprise";
        case WIFI_AUTHENTICATION_TYPE_WPA3_EXT_PSK: return "wpa3-ext";
        case WIFI_AUTHENTICATION_TYPE_WPA3_EXT_PSK_MIXED_MODE: return "wpa3-ext-mixed";
        default: return "unknown";
    }
}

int wifi_is_available(lua_State* state) {
    const ScopedWifi wifi;
    lua_pushboolean(state, static_cast<bool>(wifi));
    return 1;
}

int wifi_radio_state_binding(lua_State* state) {
    const ScopedWifi wifi;
    if (!wifi) {
        return push_no_wifi(state);
    }

    WifiRadioState radio_state = WIFI_RADIO_STATE_OFF;
    if (wifi_get_radio_state(wifi.device, &radio_state) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "read failed");
        return 2;
    }

    lua_pushstring(state, radio_state_name(radio_state));
    return 1;
}

int wifi_station_state_binding(lua_State* state) {
    const ScopedWifi wifi;
    if (!wifi) {
        return push_no_wifi(state);
    }

    WifiStationState station_state = WIFI_STATION_STATE_DISCONNECTED;
    if (wifi_get_station_state(wifi.device, &station_state) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "read failed");
        return 2;
    }

    lua_pushstring(state, station_state_name(station_state));
    return 1;
}

int wifi_ip_address(lua_State* state) {
    const ScopedWifi wifi;
    if (!wifi) {
        return push_no_wifi(state);
    }

    char address[16] = {}; // the API requires at least 16 bytes
    if (wifi_station_get_ipv4_address(wifi.device, address) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "not connected");
        return 2;
    }

    lua_pushstring(state, address);
    return 1;
}

int wifi_ssid(lua_State* state) {
    const ScopedWifi wifi;
    if (!wifi) {
        return push_no_wifi(state);
    }

    char ssid[33] = {}; // 32 bytes plus the terminator, per the API
    if (wifi_station_get_target_ssid(wifi.device, ssid) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "no target ssid");
        return 2;
    }

    lua_pushstring(state, ssid);
    return 1;
}

int wifi_rssi(lua_State* state) {
    const ScopedWifi wifi;
    if (!wifi) {
        return push_no_wifi(state);
    }

    int32_t rssi = 0;
    if (wifi_station_get_rssi(wifi.device, &rssi) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "not connected");
        return 2;
    }

    lua_pushinteger(state, rssi);
    return 1;
}

int wifi_is_scanning_binding(lua_State* state) {
    const ScopedWifi wifi;
    if (!wifi) {
        return push_no_wifi(state);
    }

    lua_pushboolean(state, wifi_is_scanning(wifi.device));
    return 1;
}

/**
 * Starts a scan. Returns immediately.
 *
 * Asynchronous on purpose: a scan takes seconds, and blocking here would block whatever
 * task the script runs on - which will be the LVGL task once Lua apps draw. A script
 * polls is_scanning() and then reads scan_results().
 */
int wifi_scan_binding(lua_State* state) {
    const ScopedWifi wifi;
    if (!wifi) {
        return push_no_wifi(state);
    }

    if (wifi_scan(wifi.device) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "scan failed to start");
        return 2;
    }

    lua_pushboolean(state, 1);
    return 1;
}

/** The most recent scan's results, as an array of tables. Empty before a scan runs. */
int wifi_scan_results(lua_State* state) {
    const ScopedWifi wifi;
    if (!wifi) {
        return push_no_wifi(state);
    }

    WifiApRecord records[MAX_SCAN_RESULTS];
    size_t count = MAX_SCAN_RESULTS; // in: capacity, out: how many were written

    if (wifi_get_scan_results(wifi.device, records, &count) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "read failed");
        return 2;
    }

    if (count > MAX_SCAN_RESULTS) {
        count = MAX_SCAN_RESULTS; // defensive: never index past the buffer
    }

    lua_newtable(state);

    for (size_t i = 0; i < count; i++) {
        lua_newtable(state);

        lua_pushstring(state, records[i].ssid);
        lua_setfield(state, -2, "ssid");

        lua_pushinteger(state, records[i].rssi);
        lua_setfield(state, -2, "rssi");

        lua_pushinteger(state, records[i].channel);
        lua_setfield(state, -2, "channel");

        lua_pushstring(state, authentication_name(records[i].authentication_type));
        lua_setfield(state, -2, "authentication");

        lua_pushboolean(state, records[i].authentication_type != WIFI_AUTHENTICATION_TYPE_OPEN);
        lua_setfield(state, -2, "secured");

        lua_seti(state, -2, static_cast<lua_Integer>(i + 1));
    }

    return 1;
}

const luaL_Reg wifi_functions[] = {
    { "is_available", wifi_is_available },
    { "radio_state", wifi_radio_state_binding },
    { "station_state", wifi_station_state_binding },
    { "ip_address", wifi_ip_address },
    { "ssid", wifi_ssid },
    { "rssi", wifi_rssi },
    { "scan", wifi_scan_binding },
    { "is_scanning", wifi_is_scanning_binding },
    { "scan_results", wifi_scan_results },
    { nullptr, nullptr }
};

// Deliberately not bound yet: station_connect() and station_disconnect(). Connecting takes
// a password, so binding it means deciding how a script obtains credentials and whether it
// should be able to join arbitrary networks unprompted. Read-only for now; revisit when
// there is a notion of what a Lua app is allowed to do.

}

void lua_bindings_open_wifi(lua_State* state) {
    luaL_newlib(state, wifi_functions);
    lua_setfield(state, -2, "wifi"); // tactility.wifi
}
