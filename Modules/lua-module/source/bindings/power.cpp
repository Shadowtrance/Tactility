// SPDX-License-Identifier: Apache-2.0
#include "bindings_private.h"

#include <tactility/device.h>
#include <tactility/drivers/power_supply.h>

extern "C" {
#include <lauxlib.h>
}

namespace {

/** Collects the started power supplies, so one supporting a given property can be picked. */
struct SupplySearch {
    Device* devices[8] = {};
    size_t count = 0;
};

bool collect_supply(Device* device, void* context) {
    auto* search = static_cast<SupplySearch*>(context);

    if (search->count < sizeof(search->devices) / sizeof(search->devices[0]) &&
        device_is_ready(device)) {
        search->devices[search->count++] = device;
    }

    return true;
}

/**
 * Holds a reference to a power supply for the duration of a call.
 *
 * The kernel API is per-device, but boards have one battery, so the binding finds it
 * rather than making every script do the find/release dance for a percentage. The
 * reference is taken and released within the call, so nothing is pinned between them -
 * unlike the device handles in device.cpp, there is no user-visible object to outlive it.
 *
 * A board can register several POWER_SUPPLY_TYPE devices, and they do not all answer the
 * same questions - on the Tab5 the battery gauge is an `ina226-power-supply` child, while
 * other drivers register the type for charge control alone. Taking whichever came first
 * therefore reports "unsupported" for a battery that is plainly present, so the search is
 * for a device that supports the property being asked about, falling back to the first
 * started one when the caller has no particular property in mind.
 */
struct ScopedPowerSupply {
    Device* device = nullptr;

    /** @param required the property to find a supplier for, or -1 for any supply */
    explicit ScopedPowerSupply(int required = -1) {
        SupplySearch search;
        device_for_each_of_type(&POWER_SUPPLY_TYPE, &search, collect_supply);

        for (size_t i = 0; i < search.count; i++) {
            const bool suitable = required < 0 ||
                power_supply_supports_property(search.devices[i],
                                               static_cast<PowerSupplyProperty>(required));
            if (suitable && device_get(search.devices[i]) == ERROR_NONE) {
                device = search.devices[i];
                return;
            }
        }
    }

    ~ScopedPowerSupply() {
        if (device != nullptr) {
            device_put(device);
        }
    }

    ScopedPowerSupply(const ScopedPowerSupply&) = delete;
    ScopedPowerSupply& operator=(const ScopedPowerSupply&) = delete;

    explicit operator bool() const { return device != nullptr; }
};

int push_no_supply(lua_State* state) {
    lua_pushnil(state);
    lua_pushstring(state, "no power supply");
    return 2;
}

/**
 * Shared body for the property readers.
 *
 * Properties are optional per driver - a board might report capacity but not current - so
 * an unsupported property returns nil plus a reason rather than a wrong number.
 */
int get_property(lua_State* state, PowerSupplyProperty property, const char* unsupported_message) {
    const ScopedPowerSupply supply(property);
    if (!supply) {
        // Either no supply at all, or none that answers this particular property
        const ScopedPowerSupply any;
        lua_pushnil(state);
        lua_pushstring(state, any ? unsupported_message : "no power supply");
        return 2;
    }

    PowerSupplyPropertyValue value = {};
    if (power_supply_get_property(supply.device, property, &value) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "read failed");
        return 2;
    }

    lua_pushinteger(state, value.int_value);
    return 1;
}

/** Battery charge, 0-100. */
int power_capacity(lua_State* state) {
    return get_property(state, POWER_SUPPLY_PROP_CAPACITY, "capacity not supported");
}

/** Battery voltage in millivolts. */
int power_voltage(lua_State* state) {
    return get_property(state, POWER_SUPPLY_PROP_VOLTAGE, "voltage not supported");
}

/** Battery current in milliamps: positive while charging, negative while discharging. */
int power_current(lua_State* state) {
    return get_property(state, POWER_SUPPLY_PROP_CURRENT, "current not supported");
}

int power_is_charging(lua_State* state) {
    const ScopedPowerSupply supply(POWER_SUPPLY_PROP_IS_CHARGING);
    if (!supply) {
        const ScopedPowerSupply any;
        lua_pushnil(state);
        lua_pushstring(state, any ? "charge state not supported" : "no power supply");
        return 2;
    }

    PowerSupplyPropertyValue value = {};
    if (power_supply_get_property(supply.device, POWER_SUPPLY_PROP_IS_CHARGING, &value) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "read failed");
        return 2;
    }

    lua_pushboolean(state, value.int_value != 0);
    return 1;
}

/** True when a power supply exists at all, so a script can adapt rather than error. */
int power_is_available(lua_State* state) {
    const ScopedPowerSupply supply;
    lua_pushboolean(state, static_cast<bool>(supply));
    return 1;
}

/**
 * Everything the board's power supply can do, as a table of booleans.
 *
 * Lets a UI hide controls it cannot drive instead of showing them and failing.
 */
int power_capabilities(lua_State* state) {
    SupplySearch search;
    device_for_each_of_type(&POWER_SUPPLY_TYPE, &search, collect_supply);

    if (search.count == 0) {
        return push_no_supply(state);
    }

    // Merged across every supply, matching what the readers do: they each pick a device
    // that answers, so reporting only one device's abilities would contradict them.
    bool capacity = false, voltage = false, current = false, is_charging = false;
    bool charge_control = false, quick_charge = false, power_off = false;

    for (size_t i = 0; i < search.count; i++) {
        Device* device = search.devices[i];
        capacity |= power_supply_supports_property(device, POWER_SUPPLY_PROP_CAPACITY);
        voltage |= power_supply_supports_property(device, POWER_SUPPLY_PROP_VOLTAGE);
        current |= power_supply_supports_property(device, POWER_SUPPLY_PROP_CURRENT);
        is_charging |= power_supply_supports_property(device, POWER_SUPPLY_PROP_IS_CHARGING);
        charge_control |= power_supply_supports_charge_control(device);
        quick_charge |= power_supply_supports_quick_charge(device);
        power_off |= power_supply_supports_power_off(device);
    }

    lua_newtable(state);

    const auto set = [state](const char* key, bool value) {
        lua_pushboolean(state, value);
        lua_setfield(state, -2, key);
    };

    set("capacity", capacity);
    set("voltage", voltage);
    set("current", current);
    set("is_charging", is_charging);
    set("charge_control", charge_control);
    set("quick_charge", quick_charge);
    set("power_off", power_off);

    return 1;
}

int power_is_allowed_to_charge(lua_State* state) {
    const ScopedPowerSupply supply;  // charge control is not a property, so any supply
    if (!supply) {
        return push_no_supply(state);
    }

    if (!power_supply_supports_charge_control(supply.device)) {
        lua_pushnil(state);
        lua_pushstring(state, "charge control not supported");
        return 2;
    }

    lua_pushboolean(state, power_supply_is_allowed_to_charge(supply.device));
    return 1;
}

int power_set_allowed_to_charge(lua_State* state) {
    luaL_checktype(state, 1, LUA_TBOOLEAN);
    const bool allowed = lua_toboolean(state, 1) != 0;

    const ScopedPowerSupply supply;
    if (!supply) {
        return push_no_supply(state);
    }

    if (power_supply_set_allowed_to_charge(supply.device, allowed) != ERROR_NONE) {
        lua_pushnil(state);
        lua_pushstring(state, "charge control not supported");
        return 2;
    }

    lua_pushboolean(state, 1);
    return 1;
}

const luaL_Reg power_functions[] = {
    { "is_available", power_is_available },
    { "capabilities", power_capabilities },
    { "capacity", power_capacity },
    { "voltage", power_voltage },
    { "current", power_current },
    { "is_charging", power_is_charging },
    { "is_allowed_to_charge", power_is_allowed_to_charge },
    { "set_allowed_to_charge", power_set_allowed_to_charge },
    { nullptr, nullptr }
};

// Deliberately not bound: power_supply_power_off(). A script calling it would take the
// device down with no confirmation and no way back, which is not something to expose
// before there is any notion of what a Lua app is allowed to do.

}

void lua_bindings_open_power(lua_State* state) {
    luaL_newlib(state, power_functions);
    lua_setfield(state, -2, "power"); // tactility.power
}
