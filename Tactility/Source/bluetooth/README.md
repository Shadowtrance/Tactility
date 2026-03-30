# Bluetooth Bridge (Tactility Layer)

Bridges the kernel `BluetoothApi` driver to the Tactility C++ application layer.

## Architecture

```
Apps / Services
      │  tt::bluetooth:: public API (Bluetooth.h)
      ▼
Bluetooth.cpp        — registers BtEventCallback, translates events, dispatches I/O
BluetoothHidHost.cpp — BLE HID central: GATT discovery, LVGL indev management
BluetoothSettings.cpp / BluetoothPairedDevice.cpp — settings persistence
      │  bluetooth_*() public C functions (TactilityKernel/drivers/bluetooth.h)
      ▼
esp32_ble*.cpp       — NimBLE kernel driver (Platforms/platform-esp32)
```

## Source Files

| File | Purpose |
|------|---------|
| `Bluetooth.cpp` | Bridge: event callback, PubSub, public C++ API (`tt::bluetooth::`) |
| `BluetoothMock.cpp` | No-op stubs for non-BLE builds |
| `BluetoothPairedDevice.cpp` | Paired peer persistence (`.device.properties` files) |
| `BluetoothSettings.cpp` | Global BT settings (`enableOnBoot`, `sppAutoStart`, `midiAutoStart`) |
| `BluetoothHidHost.cpp` | HID host: GATT discovery, report parsing, LVGL indev registration |

## Key Design Points

- **No direct struct access**: external code calls `bluetooth_scan_start(device)` etc.
  (public C API in `<tactility/drivers/bluetooth.h>`), never the `BluetoothApi` struct.
- **File I/O off NimBLE task**: the bridge callback (`bt_event_bridge`) runs on the NimBLE
  host task (4 KB stack). All `settings::load/save` calls are dispatched to `getMainDispatcher()`.
- **Scan result cache**: `Bluetooth.cpp` maintains a `std::vector<PeerRecord>` populated from
  `BT_EVENT_PEER_FOUND` events, since the kernel driver does not expose a `get_scan_results()` call.
- **Addr-type cache**: parallel cache of `{addr, addr_type}` entries used by `BluetoothHidHost.cpp`
  for `ble_gap_connect()`.
- **Settings paths**: stored under `/data/service/bluetooth/` (legacy path kept for
  backward compatibility with existing device data).

See `Platforms/platform-esp32/source/drivers/bluetooth/README.md` for the driver internals
and `bluetooth.puml` for the thread model diagram.
