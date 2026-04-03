# ESP32 Bluetooth Driver

Implements the `BluetoothApi` kernel driver interface on top of the NimBLE host stack.
Only compiled when `CONFIG_BT_NIMBLE_ENABLED=y`.

## Source Files

| File | Purpose |
|------|---------|
| `esp32_ble.cpp` | Core driver: NimBLE lifecycle, GAP event handler, advertising, radio enable/disable |
| `esp32_ble_scan.cpp` | GAP discovery, name resolution, auto-connect dispatch |
| `esp32_ble_spp.cpp` | NUS (Nordic UART Service) GATT server — `BtSerialApi` |
| `esp32_ble_midi.cpp` | BLE MIDI GATT server — `BtMidiApi` |
| `esp32_ble_hid.cpp` | HID peripheral (keyboard / mouse / gamepad) — `BtHidDeviceApi` |

Internal shared state is defined in
`Platforms/platform-esp32/private/bluetooth/esp32_ble_internal.h` (`BleCtx`).

The public C API (`bluetooth_scan_start()` etc.) is implemented in
`TactilityKernel/source/drivers/bluetooth.cpp` as thin wrappers over the `BluetoothApi`
function-pointer struct. External code must only call those public functions — never touch
`BluetoothApi` directly.

SPP, MIDI, and HID Device profiles are exposed as **child devices** created by
`esp32_ble_start_device()`. Each gets its own `DeviceType` and is found at runtime via
`bluetooth_serial_get_device()`, `bluetooth_midi_get_device()`, or
`bluetooth_hid_device_get_device()`. Their drivers have `start_device=nullptr` since
initialization is handled by the parent driver; they obtain the shared `BleCtx` via `ble_get_ctx(device)`.

## Profiles

| Profile | Role | API struct |
|---------|------|------------|
| HID Host | Central | Managed in Tactility layer (`BluetoothHidHost.cpp`) |
| HID Device | Peripheral | `BtHidDeviceApi` (child device: `BLUETOOTH_HID_DEVICE_TYPE`) |
| SPP (NUS) | Peripheral | `BtSerialApi` (child device: `BLUETOOTH_SERIAL_TYPE`) |
| MIDI | Peripheral | `BtMidiApi` (child device: `BLUETOOTH_MIDI_TYPE`) |

Only one peripheral profile (HID Device, SPP, or MIDI) can advertise at a time.

## Locking Rules

- `BleCtx::radio_mutex` — guards radio enable/disable state transitions.
- `BleCtx::data_mutex` — guards scan results and RX queues. Released before any NimBLE call.
- `std::atomic<>` — connection handles and flag bools are atomics (read/written cross-task).
- File I/O must **never** run on the NimBLE host task — dispatch via `getMainDispatcher()`.

See `bluetooth.puml` for the full thread model diagram.
