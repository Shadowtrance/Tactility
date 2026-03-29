# Bluetooth Service

Exposes four BLE profiles over a NimBLE host stack:

| Profile | Role | Use Case |
|---------|------|----------|
| HID Host | Central | Connect to BLE keyboard / mouse / gamepad as an input device |
| HID Device | Peripheral | Present Tactility as a keyboard, mouse, or gamepad to a host |
| SPP (NUS) | Peripheral | BLE serial port (Nordic UART Service) |
| MIDI | Peripheral | BLE MIDI instrument interface |

Only one peripheral profile (HID Device, SPP, or MIDI) can advertise at a time.

## Source Files

| File | Purpose |
|------|---------|
| `Bluetooth.cpp` | Public C++ service API — thin wrappers → singleton |
| `BluetoothMock.cpp` | No-op stub for non-BLE builds |
| `BluetoothPairedDevice.cpp` | `getPairedPeers()` — reads `.device.properties` files |
| `BluetoothSettings.cpp` | Per-device settings persistence |
| `BluetoothNimBLE.cpp` | Core: NimBLE lifecycle, GAP event handler, advertising |
| `BluetoothNimBLEScan.cpp` | GAP discovery, name resolution, auto-connect dispatch |
| `BluetoothNimBLESpp.cpp` | NUS (Nordic UART Service) GATT server |
| `BluetoothNimBLEMidi.cpp` | BLE MIDI GATT server |
| `BluetoothNimBLEHidDevice.cpp` | HID peripheral (keyboard / mouse / gamepad) |
| `BluetoothNimBLEHidHost.cpp` | HID central (connect to BLE keyboard/mouse) |

See `bluetooth.puml` for the thread model diagram.

## Locking Rules

- `Bluetooth::getRadioMutex()` — guards radio enable/disable state transitions.
- `Bluetooth::getDataMutex()` — guards scan results and RX queues. Always acquired as a scoped lock; released before any NimBLE call.
- `std::atomic<>` — connection handles and flag bools are atomics (read/written from multiple tasks without a mutex).
- File I/O must **never** run on the NimBLE host task — dispatch via `getMainDispatcher().dispatch()`.
