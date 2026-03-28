# Bluetooth Service — Architecture & Implementation Reference

## Overview

The Tactility Bluetooth service is a FreeRTOS/ESP-IDF service that exposes four BLE profiles
over a NimBLE host stack:

| Profile | Role | Use Case |
|---------|------|----------|
| **HID Host** | Central | Connect to BLE keyboard / mouse / gamepad as an input device |
| **HID Device** | Peripheral | Present Tactility as a keyboard, mouse, or gamepad to a host PC |
| **SPP (NUS)** | Peripheral | BLE serial port (Nordic UART Service) |
| **MIDI** | Peripheral | BLE MIDI instrument interface |

Only one peripheral profile (HID Device, SPP, or MIDI) can advertise at a time.
HID Host (central) can run simultaneously with a peripheral profile but in practice
the NimBLE controller on C6/P4+C6 (esp_hosted SDIO) cannot maintain two active
connections reliably — name resolution is therefore skipped when any profile server is
running.

---

## Source File Map

```
Tactility/Source/service/bluetooth/
├── Bluetooth.cpp                   Public C++ service API (thin wrappers → singleton)
├── BluetoothMock.cpp               No-op stub for non-BLE builds
├── BluetoothPairedDevice.cpp       getPairedPeers() — reads all .device.properties files
├── BluetoothSettings.cpp           Per-device settings persistence (load/save)
├── BluetoothNimBLE.cpp             Core: NimBLE lifecycle, GAP event handler, advertising
├── BluetoothNimBLEScan.cpp         GAP discovery, name resolution, auto-connect dispatch
├── BluetoothNimBLESpp.cpp          NUS (Nordic UART Service) GATT server
├── BluetoothNimBLEMidi.cpp         BLE MIDI GATT server
├── BluetoothNimBLEHidDevice.cpp    HID peripheral (keyboard / mouse / gamepad)
└── BluetoothNimBLEHidHost.cpp      HID central (connect to BLE keyboard/mouse)

Tactility/Private/Tactility/service/bluetooth/
└── BluetoothNimBLEInternal.h       Shared class/struct definitions and cross-module externs

Tactility/Include/Tactility/service/bluetooth/
├── Bluetooth.h                     Public C++ API for app/service use
├── BluetoothPairedDevice.h         PeerRecord vector getter
└── BluetoothSettings.h             PairedDevice settings struct + load/save declarations

TactilityKernel/include/tactility/drivers/bluetooth.h
                                    C driver API (BluetoothApi, BtHidApi, BtSerialApi, BtMidiApi)
```

All `.cpp` files under `Source/service/bluetooth/` are picked up by
`file(GLOB_RECURSE SOURCE_FILES Source/*.c*)` in `CMakeLists.txt` — no explicit
registration is needed when adding new files.

---

## Thread Model

```
┌─────────────────────────────────────────────────────┐
│ nimble_host task (NimBLE internal, 4 KB stack)      │
│  • All ble_gap_event callbacks                      │
│  • All ble_gatt callbacks (chr access, dsc access)  │
│  • gapDiscEventHandler, nameResGapCallback          │
│  • hidHostGapCb and all HID host discovery callbacks│
│  NO file I/O — stringstream blows the stack         │
└───────────────────┬─────────────────────────────────┘
                    │ getMainDispatcher().dispatch(lambda)
                    ▼
┌─────────────────────────────────────────────────────┐
│ main_dispatcher task (Tactility main task)          │
│  • settings::load / settings::save (file I/O)      │
│  • updatePairedPeers()                              │
│  • hidHostConnect() initial ble_gap_connect call    │
│  • dispatchAutoConnectHidHost                       │
└───────────────────┬─────────────────────────────────┘
                    │ publishEvent → PubSub callbacks
                    ▼
┌─────────────────────────────────────────────────────┐
│ App / service tasks (arbitrary callers)             │
│  • BtManage, BtPeerSettings — subscribe to PubSub  │
│  • sppWrite, midiSend, hidSend* — lock-free sends  │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│ esp_timer task                                      │
│  • advRestartCallback (advertising restart)         │
│  • midiKeepaliveCallback (Active Sensing keepalive) │
│  • hidEncRetryTimerCb (post-encryption delay)       │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│ LVGL task (GuiService)                              │
│  • hidHostKeyboardReadCb / hidHostMouseReadCb       │
│    (lv_indev read callbacks, called by LVGL tick)  │
└─────────────────────────────────────────────────────┘
```

**Locking rules**

- `Bluetooth::radioMutex` — guards radio enable/disable state transitions.
- `Bluetooth::dataMutex` — guards `scanResults`, `scanAddresses`, `sppRxQueue`,
  `midiRxQueue`. Always acquired as a scoped lock; released before any NimBLE call.
- `std::atomic<>` — all connection handles (`sppConnHandle`, `midiConnHandle`,
  `hidConnHandle`) and flag bools (`sppActive`, `midiActive`, `hidActive`,
  `midiUseIndicate`, `linkEncrypted`, `scanActive`, `radioState`, `pendingResetCount`)
  are atomics because they are read/written from multiple tasks without a mutex.
- File I/O must **never** run on the NimBLE host task — dispatch it via
  `getMainDispatcher().dispatch()`.

---

## Key Data Structures

### `Bluetooth` (singleton class — BluetoothNimBLEInternal.h)

One instance lives behind `bt_singleton` (a `shared_ptr`). All sub-modules access it.

| Member | Type | Purpose |
|--------|------|---------|
| `radioMutex` | `RecursiveMutex` | Radio state transitions |
| `dataMutex` | `RecursiveMutex` | Scan results + RX queues |
| `radioState` | `atomic<RadioState>` | Off / OnPending / On / OffPending |
| `scanActive` | `atomic<bool>` | True while scan + name resolution in progress |
| `pubsub` | `shared_ptr<PubSub<BtEvent>>` | Event bus for all BT events |
| `scanResults` | `vector<PeerRecord>` | Devices seen during scan |
| `scanAddresses` | `vector<ble_addr_t>` | Parallel to scanResults; stores full addr type for reconnect |
| `sppRxQueue` | `deque<vector<uint8_t>>` | NUS RX packets (capped at 16) |
| `midiRxQueue` | `deque<vector<uint8_t>>` | BLE MIDI RX packets (capped at 16) |
| `nusTxHandle` | `uint16_t` | GATT handle for NUS TX characteristic CCCD |
| `midiIoHandle` | `uint16_t` | GATT handle for MIDI I/O characteristic CCCD |
| `sppConnHandle` | `atomic<uint16_t>` | Active SPP connection (BLE_HS_CONN_HANDLE_NONE if idle) |
| `sppActive` | `atomic<bool>` | NUS GATT server started |
| `midiConnHandle` | `atomic<uint16_t>` | Active MIDI connection |
| `midiActive` | `atomic<bool>` | MIDI GATT server started |
| `midiUseIndicate` | `atomic<bool>` | Client subscribed via INDICATE (Windows) vs NOTIFY (macOS/iOS) |
| `midiKeepaliveTimer` | `esp_timer_handle_t` | 2-second Active Sensing timer |
| `advRestartTimer` | `esp_timer_handle_t` | One-shot delayed advertising restart |
| `linkEncrypted` | `atomic<bool>` | Current connection has established encryption |
| `hidConnHandle` | `atomic<uint16_t>` | Active HID device connection |
| `hidActive` | `atomic<bool>` | HID device GATT server started |
| `pendingResetCount` | `atomic<int>` | Controller reset counter during OnPending (recovery) |

### `HidHostCtx` (BluetoothNimBLEInternal.h)

One instance (heap, `unique_ptr`) lives as `hid_host_ctx` in `BluetoothNimBLEHidHost.cpp`.
Created at `ble_gap_connect` time; destroyed at disconnect or error.

| Member | Purpose |
|--------|---------|
| `bt` | Shared ownership of the singleton (keeps it alive during cleanup) |
| `connHandle` | Active connection handle |
| `hidSvcStart/End` | ATT handle range of the HID service |
| `inputRpts` | One `HidHostInputRpt` per Input Report characteristic found |
| `allChrDefHandles` | Sorted def handles of all chars in HID service (for descriptor range bounding) |
| `subscribeIdx` | Current index into `inputRpts` for CCCD write chain |
| `dscDiscIdx` | Current index into `inputRpts` for descriptor discovery chain |
| `rptRefReadIdx` | Current index for Report Reference (0x2908) read chain |
| `rptMapHandle` | Val handle of Report Map (0x2A4B); 0 = not found |
| `rptMap` | Raw Report Map bytes (accumulated over multiple reads) |
| `securityInitiated` | True after `ble_gap_security_initiate()` has been called once |
| `typeResolutionDone` | True after rptRef reads + Report Map parse complete |
| `readyBlockFired` | Prevents duplicate CCCD subscription if enc timer fires after subscribe |
| `kbIndev` / `mouseIndev` | LVGL input device handles (freed via LVGL dispatch on disconnect) |
| `mouseCursor` | LVGL mouse cursor object |
| `peerAddr` | 6-byte address of the connected HID peripheral |

### `HidHostInputRpt` (BluetoothNimBLEInternal.h)

One entry per `INPUT` report characteristic found on the remote HOGP server.

| Member | Purpose |
|--------|---------|
| `valHandle` | ATT value handle (for ble_gattc_read, CCCD subscription) |
| `cccdHandle` | Handle of the Client Characteristic Configuration descriptor |
| `rptRefHandle` | Handle of the Report Reference descriptor (0x2908); 0 = none |
| `reportId` | Report ID from 0x2908 (0 = no prefix byte in notification data) |
| `type` | `HidReportType::{Unknown, Keyboard, Mouse, Consumer}` — from Report Map parse |

---

## NimBLE Lifecycle (BluetoothNimBLE.cpp)

```
setEnabled(true)
  └─► nimble_port_init()
      nimble_port_freertos_init(bleHostTask)
      bt_singleton = make_shared<Bluetooth>()
      radioState = OnPending
      publishEvent(RadioStateOnPending)

bleHostTask  (runs nimble_port_run — blocks until nimble_port_stop())
  └─► NimBLE stack initializes → calls onSync()

onSync()
  ├── hidDeviceInitGatt()        register all GATT service tables
  ├── sppInitGattHandles()       resolve nus_tx_handle from NimBLE
  ├── midiInitGattHandles()      resolve midi_io_handle
  ├── hidDeviceInitGattHandles() resolve 4 HID input report handles
  ├── ble_svc_gap_device_name_set(device name from settings)
  ├── radioState = On
  └── publishEvent(RadioStateOn)

onReset(reason)
  ├── radioState = OnPending
  ├── pendingResetCount++
  └── (NimBLE calls onSync again after internal reset)

setEnabled(false)
  └─► dispatchDisable()
      ├── switchGattProfile(HidProfile::None)  unregister HID GATT tables
      ├── nimble_port_stop()
      ├── nimble_port_deinit()
      ├── bt_singleton.reset()
      ├── radioState = Off
      └── publishEvent(RadioStateOff)
```

---

## Advertising

Three advertising entry points exist, all in `BluetoothNimBLE.cpp`:

| Function | Profile | AD Type | Description |
|----------|---------|---------|-------------|
| `startAdvertising(svcUuid)` | SPP or MIDI | Complete 128-bit UUID | Connectable undirected, 100 ms interval |
| `startAdvertisingHid(appearance)` | HID Device | 16-bit UUID 0x1812 + appearance | Connectable undirected, HID flags |
| `scheduleAdvRestart(bt, delay_us)` | any | — | Re-calls one of the above after `delay_us` |

**500 ms restart delay**: After a failed `BLE_GAP_EVENT_CONNECT`, advertising is restarted
500 ms later via `scheduleAdvRestart`. Without this delay, a peer (e.g. Windows) retrying
immediately sees NimBLE still cleaning up internal SMP/connection state → EAGAIN loop.

**Immediate restart on disconnect**: When a client disconnects, advertising restarts
immediately so a reconnect can happen without the user doing anything.

---

## GAP Event Handler (BluetoothNimBLE.cpp — `gapEventHandler`)

All peripheral (server-role) GAP events flow here. Key events:

| Event | Action |
|-------|--------|
| `CONNECT` success | Log only. **Do NOT** call `ble_gap_security_initiate()` — the peer (Windows/macOS) initiates encryption itself. Calling it creates a race with NimBLE's REPEAT_PAIRING retry logic. |
| `CONNECT` failure | `scheduleAdvRestart(bt, 500_000)` |
| `DISCONNECT` | Clear the matching conn handle; restart advertising for the active profile |
| `SUBSCRIBE` | Match `attr_handle` to `nus_tx_handle`, `midi_io_handle`, or one of 4 HID handles; set the corresponding conn handle; persist profile via `getMainDispatcher().dispatch` |
| `MTU` | Log only |
| `CONN_UPDATE` | Accept peer's proposed parameters |
| `CONN_UPDATE_REQ` | Accept whatever the central requests |
| `ENC_CHANGE` | Set `linkEncrypted`; dispatch file I/O to persist `PairedDevice`; profile-specific post-encryption action (MIDI Active Sensing, HID enc retry timer) |
| `REPEAT_PAIRING` | Delete old bond and retry (standard bonded re-pair flow) |
| `PASSKEY_ACTION` | Publish `PairRequest` event if user confirmation or key entry needed |
| `NOTIFY_TX` | Log MIDI/SPP notification results |

**SUBSCRIBE cross-profile guard**: When a NUS CCCD subscription arrives but
`sppActive == false`, it is ignored. Windows subscribes to **all** CCCDs on connect
regardless of which server is actually running — the guard prevents a false
`sppConnHandle` being set when only MIDI or HID is active.

**ENC_CHANGE profile fallback** (for `PairedDevice.profileId`):
```
midiActive  → BT_PROFILE_MIDI
sppActive   → BT_PROFILE_SPP
hidActive   → BT_PROFILE_HID_DEVICE
else        → BT_PROFILE_HID_HOST  (host connected, bonded through central path)
```

---

## Scan + Name Resolution (BluetoothNimBLEScan.cpp)

### Scan flow

```
scanStart()
  └─► ble_gap_disc() with passive scan params
      gapDiscEventHandler receives BLE_GAP_EVENT_DISC per advertisement
        └─► deduplicate by address in scanResults (smart-merge: non-empty name wins)
            publishEvent(PeerFound)

BLE_GAP_EVENT_DISC_COMPLETE (hardware scan done)
  └─► resolveNextUnnamedPeer(bt, 0)  ← do NOT fire ScanFinished yet
```

### Name resolution (GATT Device Name)

After hardware scan completes, unnamed devices are resolved by briefly connecting and
reading the Generic Access Device Name characteristic (UUID 0x2A00) — the same technique
used by Windows and Android.

```
resolveNextUnnamedPeer(bt, start_idx)
  │  Skip if: midiActive || sppActive || hidActive || hid_host_ctx
  │  (initiating a central connection while advertising as peripheral is unsafe)
  │
  ├─► for each scanResults[i] where name.empty():
  │       ble_gap_connect(addr, timeout=1500ms, nameResGapCallback, (void*)i)
  │
  ├─► nameResGapCallback(CONNECT success)
  │       ble_gattc_read_by_uuid(0x2A00, nameReadCallback, same (void*)i)
  │
  ├─► nameReadCallback(status=0, attr≠NULL)
  │       copy name into scanResults[i].name
  │       publishEvent(PeerFound)
  │       ble_gap_terminate(conn_handle, USER_TERM)
  │
  ├─► nameResGapCallback(DISCONNECT)
  │       resolveNextUnnamedPeer(bt, i+1)  ← advance chain
  │
  └─► when no more unnamed devices:
          bt->setScanning(false)
          publishEvent(ScanFinished)
          dispatchAutoConnectHidHost(bt)
```

**Index-as-arg pattern**: The scan result index `i` is cast to `void*` and passed as
the NimBLE `arg`. This eliminates heap allocation entirely — if NimBLE fires a stale
callback (e.g. `ble_gap_master_connect_cancelled`) after the initiating code has moved
on, the callback reads `bt_singleton` directly and the stale index is harmless (it
simply accesses the already-populated entry or an out-of-range index guarded by bounds
checks). This avoids a use-after-free crash that occurred when a `NameResCtx*` was
freed in one callback and then accessed again by a subsequent cancellation callback.

**`ScanFinished` timing**: `setScanning(false)` and `ScanFinished` are only published at
the two terminal points of `resolveNextUnnamedPeer` (skip path and completion path).
This keeps the BtManage spinner active for the entire scan + resolution phase, not just
the hardware scan portion.

### Auto-connect (HID Host)

After name resolution completes (or is skipped), `dispatchAutoConnectHidHost` runs on
the main dispatcher and checks scan results against saved `PairedDevice` records. The
first saved device with `profileId == BT_PROFILE_HID_HOST` and `autoConnect == true`
found in the scan results triggers `hidHostConnect`.

---

## SPP / NUS (BluetoothNimBLESpp.cpp)

**Protocol**: Nordic UART Service (NUS) — a custom 128-bit GATT service widely used as
a BLE serial port replacement.

**UUIDs**:
- Service: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- RX char (client writes to device): `6E400002-...`
- TX char (device notifies client): `6E400003-...`

**GATT table ownership**: `nus_chars_with_handle[]` lives in this file. The HID Device
file embeds a pointer to it via `extern const ble_gatt_chr_def nus_chars_with_handle[]`
(declared in the internal header) so the NUS service definition can be included in the
unified `gatt_svcs_*` tables that also hold HID and MIDI services.

**RX queue**: Incoming data is queued (max 16 packets) under `dataMutex`.
`publishEvent(SppDataReceived)` notifies subscribers without requiring polling.

**TX path**: `sppWrite()` calls `ble_gatts_notify_custom()` with an `os_mbuf` allocated
from the flat data. If `notify_custom` returns non-zero, `os_mbuf_free_chain(om)` is
called — NimBLE only takes mbuf ownership on success.

---

## MIDI (BluetoothNimBLEMidi.cpp)

**Protocol**: BLE MIDI (MIDI over Bluetooth Low Energy Specification v1.0).

**UUID**: `03B80E5A-EDE8-4B33-A751-6CE34EC4C700` (service + I/O characteristic share this UUID).

**BLE MIDI framing**: Every packet prepends a 2-byte header `[0x80][0x80|timestamp]`
before the raw MIDI bytes. `midiSend()` handles this automatically. `midiRead()` returns
the raw packet including the 2-byte header — callers strip it if needed.

**Active Sensing keepalive**: Windows's BLE MIDI driver disconnects after ~8–10 seconds
of silence. A 2-second `esp_timer` fires `midiKeepaliveCallback` while connected,
sending MIDI byte `0xFE` (Active Sensing) wrapped in a BLE MIDI header. An immediate
Active Sensing packet is also sent at CCCD subscribe time.

**NOTIFY vs INDICATE**: macOS/iOS subscribe via NOTIFY; Windows subscribes via INDICATE.
`midiUseIndicate` (atomic) tracks which to use for `midiSend()` and the keepalive.

---

## HID Device (BluetoothNimBLEHidDevice.cpp)

**Profiles**:

| `HidProfile` | Appearance | Reports exposed |
|-------------|-----------|----------------|
| `None` | — | Empty placeholder (registered to avoid re-calling `ble_gatts_add_svcs`) |
| `KbConsumer` | 0x03C1 Keyboard | Keyboard (8 bytes) + Consumer (2 bytes) |
| `Mouse` | 0x03C2 Mouse | Mouse (4 bytes) |
| `KbMouse` | 0x03C0 Generic HID | Keyboard + Consumer + Mouse |
| `Gamepad` | 0x03C4 Gamepad | Gamepad (8 bytes) |

**GATT table structure** (per profile):

```
HID Service (0x1812)
  ├── HID Information (0x2A4A)  — R, fixed value {0x0111, 0x00, 0x02}
  ├── Report Map (0x2A4B)       — R, USB HID descriptor bytes
  ├── HID Control Point (0x2A4C) — W no response
  ├── Protocol Mode (0x2A4E)    — R/W, Boot vs Report (always Report=0x01)
  └── Input Report (0x2A4D) × N — R/Notify
        └── Report Reference (0x2908) — R, {reportId, INPUT=0x01}

NUS Service (6E400001-...)
  └── (embedded in same gatt_svcs_* table)

MIDI Service (03B80E5A-...)
  └── (embedded in same gatt_svcs_* table)
```

**`switchGattProfile(profile)`**: Calls `ble_gatts_add_svcs()` with the new service
table. `active_hid_rpt_map` / `active_hid_rpt_map_len` are only updated **after**
`ble_gatts_add_svcs` succeeds to maintain GATT state consistency.

**`hidDeviceInitGatt()`**: Registers `gatt_svcs_none` (NUS + MIDI + empty HID) on
`onSync`. Called once — NimBLE does not support repeated registration of services with
the same UUID without a full restart.

**`hidDeviceInitGattHandles()`**: After `onSync` registration, resolves the 4 report
characteristic handles (`hid_kb_input_handle`, etc.) by calling `ble_gatts_find_chr`
with the HID service UUID and report characteristic UUID.

**Report format** (HOGP §4.4 — no Report ID prefix in notification data):
- Keyboard: 8 bytes — `[modifier][0x00][key0]..[key5]`
- Consumer: 2 bytes — 16-bit HID Consumer usage, little-endian
- Mouse: 4 bytes — `[buttons][dx][dy][wheel]`
- Gamepad: 8 bytes — `[buttons_lo][buttons_hi][lx][ly][rx][ry][l2][r2]`

---

## HID Host (BluetoothNimBLEHidHost.cpp)

The HID Host implements the Bluetooth HID over GATT Profile (HOGP) central role.
It discovers the HID service, reads all Input Report characteristics, subscribes to
their CCCDs, registers LVGL input devices, and forwards HID reports as LVGL key/mouse
events.

### Connection sequence

```
hidHostConnect(addr)
  └─► (main_dispatcher) ble_gap_connect(addr, timeout=10s, hidHostGapCb)
      hid_host_ctx = make_unique<HidHostCtx>()

hidHostGapCb CONNECT
  └─► ble_gattc_disc_svc_by_uuid(0x1812)  ← find HID service

svcDiscCb (HID service found)
  └─► ble_gattc_disc_all_chrs(hidSvcStart, hidSvcEnd)

chrDiscCb (for each characteristic)
  ├── collect inputRpts (0x2A4D INPUT chars)
  ├── collect allChrDefHandles (sorted after BLE_HS_EDONE)
  ├── note rptMapHandle (0x2A4B)
  └─► BLE_HS_EDONE: hidHostStartDscDisc()

hidHostStartDscDisc()  — serial descriptor discovery per input report
  ├── For each inputRpt[dscDiscIdx]:
  │     dsc range = [valHandle+1, next_def_handle-1]
  │     ble_gattc_disc_all_dscs(range)
  └─► dscDiscCb: record cccdHandle (0x2902) and rptRefHandle (0x2908)
      BLE_HS_EDONE: advance dscDiscIdx, loop or → hidHostStartRptRefRead()

hidHostStartRptRefRead()  — read Report Reference (0x2908) per input report
  └─► rptRefCb(status=0, attr≠NULL):
        parse {reportId, reportType}
        ctx.rptRefReadIdx++; hidHostStartRptRefRead()   ← advance immediately
      rptRefCb(any other status / EDONE): treat as no-op
      All done: ble_gattc_read(rptMapHandle) → hidHostRptMapReadCb()

hidHostRptMapReadCb()
  └─► Accumulate Report Map bytes
      BLE_HS_EDONE: parse map → assign HidReportType to each inputRpt by reportId
      typeResolutionDone = true
      ble_gap_security_initiate(connHandle)

hidHostGapCb ENC_CHANGE
  └─► (if !typeResolutionDone) schedule hidEncRetryTimer (500 ms)
      (if  typeResolutionDone) hidHostSubscribeNext()

hidEncRetryTimerCb (backstop — fires if typeResolution stalled)
  └─► if (!typeResolutionDone): force typeResolutionDone=true
      hidHostSubscribeNext()

hidHostSubscribeNext()  — write CCCD 0x0001 for each inputRpt serially
  └─► All subscribed:
        register kbIndev / mouseIndev with LVGL
        save PairedDevice to settings (preserving existing autoConnect value)
        publishEvent(ProfileStateChanged)
        readyBlockFired = true
```

### Report type detection

1. **Report Reference (0x2908)**: Each Input Report characteristic may have a 2-byte
   descriptor `{reportId, reportType}`. `reportId` is used to match Report Map entries
   and to strip the prefix byte from notification data when `reportId != 0`.

2. **Report Map (0x2A4B)**: USB HID descriptor binary. Parsed linearly — tracks
   `USAGE_PAGE`, `USAGE`, `REPORT_ID`, and `REPORT_COUNT` to determine whether each
   Report ID describes a Keyboard, Mouse, or Consumer report. The assigned
   `HidReportType` is stored in `HidHostInputRpt.type`.

3. **Length heuristic fallback**: For devices with `reportId == 0` and no parseable
   Report Map, report type falls back to length: 8 bytes → Keyboard, 4 bytes → Mouse,
   2 bytes → Consumer.

### HOGP compliance note (report ID prefix)

HOGP §4.4: notification data does **not** include a Report ID prefix byte.
The Report ID is used only to identify the report type during setup.
`hidHostKeyboardReadCb` and `hidHostMouseReadCb` strip the prefix byte only when
`reportId != 0` (some devices incorrectly include it).

### LVGL input devices

- **Keyboard**: `lv_indev_t` with `LV_INDEV_TYPE_KEYPAD`. Read callback
  `hidHostKeyboardReadCb` drains a FreeRTOS queue of `{keycode, pressed}` pairs,
  maps USB HID keycodes to LVGL keycodes, and returns `LV_INDEV_STATE_PRESSED` /
  `_RELEASED`.
- **Mouse**: `lv_indev_t` with `LV_INDEV_TYPE_POINTER`. Read callback
  `hidHostMouseReadCb` reads atomic x/y/button state accumulated by the notification
  callback. Mouse position is clamped to display bounds and compensated for display
  rotation.

### Disconnect / cleanup

On `BLE_GAP_EVENT_DISCONNECT`:
1. `esp_timer_stop(hid_enc_retry_timer)` — cancel pending timer.
2. LVGL indev and cursor deletion dispatched to LVGL task (same task that calls the
   read callbacks) to avoid a race where `hidHostKeyboardReadCb` runs on a deleted
   indev.
3. `hid_host_ctx.reset()` — destroys context, releases resources.
4. `publishEvent(ProfileStateChanged)`.

---

## Pairing and Security

NimBLE manages BLE bonding and LTK storage via `ble_store_config_init()` (NVS).
Tactility maintains a **separate** per-device settings file so `getPairedPeers()` can
enumerate bonded devices without needing to iterate NVS.

**Pairing flow (peripheral / HID Device / SPP / MIDI)**:

1. Central connects and immediately initiates encryption (SMP).
2. NimBLE calls `BLE_GAP_EVENT_REPEAT_PAIRING` if the stored LTK is stale (e.g. after
   NVS clear or firmware flash with new bond slot). Handler: delete old bond and return
   `BLE_GAP_REPEAT_PAIRING_RETRY` — NimBLE will re-pair automatically.
3. On `BLE_GAP_EVENT_ENC_CHANGE` (status 0): `linkEncrypted = true`; dispatch file I/O
   to save `PairedDevice`. Profile ID is inferred from `midiActive` / `sppActive` /
   `hidActive` flags (see ENC_CHANGE profile fallback above).
4. `BLE_GAP_EVENT_PASSKEY_ACTION`: if the SM requires user confirmation or key entry,
   `publishEvent(PairRequest)` — the UI subscribes and prompts the user.

**Do NOT call `ble_gap_security_initiate()` in `BLE_GAP_EVENT_CONNECT`**: The central
(Windows/macOS) always initiates encryption itself. Calling it creates a race with
NimBLE's REPEAT_PAIRING RETRY path and causes Windows to disconnect. The one exception
is the HID Host path where Tactility is the *central* — `ble_gap_security_initiate()`
is called there after type resolution completes.

---

## Settings Persistence (BluetoothSettings.cpp)

Each bonded device is stored as a Java-style `.properties` file on the FAT filesystem:

```
Path:  /data/service/bluetooth/<addrHex>.device.properties
       e.g. /data/service/bluetooth/fd353f43ca48.device.properties

Keys:
  addr        = fd353f43ca48          (hex, no colons, lowercase)
  name        = My Keyboard           (may be empty)
  profileId   = 0                     (BtProfileId value: 0=HID_HOST, 1=HID_DEVICE, ...)
  autoConnect = true
```

`settings::addrToHex(addr)` converts `array<uint8_t,6>` to the 12-char lowercase hex
key. `settings::load(addrHex, device)` uses `strtoul` (safe when exceptions are
disabled — `std::stoul` would call `terminate()` on invalid input).

`getPairedPeers()` (BluetoothPairedDevice.cpp) calls `settings::loadAll()` which
enumerates all `*.device.properties` files and cross-references each with NimBLE's
connection state to populate the `connected` flag.

**Auto-connect preservation**: When saving a `PairedDevice` on reconnection, the
existing file must be loaded first and its `autoConnect` value copied over. Otherwise
every reconnection would reset `autoConnect` to `true`. The HID Host subscribe-complete
save does this explicitly.

---

## Public API Layers

### C++ API (`Bluetooth.h`)

Used by Tactility apps and services. Thin wrappers that forward to the singleton.

```cpp
// Radio
void setEnabled(bool enabled);
RadioState getRadioState();

// Scan
void scanStart();
void scanStop();
bool isScanning();
vector<PeerRecord> getScanResults();
vector<PeerRecord> getPairedPeers();

// Pairing
void pair(const array<uint8_t,6>& addr);
void unpair(const array<uint8_t,6>& addr);
void connect(const array<uint8_t,6>& addr, int profileId);
void disconnect(const array<uint8_t,6>& addr, int profileId);

// HID Host
void hidHostConnect(const array<uint8_t,6>& addr);
void hidHostDisconnect();
bool hidHostIsConnected();

// HID Device
bool hidDeviceStart(uint16_t appearance = 0x03C1);
void hidDeviceStop();
bool hidSendKeyboard(const uint8_t[8]);
bool hidSendConsumer(const uint8_t[2]);
bool hidSendMouse(const uint8_t[4]);
bool hidSendGamepad(const uint8_t[8]);

// SPP
bool sppStart();  void sppStop();
bool sppWrite(const uint8_t*, size_t);
size_t sppRead(uint8_t*, size_t);

// MIDI
bool midiStart();  void midiStop();
bool midiSend(const uint8_t*, size_t);
size_t midiRead(uint8_t*, size_t);

// Events
shared_ptr<PubSub<BtEvent>> getPubsub();
```

### C Driver API (`bluetooth.h`)

Used by device drivers (C code). Function-pointer vtable structs:
- `BluetoothApi` — radio, scan, pair, connect, events
- `BtHidApi` — HID host connect/disconnect, HID device start/stop/send
- `BtSerialApi` — SPP start/stop/read/write
- `BtMidiApi` — MIDI start/stop/send/read

Implemented by `nimble_hid_api`, `nimble_serial_api`, `nimble_midi_api` (defined in
respective `.cpp` files) and registered with the kernel device registry.

### Events (`BtEvent` enum)

| Event | When fired |
|-------|-----------|
| `RadioStateOnPending` | `setEnabled(true)` called, NimBLE initializing |
| `RadioStateOn` | `onSync` completed, radio ready |
| `RadioStateOffPending` | `setEnabled(false)` called |
| `RadioStateOff` | NimBLE fully shut down |
| `ScanStarted` | `scanStart()` called successfully |
| `PeerFound` | New device seen in scan, or name resolved |
| `ScanFinished` | Scan + name resolution fully complete |
| `PairRequest` | User action required (passkey confirm/entry) |
| `PairSuccess` | Pairing completed |
| `PairFailed` | Pairing failed |
| `ProfileStateChanged` | HID host connect/disconnect; HID device subscribe |
| `SppDataReceived` | New data in `sppRxQueue` |
| `MidiDataReceived` | New data in `midiRxQueue` |

---

## BtManage App Integration

`BtManage` subscribes to the BT pubsub and maintains a `BtState` object. Key events
that trigger UI refresh:

- `RadioStateOn`: calls `updatePairedPeers()` — needed because BT may finish
  initializing after `BtManage` opens, and paired devices (saved but offline) would
  not appear without this refresh.
- `ScanFinished`: calls `updatePairedPeers()` and rebuilds the scan results list.
- `PeerFound`: rebuilds only the Available section.
- `ProfileStateChanged`: rebuilds Paired section connection state.

Tapping a **paired** device navigates to `BtPeerSettings` (passes addr hex via Bundle).
Tapping an **unpaired** device calls `pair()`.

`BtPeerSettings` provides: Connect / Disconnect buttons (swap visibility based on live
connection state), Forget (with confirmation dialog), and Auto-connect toggle. It
subscribes to the BT pubsub to receive live connection state updates.

---

## Known Gotchas & Design Decisions

### No `ble_gap_security_initiate()` on CONNECT
See GAP Event Handler section. Calling it here causes Windows disconnects due to race
with NimBLE's REPEAT_PAIRING RETRY mechanism.

### esp_hosted (P4 + C6 via SDIO) timing
`BLE_GAP_EVENT_ENC_CHANGE` can arrive **before** `BLE_GAP_EVENT_CONNECT` on esp_hosted.
The `linkEncrypted` flag handles this — it is checked on CONNECT but its value is
already correct.

### Name resolution skip condition
`resolveNextUnnamedPeer` skips entirely when `midiActive || sppActive || hidActive ||
hid_host_ctx`. Simultaneously initiating a central connection while a peripheral
connection or HID host connection attempt is in progress fails with `BLE_HS_EALREADY`
on the C6 controller over SDIO.

### `std::stoul` is unsafe without exceptions
ESP-IDF builds have C++ exceptions disabled. `std::stoul` calls `terminate()` on
invalid input. Use `settings::load()` (which uses `strtoul` internally) or
`strtoul` directly for hex address parsing.

### `os_mbuf_free_chain` on send failure
`ble_gatts_notify_custom` and `ble_gatts_indicate_custom` take ownership of the mbuf
**only on success** (return 0). On failure, the caller must call `os_mbuf_free_chain(om)`
to avoid a memory leak.

### GATT handle resolution timing
`ble_gatts_find_chr` / `ble_gatts_find_dsc` only work after `ble_gatts_add_svcs` has
been called (in `onSync`). Handle resolution in `sppInitGattHandles`,
`midiInitGattHandles`, `hidDeviceInitGattHandles` must happen in `onSync`, not at
service startup time.

### `current_hid_profile` consistency
`active_hid_rpt_map` and `active_hid_rpt_map_len` must only be updated **after**
`ble_gatts_add_svcs` returns success. If the call fails, the old profile's report map
continues to be served — better than advertising a profile with a mismatched map.

### HID host `hid_key_queue` deletion race
The FreeRTOS queue used for keyboard events must be deleted on the LVGL task, not the
NimBLE host task. The LVGL indev read callback runs on the LVGL task and would access
a deleted queue if it were freed on disconnect from the NimBLE task. Deletion is
dispatched via `lvgl::lock` + LVGL task dispatch.

### Descriptor discovery range bounding
Without `allChrDefHandles`, the descriptor discovery range for the last characteristic
in a service defaults to `hidSvcEnd` (which NimBLE sometimes reports as 65535). This
causes descriptor discovery to sweep far beyond the HID service, picking up unrelated
descriptors. Sorting all def handles after characteristic discovery and computing
`[val_handle+1, next_def_handle-1]` per characteristic avoids this.

### Post-encryption type resolution timeout
After `ble_gap_security_initiate()` succeeds, `BLE_GAP_EVENT_ENC_CHANGE` triggers CCCD
subscriptions. If type resolution (rptRef reads + Report Map parse) has not completed
yet, a 500 ms `hidEncRetryTimer` is started. If the timer fires and
`typeResolutionDone == false` (e.g. due to `BLE_ERR_INV_HCI_CMD_PARMS` preventing
EDONE from firing), type resolution is force-completed and subscriptions proceed anyway.
The `readyBlockFired` flag prevents duplicate execution if the timer fires after
subscriptions are already done.
