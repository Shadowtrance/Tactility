# usb-host-midi-module

USB MIDI 1.0 class driver. Receives MIDI messages from any USB MIDI device and dispatches them via a callback.

## What it does

- Registers a raw USB host client (no extra managed component needed — uses the built-in `usb` component)
- On device connect: scans the configuration descriptor for an Audio class / MIDI Streaming subclass interface (bInterfaceClass=0x01, bInterfaceSubClass=0x03) and its bulk or interrupt IN endpoint
- Submits recurring bulk transfers to read USB MIDI Event Packets (4 bytes each) from the device
- Decodes and dispatches each packet via the registered callback
- On disconnect: releases the interface and closes the device cleanly

## API

```c
#include <usb_host_midi_module.h>

// Register a callback (call before or after module start)
usb_midi_set_callback(my_callback, my_context);

// In the callback:
void my_callback(const usb_midi_message_t* msg, void* ctx) {
    uint8_t type    = msg->status & 0xF0;  // 0x80=NoteOff, 0x90=NoteOn, 0xB0=CC, 0xE0=PitchBend
    uint8_t channel = msg->status & 0x0F;  // 0–15
    // msg->data1, msg->data2 — note/controller/value bytes
}
```

The callback runs on the `midi_client` FreeRTOS task. Keep it short or post to a queue.

## Dependencies

```yaml
dependencies:
  - Drivers/usb-host-module
```

Only `TactilityKernel` and `usb` are required in CMake.

## Notes

- Supports one MIDI device at a time. Multiple simultaneous devices are not currently handled.
- Does not claim the Audio Control interface (subclass 0x01) — only the MIDI Streaming interface (subclass 0x03). This is sufficient for receiving notes, CC, pitch bend, and other channel messages.
- System Exclusive (SysEx) messages arrive as raw packets with CIN 0x04–0x07; they are dispatched as-is without reassembly.
- MIDI devices are FS (Full Speed). They work when connected directly to the host port. Through a hub, they will fail until ESP-IDF implements split transaction support (IDF-10023).
