// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <tactility/module.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decoded USB MIDI message.
 *
 * `status` encodes both message type and channel:
 *   type    = status & 0xF0  (0x80=NoteOff, 0x90=NoteOn, 0xB0=CC, 0xC0=PC, 0xE0=PitchBend, ...)
 *   channel = status & 0x0F  (0–15)
 *
 * For System Common / Realtime messages (status >= 0xF0) the channel nibble has no meaning.
 */
typedef struct {
    uint8_t cable;   /**< USB cable number (0–15, almost always 0) */
    uint8_t status;  /**< MIDI status byte */
    uint8_t data1;   /**< First data byte  */
    uint8_t data2;   /**< Second data byte */
} usb_midi_message_t;

/** Callback invoked (from the MIDI client task) for every inbound MIDI message. */
typedef void (*usb_midi_message_cb_t)(const usb_midi_message_t* msg, void* user_data);

/**
 * @brief Register a callback for incoming MIDI messages.
 *
 * Replaces any previously registered callback. Pass NULL to disable.
 * @param callback  Function to call on each message.
 * @param user_data Opaque pointer forwarded to the callback unchanged.
 */
void usb_midi_set_callback(usb_midi_message_cb_t callback, void* user_data);

/** @return true if a MIDI device is currently connected and streaming. */
bool usb_midi_is_connected(void);

extern struct Module usb_host_midi_module;

#ifdef __cplusplus
}
#endif
