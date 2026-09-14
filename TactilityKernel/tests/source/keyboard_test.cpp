#include "doctest.h"

#include <tactility/drivers/keyboard.h>

TEST_CASE("keyboard_key_from_hid_usage maps letters and digits") {
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x04, false, false), (uint32_t)'a');
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x1D, false, false), (uint32_t)'z');
    CHECK_EQ(keyboard_key_from_hid_usage(KEYBOARD_HID_MOD_LEFT_SHIFT, 0x04, false, false), (uint32_t)'A');
    // Right Shift behaves like Left Shift
    CHECK_EQ(keyboard_key_from_hid_usage(KEYBOARD_HID_MOD_RIGHT_SHIFT, 0x04, false, false), (uint32_t)'A');
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x1E, false, false), (uint32_t)'1');
    CHECK_EQ(keyboard_key_from_hid_usage(KEYBOARD_HID_MOD_LEFT_SHIFT, 0x1E, false, false), (uint32_t)'!');
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x2C, false, false), (uint32_t)' ');
}

TEST_CASE("keyboard_key_from_hid_usage applies caps lock to letters only") {
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x04, true, false), (uint32_t)'A');
    // Caps Lock + Shift on a letter cancels out
    CHECK_EQ(keyboard_key_from_hid_usage(KEYBOARD_HID_MOD_LEFT_SHIFT, 0x04, true, false), (uint32_t)'a');
    // Caps Lock does not affect non-letters
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x1E, true, false), (uint32_t)'1');
}

TEST_CASE("keyboard_key_from_hid_usage maps control keys to CodePoint values") {
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x28, false, false), (uint32_t)CODEPOINT_ENTER);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x29, false, false), (uint32_t)CODEPOINT_ESCAPE);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x2A, false, false), (uint32_t)CODEPOINT_BACKSPACE);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x2B, false, false), (uint32_t)CODEPOINT_TAB);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x4C, false, false), (uint32_t)CODEPOINT_DELETE);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x52, false, false), (uint32_t)CODEPOINT_ARROW_UP);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x51, false, false), (uint32_t)CODEPOINT_ARROW_DOWN);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x50, false, false), (uint32_t)CODEPOINT_ARROW_LEFT);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x4F, false, false), (uint32_t)CODEPOINT_ARROW_RIGHT);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x4A, false, false), (uint32_t)CODEPOINT_HOME);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x4D, false, false), (uint32_t)CODEPOINT_END);
}

TEST_CASE("keyboard_key_from_hid_usage maps keypad with num lock") {
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x62, false, true), (uint32_t)'0');
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x59, false, true), (uint32_t)'1');
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x57, false, false), (uint32_t)'+');
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x58, false, false), (uint32_t)CODEPOINT_ENTER);
    // Without num lock the keypad doubles as navigation
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x62, false, false), 0u);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x5A, false, false), (uint32_t)CODEPOINT_ARROW_DOWN);
}

TEST_CASE("keyboard_key_from_hid_usage does not suppress ctrl/alt chords") {
    CHECK_EQ(keyboard_key_from_hid_usage(KEYBOARD_HID_MOD_LEFT_CTRL, 0x06, false, false), (uint32_t)'c');
    CHECK_EQ(keyboard_key_from_hid_usage(KEYBOARD_HID_MOD_LEFT_ALT, 0x06, false, false), (uint32_t)'c');
}

TEST_CASE("keyboard_key_from_hid_usage returns 0 for unmapped keys") {
    // Reserved usage and function keys have no codepoint mapping here
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x00, false, false), 0u);
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0x3A, false, false), 0u); // F1
    CHECK_EQ(keyboard_key_from_hid_usage(0, 0xE0, false, false), 0u); // Left Ctrl itself
}
