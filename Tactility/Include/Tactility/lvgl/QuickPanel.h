#pragma once

#include <lvgl.h>

namespace tt::lvgl {

/**
 * Create the quick-panel drag handle and drawer, parented to lv_layer_top() so
 * they float above whatever app is currently showing.
 * No-ops if no pointer (touch) input device is present.
 * Needs to be called with the LVGL lock held.
 */
void quickpanel_create();

/** Tear down the quick-panel drawer and unsubscribe from events. */
void quickpanel_destroy();

} // namespace tt::lvgl
