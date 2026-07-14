#include <Tactility/lvgl/QuickPanel.h>

#include <Tactility/app/App.h>
#include <Tactility/lvgl/LvglSync.h>
#include <Tactility/lvgl/SliderBox.h>
#include <Tactility/lvgl/Statusbar.h>
#include <Tactility/bluetooth/Bluetooth.h>
#include <Tactility/service/audio/Audio.h>
#include <Tactility/service/wifi/Wifi.h>

#include <tactility/drivers/bluetooth.h>
#include <Tactility/settings/DisplaySettings.h>
#include <Tactility/settings/QuickPanelSettings.h>

#include <Tactility/hal/display/DisplayDevice.h>
#include <tactility/hal/Device.h>
#include <tactility/hal/touch/TouchDevice.h>
#include <tactility/lvgl_fonts.h>
#include <tactility/lvgl_icon_shared.h>
#include <tactility/lvgl_module.h>

#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <vector>

namespace tt::lvgl {

namespace {

constexpr int32_t PRIMARY_TILE_LIMIT = 4;
constexpr int32_t TILES_PER_PAGE = 4;

struct QuickPanelTile {
    const char* id; // stable identifier for persistence, e.g. "mic_on"
    const char* labelOn;
    const char* labelOff;
    const char* icon; // LVGL_ICON_SHARED_* glyph, rendered with the shared icon font; nullptr for none
    const char* longPressAppId = nullptr; // app::start() target on long-press
    std::function<bool()> getState;
    std::function<void(bool)> setState;
    std::function<bool()> isSupported = nullptr; // unset/null treated as always supported
    bool hidden = false; // user-hidden via edit mode; persisted
    lv_obj_t* tileObj = nullptr;
    lv_obj_t* labelObj = nullptr;
    bool wasLongPressed = false;
};

enum class DrawerStage {
    Closed,
    Collapsed, // primary tile grid only
    Expanded   // primary grid + brightness slider + paged secondary tiles + volume sliders
};

constexpr lv_color_t TILE_OFF_COLOR = LV_COLOR_MAKE(0x3a, 0x3a, 0x3e);
constexpr lv_color_t TILE_ON_COLOR = LV_COLOR_MAKE(0x9e, 0xcb, 0xff);
constexpr lv_color_t TILE_EDIT_HIDDEN_COLOR = LV_COLOR_MAKE(0x20, 0x20, 0x22);
constexpr lv_color_t PAGE_DOT_INACTIVE_COLOR = LV_COLOR_MAKE(0x55, 0x55, 0x58);
constexpr lv_color_t PAGE_DOT_ACTIVE_COLOR = LV_COLOR_MAKE(0xe0, 0xe0, 0xe0);

lv_obj_t* dragHandle = nullptr;
lv_obj_t* scrim = nullptr;
lv_obj_t* panel = nullptr;
lv_obj_t* primaryGrid = nullptr;
lv_obj_t* expandedSection = nullptr; // hidden until Expanded; holds the chrome row + the scrollable content below it
lv_obj_t* expandedScroll = nullptr; // scrollable region inside expandedSection: brightness slider, tile pager, volume sliders
lv_obj_t* editButton = nullptr;
lv_obj_t* editButtonLabel = nullptr;
lv_obj_t* closeButton = nullptr;
lv_obj_t* brightnessSlider = nullptr;
lv_obj_t* tilePager = nullptr;
lv_obj_t* pageIndicator = nullptr;
lv_obj_t* inputVolumeSlider = nullptr;
lv_obj_t* outputVolumeSlider = nullptr;
lv_timer_t* brightnessSaveTimer = nullptr; // debounces persisting backlightDuty while dragging; see onBrightnessChanged
DrawerStage stage = DrawerStage::Closed;
bool editMode = false;
// lv_layer_top()'s scrollable flag as it was before quickpanel_create()
// touched it, so quickpanel_destroy() can put it back rather than leaking
// this input-behavior change into whatever else uses the shared top layer
// (screensavers, dialogs, etc.) after the drawer is torn down.
bool layerTopWasScrollableBeforeCreate = true;
// Bumped on every quickpanel_destroy(); lets a still-queued lv_async_call
// (e.g. from onEditButtonClicked) detect that the drawer it captured no longer
// exists and bail out instead of touching freed objects.
uint32_t instanceGeneration = 0;

std::vector<QuickPanelTile> tiles; // order matches persisted/display order

PubSub<service::audio::AudioEvent>::SubscriptionHandle audioSubscription = nullptr;
PubSub<service::wifi::WifiEvent>::SubscriptionHandle wifiSubscription = nullptr;
Device* btEventDevice = nullptr; // BT device we registered our event callback with

void scheduleRefreshTiles(); // fwd decl — defined after the tile/layout builders

bool isWifiOn() {
    auto state = service::wifi::getRadioState();
    return state != service::wifi::RadioState::Off && state != service::wifi::RadioState::OffPending;
}

#ifdef CONFIG_BT_NIMBLE_ENABLED
bool isBtOn() {
    auto state = bluetooth::getRadioState();
    return state == bluetooth::RadioState::On || state == bluetooth::RadioState::OnPending;
}

void setBtEnabled(bool enabled) {
    Device* dev = device_find_first_by_type(&BLUETOOTH_TYPE);
    if (dev == nullptr) return;
    if (enabled) {
        bluetooth::start(dev);
    } else {
        bluetooth::stop(dev);
    }
}

bool isBtSupported() {
    return device_find_first_by_type(&BLUETOOTH_TYPE) != nullptr;
}
#endif

bool hasTouch() {
    auto devices = hal::findDevices<hal::touch::TouchDevice>(hal::Device::Type::Touch);
    for (const auto& device : devices) {
        if (device->getLvglIndev() != nullptr) {
            return true;
        }
    }
    return false;
}

bool isTileSupported(const QuickPanelTile& tile) {
    return !tile.isSupported || tile.isSupported();
}

// Height of the empty strip below the tile grid, reserved as a tile-free grab
// target for the second swipe-down (Collapsed -> Expanded) without fat-
// fingering a toggle tile underneath it. Scaled to a percentage of screen height instead, clamped so
// it's never too cramped to grab on a tiny screen or absurdly tall on a big one.
int32_t grabStripHeight() {
    int32_t screenHeight = lv_display_get_vertical_resolution(nullptr);
    int32_t scaled = screenHeight * 15 / 100;
    if (scaled < 24) {
        return 24;
    }
    if (scaled > 80) {
        return 80;
    }
    return scaled;
}

// Returns `compact` on small/low-resolution screens (device.properties
// uiDensity=compact) and `regular` otherwise. Matches the
// existing convention used throughout the codebase (Statusbar.cpp, button.cpp,
// Launcher.cpp) for scaling padding/gaps down on tiny screens, rather than
// inventing separate percentage-of-screen math for every value - the
// grab-strip (see grabStripHeight()) is the one exception, since it's a
// genuinely proportional concept rather than a density preference.
int32_t densityValue(int32_t compact, int32_t regular) {
    return (lvgl_get_ui_density() == LVGL_UI_DENSITY_COMPACT) ? compact : regular;
}

// True on small absolute resolutions (e.g. lilygo-tdeck's 320x240) regardless of
// the uiDensity flag - that flag is reserved for genuinely tiny screens
// (cardputer-class) and tdeck-class devices intentionally don't set it, but
// their resolution is still small enough that full-size tile content (icon +
// two-line wrapped label at FONT_SIZE_DEFAULT) makes each tile so tall that the
// primary grid alone eats most of the screen, leaving barely anything for the
// rest of the expanded view. Checked against the shorter screen dimension so it
// reads the same in either orientation.
bool isSmallScreen() {
    int32_t w = lv_display_get_horizontal_resolution(nullptr);
    int32_t h = lv_display_get_vertical_resolution(nullptr);
    int32_t shorterSide = (w < h) ? w : h;
    return shorterSide <= 240;
}

void saveTileSettings() {
    settings::quickpanel::QuickPanelSettings settings;
    for (const auto& tile : tiles) {
        settings.tileOrder.emplace_back(tile.id);
        if (tile.hidden) {
            settings.hiddenTileIds.emplace_back(tile.id);
        }
    }
    settings::quickpanel::save(settings);
}

// Applies persisted order/hidden-state to the in-memory tile list. Unknown
// persisted IDs are ignored; tiles missing from the persisted order keep their
// built-in relative order, appended after the ones that were persisted.
void applyTileSettings(const settings::quickpanel::QuickPanelSettings& settings) {
    if (!settings.tileOrder.empty()) {
        std::vector<QuickPanelTile> ordered;
        for (const auto& id : settings.tileOrder) {
            auto it = std::find_if(tiles.begin(), tiles.end(), [&](const QuickPanelTile& tile) {
                return tile.id == id;
            });
            if (it != tiles.end()) {
                ordered.push_back(std::move(*it));
                tiles.erase(it);
            }
        }
        for (auto& remaining : tiles) {
            ordered.push_back(std::move(remaining));
        }
        tiles = std::move(ordered);
    }

    for (auto& tile : tiles) {
        tile.hidden = std::find(settings.hiddenTileIds.begin(), settings.hiddenTileIds.end(), std::string(tile.id))
            != settings.hiddenTileIds.end();
    }
}

// Panel height is normally LV_SIZE_CONTENT (driven by primaryGrid + expandedSection
// visibility) so it naturally fits whatever the current stage shows instead of a
// guessed fraction of the screen. To animate a stage transition smoothly, we
// temporarily pin an explicit pixel height for the duration of the anim, then
// hand control back to LV_SIZE_CONTENT once it settles.
void animatePanelY(int32_t targetY, uint32_t durationMs = 200) {
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, panel);
    lv_anim_set_values(&anim, lv_obj_get_y(panel), targetY);
    lv_anim_set_exec_cb(&anim, [](void* var, int32_t value) {
        lv_obj_set_y(static_cast<lv_obj_t*>(var), value);
    });
    lv_anim_set_duration(&anim, durationMs);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
    lv_anim_start(&anim);
}

void animatePanelHeight(int32_t fromHeight, int32_t toHeight, lv_anim_completed_cb_t completedCb = nullptr) {
    lv_obj_set_height(panel, fromHeight);
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, panel);
    lv_anim_set_values(&anim, fromHeight, toHeight);
    lv_anim_set_exec_cb(&anim, [](void* var, int32_t value) {
        lv_obj_set_height(static_cast<lv_obj_t*>(var), value);
    });
    lv_anim_set_duration(&anim, 200);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
    if (completedCb != nullptr) {
        lv_anim_set_completed_cb(&anim, completedCb);
    }
    lv_anim_start(&anim);
}

// Measure the full natural content height the panel would have for a given
// expandedSection visibility, without leaving that visibility applied. Capped
// to the screen height so panel itself never has to scroll/overflow
// lv_layer_top() - any excess becomes expandedScroll's problem.
//
// expandedSection/expandedScroll normally use flex_grow(1) so they fill
// whatever space panel has left once panel's height is pinned to the capped
// pixel value. But flex_grow has no well-defined meaning while panel's own
// height is LV_SIZE_CONTENT (which is what we set it to here, specifically to
// measure the *natural*, uncapped size) - LVGL's flex resolution for a
// grow-child of a content-sized parent isn't a stable "give me my natural
// size", and was observed to settle at a tiny/near-zero height on repeat opens
// (worked once "by luck" on the very first open, then stuck squashed every
// open after). Fix: temporarily clear flex_grow on both so they fall back to
// real LV_SIZE_CONTENT sizing for this measurement, then restore flex_grow
// for actual capped/animated display.
int32_t measureHeightWithExpandedVisible(bool visible) {
    bool wasHidden = lv_obj_has_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
    if (visible) {
        lv_obj_remove_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
    }

    lv_obj_set_flex_grow(expandedSection, 0);
    lv_obj_set_height(expandedSection, LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(expandedScroll, 0);
    lv_obj_set_height(expandedScroll, LV_SIZE_CONTENT);

    lv_obj_set_height(panel, LV_SIZE_CONTENT);
    lv_obj_update_layout(panel);
    int32_t height = lv_obj_get_height(panel);
    int32_t screenHeight = lv_display_get_vertical_resolution(nullptr);
    if (height > screenHeight) {
        height = screenHeight;
    }

    lv_obj_set_flex_grow(expandedSection, 1);
    lv_obj_set_flex_grow(expandedScroll, 1);

    if (wasHidden) {
        lv_obj_add_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_remove_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
    }
    return height;
}

void onCollapseAnimCompleted(lv_anim_t*) {
    lv_obj_add_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_height(panel, LV_SIZE_CONTENT);
}

void setEditMode(bool enabled); // fwd decl, defined after rebuildTileLayout()

void setStage(DrawerStage newStage) {
    DrawerStage previousStage = stage;
    stage = newStage;

    // Closing the drawer while still in edit mode must commit (save + drop the
    // per-tile edit borders) first - otherwise hidden-tile changes could be
    // lost and the drawer would reopen still showing edit-mode styling. Every
    // close path (swipe-up, scrim-tap, close button) funnels through here.
    if (newStage == DrawerStage::Closed && editMode) {
        setEditMode(false);
    }

    // A rapid second swipe (e.g. Expanded -> Collapsed -> Closed before the first
    // transition's 200ms height animation finished) could leave two animations
    // racing on `panel` at once - the old height anim's exec_cb still firing each
    // frame and fighting whatever the new stage change just set, and its
    // completion callback (onCollapseAnimCompleted) still pending. Cancel
    // anything still running on panel before starting a new transition.
    lv_anim_delete(panel, nullptr);

    // dragHandle's tiny top strip only needs to catch the very first pull
    // (Closed -> Collapsed). Once anything is open, panel itself must be the
    // sole gesture target - LVGL pins a swipe's target to whatever was pressed
    // at touch-down, so if dragHandle stayed clickable while open, any swipe
    // that happened to start near the top of the screen would keep landing on
    // dragHandle (which only ever knows how to do Collapsed) instead of panel
    // (which correctly distinguishes Expanded/Collapsed/Closed). This was why
    // swipes seemed to "stick" at the wrong stage and TOP swipes silently did
    // nothing whenever the press started up near the handle's strip.
    if (newStage == DrawerStage::Closed) {
        lv_obj_add_flag(dragHandle, LV_OBJ_FLAG_CLICKABLE);
    } else {
        lv_obj_remove_flag(dragHandle, LV_OBJ_FLAG_CLICKABLE);
    }

    if (newStage == DrawerStage::Closed) {
        animatePanelY(-lv_obj_get_height(panel));
        lv_obj_add_flag(scrim, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_height(panel, LV_SIZE_CONTENT);
        return;
    }

    lv_obj_remove_flag(scrim, LV_OBJ_FLAG_HIDDEN);

    if (previousStage == DrawerStage::Collapsed && newStage == DrawerStage::Expanded) {
        int32_t fromHeight = lv_obj_get_height(panel);
        int32_t toHeight = measureHeightWithExpandedVisible(true);
        lv_obj_remove_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
        animatePanelHeight(fromHeight, toHeight);
    } else if (previousStage == DrawerStage::Expanded && newStage == DrawerStage::Collapsed) {
        int32_t fromHeight = lv_obj_get_height(panel);
        int32_t toHeight = measureHeightWithExpandedVisible(false);
        animatePanelHeight(fromHeight, toHeight, onCollapseAnimCompleted);
    } else {
        // Opening from Closed: no height animation needed, just slide in.
        lv_obj_add_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_height(panel, LV_SIZE_CONTENT);
    }

    animatePanelY(0);
}

// lv_layer_top() is resized on rotation; while closed, the panel's off-screen
// resting position must track its (rotation-dependent) content height. The
// grab-strip is also screen-size-dependent (see grabStripHeight()), so it needs
// re-applying too - a stale strip from the previous orientation would otherwise
// leave the wrong amount of dead space until the next full create/destroy cycle.
void onLayerTopResized(lv_event_t* event) {
    lv_obj_set_style_pad_bottom(panel, grabStripHeight(), LV_STATE_DEFAULT);
    if (stage == DrawerStage::Closed) {
        lv_obj_set_y(panel, -lv_obj_get_height(panel));
    }
}

void onHandleGesture(lv_event_t* event) {
    auto* indev = lv_indev_active();
    if (indev != nullptr && lv_indev_get_gesture_dir(indev) == LV_DIR_BOTTOM) {
        lv_event_stop_bubbling(event);
        setStage(DrawerStage::Collapsed);
    }
}

// While open, swipe down again to expand, swipe up to step back/close. Only
// fires for drags that started directly on `panel`'s own background (not
// bubbled from tilePager, which handles its own horizontal paging swipes and
// does not have GESTURE_BUBBLE removed, so a horizontal drag there never
// reaches here; a vertical drag inside the pager still bubbles up correctly).
void onPanelGesture(lv_event_t* event) {
    auto* indev = lv_indev_active();
    if (indev == nullptr) {
        return;
    }
    lv_dir_t dir = lv_indev_get_gesture_dir(indev);
    if (dir == LV_DIR_BOTTOM && stage == DrawerStage::Collapsed) {
        lv_event_stop_bubbling(event);
        setStage(DrawerStage::Expanded);
    } else if (dir == LV_DIR_TOP) {
        lv_event_stop_bubbling(event);
        if (stage == DrawerStage::Expanded) {
            setStage(DrawerStage::Collapsed);
        } else {
            setStage(DrawerStage::Closed);
        }
    }
}

void onScrimClicked(lv_event_t* event) {
    // setStage(Closed) commits edit mode (saves + drops edit styling) before
    // closing, so it's safe to close from here regardless of editMode.
    setStage(DrawerStage::Closed);
}

void onTileLongPressed(lv_event_t* event) {
    auto* tile = static_cast<QuickPanelTile*>(lv_event_get_user_data(event));
    tile->wasLongPressed = true;
    if (editMode || tile->longPressAppId == nullptr) {
        return;
    }
    setStage(DrawerStage::Closed);
    app::start(tile->longPressAppId);
}

void updateTileVisual(QuickPanelTile& tile, bool on) {
    if (editMode && tile.hidden) {
        lv_obj_set_style_bg_color(tile.tileObj, TILE_EDIT_HIDDEN_COLOR, LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(tile.tileObj, lv_color_white(), LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(tile.tileObj, LV_OPA_50, LV_STATE_DEFAULT);
    } else {
        lv_obj_set_style_bg_color(tile.tileObj, on ? TILE_ON_COLOR : TILE_OFF_COLOR, LV_STATE_DEFAULT);
        lv_obj_set_style_text_color(tile.tileObj, on ? lv_color_black() : lv_color_white(), LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(tile.tileObj, LV_OPA_COVER, LV_STATE_DEFAULT);
    }
    // In edit mode every tile gets a border, regardless of hidden state,
    // as a visual hint that tiles are tappable-to-hide right now (not obvious
    // otherwise, especially to anyone who hasn't seen this drawer before).
    if (editMode) {
        lv_obj_set_style_border_width(tile.tileObj, 2, LV_STATE_DEFAULT);
        lv_obj_set_style_border_color(tile.tileObj, lv_color_white(), LV_STATE_DEFAULT);
        lv_obj_set_style_border_opa(tile.tileObj, LV_OPA_70, LV_STATE_DEFAULT);
        lv_obj_set_style_border_side(tile.tileObj, LV_BORDER_SIDE_FULL, LV_STATE_DEFAULT);
    } else {
        lv_obj_set_style_border_width(tile.tileObj, 0, LV_STATE_DEFAULT);
    }
    lv_label_set_text(tile.labelObj, on ? tile.labelOn : tile.labelOff);
}

void rebuildTileLayout(); // fwd decl, defined after grid/page builders

void onTileClicked(lv_event_t* event) {
    auto* tile = static_cast<QuickPanelTile*>(lv_event_get_user_data(event));
    if (tile->wasLongPressed) {
        tile->wasLongPressed = false;
        return;
    }
    if (editMode) {
        tile->hidden = !tile->hidden;
        updateTileVisual(*tile, tile->getState());
        return;
    }
    bool newState = !tile->getState();
    tile->setState(newState);
    updateTileVisual(*tile, newState);
}

void updateEditButtonLabel() {
    if (editButtonLabel != nullptr) {
        // Both states ("Done" checkmark and the edit pencil) are Material Symbols
        // glyphs, not regular text - always use the shared icon font. This was
        // backwards before (text font for the checkmark state), which rendered as
        // a tofu/missing-glyph box since the regular text font doesn't have that
        // codepoint at all - looked exactly like a broken font/glyph, but the
        // actual checkmark glyph data was fine all along.
        lv_label_set_text(editButtonLabel, editMode ? LVGL_ICON_SHARED_CHECK : LVGL_ICON_SHARED_EDIT);
        lv_obj_set_style_text_font(editButtonLabel, lvgl_get_shared_icon_font(), LV_PART_MAIN);
    }
}

// Enters/leaves edit mode. Leaving ("Done") persists whatever hidden-state
// changes were made, once, instead of writing to flash on every single tile
// tap. Shared by the explicit edit button and by setStage()'s Closed path, so
// closing the drawer (swipe-up, scrim-tap, close button) while still editing
// commits the change first instead of silently dropping it or leaving the
// drawer stuck in edit mode (with its per-tile borders) on next reopen.
//
// Deferred via lv_async_call: rebuildTileLayout() deletes the tile objects, and
// doing that synchronously inside the same input-event dispatch that triggered
// it corrupts LVGL's indev state (still mid-press, tracking pointers into the
// about-to-be-deleted tree) which froze the whole GUI task in testing.
void setEditMode(bool enabled) {
    if (editMode == enabled) {
        return;
    }
    bool wasEditing = editMode;
    editMode = enabled;
    updateEditButtonLabel();
    if (wasEditing) {
        saveTileSettings();
    }
    // Capture the current instance generation so this callback can detect if
    // quickpanel_destroy() ran before it fires (e.g. the drawer was torn
    // down while this async call was still queued) and bail out instead of
    // touching freed primaryGrid/tilePager/panel objects.
    auto* capturedGeneration = new uint32_t(instanceGeneration);
    lv_async_call([](void* userData) {
        auto* generationPtr = static_cast<uint32_t*>(userData);
        uint32_t capturedGeneration = *generationPtr;
        delete generationPtr;
        if (capturedGeneration != instanceGeneration) {
            return; // drawer was destroyed/recreated since this was queued
        }
        rebuildTileLayout();
        // rebuildTileLayout() can change content height (tiles appear/disappear),
        // but panel's height may currently be pinned to a fixed pixel value left
        // over from the last open/close animation rather than tracking content -
        // re-measure immediately instead of only catching up the next time the
        // drawer is closed and reopened. Goes through the same capped measurement
        // as setStage() so panel can't grow past the screen again here either.
        if (stage == DrawerStage::Expanded) {
            lv_obj_set_height(panel, measureHeightWithExpandedVisible(true));
        } else if (stage == DrawerStage::Collapsed) {
            lv_obj_set_height(panel, LV_SIZE_CONTENT);
        }
    }, capturedGeneration);
}

void onEditButtonClicked(lv_event_t* event) {
    setEditMode(!editMode);
}

// Always-reachable close button in the chrome row (next to edit). Needed
// because expandedScroll is scrollable for overflow content - a swipe starting
// inside scrolled content can't be claimed as the close gesture (LVGL fully
// blocks gesture detection for the whole drag once something claims it as a
// scroll), and scrim-tap doesn't help either once panel covers the whole
// screen. This button works regardless of scroll position or screen size.
void onCloseButtonClicked(lv_event_t* event) {
    setStage(DrawerStage::Closed);
}

void onInputVolumeChanged(lv_event_t* event) {
    auto* sliderBox = static_cast<lv_obj_t*>(lv_event_get_target(event));
    auto percent = static_cast<float>(sliderbox_get_value(sliderBox));
    service::audio::setInputVolume(percent);
}

void onOutputVolumeChanged(lv_event_t* event) {
    auto* sliderBox = static_cast<lv_obj_t*>(lv_event_get_target(event));
    auto percent = static_cast<float>(sliderbox_get_value(sliderBox));
    service::audio::setOutputVolume(percent);
}

void onBrightnessChanged(lv_event_t* event) {
    auto* sliderBox = static_cast<lv_obj_t*>(lv_event_get_target(event));
    auto display = hal::findFirstDevice<hal::display::DisplayDevice>(hal::Device::Type::Display);
    if (display == nullptr) {
        return;
    }
    auto duty = static_cast<uint8_t>(sliderbox_get_value(sliderBox));
    display->setBacklightDuty(duty); // live preview, cheap, fine to call on every drag tick

    // SliderBox's LV_EVENT_VALUE_CHANGED fires continuously while dragging, not
    // just on release, so persisting here directly meant a synchronous flash
    // write on every tick of the drag. Debounce: restart a short one-shot timer
    // on every change, only actually save once dragging has paused. The timer
    // is file-scoped (not a function-local static) so quickpanel_destroy()
    // can cancel it if the drawer closes mid-drag, before it fires against a
    // freed brightnessSlider.
    if (brightnessSaveTimer != nullptr) {
        lv_timer_delete(brightnessSaveTimer);
    }
    brightnessSaveTimer = lv_timer_create([](lv_timer_t*) {
        auto settings = settings::display::loadOrGetDefault();
        settings.backlightDuty = static_cast<uint8_t>(sliderbox_get_value(brightnessSlider));
        settings::display::save(settings);
        brightnessSaveTimer = nullptr;
    }, 300, nullptr);
    lv_timer_set_repeat_count(brightnessSaveTimer, 1);
}

lv_obj_t* createTile(lv_obj_t* parent, QuickPanelTile& tile) {
    bool smallScreen = isSmallScreen();

    auto* tileObj = lv_obj_create(parent);
    lv_obj_set_size(tileObj, LV_PCT(48), LV_SIZE_CONTENT);
    lv_obj_set_style_radius(tileObj, LV_RADIUS_CIRCLE, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(tileObj, densityValue(4, smallScreen ? 6 : 10), LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(tileObj, densityValue(1, smallScreen ? 1 : 2), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(tileObj, 0, LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(tileObj, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(tileObj, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(tileObj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(tileObj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(tileObj, onTileClicked, LV_EVENT_CLICKED, &tile);
    lv_obj_add_event_cb(tileObj, onTileLongPressed, LV_EVENT_LONG_PRESSED, &tile);

    if (tile.icon != nullptr) {
        auto* icon = lv_label_create(tileObj);
        lv_obj_set_style_text_font(icon, lvgl_get_shared_icon_font(), LV_PART_MAIN);
        lv_label_set_text(icon, tile.icon);
    }

    // Small absolute resolution (not just compact density - see isSmallScreen())
    // drops to the smallest label font and clips to one line instead of wrapping
    // to two, since a tall two-line wrapped label at full size is what made
    // primary-grid tiles eat most of a 240px-tall screen.
    auto* label = lv_label_create(tileObj);
    lv_obj_set_style_text_font(label, lvgl_get_text_font(smallScreen ? FONT_SIZE_SMALL : FONT_SIZE_DEFAULT), LV_PART_MAIN);
    lv_label_set_long_mode(label, smallScreen ? LV_LABEL_LONG_MODE_CLIP : LV_LABEL_LONG_MODE_WRAP);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    tile.tileObj = tileObj;
    tile.labelObj = label;
    return tileObj;
}

lv_obj_t* createTileGridContainer(lv_obj_t* parent) {
    auto* grid = lv_obj_create(parent);
    lv_obj_set_size(grid, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(grid, densityValue(4, 8), LV_STATE_DEFAULT);
    lv_obj_set_style_pad_gap(grid, densityValue(4, 8), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(grid, 0, LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(grid, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    return grid;
}

// Rebuilds the page-count dots to match `pageCount`, marking dot 0 active
// (matches tilePager always being scrolled to its first page right after a rebuild).
void rebuildPageIndicator(int32_t pageCount) {
    lv_obj_clean(pageIndicator);
    if (pageCount <= 1) {
        lv_obj_add_flag(pageIndicator, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_remove_flag(pageIndicator, LV_OBJ_FLAG_HIDDEN);
    for (int32_t i = 0; i < pageCount; ++i) {
        auto* dot = lv_obj_create(pageIndicator);
        lv_obj_remove_style_all(dot);
        lv_obj_set_size(dot, 6, 6);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(dot, i == 0 ? PAGE_DOT_ACTIVE_COLOR : PAGE_DOT_INACTIVE_COLOR, LV_STATE_DEFAULT);
    }
}

// Recolors the dots to mark whichever page tilePager is currently scrolled to.
void onTilePagerScroll(lv_event_t* event) {
    int32_t pageWidth = lv_obj_get_width(tilePager);
    if (pageWidth <= 0) {
        return;
    }
    int32_t currentPage = (lv_obj_get_scroll_x(tilePager) + pageWidth / 2) / pageWidth;
    uint32_t dotCount = lv_obj_get_child_count(pageIndicator);
    for (uint32_t i = 0; i < dotCount; ++i) {
        auto* dot = lv_obj_get_child(pageIndicator, static_cast<int32_t>(i));
        lv_obj_set_style_bg_color(dot, static_cast<int32_t>(i) == currentPage ? PAGE_DOT_ACTIVE_COLOR : PAGE_DOT_INACTIVE_COLOR, LV_STATE_DEFAULT);
    }
}

lv_obj_t* createVolumeRow(lv_obj_t* parent, const char* label) {
    auto* row = lv_obj_create(parent);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(row, 0, LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(row, 0, LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    auto* row_label = lv_label_create(row);
    lv_label_set_text(row_label, label);

    return row;
}

// (Re)builds primaryGrid and tilePager contents from the current `tiles` vector +
// editMode + hidden state. Editing/reordering/hide-toggling all funnel through
// this rather than trying to animate incremental moves.
void rebuildTileLayout() {
    lv_obj_clean(primaryGrid);
    lv_obj_clean(tilePager);

    // lv_obj_clean() above just freed every existing tile/label object. Any tile
    // that doesn't get recreated below (filtered out, e.g. hidden-and-not-in-edit-
    // mode) must have its pointers nulled here, otherwise it's left holding a
    // dangling pointer that still passes a "!= nullptr" check elsewhere
    // (refreshTiles(), called from the audio/wifi pubsub handlers) and gets
    // dereferenced -> use-after-free.
    for (auto& tile : tiles) {
        tile.tileObj = nullptr;
        tile.labelObj = nullptr;
    }

    std::vector<QuickPanelTile*> visibleTiles;
    for (auto& tile : tiles) {
        if (!isTileSupported(tile)) {
            continue;
        }
        if (!editMode && tile.hidden) {
            continue;
        }
        visibleTiles.push_back(&tile);
    }

    int32_t primaryCount = std::min<int32_t>(PRIMARY_TILE_LIMIT, static_cast<int32_t>(visibleTiles.size()));
    for (int32_t i = 0; i < primaryCount; ++i) {
        createTile(primaryGrid, *visibleTiles[static_cast<size_t>(i)]);
    }

    lv_obj_t* currentPage = nullptr;
    int32_t pageCount = 0;
    for (size_t i = static_cast<size_t>(primaryCount); i < visibleTiles.size(); ++i) {
        if ((i - static_cast<size_t>(primaryCount)) % TILES_PER_PAGE == 0) {
            currentPage = createTileGridContainer(tilePager);
            lv_obj_set_width(currentPage, LV_PCT(100));
            ++pageCount;
        }
        createTile(currentPage, *visibleTiles[i]);
    }

    // Only let tilePager claim horizontal scroll/gesture when there's actually
    // more than one page to swipe between. With exactly one (or zero) pages,
    // leaving it scrollable risks LVGL provisionally claiming a vertical
    // open/close swipe as a horizontal scroll attempt before the direction
    // restriction kicks in, which silently ate the close gesture in testing
    // whenever a single secondary tile (e.g. Wi-Fi) was present.
    if (pageCount > 1) {
        lv_obj_add_flag(tilePager, LV_OBJ_FLAG_SCROLLABLE);
    } else {
        lv_obj_remove_flag(tilePager, LV_OBJ_FLAG_SCROLLABLE);
    }
    rebuildPageIndicator(pageCount);

    for (auto& tile : tiles) {
        if (tile.tileObj != nullptr) {
            updateTileVisual(tile, tile.getState());
        }
    }
}

void refreshTiles() {
    for (auto& tile : tiles) {
        if (tile.tileObj == nullptr) {
            continue;
        }
        updateTileVisual(tile, tile.getState());
    }

    if (inputVolumeSlider != nullptr) {
        sliderbox_set_value(inputVolumeSlider, static_cast<int32_t>(service::audio::getInputVolume()), LV_ANIM_OFF);
    }
    if (outputVolumeSlider != nullptr) {
        sliderbox_set_value(outputVolumeSlider, static_cast<int32_t>(service::audio::getOutputVolume()), LV_ANIM_OFF);
    }
}

// Deferred tile refresh: scheduled via lv_async_call so it runs on the LVGL
// task's next timer tick, outside of any pubsub or AudioService mutex hold.
// Direct call from pubsub callbacks risks an ABBA deadlock:
//   pubsub->publish() holds PubSub::mutex, then onAudioEvent calls
//   tile.getState() which locks AudioService::mutex; meanwhile the persist
//   timer holds AudioService::mutex and calls pubsub->publish() which waits
//   for PubSub::mutex — classic lock-order inversion that freezes both tasks.
// Deferring via lv_async_call breaks the chain: the callback fires after
// PubSub::mutex is released, so both mutexes are always acquired in the
// same order (AudioService first, then PubSub for publish, never nested).
void scheduleRefreshTiles() {
    auto* capturedGeneration = new uint32_t(instanceGeneration);
    // lv_async_call touches LVGL's timer list; must hold the LVGL lock.
    // From the LVGL task (e.g. fired via setState → pubsub) the recursive
    // mutex already covers this; from external tasks (persist timer, BT/wifi
    // event tasks) we take it here before calling in.
    if (lock(defaultLockTime)) {
        lv_async_call([](void* userData) {
            auto* generationPtr = static_cast<uint32_t*>(userData);
            uint32_t captured = *generationPtr;
            delete generationPtr;
            if (captured != instanceGeneration) {
                return;
            }
            refreshTiles();
        }, capturedGeneration);
        unlock();
    } else {
        delete capturedGeneration;
    }
}

void onAudioEvent(service::audio::AudioEvent) {
    scheduleRefreshTiles();
}

void onWifiEvent(service::wifi::WifiEvent) {
    scheduleRefreshTiles();
}

#ifdef CONFIG_BT_NIMBLE_ENABLED
void onBtEvent(Device*, void*, BtEvent event) {
    if (event.type == BT_EVENT_RADIO_STATE_CHANGED) {
        scheduleRefreshTiles();
    }
}
#endif

} // namespace

void quickpanel_create() {
    if (!hasTouch()) {
        return;
    }

    // lv_layer_top() itself defaults to LV_OBJ_FLAG_SCROLLABLE=true (LVGL core only
    // ever disables CLICKABLE on it, never SCROLLABLE - see lv_display.c's
    // lv_display_create()). Scrollable doesn't require the object itself to be
    // clickable - a drag starting on any clickable child (panel, in our case)
    // that isn't itself scrollable still gets walked up to the nearest scrollable
    // ancestor, which was top_layer. Since panel's content can be taller than the
    // screen once expanded, that meant the open/close swipe was being consumed as
    // a literal scroll of the whole top layer instead of ever reaching panel's
    // own gesture handler.
    // Save the prior state so quickpanel_destroy() can restore it.
    layerTopWasScrollableBeforeCreate = lv_obj_has_flag(lv_layer_top(), LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(lv_layer_top(), LV_OBJ_FLAG_SCROLLABLE);

    // Drag handle: thin strip over the statusbar that catches the pull-down gesture.
    // Lives on lv_layer_top() (not as a flex sibling of the statusbar) so it overlays
    // without affecting vertical_container's column layout.
    dragHandle = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(dragHandle);
    lv_obj_set_size(dragHandle, LV_PCT(100), statusbar_get_height());
    lv_obj_set_pos(dragHandle, 0, 0);
    lv_obj_remove_flag(dragHandle, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(dragHandle, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_flag(dragHandle, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(dragHandle, onHandleGesture, LV_EVENT_GESTURE, nullptr);

    // Scrim: full-screen tap-outside-to-close target, behind the panel
    scrim = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(scrim);
    lv_obj_set_size(scrim, LV_PCT(100), LV_PCT(100));
    lv_obj_set_pos(scrim, 0, 0);
    lv_obj_set_style_bg_color(scrim, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(scrim, LV_OPA_50, LV_STATE_DEFAULT);
    lv_obj_remove_flag(scrim, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(scrim, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(scrim, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(scrim, onScrimClicked, LV_EVENT_CLICKED, nullptr);

    // Panel: drawer content, slides down from above the screen. Height is
    // LV_SIZE_CONTENT so it naturally fits whichever stage's content is visible
    // (collapsed: primary tile grid only; expanded: + brightness + paged tiles +
    // volume sliders) instead of a guessed fraction of the screen. Initial
    // off-screen Y is set after children exist below, once the content height
    // is known.
    panel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(panel, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_pos(panel, 0, 0);
    lv_obj_set_style_border_width(panel, 0, LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(panel, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(panel, LV_OPA_80, LV_STATE_DEFAULT);
    // Extra bottom padding gives a tile-free strip to grab for the second swipe-down
    // (collapsed -> expanded) without fat-fingering a toggle tile underneath it.
    // Scaled to screen size - see grabStripHeight().
    lv_obj_set_style_pad_bottom(panel, grabStripHeight(), LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_event_cb(panel, onPanelGesture, LV_EVENT_GESTURE, nullptr);
    lv_obj_add_event_cb(lv_layer_top(), onLayerTopResized, LV_EVENT_SIZE_CHANGED, nullptr);

    // Primary tile grid: always visible (both Collapsed and Expanded), capped at
    // PRIMARY_TILE_LIMIT tiles.
    primaryGrid = createTileGridContainer(panel);

    // Expanded-only content: a fixed chrome row (edit/close buttons, never
    // scrolls, always reachable) + a scrollable region below it holding
    // whatever doesn't fit (brightness slider, paged secondary tiles, volume
    // sliders). Capped to fit the screen - see the height cap applied in
    // setStage()'s Expanded branch - so expandedScroll, not panel or
    // lv_layer_top(), is the only thing that ever scrolls for overflow.
    expandedSection = lv_obj_create(panel);
    lv_obj_set_size(expandedSection, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(expandedSection, 1); // fills whatever space panel has left once panel's height is capped
    lv_obj_set_style_pad_all(expandedSection, 0, LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(expandedSection, 0, LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(expandedSection, LV_FLEX_FLOW_COLUMN);
    lv_obj_remove_flag(expandedSection, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(expandedSection, LV_OBJ_FLAG_HIDDEN);

    // Chrome row: edit + close buttons, pinned to opposite ends (SPACE_BETWEEN)
    // rather than bunched together - clearer separation between "secondary/
    // editing action" and "dismiss", and frees up the middle of the row. Fixed,
    // never part of the scrollable area, so both stay reachable regardless of
    // content overflow or screen size.
    auto* chromeRow = lv_obj_create(expandedSection);
    lv_obj_set_size(chromeRow, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(chromeRow, densityValue(4, 8), LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(chromeRow, densityValue(4, 8), LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(chromeRow, 0, LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(chromeRow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(chromeRow, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(chromeRow, LV_OBJ_FLAG_SCROLLABLE);

    // Chrome buttons are sized off the icon font height (not LVGL's default
    // button padding, which is generous/desktop-sized) so they scale with the
    // same per-device icon size everything else uses, and shrink further on
    // small screens via isSmallScreen() - on a 240px-tall panel, full-size
    // buttons alone were eating a large chunk of the already-cramped expanded view.
    int32_t chromeButtonSize = lvgl_get_shared_icon_font_height() + (isSmallScreen() ? 8 : 16);

    editButton = lv_button_create(chromeRow);
    lv_obj_set_size(editButton, chromeButtonSize, chromeButtonSize);
    lv_obj_set_style_pad_all(editButton, 0, LV_STATE_DEFAULT);
    lv_obj_remove_flag(editButton, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(editButton, onEditButtonClicked, LV_EVENT_CLICKED, nullptr);

    editButtonLabel = lv_label_create(editButton);
    lv_obj_center(editButtonLabel);
    updateEditButtonLabel();

    closeButton = lv_button_create(chromeRow);
    lv_obj_set_size(closeButton, chromeButtonSize, chromeButtonSize);
    lv_obj_set_style_pad_all(closeButton, 0, LV_STATE_DEFAULT);
    lv_obj_remove_flag(closeButton, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(closeButton, onCloseButtonClicked, LV_EVENT_CLICKED, nullptr);

    auto* closeButtonLabel = lv_label_create(closeButton);
    lv_obj_set_style_text_font(closeButtonLabel, lvgl_get_shared_icon_font(), LV_PART_MAIN);
    lv_label_set_text(closeButtonLabel, LVGL_ICON_SHARED_CLOSE);
    lv_obj_center(closeButtonLabel);

    // Scrollable region: everything that might not fit (brightness, pager,
    // volume sliders). This - not panel, not lv_layer_top() - is the only thing
    // allowed to scroll, so an overflowing layout never blocks the swipe-to-close
    // gesture for presses that start above it (chromeRow, primaryGrid); presses
    // starting inside it use the close button instead, since LVGL fully blocks
    // gesture detection for the whole drag once something claims it as a scroll.
    expandedScroll = lv_obj_create(expandedSection);
    lv_obj_set_size(expandedScroll, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_grow(expandedScroll, 1); // fills whatever space expandedSection has left under chromeRow
    lv_obj_set_style_pad_all(expandedScroll, 0, LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(expandedScroll, 0, LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(expandedScroll, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(expandedScroll, LV_DIR_VER);

    auto display = hal::findFirstDevice<hal::display::DisplayDevice>(hal::Device::Type::Display);
    if (display != nullptr && display->supportsBacklightDuty()) {
        auto* brightness_row = createVolumeRow(expandedScroll, "Brightness");
        auto displaySettings = settings::display::loadOrGetDefault();
        brightnessSlider = sliderbox_create(brightness_row, 0, 255, 16, displaySettings.backlightDuty);
        sliderbox_add_value_changed_cb(brightnessSlider, onBrightnessChanged, nullptr);
    }

    // Tile pager: secondary tiles (beyond PRIMARY_TILE_LIMIT), paged horizontally.
    // Lives inside expandedScroll (which scrolls vertically); the pager's own
    // horizontal scroll-snap is a different axis so the two don't fight.
    tilePager = lv_obj_create(expandedScroll);
    lv_obj_set_size(tilePager, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(tilePager, 0, LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(tilePager, 0, LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(tilePager, LV_FLEX_FLOW_ROW);
    lv_obj_set_scroll_snap_x(tilePager, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scroll_dir(tilePager, LV_DIR_HOR);
    lv_obj_add_flag(tilePager, LV_OBJ_FLAG_SCROLL_ONE);
    lv_obj_add_event_cb(tilePager, onTilePagerScroll, LV_EVENT_SCROLL, nullptr);

    // Page indicator: small dots below the pager, one per page.
    // Hidden when there's 0-1 pages (rebuildPageIndicator
    // controls this each time tiles change).
    pageIndicator = lv_obj_create(expandedScroll);
    lv_obj_remove_style_all(pageIndicator);
    lv_obj_set_size(pageIndicator, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(pageIndicator, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(pageIndicator, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(pageIndicator, 6, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(pageIndicator, 4, LV_STATE_DEFAULT);
    lv_obj_remove_flag(pageIndicator, LV_OBJ_FLAG_SCROLLABLE);

    tiles.clear();
    tiles.push_back({
        .id = "wifi",
        .labelOn = "Wi-Fi On",
        .labelOff = "Wi-Fi Off",
        .icon = LVGL_ICON_SHARED_WIFI,
        .longPressAppId = "WifiManage",
        .getState = isWifiOn,
        .setState = service::wifi::setEnabled
    });

#ifdef CONFIG_BT_NIMBLE_ENABLED
    tiles.push_back({
        .id = "bluetooth",
        .labelOn = "BT On",
        .labelOff = "BT Off",
        .icon = LVGL_ICON_SHARED_BLUETOOTH,
        .longPressAppId = "BtManage",
        .getState = isBtOn,
        .setState = setBtEnabled,
        .isSupported = isBtSupported
    });
#endif
    tiles.push_back({
        .id = "speaker_mute",
        .labelOn = "Speaker Muted",
        .labelOff = "Speaker Mute",
        .icon = LVGL_ICON_SHARED_VOLUME_OFF,
        .longPressAppId = "AudioSettings",
        .getState = service::audio::isOutputMuted,
        .setState = service::audio::setOutputMuted,
        .isSupported = service::audio::isOutputAvailable
    });
    tiles.push_back({
        .id = "mic_mute",
        .labelOn = "Mic Muted",
        .labelOff = "Mic Mute",
        .icon = LVGL_ICON_SHARED_MIC_OFF,
        .longPressAppId = "AudioSettings",
        .getState = service::audio::isInputMuted,
        .setState = service::audio::setInputMuted,
        .isSupported = service::audio::isInputAvailable
    });
    tiles.push_back({
        .id = "speaker_on",
        .labelOn = "Speaker On",
        .labelOff = "Speaker Off",
        .icon = LVGL_ICON_SHARED_VOLUME_UP,
        .longPressAppId = "AudioSettings",
        .getState = service::audio::isOutputEnabled,
        .setState = service::audio::setOutputEnabled,
        .isSupported = service::audio::isOutputAvailable
    });
    tiles.push_back({
        .id = "mic_on",
        .labelOn = "Mic On",
        .labelOff = "Mic Off",
        .icon = LVGL_ICON_SHARED_MIC,
        .longPressAppId = "AudioSettings",
        .getState = service::audio::isInputEnabled,
        .setState = service::audio::setInputEnabled,
        .isSupported = service::audio::isInputAvailable
    });

    // TODO: throwaway test tiles to exercise multi-page paging (>1 page); remove
    // once real secondary tiles (Bluetooth, GPS, etc.) make this unnecessary.
    static bool testTileState[4] = {};
    static const std::array<std::string, 4> testTileIds = {
        "test_0", "test_1", "test_2", "test_3"
    };
    for (int i = 0; i < 4; ++i) {
        tiles.push_back({
            .id = testTileIds[static_cast<size_t>(i)].c_str(),
            .labelOn = "Test On",
            .labelOff = "Test Off",
            .icon = LVGL_ICON_SHARED_CIRCLE,
            .getState = [i]{ return testTileState[i]; },
            .setState = [i](bool v){ testTileState[i] = v; }
        });
    }

    applyTileSettings(settings::quickpanel::loadOrGetDefault());
    rebuildTileLayout();

    // Volume sliders: appended after the tile pager, inside expandedScroll. Gated
    // per-direction (not the blanket isAvailable()) so a mic-only or speaker-only
    // device doesn't show a dead slider for the direction it doesn't have.
    if (service::audio::isInputAvailable()) {
        auto* input_volume_row = createVolumeRow(expandedScroll, "Microphone Volume");
        inputVolumeSlider = sliderbox_create(input_volume_row, 0, 100, 10, static_cast<int32_t>(service::audio::getInputVolume()));
        sliderbox_add_value_changed_cb(inputVolumeSlider, onInputVolumeChanged, nullptr);
    }

    if (service::audio::isOutputAvailable()) {
        auto* output_volume_row = createVolumeRow(expandedScroll, "Speaker Volume");
        outputVolumeSlider = sliderbox_create(output_volume_row, 0, 100, 10, static_cast<int32_t>(service::audio::getOutputVolume()));
        sliderbox_add_value_changed_cb(outputVolumeSlider, onOutputVolumeChanged, nullptr);
    }

    refreshTiles();

    // Force a layout pass so panel's LV_SIZE_CONTENT height is known, then park it off-screen.
    lv_obj_update_layout(panel);
    lv_obj_set_y(panel, -lv_obj_get_height(panel));

    audioSubscription = service::audio::getPubsub()->subscribe(onAudioEvent);
    wifiSubscription = service::wifi::getPubsub()->subscribe(onWifiEvent);
#ifdef CONFIG_BT_NIMBLE_ENABLED
    btEventDevice = device_find_first_by_type(&BLUETOOTH_TYPE);
    if (btEventDevice != nullptr) {
        bluetooth_add_event_callback(btEventDevice, nullptr, onBtEvent);
    }
#endif
}

void quickpanel_destroy() {
    // Invalidate any in-flight lv_async_call queued by onEditButtonClicked - it
    // checks this against what it captured and no-ops instead of touching the
    // objects we're about to delete below.
    ++instanceGeneration;

    // Put lv_layer_top()'s scrollable flag back the way quickpanel_create()
    // found it, so this drawer's input-behavior tweak doesn't leak into other
    // code that shares the same top layer after it's torn down.
    if (layerTopWasScrollableBeforeCreate) {
        lv_obj_add_flag(lv_layer_top(), LV_OBJ_FLAG_SCROLLABLE);
    } else {
        lv_obj_remove_flag(lv_layer_top(), LV_OBJ_FLAG_SCROLLABLE);
    }

    if (brightnessSaveTimer != nullptr) {
        lv_timer_delete(brightnessSaveTimer);
        brightnessSaveTimer = nullptr;
    }

    if (audioSubscription != nullptr) {
        service::audio::getPubsub()->unsubscribe(audioSubscription);
        audioSubscription = nullptr;
    }
    if (wifiSubscription != nullptr) {
        service::wifi::getPubsub()->unsubscribe(wifiSubscription);
        wifiSubscription = nullptr;
    }
#ifdef CONFIG_BT_NIMBLE_ENABLED
    if (btEventDevice != nullptr) {
        bluetooth_remove_event_callback(btEventDevice, onBtEvent);
        btEventDevice = nullptr;
    }
#endif

    tiles.clear();
    inputVolumeSlider = nullptr;
    outputVolumeSlider = nullptr;
    brightnessSlider = nullptr;

    if (dragHandle != nullptr) {
        lv_obj_delete(dragHandle);
    }
    if (scrim != nullptr) {
        lv_obj_delete(scrim);
    }
    if (panel != nullptr) {
        lv_obj_remove_event_cb(lv_layer_top(), onLayerTopResized);
        lv_obj_delete(panel);
    }

    dragHandle = nullptr;
    scrim = nullptr;
    panel = nullptr;
    primaryGrid = nullptr;
    expandedSection = nullptr;
    expandedScroll = nullptr;
    editButton = nullptr;
    editButtonLabel = nullptr;
    closeButton = nullptr;
    tilePager = nullptr;
    pageIndicator = nullptr;
    stage = DrawerStage::Closed;
    editMode = false;
}

} // namespace tt::lvgl
