#include <Tactility/Tactility.h>

#include <tactility/lvgl_icon_shared.h>

#include <Tactility/Logger.h>
#include <Tactility/app/App.h>
#include <Tactility/lvgl/Toolbar.h>
#include <Tactility/service/audio/Audio.h>

#include <lvgl.h>
#include <tactility/lvgl_module.h>

namespace tt::app::audiosettings {

static const auto LOGGER = Logger("AudioSettings");

class AudioSettingsApp final : public App {

    lv_obj_t* inputEnabledSwitch = nullptr;
    lv_obj_t* inputMuteSwitch = nullptr;
    lv_obj_t* inputVolumeSlider = nullptr;

    lv_obj_t* outputEnabledSwitch = nullptr;
    lv_obj_t* outputMuteSwitch = nullptr;
    lv_obj_t* outputVolumeSlider = nullptr;

    static void onInputEnabledSwitch(lv_event_t* event) {
        auto* sw = static_cast<lv_obj_t*>(lv_event_get_target(event));
        bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
        service::audio::setInputEnabled(enabled);
    }

    static void onOutputEnabledSwitch(lv_event_t* event) {
        auto* sw = static_cast<lv_obj_t*>(lv_event_get_target(event));
        bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
        service::audio::setOutputEnabled(enabled);
    }

    static void onInputMuteSwitch(lv_event_t* event) {
        auto* sw = static_cast<lv_obj_t*>(lv_event_get_target(event));
        bool muted = lv_obj_has_state(sw, LV_STATE_CHECKED);
        service::audio::setInputMuted(muted);
    }

    static void onOutputMuteSwitch(lv_event_t* event) {
        auto* sw = static_cast<lv_obj_t*>(lv_event_get_target(event));
        bool muted = lv_obj_has_state(sw, LV_STATE_CHECKED);
        service::audio::setOutputMuted(muted);
    }

    static void onInputVolumeSlider(lv_event_t* event) {
        auto* slider = static_cast<lv_obj_t*>(lv_event_get_target(event));
        float percent = static_cast<float>(lv_slider_get_value(slider));
        service::audio::setInputVolume(percent);
    }

    static void onOutputVolumeSlider(lv_event_t* event) {
        auto* slider = static_cast<lv_obj_t*>(lv_event_get_target(event));
        float percent = static_cast<float>(lv_slider_get_value(slider));
        service::audio::setOutputVolume(percent);
    }

    static lv_obj_t* createSection(lv_obj_t* parent, const char* title) {
        auto* wrapper = lv_obj_create(parent);
        lv_obj_set_size(wrapper, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(wrapper, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_hor(wrapper, 0, LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(wrapper, 0, LV_STATE_DEFAULT);

        auto* title_label = lv_label_create(wrapper);
        lv_label_set_text(title_label, title);

        return wrapper;
    }

    static lv_obj_t* createSwitchRow(lv_obj_t* parent, const char* label, lv_event_cb_t cb, void* userData) {
        auto* row = lv_obj_create(parent);
        lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_pad_all(row, 0, LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(row, 0, LV_STATE_DEFAULT);

        auto* row_label = lv_label_create(row);
        lv_label_set_text(row_label, label);
        lv_obj_align(row_label, LV_ALIGN_LEFT_MID, 0, 0);

        auto* sw = lv_switch_create(row);
        lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, userData);

        return sw;
    }

    static lv_obj_t* createSliderRow(lv_obj_t* parent, const char* label, lv_event_cb_t cb, void* userData) {
        auto* row = lv_obj_create(parent);
        lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);
        lv_obj_set_style_pad_all(row, 0, LV_STATE_DEFAULT);
        lv_obj_set_style_border_width(row, 0, LV_STATE_DEFAULT);

        auto* row_label = lv_label_create(row);
        lv_label_set_text(row_label, label);
        lv_obj_align(row_label, LV_ALIGN_LEFT_MID, 0, 0);

        auto* slider = lv_slider_create(row);
        lv_obj_set_width(slider, LV_PCT(50));
        lv_obj_align(slider, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_slider_set_range(slider, 0, 100);
        lv_obj_add_event_cb(slider, cb, LV_EVENT_RELEASED, userData);

        return slider;
    }

public:

    void onShow(AppContext& app, lv_obj_t* parent) override {
        lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(parent, 0, LV_STATE_DEFAULT);

        lvgl::toolbar_create(parent, app);

        auto* main_wrapper = lv_obj_create(parent);
        lv_obj_set_flex_flow(main_wrapper, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_width(main_wrapper, LV_PCT(100));
        lv_obj_set_flex_grow(main_wrapper, 1);

        if (!service::audio::isAvailable()) {
            auto* label = lv_label_create(main_wrapper);
            lv_label_set_text(label, "No audio hardware available");
            lv_obj_center(label);
            return;
        }

        auto* input_section = createSection(main_wrapper, "Microphone");
        inputEnabledSwitch = createSwitchRow(input_section, "Enabled", onInputEnabledSwitch, this);
        inputMuteSwitch = createSwitchRow(input_section, "Mute", onInputMuteSwitch, this);
        inputVolumeSlider = createSliderRow(input_section, "Volume", onInputVolumeSlider, this);

        auto* output_section = createSection(main_wrapper, "Speaker");
        outputEnabledSwitch = createSwitchRow(output_section, "Enabled", onOutputEnabledSwitch, this);
        outputMuteSwitch = createSwitchRow(output_section, "Mute", onOutputMuteSwitch, this);
        outputVolumeSlider = createSliderRow(output_section, "Volume", onOutputVolumeSlider, this);

        refresh();
    }

    void onHide(AppContext& app) override {
        inputEnabledSwitch = nullptr;
        inputMuteSwitch = nullptr;
        inputVolumeSlider = nullptr;
        outputEnabledSwitch = nullptr;
        outputMuteSwitch = nullptr;
        outputVolumeSlider = nullptr;
    }

    void refresh() const {
        if (inputEnabledSwitch) {
            if (service::audio::isInputEnabled()) lv_obj_add_state(inputEnabledSwitch, LV_STATE_CHECKED);
            else lv_obj_remove_state(inputEnabledSwitch, LV_STATE_CHECKED);
        }
        if (inputMuteSwitch) {
            if (service::audio::isInputMuted()) lv_obj_add_state(inputMuteSwitch, LV_STATE_CHECKED);
            else lv_obj_remove_state(inputMuteSwitch, LV_STATE_CHECKED);
        }
        if (inputVolumeSlider && !lv_obj_has_state(inputVolumeSlider, LV_STATE_PRESSED)) {
            lv_slider_set_value(inputVolumeSlider, static_cast<int32_t>(service::audio::getInputVolume()), LV_ANIM_OFF);
        }

        if (outputEnabledSwitch) {
            if (service::audio::isOutputEnabled()) lv_obj_add_state(outputEnabledSwitch, LV_STATE_CHECKED);
            else lv_obj_remove_state(outputEnabledSwitch, LV_STATE_CHECKED);
        }
        if (outputMuteSwitch) {
            if (service::audio::isOutputMuted()) lv_obj_add_state(outputMuteSwitch, LV_STATE_CHECKED);
            else lv_obj_remove_state(outputMuteSwitch, LV_STATE_CHECKED);
        }
        if (outputVolumeSlider && !lv_obj_has_state(outputVolumeSlider, LV_STATE_PRESSED)) {
            lv_slider_set_value(outputVolumeSlider, static_cast<int32_t>(service::audio::getOutputVolume()), LV_ANIM_OFF);
        }
    }
};

extern const AppManifest manifest = {
    .appId = "AudioSettings",
    .appName = "Audio",
    .appIcon = LVGL_ICON_SHARED_MUSIC_NOTE,
    .appCategory = Category::Settings,
    .createApp = create<AudioSettingsApp>
};

} // namespace tt::app::audiosettings
