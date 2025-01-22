#include "YellowDisplay.h"
#include "YellowDisplayConstants.h"
#include "YellowTouch.h"
#include "Log.h"

#include <TactilityCore.h>
#include <esp_lcd_panel_commands.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"

#define TAG "yellow_display"

static bool isBacklightInitialized = false;

static bool initBacklight() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = CYD8048S043_LCD_BACKLIGHT_LEDC_MODE,
        .duty_resolution = CYD8048S043_LCD_BACKLIGHT_LEDC_DUTY_RES,
        .timer_num = CYD8048S043_LCD_BACKLIGHT_LEDC_TIMER,
        .freq_hz = CYD8048S043_LCD_BACKLIGHT_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };

    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        TT_LOG_E(TAG, "Backlight led timer config failed");
        return false;
    }

    return true;
}

static bool setBacklight(uint8_t duty) {
    ledc_channel_config_t ledc_channel = {
        .gpio_num = CYD8048S043_LCD_PIN_BACKLIGHT,
        .speed_mode = CYD8048S043_LCD_BACKLIGHT_LEDC_MODE,
        .channel = CYD8048S043_LCD_BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = CYD8048S043_LCD_BACKLIGHT_LEDC_TIMER,
        .duty = duty,
        .hpoint = 0,
        .flags = {
            .output_invert = false
        }
    };

    if (ledc_channel_config(&ledc_channel) != ESP_OK) {
        TT_LOG_E(TAG, "Backlight init failed");
        return false;
    }

    return true;
}

bool YellowDisplay::start() {
    TT_LOG_I(TAG, "Starting");

    const esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = 14000000,
            .h_res = CYD8048S043_LCD_HORIZONTAL_RESOLUTION,
            .v_res = CYD8048S043_LCD_VERTICAL_RESOLUTION,

            .hsync_pulse_width = 4,
            .hsync_back_porch = 8,
            .hsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .vsync_back_porch = 8,
            .vsync_front_porch = 8,
            .flags = {
                .hsync_idle_low = false,
                .vsync_idle_low = false,
                .de_idle_high = false,
                .pclk_active_neg = true,
                .pclk_idle_high = false
            }
        },

        .data_width = 16,
        .bits_per_pixel = 0,
        .num_fbs = 2,
        .bounce_buffer_size_px = 10 * CYD8048S043_LCD_HORIZONTAL_RESOLUTION,
        .sram_trans_align = 8,
        .psram_trans_align = 64,

        .hsync_gpio_num = CYD8048S043_LCD_PIN_HSYNC,
        .vsync_gpio_num = CYD8048S043_LCD_PIN_VSYNC,
        .de_gpio_num = CYD8048S043_LCD_PIN_DE,
        .pclk_gpio_num = CYD8048S043_LCD_PIN_PCLK,
        .disp_gpio_num = CYD8048S043_LCD_PIN_DISP_EN,
        .data_gpio_nums = {
            CYD8048S043_LCD_PIN_DATA0,
            CYD8048S043_LCD_PIN_DATA1,
            CYD8048S043_LCD_PIN_DATA2,
            CYD8048S043_LCD_PIN_DATA3,
            CYD8048S043_LCD_PIN_DATA4,
            CYD8048S043_LCD_PIN_DATA5,
            CYD8048S043_LCD_PIN_DATA6,
            CYD8048S043_LCD_PIN_DATA7,
            CYD8048S043_LCD_PIN_DATA8,
            CYD8048S043_LCD_PIN_DATA9,
            CYD8048S043_LCD_PIN_DATA10,
            CYD8048S043_LCD_PIN_DATA11,
            CYD8048S043_LCD_PIN_DATA12,
            CYD8048S043_LCD_PIN_DATA13,
            CYD8048S043_LCD_PIN_DATA14,
            CYD8048S043_LCD_PIN_DATA15
        },
        .flags = {
            .disp_active_low = 0,
            .refresh_on_demand = 0,
            .fb_in_psram = true,
            .double_fb = true,
            .no_fb = 0,
            .bb_invalidate_cache = 0
        }
    };

    if (esp_lcd_new_rgb_panel(&panel_config, &panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create panel");
        return false;
    }

    if (esp_lcd_panel_reset(panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to reset panel");
        return false;
    }

    if (esp_lcd_panel_init(panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to init panel");
        return false;
    }

    gpio_set_level(CYD8048S043_LCD_PIN_BACKLIGHT, 1);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = ioHandle,
        .panel_handle = panelHandle,
        .control_handle = nullptr,
        .buffer_size = CYD8048S043_LCD_DRAW_BUFFER_SIZE,
        .double_buffer = true,
        .trans_size = 0,
        .hres = CYD8048S043_LCD_HORIZONTAL_RESOLUTION,
        .vres = CYD8048S043_LCD_VERTICAL_RESOLUTION,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = false,
            .swap_bytes = false,
            .full_refresh = false,
            .direct_mode = false,
        }
    };

    const lvgl_port_display_rgb_cfg_t rgb_cfg = {
        .flags = {
            .bb_mode = true,
            .avoid_tearing = false,
        }
    };

    displayHandle = lvgl_port_add_disp_rgb(&disp_cfg, &rgb_cfg);
    TT_LOG_I(TAG, "Finished");
    return displayHandle != nullptr;
}

bool YellowDisplay::stop() {
    tt_assert(displayHandle != nullptr);

    lvgl_port_remove_disp(displayHandle);

    if (esp_lcd_panel_del(panelHandle) != ESP_OK) {
        return false;
    }

    if (esp_lcd_panel_io_del(ioHandle) != ESP_OK) {
        return false;
    }

    displayHandle = nullptr;
    return true;
}

void YellowDisplay::setBacklightDuty(uint8_t backlightDuty) {
    if (!isBacklightInitialized) {
        tt_check(initBacklight());
        isBacklightInitialized = true;
    }

    if (!setBacklight(backlightDuty)) {
        TT_LOG_E(TAG, "Failed to configure display backlight");
    }
}

tt::hal::Touch* _Nullable YellowDisplay::createTouch() {
    return static_cast<tt::hal::Touch*>(new YellowTouch());
}

tt::hal::Display* createDisplay() {
    return static_cast<tt::hal::Display*>(new YellowDisplay());
}
