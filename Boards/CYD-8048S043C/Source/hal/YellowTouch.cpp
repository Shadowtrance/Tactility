#include "YellowTouch.h"
#include "YellowTouchConstants.h"
#include "Log.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_touch.h"
#include "esp_lvgl_port.h"

#define TAG "yellow_touch"

uint16_t map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) { 
    return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    *x = map(*x, CYD8048S043_TOUCH_H_RES_MIN, CYD8048S043_TOUCH_H_RES_MAX, 0, 800);
    *y = map(*y, CYD8048S043_TOUCH_V_RES_MIN, CYD8048S043_TOUCH_V_RES_MAX, 0, 480);

    ESP_LOGI(TAG, "Touch X: %d Y: %d\n", *x, *y);
}

bool YellowTouch::start(lv_display_t* display) {
    TT_LOG_I(TAG, "Starting");
    vTaskDelay(pdMS_TO_TICKS(100));
    const esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    if (esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CYD8048S043_TOUCH_I2C_PORT, &io_config, &ioHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Touch I2C IO init failed");
        return false;
    }

    esp_lcd_touch_config_t config = {
        .x_max = 800,
        .y_max = 480,
        .rst_gpio_num = GPIO_NUM_38, //GPIO_NUM_38,
        .int_gpio_num = GPIO_NUM_NC, //GPIO_NUM_18 (with mod, NC otherwise),
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .process_coordinates = process_coordinates,
        .interrupt_callback = nullptr,
        .user_data = nullptr,
        .driver_data = nullptr
    };

    // if (esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CYD8048S043_TOUCH_I2C_PORT, &touch_io_config, &ioHandle) != ESP_OK) {
    //     TT_LOG_E(TAG, "Touch I2C IO init failed");
    //     return false;
    // }

    if (esp_lcd_touch_new_i2c_gt911(ioHandle, &config, &touchHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Driver init failed");
        cleanup();
        return false;
    }

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = display,
        .handle = touchHandle,
    };

    TT_LOG_I(TAG, "Adding touch to LVGL");
    deviceHandle = lvgl_port_add_touch(&touch_cfg);
    if (deviceHandle == nullptr) {
        TT_LOG_E(TAG, "Adding touch failed");
        cleanup();
        return false;
    }

    return true;
}

bool YellowTouch::stop() {
    cleanup();
    return true;
}

void YellowTouch::cleanup() {
    if (deviceHandle != nullptr) {
        lv_indev_delete(deviceHandle);
        deviceHandle = nullptr;
    }

    if (touchHandle != nullptr) {
        esp_lcd_touch_del(touchHandle);
        touchHandle = nullptr;
    }

    if (ioHandle != nullptr) {
        esp_lcd_panel_io_del(ioHandle);
        ioHandle = nullptr;
    }
}
