#pragma once

#include "Tactility/hal/touch/TouchDevice.h"
#include <Tactility/TactilityCore.h>
#include <esp_lcd_panel_io_interface.h>
#include <esp_lcd_touch.h>

//Touch INT note:
//Install a 0 ohm resistor or solder bridge across R17, this connects the GT911's INT pin to GPIO18.
//If installed, remove (pullup resistor) R5 (GPIO18 to 3.3V).
//The GT911's INT pin is not strong enough to drive GPIO18 low with this resistor installed.
//If installed, remove U1 (XPT2046), not needed, ideally shouldn't be there anyway and may conflict with GPIO18.

class YellowTouch : public tt::hal::touch::TouchDevice {

private:

    std::string getName() const final { return "GT911"; }
    std::string getDescription() const final { return "I2C touch driver"; }

    esp_lcd_panel_io_handle_t ioHandle = nullptr;
    esp_lcd_touch_handle_t touchHandle = nullptr;
    lv_indev_t* _Nullable deviceHandle = nullptr;
    void cleanup();

public:

    bool start(lv_display_t* display) override;
    bool stop() override;
    lv_indev_t* _Nullable getLvglIndev() override { return deviceHandle; }
};
