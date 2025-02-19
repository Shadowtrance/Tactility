#include "UnPhoneTouch.h"
#include "UnPhoneDisplayConstants.h"

#include <Tactility/Log.h>

std::shared_ptr<Xpt2046Touch> createTouch() {
    auto configuration = std::make_unique<Xpt2046Touch::Configuration>(
        UNPHONE_LCD_SPI_HOST,
        GPIO_NUM_38,
        320,
        480
    );

    return std::make_shared<Xpt2046Touch>(std::move(configuration));
}
