#include "YellowSdCard.h"

#define TAG "cyd8048s043c_sdcard"

#include "lvgl/LvglSync.h"
#include "hal/SpiSdCard.h"

#define SDCARD_SPI_HOST SPI2_HOST
#define SDCARD_PIN_CS GPIO_NUM_10

std::shared_ptr<SdCard> createYellowSdCard() {
    auto* configuration = new tt::hal::SpiSdCard::Config(
        SDCARD_PIN_CS,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        SdCard::MountBehaviour::AtBoot,
        nullptr,
        std::vector<gpio_num_t>(),
        SDCARD_SPI_HOST
    );

    auto* sdcard = (SdCard*) new SpiSdCard(
        std::unique_ptr<SpiSdCard::Config>(configuration)
    );

    return std::shared_ptr<SdCard>(sdcard);
}

