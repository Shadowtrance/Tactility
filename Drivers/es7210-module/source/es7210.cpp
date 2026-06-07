// SPDX-License-Identifier: Apache-2.0
#include <drivers/es7210.h>

#include <tactility/device.h>
#include <tactility/drivers/audio_codec.h>
#include <tactility/drivers/audio_codec_adapters.h>
#include <tactility/drivers/i2c_controller.h>
#include <tactility/drivers/i2s_controller.h>
#include <tactility/log.h>

#include <es7210_adc.h>
#include <esp_codec_dev.h>
#include <esp_codec_dev_defaults.h>

#define TAG "ES7210"

namespace {

constexpr uint32_t NATIVE_SAMPLE_RATE = 16000;

struct Es7210Data {
    const audio_codec_ctrl_if_t* ctrlIf = nullptr;
    const audio_codec_data_if_t* dataIf = nullptr;
    const audio_codec_if_t* codecIf = nullptr;
    esp_codec_dev_handle_t codecDevice = nullptr;
    bool isOpen = false;
};

#define GET_CONFIG(device) (static_cast<const Es7210Config*>((device)->config))
#define GET_DATA(device) (static_cast<Es7210Data*>(device_get_driver_data(device)))

// region AudioCodecApi

error_t open_(Device* device, const struct AudioCodecStreamConfig* config) {
    auto* data = GET_DATA(device);
    if (data == nullptr || data->codecDevice == nullptr) {
        return ERROR_RESOURCE;
    }

    if (config->direction == AUDIO_CODEC_DIR_OUTPUT || config->direction == AUDIO_CODEC_DIR_BOTH) {
        LOG_E(TAG, "ES7210 is input-only");
        return ERROR_NOT_SUPPORTED;
    }

    if (data->isOpen) {
        return ERROR_NONE;
    }

    esp_codec_dev_sample_info_t sampleInfo = {
        .bits_per_sample = config->bits_per_sample,
        .channel = config->channels,
        .channel_mask = 0,
        .sample_rate = config->sample_rate,
        .mclk_multiple = 0,
    };

    if (esp_codec_dev_open(data->codecDevice, &sampleInfo) != ESP_CODEC_DEV_OK) {
        LOG_E(TAG, "Failed to open codec device");
        return ERROR_RESOURCE;
    }

    data->isOpen = true;
    return ERROR_NONE;
}

error_t close_(Device* device) {
    auto* data = GET_DATA(device);
    if (data == nullptr || data->codecDevice == nullptr) {
        return ERROR_RESOURCE;
    }

    if (data->isOpen) {
        esp_codec_dev_close(data->codecDevice);
        data->isOpen = false;
    }

    return ERROR_NONE;
}

error_t read_(Device* device, void* buffer, size_t size, size_t* bytesRead, TickType_t timeout) {
    (void) timeout;
    auto* data = GET_DATA(device);
    if (data == nullptr || !data->isOpen) {
        return ERROR_RESOURCE;
    }

    // esp_codec_dev_read returns the number of bytes read (>= 0) on success, or a negative
    // ESP_CODEC_DEV_* error code on failure -- it does NOT return ESP_CODEC_DEV_OK (0) for
    // a successful nonzero-length read.
    int result = esp_codec_dev_read(data->codecDevice, buffer, (int) size);
    if (result < 0) {
        return ERROR_RESOURCE;
    }

    if (bytesRead != nullptr) {
        *bytesRead = (size_t) result;
    }
    return ERROR_NONE;
}

error_t write_(Device* device, const void* buffer, size_t size, size_t* bytesWritten, TickType_t timeout) {
    (void) device;
    (void) buffer;
    (void) size;
    (void) bytesWritten;
    (void) timeout;
    return ERROR_NOT_SUPPORTED;
}

error_t setVolume(Device* device, enum AudioCodecDirection direction, float volumePercent) {
    auto* data = GET_DATA(device);
    if (data == nullptr || data->codecDevice == nullptr || direction != AUDIO_CODEC_DIR_INPUT) {
        return ERROR_NOT_SUPPORTED;
    }

    // ES7210 mic gain range is roughly 0..37.5 dB; map 0..100% linearly onto it.
    float db = (volumePercent / 100.0f) * 37.5f;
    return (esp_codec_dev_set_in_gain(data->codecDevice, db) == ESP_CODEC_DEV_OK) ? ERROR_NONE : ERROR_RESOURCE;
}

error_t getVolume(Device* device, enum AudioCodecDirection direction, float* volumePercent) {
    auto* data = GET_DATA(device);
    if (data == nullptr || data->codecDevice == nullptr || direction != AUDIO_CODEC_DIR_INPUT || volumePercent == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }

    float db = 0.0f;
    if (esp_codec_dev_get_in_gain(data->codecDevice, &db) != ESP_CODEC_DEV_OK) {
        return ERROR_RESOURCE;
    }
    *volumePercent = (db / 37.5f) * 100.0f;
    return ERROR_NONE;
}

error_t setMute(Device* device, enum AudioCodecDirection direction, bool muted) {
    auto* data = GET_DATA(device);
    if (data == nullptr || data->codecDevice == nullptr || direction != AUDIO_CODEC_DIR_INPUT) {
        return ERROR_NOT_SUPPORTED;
    }

    return (esp_codec_dev_set_in_mute(data->codecDevice, muted) == ESP_CODEC_DEV_OK) ? ERROR_NONE : ERROR_RESOURCE;
}

error_t getMute(Device* device, enum AudioCodecDirection direction, bool* muted) {
    auto* data = GET_DATA(device);
    if (data == nullptr || data->codecDevice == nullptr || direction != AUDIO_CODEC_DIR_INPUT || muted == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }

    return (esp_codec_dev_get_in_mute(data->codecDevice, muted) == ESP_CODEC_DEV_OK) ? ERROR_NONE : ERROR_RESOURCE;
}

error_t getNativeSampleRate(Device* device, enum AudioCodecDirection direction, uint32_t* rateHz) {
    (void) device;
    if (direction != AUDIO_CODEC_DIR_INPUT || rateHz == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }
    *rateHz = NATIVE_SAMPLE_RATE;
    return ERROR_NONE;
}

error_t getCapabilities(Device* device, enum AudioCodecDirection* supportedDirections) {
    (void) device;
    if (supportedDirections == nullptr) {
        return ERROR_RESOURCE;
    }
    *supportedDirections = AUDIO_CODEC_DIR_INPUT;
    return ERROR_NONE;
}

const struct AudioCodecApi API = {
    .open = open_,
    .close = close_,
    .read = read_,
    .write = write_,
    .set_volume = setVolume,
    .get_volume = getVolume,
    .set_mute = setMute,
    .get_mute = getMute,
    .get_native_sample_rate = getNativeSampleRate,
    .get_capabilities = getCapabilities,
};

// endregion

// region Driver lifecycle

error_t startDevice(Device* device) {
    const auto* config = GET_CONFIG(device);

    auto* i2cController = device_get_parent(device);
    if (i2cController == nullptr || device_get_type(i2cController) != &I2C_CONTROLLER_TYPE) {
        LOG_E(TAG, "Parent is not an I2C controller");
        return ERROR_RESOURCE;
    }

    auto* i2sController = device_find_by_name(config->i2s_device_name);
    if (i2sController == nullptr || device_get_type(i2sController) != &I2S_CONTROLLER_TYPE) {
        LOG_E(TAG, "I2S controller '%s' not found", config->i2s_device_name);
        return ERROR_RESOURCE;
    }

    auto* data = new Es7210Data();

    data->ctrlIf = audio_codec_adapter_new_i2c_ctrl(i2cController, config->address);
    data->dataIf = audio_codec_adapter_new_i2s_data(i2sController);
    if (data->ctrlIf == nullptr || data->dataIf == nullptr) {
        LOG_E(TAG, "Failed to create adapters");
        delete data;
        return ERROR_RESOURCE;
    }

    if (data->ctrlIf->open(data->ctrlIf, nullptr, 0) != ESP_CODEC_DEV_OK) {
        LOG_E(TAG, "Failed to open control interface");
        delete data;
        return ERROR_RESOURCE;
    }

    if (data->dataIf->open(data->dataIf, nullptr, 0) != ESP_CODEC_DEV_OK) {
        LOG_E(TAG, "Failed to open data interface");
        delete data;
        return ERROR_RESOURCE;
    }

    es7210_codec_cfg_t codecConfig = {};
    codecConfig.ctrl_if = data->ctrlIf;
    codecConfig.master_mode = false;
    codecConfig.mic_selected = (config->mic_selected_mask != 0)
        ? config->mic_selected_mask
        : (uint8_t) (ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3 | ES7210_SEL_MIC4);
    codecConfig.mclk_src = ES7210_MCLK_FROM_PAD;
    codecConfig.mclk_div = 0;

    data->codecIf = es7210_codec_new(&codecConfig);
    if (data->codecIf == nullptr) {
        LOG_E(TAG, "Failed to create ES7210 codec interface");
        delete data;
        return ERROR_RESOURCE;
    }

    esp_codec_dev_cfg_t devConfig = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = data->codecIf,
        .data_if = data->dataIf,
    };

    data->codecDevice = esp_codec_dev_new(&devConfig);
    if (data->codecDevice == nullptr) {
        LOG_E(TAG, "Failed to create codec device");
        delete data;
        return ERROR_RESOURCE;
    }

    device_set_driver_data(device, data);
    return ERROR_NONE;
}

error_t stopDevice(Device* device) {
    auto* data = GET_DATA(device);
    if (data == nullptr) {
        return ERROR_NONE;
    }

    if (data->isOpen) {
        esp_codec_dev_close(data->codecDevice);
    }

    if (data->codecDevice != nullptr) {
        esp_codec_dev_delete(data->codecDevice);
    }
    if (data->codecIf != nullptr) {
        audio_codec_delete_codec_if(data->codecIf);
    }
    if (data->dataIf != nullptr) {
        audio_codec_delete_data_if(data->dataIf);
    }
    if (data->ctrlIf != nullptr) {
        audio_codec_delete_ctrl_if(data->ctrlIf);
    }

    device_set_driver_data(device, nullptr);
    delete data;
    return ERROR_NONE;
}

// endregion

const char* COMPATIBLE[] = { "everest,es7210", nullptr };

} // namespace

extern "C" {

Driver es7210_driver = {
    .name = "es7210",
    .compatible = COMPATIBLE,
    .start_device = startDevice,
    .stop_device = stopDevice,
    .api = &API,
    .device_type = &AUDIO_CODEC_TYPE,
    .owner = nullptr,
    .internal = nullptr,
};

}
