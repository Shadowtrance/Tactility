// SPDX-License-Identifier: Apache-2.0
#include <tactility/drivers/audio_codec_adapters.h>
#include <tactility/drivers/i2s_controller.h>
#include <tactility/freertos/freertos.h>

#include <stdlib.h>

#define I2S_TIMEOUT_TICKS pdMS_TO_TICKS(1000)

struct I2sDataContext {
    audio_codec_data_if_t base;
    struct Device* i2s_controller;
    bool is_open;
};

static bool data_is_open(const audio_codec_data_if_t* handle) {
    const struct I2sDataContext* context = (const struct I2sDataContext*) handle;
    return context->is_open;
}

static int data_open(const audio_codec_data_if_t* handle, void* data_cfg, int cfg_size) {
    (void) data_cfg;
    (void) cfg_size;
    struct I2sDataContext* context = (struct I2sDataContext*) handle;
    context->is_open = true;
    return ESP_CODEC_DEV_OK;
}

static int data_enable(const audio_codec_data_if_t* handle, esp_codec_dev_type_t dev_type, bool enable) {
    (void) handle;
    (void) dev_type;
    (void) enable;
    // Channel enablement is handled by i2s_controller_set_config()/reset() at format-set time;
    // there is no separate per-direction enable in the I2sControllerApi.
    return ESP_CODEC_DEV_OK;
}

static int data_set_fmt(const audio_codec_data_if_t* handle, esp_codec_dev_type_t dev_type, esp_codec_dev_sample_info_t* fs) {
    struct I2sDataContext* context = (struct I2sDataContext*) handle;
    if (!context->is_open || fs == NULL) {
        return ESP_CODEC_DEV_INVALID_ARG;
    }

    struct I2sConfig config = {
        .communication_format = I2S_FORMAT_STAND_I2S,
        .sample_rate = fs->sample_rate,
        .bits_per_sample = fs->bits_per_sample,
        .channel_left = 0,
        .channel_right = (fs->channel > 1) ? 1 : I2S_CHANNEL_NONE,
    };

    if (i2s_controller_set_config(context->i2s_controller, &config) != ERROR_NONE) {
        return ESP_CODEC_DEV_DRV_ERR;
    }

    if ((dev_type & ESP_CODEC_DEV_TYPE_IN) != 0 && fs->channel > 2) {
        struct I2sTdmRxConfig tdm_config = {
            .sample_rate_hz = fs->sample_rate,
            .mclk_multiple = (fs->mclk_multiple != 0) ? (uint32_t) fs->mclk_multiple : 256,
            .bclk_div = 8,
            .slot_count = fs->channel,
            .bits_per_sample = fs->bits_per_sample,
            .slot_bit_width = 0,
        };
        error_t error = i2s_controller_set_rx_tdm_config(context->i2s_controller, &tdm_config);
        if (error != ERROR_NONE && error != ERROR_NOT_SUPPORTED) {
            return ESP_CODEC_DEV_DRV_ERR;
        }
    }

    return ESP_CODEC_DEV_OK;
}

static int data_read(const audio_codec_data_if_t* handle, uint8_t* data, int size) {
    struct I2sDataContext* context = (struct I2sDataContext*) handle;
    if (!context->is_open) {
        return ESP_CODEC_DEV_WRONG_STATE;
    }
    size_t bytes_read = 0;
    error_t error = i2s_controller_read(context->i2s_controller, data, (size_t) size, &bytes_read, I2S_TIMEOUT_TICKS);
    if (error != ERROR_NONE) {
        return ESP_CODEC_DEV_READ_FAIL;
    }
    return (int) bytes_read;
}

static int data_write(const audio_codec_data_if_t* handle, uint8_t* data, int size) {
    struct I2sDataContext* context = (struct I2sDataContext*) handle;
    if (!context->is_open) {
        return ESP_CODEC_DEV_WRONG_STATE;
    }
    size_t bytes_written = 0;
    error_t error = i2s_controller_write(context->i2s_controller, data, (size_t) size, &bytes_written, I2S_TIMEOUT_TICKS);
    if (error != ERROR_NONE) {
        return ESP_CODEC_DEV_WRITE_FAIL;
    }
    return (int) bytes_written;
}

static int data_close(const audio_codec_data_if_t* handle) {
    struct I2sDataContext* context = (struct I2sDataContext*) handle;
    context->is_open = false;
    return ESP_CODEC_DEV_OK;
}

const audio_codec_data_if_t* audio_codec_adapter_new_i2s_data(struct Device* i2s_controller) {
    if (i2s_controller == NULL) {
        return NULL;
    }

    struct I2sDataContext* context = (struct I2sDataContext*) calloc(1, sizeof(struct I2sDataContext));
    if (context == NULL) {
        return NULL;
    }

    context->i2s_controller = i2s_controller;
    context->is_open = false;
    context->base.open = data_open;
    context->base.is_open = data_is_open;
    context->base.enable = data_enable;
    context->base.set_fmt = data_set_fmt;
    context->base.read = data_read;
    context->base.write = data_write;
    context->base.close = data_close;

    return &context->base;
}

// Note: esp_codec_dev already provides audio_codec_delete_data_if() (calls ->close then frees);
// no adapter-specific cleanup is needed, so we don't redefine it here.
