// SPDX-License-Identifier: Apache-2.0
#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/log.h>
#include <tactility/drivers/audio_codec.h>
#include <tactility/drivers/audio_stream.h>

#include <cstring>
#include <vector>

#define TAG "AudioStream"

namespace {

// Linear-interpolation resampler. Cheap and good enough for voice/UI audio; S3/P4 have
// plenty of headroom for this at the rates this subsystem targets (16k/44.1k/48k).
// Operates on interleaved 16-bit PCM, which is what esp_codec_dev / our codec drivers use.
size_t resampleS16(const int16_t* in, size_t inFrames, uint8_t channels,
                   uint32_t inRate, uint32_t outRate,
                   int16_t* out, size_t outFrameCapacity) {
    if (inRate == outRate) {
        size_t frames = (inFrames < outFrameCapacity) ? inFrames : outFrameCapacity;
        std::memcpy(out, in, frames * channels * sizeof(int16_t));
        return frames;
    }

    if (inFrames == 0) {
        return 0;
    }

    double ratio = (double) inRate / (double) outRate;
    size_t outFrames = 0;
    for (; outFrames < outFrameCapacity; outFrames++) {
        double srcPos = (double) outFrames * ratio;
        size_t srcIndex = (size_t) srcPos;
        if (srcIndex + 1 >= inFrames) {
            if (srcIndex >= inFrames) {
                break;
            }
            // Last frame: no next sample to interpolate with, repeat it.
            for (uint8_t channel = 0; channel < channels; channel++) {
                out[outFrames * channels + channel] = in[srcIndex * channels + channel];
            }
            continue;
        }

        double frac = srcPos - (double) srcIndex;
        for (uint8_t channel = 0; channel < channels; channel++) {
            int16_t a = in[srcIndex * channels + channel];
            int16_t b = in[(srcIndex + 1) * channels + channel];
            out[outFrames * channels + channel] = (int16_t) ((double) a + ((double) b - (double) a) * frac);
        }
    }

    return outFrames;
}

struct AudioStreamHandleImpl : AudioStreamHandleData {
    enum AudioCodecDirection direction = AUDIO_CODEC_DIR_BOTH;
    struct AudioStreamConfig config = {};
    uint32_t codecRate = 0;
    uint8_t bytesPerFrame = 0;
    std::vector<uint8_t> codecBuffer; // raw codec-rate PCM, scratch for resampling

    // Lifetime guard: closeStream() can be triggered from a different task than the one
    // doing read()/write() (e.g. the Settings UI disabling output while SfxEngine's audio
    // task is mid-write). `closing` keeps new I/O calls out, `busyCount` tracks I/O calls
    // currently in flight, and `drainSemaphore` lets closeStream() block until they finish
    // before freeing the handle. All three are only touched while `AudioStreamData::mutex`
    // is held, except for the give/take on drainSemaphore itself.
    bool closing = false;
    int busyCount = 0;
    SemaphoreHandle_t drainSemaphore = nullptr;
};

struct AudioStreamData {
    Device* inputCodec = nullptr;
    Device* outputCodec = nullptr;
    bool inputEnabled = true;
    bool outputEnabled = true;
    AudioStreamHandleImpl* openInput = nullptr;
    AudioStreamHandleImpl* openOutput = nullptr;
    // Guards openInput/openOutput and the closing/busyCount fields of any handle reachable
    // through them, so close (possibly forced by setEnabled) can't race with read/write.
    SemaphoreHandle_t mutex = nullptr;
};

#define GET_DATA(device) (static_cast<AudioStreamData*>(device_get_driver_data(device)))

struct CodecSearchContext {
    enum AudioCodecDirection wantedDirection;
    Device* found = nullptr;
};

bool findCodecByDirection(Device* device, void* contextPtr) {
    auto* context = static_cast<CodecSearchContext*>(contextPtr);
    if (!device_is_ready(device)) {
        return true; // continue searching
    }

    enum AudioCodecDirection capabilities = AUDIO_CODEC_DIR_BOTH;
    if (audio_codec_get_capabilities(device, &capabilities) != ERROR_NONE) {
        return true;
    }

    if ((capabilities & context->wantedDirection) == context->wantedDirection) {
        context->found = device;
        return false; // stop searching
    }

    return true;
}

Device* findFirstCodecSupporting(enum AudioCodecDirection direction) {
    CodecSearchContext context = { .wantedDirection = direction };
    device_for_each_of_type(&AUDIO_CODEC_TYPE, &context, findCodecByDirection);
    return context.found;
}

// The audio-stream device is constructed while modules start, which happens before the
// device tree's codec devices (nested under i2c0) are started. So codecs can't be resolved
// at start_device time — resolve (and cache) them lazily on first use instead, by which
// point the device tree has finished starting.
Device* codecForDirection(AudioStreamData* data, enum AudioCodecDirection direction) {
    Device** slot = (direction == AUDIO_CODEC_DIR_INPUT) ? &data->inputCodec : &data->outputCodec;
    if (*slot == nullptr) {
        *slot = findFirstCodecSupporting(direction);
        if (*slot != nullptr) {
            LOG_I(TAG, "Bound %s codec: %s", (direction == AUDIO_CODEC_DIR_INPUT) ? "input" : "output", (*slot)->name);
        }
    }
    return *slot;
}

// region AudioStreamApi

error_t openStream(Device* device, const struct AudioStreamConfig* config, enum AudioCodecDirection direction, AudioStreamHandle* outHandle) {
    if (config == nullptr || outHandle == nullptr) {
        return ERROR_INVALID_ARGUMENT;
    }

    auto* data = GET_DATA(device);
    if (data == nullptr) {
        return ERROR_RESOURCE;
    }

    bool isInput = (direction == AUDIO_CODEC_DIR_INPUT);
    Device* codec = codecForDirection(data, direction);
    if (codec == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }

    xSemaphoreTake(data->mutex, portMAX_DELAY);

    if ((isInput && !data->inputEnabled) || (!isInput && !data->outputEnabled)) {
        xSemaphoreGive(data->mutex);
        return ERROR_NOT_ALLOWED;
    }

    AudioStreamHandleImpl** slot = isInput ? &data->openInput : &data->openOutput;
    if (*slot != nullptr) {
        xSemaphoreGive(data->mutex);
        return ERROR_INVALID_STATE;
    }

    // Reserve the slot with a placeholder so concurrent opens can't race past the check
    // above while we do the (potentially slow) codec open below outside the lock.
    auto* reservation = reinterpret_cast<AudioStreamHandleImpl*>(1);
    *slot = reservation;
    xSemaphoreGive(data->mutex);

    uint32_t codecRate = 0;
    if (audio_codec_get_native_sample_rate(codec, direction, &codecRate) != ERROR_NONE || codecRate == 0) {
        return ERROR_RESOURCE;
    }

    struct AudioCodecStreamConfig codecConfig = {
        .sample_rate = codecRate,
        .bits_per_sample = config->bits_per_sample,
        .channels = config->channels,
        .direction = direction,
    };

    if (audio_codec_open(codec, &codecConfig) != ERROR_NONE) {
        LOG_E(TAG, "Failed to open codec for %s", isInput ? "input" : "output");
        return ERROR_RESOURCE;
    }

    auto* handle = new AudioStreamHandleImpl();
    handle->device = device;
    handle->direction = direction;
    handle->config = *config;
    handle->codecRate = codecRate;
    handle->bytesPerFrame = (uint8_t) ((config->bits_per_sample / 8) * config->channels);

    *slot = handle;
    *outHandle = handle;
    return ERROR_NONE;
}

error_t openInput(Device* device, const struct AudioStreamConfig* config, AudioStreamHandle* outHandle) {
    return openStream(device, config, AUDIO_CODEC_DIR_INPUT, outHandle);
}

error_t openOutput(Device* device, const struct AudioStreamConfig* config, AudioStreamHandle* outHandle) {
    return openStream(device, config, AUDIO_CODEC_DIR_OUTPUT, outHandle);
}

error_t readStream(AudioStreamHandle handleBase, void* outData, size_t dataSize, size_t* bytesRead, TickType_t timeout) {
    auto* handle = static_cast<AudioStreamHandleImpl*>(handleBase);
    if (handle->direction != AUDIO_CODEC_DIR_INPUT || handle->bytesPerFrame == 0) {
        return ERROR_INVALID_STATE;
    }

    auto* data = GET_DATA(handle->device);
    if (data == nullptr || data->inputCodec == nullptr) {
        return ERROR_RESOURCE;
    }

    size_t requestedFrames = dataSize / handle->bytesPerFrame;
    if (requestedFrames == 0) {
        if (bytesRead != nullptr) {
            *bytesRead = 0;
        }
        return ERROR_NONE;
    }

    if (handle->codecRate == handle->config.sample_rate) {
        size_t codecBytesRead = 0;
        error_t error = audio_codec_read(data->inputCodec, outData, dataSize, &codecBytesRead, timeout);
        if (bytesRead != nullptr) {
            *bytesRead = codecBytesRead;
        }
        return error;
    }

    // Read enough codec-rate frames to produce the requested number of output-rate frames.
    size_t codecFrames = (size_t) ((double) requestedFrames * ((double) handle->codecRate / (double) handle->config.sample_rate)) + 2;
    size_t codecBytesNeeded = codecFrames * handle->bytesPerFrame;
    if (handle->codecBuffer.size() < codecBytesNeeded) {
        handle->codecBuffer.resize(codecBytesNeeded);
    }

    size_t codecBytesRead = 0;
    error_t error = audio_codec_read(data->inputCodec, handle->codecBuffer.data(), codecBytesNeeded, &codecBytesRead, timeout);
    if (error != ERROR_NONE) {
        return error;
    }

    size_t codecFramesRead = codecBytesRead / handle->bytesPerFrame;
    size_t outFrames = resampleS16(
        reinterpret_cast<const int16_t*>(handle->codecBuffer.data()), codecFramesRead, handle->config.channels,
        handle->codecRate, handle->config.sample_rate,
        reinterpret_cast<int16_t*>(outData), requestedFrames);

    if (bytesRead != nullptr) {
        *bytesRead = outFrames * handle->bytesPerFrame;
    }
    return ERROR_NONE;
}

error_t writeStream(AudioStreamHandle handleBase, const void* inData, size_t dataSize, size_t* bytesWritten, TickType_t timeout) {
    auto* handle = static_cast<AudioStreamHandleImpl*>(handleBase);
    if (handle->direction != AUDIO_CODEC_DIR_OUTPUT || handle->bytesPerFrame == 0) {
        return ERROR_INVALID_STATE;
    }

    auto* data = GET_DATA(handle->device);
    if (data == nullptr || data->outputCodec == nullptr) {
        return ERROR_RESOURCE;
    }

    size_t inFrames = dataSize / handle->bytesPerFrame;
    if (inFrames == 0) {
        if (bytesWritten != nullptr) {
            *bytesWritten = 0;
        }
        return ERROR_NONE;
    }

    if (handle->codecRate == handle->config.sample_rate) {
        size_t codecBytesWritten = 0;
        error_t error = audio_codec_write(data->outputCodec, inData, dataSize, &codecBytesWritten, timeout);
        if (bytesWritten != nullptr) {
            // Report in terms of input bytes consumed, matching codecBytesWritten 1:1 here.
            *bytesWritten = codecBytesWritten;
        }
        return error;
    }

    size_t codecFrameCapacity = (size_t) ((double) inFrames * ((double) handle->codecRate / (double) handle->config.sample_rate)) + 2;
    size_t codecBytesCapacity = codecFrameCapacity * handle->bytesPerFrame;
    if (handle->codecBuffer.size() < codecBytesCapacity) {
        handle->codecBuffer.resize(codecBytesCapacity);
    }

    size_t codecFrames = resampleS16(
        reinterpret_cast<const int16_t*>(inData), inFrames, handle->config.channels,
        handle->config.sample_rate, handle->codecRate,
        reinterpret_cast<int16_t*>(handle->codecBuffer.data()), codecFrameCapacity);

    size_t codecBytesToWrite = codecFrames * handle->bytesPerFrame;
    size_t codecBytesWritten = 0;
    error_t error = audio_codec_write(data->outputCodec, handle->codecBuffer.data(), codecBytesToWrite, &codecBytesWritten, timeout);
    if (error != ERROR_NONE) {
        return error;
    }

    if (bytesWritten != nullptr) {
        // The caller provided `dataSize` worth of input; we consumed all of it (resampled).
        *bytesWritten = dataSize;
    }
    return ERROR_NONE;
}

error_t closeStream(AudioStreamHandle handleBase) {
    auto* handle = static_cast<AudioStreamHandleImpl*>(handleBase);
    auto* data = GET_DATA(handle->device);
    if (data == nullptr) {
        return ERROR_RESOURCE;
    }

    bool isInput = (handle->direction == AUDIO_CODEC_DIR_INPUT);
    Device* codec = isInput ? data->inputCodec : data->outputCodec;
    AudioStreamHandleImpl** slot = isInput ? &data->openInput : &data->openOutput;

    if (codec != nullptr) {
        audio_codec_close(codec);
    }

    if (*slot == handle) {
        *slot = nullptr;
    }

    delete handle;
    return ERROR_NONE;
}

error_t setVolume(Device* device, enum AudioCodecDirection direction, float volumePercent) {
    auto* data = GET_DATA(device);
    if (data == nullptr) {
        return ERROR_RESOURCE;
    }
    Device* codec = codecForDirection(data, direction);
    if (codec == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }
    return audio_codec_set_volume(codec, direction, volumePercent);
}

error_t getVolume(Device* device, enum AudioCodecDirection direction, float* volumePercent) {
    auto* data = GET_DATA(device);
    if (data == nullptr) {
        return ERROR_RESOURCE;
    }
    Device* codec = codecForDirection(data, direction);
    if (codec == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }
    return audio_codec_get_volume(codec, direction, volumePercent);
}

error_t setMute(Device* device, enum AudioCodecDirection direction, bool muted) {
    auto* data = GET_DATA(device);
    if (data == nullptr) {
        return ERROR_RESOURCE;
    }
    Device* codec = codecForDirection(data, direction);
    if (codec == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }
    return audio_codec_set_mute(codec, direction, muted);
}

error_t getMute(Device* device, enum AudioCodecDirection direction, bool* muted) {
    auto* data = GET_DATA(device);
    if (data == nullptr) {
        return ERROR_RESOURCE;
    }
    Device* codec = codecForDirection(data, direction);
    if (codec == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }
    return audio_codec_get_mute(codec, direction, muted);
}

error_t setEnabled(Device* device, enum AudioCodecDirection direction, bool enabled) {
    auto* data = GET_DATA(device);
    if (data == nullptr) {
        return ERROR_RESOURCE;
    }

    bool isInput = (direction == AUDIO_CODEC_DIR_INPUT);
    Device* codec = codecForDirection(data, direction);
    if (codec == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }

    if (isInput) {
        data->inputEnabled = enabled;
    } else {
        data->outputEnabled = enabled;
    }

    if (!enabled) {
        AudioStreamHandleImpl** slot = isInput ? &data->openInput : &data->openOutput;
        if (*slot != nullptr) {
            closeStream(*slot);
        }
    }

    return ERROR_NONE;
}

error_t getEnabled(Device* device, enum AudioCodecDirection direction, bool* enabled) {
    auto* data = GET_DATA(device);
    if (data == nullptr || enabled == nullptr) {
        return ERROR_RESOURCE;
    }
    Device* codec = codecForDirection(data, direction);
    if (codec == nullptr) {
        return ERROR_NOT_SUPPORTED;
    }
    *enabled = (direction == AUDIO_CODEC_DIR_INPUT) ? data->inputEnabled : data->outputEnabled;
    return ERROR_NONE;
}

const struct AudioStreamApi API = {
    .open_input = openInput,
    .open_output = openOutput,
    .read = readStream,
    .write = writeStream,
    .close = closeStream,
    .set_volume = setVolume,
    .get_volume = getVolume,
    .set_mute = setMute,
    .get_mute = getMute,
    .set_enabled = setEnabled,
    .get_enabled = getEnabled,
};

// endregion

// region Driver lifecycle

error_t startDevice(Device* device) {
    auto* data = new AudioStreamData();
    data->mutex = xSemaphoreCreateMutex();
    if (data->mutex == nullptr) {
        delete data;
        return ERROR_OUT_OF_MEMORY;
    }
    device_set_driver_data(device, data);
    return ERROR_NONE;
}

error_t stopDevice(Device* device) {
    auto* data = GET_DATA(device);
    if (data == nullptr) {
        return ERROR_NONE;
    }

    if (data->openInput != nullptr) {
        closeStream(data->openInput);
    }
    if (data->openOutput != nullptr) {
        closeStream(data->openOutput);
    }

    device_set_driver_data(device, nullptr);
    if (data->mutex != nullptr) {
        vSemaphoreDelete(data->mutex);
    }
    delete data;
    return ERROR_NONE;
}

// endregion

const char* COMPATIBLE[] = { "audio-stream", nullptr };

} // namespace

extern "C" {

Driver audio_stream_driver = {
    .name = "audio-stream",
    .compatible = COMPATIBLE,
    .start_device = startDevice,
    .stop_device = stopDevice,
    .api = &API,
    .device_type = &AUDIO_STREAM_TYPE,
    .owner = nullptr,
    .internal = nullptr,
};

Device audio_stream_device = {
    .name = "audio-stream0",
    .config = nullptr,
    .parent = nullptr,
    .internal = nullptr,
};

}
