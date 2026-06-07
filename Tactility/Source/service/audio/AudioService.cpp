#include <Tactility/service/audio/AudioService.h>

#include <Tactility/Logger.h>
#include <Tactility/service/ServiceManifest.h>
#include <Tactility/service/ServiceRegistration.h>
#include <Tactility/settings/AudioSettings.h>

#include <tactility/device.h>
#include <tactility/drivers/audio_codec.h>
#include <tactility/drivers/audio_stream.h>

namespace tt::service::audio {

static const auto LOGGER = Logger("AudioService");
extern const ServiceManifest manifest;

constexpr TickType_t PERSIST_INTERVAL_TICKS = pdMS_TO_TICKS(2000);

void AudioService::findStreamDevice() {
    streamDevice = nullptr;
    device_for_each_of_type(&AUDIO_STREAM_TYPE, &streamDevice, [](Device* device, void* context) -> bool {
        if (device_is_ready(device)) {
            *static_cast<Device**>(context) = device;
            return false;
        }
        return true;
    });
}

void AudioService::primeFromSettings() const {
    if (streamDevice == nullptr) {
        return;
    }

    auto settings = settings::audio::loadOrGetDefault();

    audio_stream_set_enabled(streamDevice, AUDIO_CODEC_DIR_INPUT, settings.inputEnabled);
    audio_stream_set_enabled(streamDevice, AUDIO_CODEC_DIR_OUTPUT, settings.outputEnabled);
    audio_stream_set_mute(streamDevice, AUDIO_CODEC_DIR_INPUT, settings.inputMuted);
    audio_stream_set_mute(streamDevice, AUDIO_CODEC_DIR_OUTPUT, settings.outputMuted);
    audio_stream_set_volume(streamDevice, AUDIO_CODEC_DIR_INPUT, settings.inputVolume);
    audio_stream_set_volume(streamDevice, AUDIO_CODEC_DIR_OUTPUT, settings.outputVolume);
}

void AudioService::schedulePersist() {
    auto lock = mutex.asScopedLock();
    lock.lock();
    persistPending = true;
}

void AudioService::persistIfPending() {
    bool shouldPersist;
    {
        auto lock = mutex.asScopedLock();
        lock.lock();
        shouldPersist = persistPending;
        persistPending = false;
    }

    if (!shouldPersist || streamDevice == nullptr) {
        return;
    }

    settings::audio::AudioSettings settings;
    settings.inputEnabled = isInputEnabled();
    settings.outputEnabled = isOutputEnabled();
    settings.inputMuted = isInputMuted();
    settings.outputMuted = isOutputMuted();
    settings.inputVolume = getInputVolume();
    settings.outputVolume = getOutputVolume();
    settings::audio::save(settings);
}

bool AudioService::onStart(ServiceContext& serviceContext) {
    findStreamDevice();

    if (streamDevice == nullptr) {
        LOGGER.info("No audio stream device found; service is unavailable");
        return true;
    }

    primeFromSettings();

    persistTimer = std::make_unique<Timer>(Timer::Type::Periodic, PERSIST_INTERVAL_TICKS, [this] {
        persistIfPending();
    });
    persistTimer->start();

    return true;
}

void AudioService::onStop(ServiceContext& serviceContext) {
    if (persistTimer) {
        persistTimer->stop();
        persistTimer.reset();
    }
    persistIfPending();
    streamDevice = nullptr;
}

bool AudioService::isAvailable() const {
    return streamDevice != nullptr;
}

bool AudioService::isInputEnabled() const {
    bool enabled = false;
    if (streamDevice != nullptr) {
        audio_stream_get_enabled(streamDevice, AUDIO_CODEC_DIR_INPUT, &enabled);
    }
    return enabled;
}

void AudioService::setInputEnabled(bool enabled) {
    if (streamDevice == nullptr) {
        return;
    }
    if (audio_stream_set_enabled(streamDevice, AUDIO_CODEC_DIR_INPUT, enabled) == ERROR_NONE) {
        schedulePersist();
        pubsub->publish(AudioEvent::InputEnabledChanged);
    }
}

bool AudioService::isOutputEnabled() const {
    bool enabled = false;
    if (streamDevice != nullptr) {
        audio_stream_get_enabled(streamDevice, AUDIO_CODEC_DIR_OUTPUT, &enabled);
    }
    return enabled;
}

void AudioService::setOutputEnabled(bool enabled) {
    if (streamDevice == nullptr) {
        return;
    }
    if (audio_stream_set_enabled(streamDevice, AUDIO_CODEC_DIR_OUTPUT, enabled) == ERROR_NONE) {
        schedulePersist();
        pubsub->publish(AudioEvent::OutputEnabledChanged);
    }
}

float AudioService::getInputVolume() const {
    float volume = 0.0f;
    if (streamDevice != nullptr) {
        audio_stream_get_volume(streamDevice, AUDIO_CODEC_DIR_INPUT, &volume);
    }
    return volume;
}

void AudioService::setInputVolume(float percent) {
    if (streamDevice == nullptr) {
        return;
    }
    if (audio_stream_set_volume(streamDevice, AUDIO_CODEC_DIR_INPUT, percent) == ERROR_NONE) {
        schedulePersist();
        pubsub->publish(AudioEvent::InputVolumeChanged);
    }
}

float AudioService::getOutputVolume() const {
    float volume = 0.0f;
    if (streamDevice != nullptr) {
        audio_stream_get_volume(streamDevice, AUDIO_CODEC_DIR_OUTPUT, &volume);
    }
    return volume;
}

void AudioService::setOutputVolume(float percent) {
    if (streamDevice == nullptr) {
        return;
    }
    if (audio_stream_set_volume(streamDevice, AUDIO_CODEC_DIR_OUTPUT, percent) == ERROR_NONE) {
        schedulePersist();
        pubsub->publish(AudioEvent::OutputVolumeChanged);
    }
}

bool AudioService::isInputMuted() const {
    bool muted = false;
    if (streamDevice != nullptr) {
        audio_stream_get_mute(streamDevice, AUDIO_CODEC_DIR_INPUT, &muted);
    }
    return muted;
}

void AudioService::setInputMuted(bool muted) {
    if (streamDevice == nullptr) {
        return;
    }
    if (audio_stream_set_mute(streamDevice, AUDIO_CODEC_DIR_INPUT, muted) == ERROR_NONE) {
        schedulePersist();
        pubsub->publish(AudioEvent::InputMuteChanged);
    }
}

bool AudioService::isOutputMuted() const {
    bool muted = false;
    if (streamDevice != nullptr) {
        audio_stream_get_mute(streamDevice, AUDIO_CODEC_DIR_OUTPUT, &muted);
    }
    return muted;
}

void AudioService::setOutputMuted(bool muted) {
    if (streamDevice == nullptr) {
        return;
    }
    if (audio_stream_set_mute(streamDevice, AUDIO_CODEC_DIR_OUTPUT, muted) == ERROR_NONE) {
        schedulePersist();
        pubsub->publish(AudioEvent::OutputMuteChanged);
    }
}

std::shared_ptr<AudioService> findAudioService() {
    auto service = findServiceById(manifest.id);
    assert(service != nullptr);
    return std::static_pointer_cast<AudioService>(service);
}

extern const ServiceManifest manifest = {
    .id = "Audio",
    .createService = create<AudioService>
};

} // namespace tt::service::audio
