#pragma once

#include <Tactility/PubSub.h>

#include <memory>

namespace tt::service::audio {

enum class AudioEvent {
    InputEnabledChanged,
    OutputEnabledChanged,
    InputVolumeChanged,
    OutputVolumeChanged,
    InputMuteChanged,
    OutputMuteChanged
};

/** @return the audio pubsub that broadcasts AudioEvent objects */
std::shared_ptr<PubSub<AudioEvent>> getPubsub();

/** @return true when an AUDIO_STREAM_TYPE device is bound and ready */
bool isAvailable();

bool isInputEnabled();
void setInputEnabled(bool enabled);

bool isOutputEnabled();
void setOutputEnabled(bool enabled);

float getInputVolume();
void setInputVolume(float percent);

float getOutputVolume();
void setOutputVolume(float percent);

bool isInputMuted();
void setInputMuted(bool muted);

bool isOutputMuted();
void setOutputMuted(bool muted);

} // namespace tt::service::audio
