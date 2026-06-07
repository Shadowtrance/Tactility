#include <Tactility/service/audio/Audio.h>
#include <Tactility/service/audio/AudioService.h>

namespace tt::service::audio {

std::shared_ptr<PubSub<AudioEvent>> getPubsub() {
    return findAudioService()->getPubsub();
}

bool isAvailable() {
    return findAudioService()->isAvailable();
}

bool isInputEnabled() {
    return findAudioService()->isInputEnabled();
}

void setInputEnabled(bool enabled) {
    findAudioService()->setInputEnabled(enabled);
}

bool isOutputEnabled() {
    return findAudioService()->isOutputEnabled();
}

void setOutputEnabled(bool enabled) {
    findAudioService()->setOutputEnabled(enabled);
}

float getInputVolume() {
    return findAudioService()->getInputVolume();
}

void setInputVolume(float percent) {
    findAudioService()->setInputVolume(percent);
}

float getOutputVolume() {
    return findAudioService()->getOutputVolume();
}

void setOutputVolume(float percent) {
    findAudioService()->setOutputVolume(percent);
}

bool isInputMuted() {
    return findAudioService()->isInputMuted();
}

void setInputMuted(bool muted) {
    findAudioService()->setInputMuted(muted);
}

bool isOutputMuted() {
    return findAudioService()->isOutputMuted();
}

void setOutputMuted(bool muted) {
    findAudioService()->setOutputMuted(muted);
}

} // namespace tt::service::audio
