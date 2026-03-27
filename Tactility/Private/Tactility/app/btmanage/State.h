#pragma once

#include <Tactility/service/bluetooth/Bluetooth.h>
#include <Tactility/RecursiveMutex.h>

namespace tt::app::btmanage {

class State final {

    mutable RecursiveMutex mutex;
    bool scanning = false;
    service::bluetooth::RadioState radioState = service::bluetooth::RadioState::Off;
    std::vector<service::bluetooth::PeerRecord> scanResults;
    std::vector<service::bluetooth::PeerRecord> pairedPeers;

public:
    State() = default;

    void setScanning(bool isScanning);
    bool isScanning() const;

    void setRadioState(service::bluetooth::RadioState state);
    service::bluetooth::RadioState getRadioState() const;

    void updateScanResults();
    void updatePairedPeers();

    std::vector<service::bluetooth::PeerRecord> getScanResults() const {
        auto lock = mutex.asScopedLock();
        lock.lock();
        return scanResults;
    }

    std::vector<service::bluetooth::PeerRecord> getPairedPeers() const {
        auto lock = mutex.asScopedLock();
        lock.lock();
        return pairedPeers;
    }
};

} // namespace tt::app::btmanage
