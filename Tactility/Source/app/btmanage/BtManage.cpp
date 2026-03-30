#include <Tactility/app/btmanage/BtManagePrivate.h>
#include <Tactility/app/btmanage/View.h>

#include <Tactility/app/AppContext.h>
#include <Tactility/app/AppManifest.h>
#include <Tactility/Logger.h>
#include <Tactility/LogMessages.h>
#include <Tactility/lvgl/LvglSync.h>

#include <tactility/lvgl_icon_shared.h>

namespace tt::app::btmanage {

static const auto LOGGER = Logger("BtManage");

extern const AppManifest manifest;

static void onBtToggled(bool enabled) {
    bluetooth::setEnabled(enabled);
}

static void onScanToggled(bool enabled) {
    if (enabled) {
        bluetooth::scanStart();
    } else {
        bluetooth::scanStop();
    }
}

static void onConnectPeer(const std::array<uint8_t, 6>& addr, int profileId) {
    bluetooth::connect(addr, profileId);
}

static void onDisconnectPeer(const std::array<uint8_t, 6>& addr, int profileId) {
    bluetooth::disconnect(addr, profileId);
}

static void onPairPeer(const std::array<uint8_t, 6>& addr) {
    // Clicking an unrecognised scan result initiates a HID host connection.
    // Bond exchange happens automatically during the first connection.
    bluetooth::hidHostConnect(addr);
}

static void onForgetPeer(const std::array<uint8_t, 6>& addr) {
    bluetooth::unpair(addr);
}

BtManage::BtManage() {
    bindings = (Bindings) {
        .onBtToggled = onBtToggled,
        .onScanToggled = onScanToggled,
        .onConnectPeer = onConnectPeer,
        .onDisconnectPeer = onDisconnectPeer,
        .onPairPeer = onPairPeer,
        .onForgetPeer = onForgetPeer,
    };
}

void BtManage::lock() {
    mutex.lock();
}

void BtManage::unlock() {
    mutex.unlock();
}

void BtManage::requestViewUpdate() {
    lock();
    if (isViewEnabled) {
        if (lvgl::lock(1000)) {
            view.update();
            lvgl::unlock();
        } else {
            LOGGER.error(LOG_MESSAGE_MUTEX_LOCK_FAILED_FMT, "LVGL");
        }
    }
    unlock();
}

void BtManage::onBtEvent(bluetooth::BtEvent event) {
    auto radio_state = bluetooth::getRadioState();
    LOGGER.info("Update with state {}", bluetooth::radioStateToString(radio_state));
    getState().setRadioState(radio_state);
    switch (event) {
        using enum bluetooth::BtEvent;
        case ScanStarted:
            getState().setScanning(true);
            break;
        case ScanFinished:
            getState().setScanning(false);
            getState().updateScanResults();
            getState().updatePairedPeers();
            break;
        case PeerFound:
            getState().updateScanResults();
            break;
        case PairSuccess:
        case PairFailed:
            getState().updatePairedPeers();
            break;
        case ProfileStateChanged:
            getState().updateScanResults();
            getState().updatePairedPeers();
            break;
        case RadioStateOn:
            getState().updatePairedPeers();
            if (!bluetooth::isScanning()) {
                bluetooth::scanStart();
            }
            break;
        default:
            break;
    }

    requestViewUpdate();
}

void BtManage::onShow(AppContext& app, lv_obj_t* parent) {
    // Initialise state and view before subscribing to avoid incoming events
    // racing with state initialisation.
    state.setRadioState(bluetooth::getRadioState());
    state.setScanning(bluetooth::isScanning());
    state.updateScanResults();
    state.updatePairedPeers();

    lock();
    isViewEnabled = true;
    view.init(app, parent);
    view.update();
    unlock();

    btSubscription = bluetooth::getPubsub()->subscribe([this](auto event) {
        onBtEvent(event);
    });

    auto radio_state = bluetooth::getRadioState();
    bool can_scan = radio_state == bluetooth::RadioState::On;
    LOGGER.info("Radio: {}, Scanning: {}, Can scan: {}",
        bluetooth::radioStateToString(radio_state),
        bluetooth::isScanning(),
        can_scan);
    if (can_scan && !bluetooth::isScanning()) {
        bluetooth::scanStart();
    }
}

void BtManage::onHide(AppContext& app) {
    lock();
    bluetooth::getPubsub()->unsubscribe(btSubscription);
    btSubscription = nullptr;
    isViewEnabled = false;
    unlock();
}

extern const AppManifest manifest = {
    .appId = "BtManage",
    .appName = "Bluetooth",
    .appIcon = LVGL_ICON_SHARED_BLUETOOTH,
    .appCategory = Category::Settings,
    .createApp = create<BtManage>
};

LaunchId start() {
    return app::start(manifest.appId);
}

} // namespace tt::app::btmanage
