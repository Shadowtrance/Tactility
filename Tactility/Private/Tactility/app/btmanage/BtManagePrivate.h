#pragma once

#include "./View.h"
#include "./State.h"

#include <Tactility/app/App.h>

#include <Tactility/PubSub.h>
#include <Tactility/Mutex.h>
#include <Tactility/bluetooth/Bluetooth.h>

namespace tt::app::btmanage {

class BtManage final : public App {

    PubSub<bluetooth::BtEvent>::SubscriptionHandle btSubscription = nullptr;
    Mutex mutex;
    Bindings bindings = { };
    State state;
    View view = View(&bindings, &state);
    bool isViewEnabled = false;

    void onBtEvent(bluetooth::BtEvent event);

public:

    BtManage();

    void lock();
    void unlock();

    void onShow(AppContext& app, lv_obj_t* parent) override;
    void onHide(AppContext& app) override;

    Bindings& getBindings() { return bindings; }
    State& getState() { return state; }

    void requestViewUpdate();
};

} // namespace tt::app::btmanage
