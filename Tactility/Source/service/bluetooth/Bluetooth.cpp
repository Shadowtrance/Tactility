#include <Tactility/service/bluetooth/Bluetooth.h>

#include <Tactility/CoreDefines.h>
#include <tactility/check.h>
#include <Tactility/service/ServiceManifest.h>
#include <Tactility/service/ServiceRegistration.h>

namespace tt::service::bluetooth {

const char* radioStateToString(RadioState state) {
    switch (state) {
        using enum RadioState;
        case Off:
            return TT_STRINGIFY(Off);
        case OnPending:
            return TT_STRINGIFY(OnPending);
        case On:
            return TT_STRINGIFY(On);
        case OffPending:
            return TT_STRINGIFY(OffPending);
    }
    check(false, "not implemented");
}

extern const ServiceManifest manifest;

std::shared_ptr<ServiceContext> findServiceContext() {
    return findServiceContextById(manifest.id);
}

} // namespace tt::service::bluetooth
