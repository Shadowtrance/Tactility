#pragma once

#include <string>
#include <vector>

namespace tt::settings::quickpanel {

struct QuickPanelSettings {
    /** Ordered list of tile IDs, e.g. "mic_on", "wifi". Empty means use built-in default order. */
    std::vector<std::string> tileOrder;
    /** Tile IDs that are hidden from the drawer. */
    std::vector<std::string> hiddenTileIds;
};

bool load(QuickPanelSettings& settings);

QuickPanelSettings getDefault();

QuickPanelSettings loadOrGetDefault();

bool save(const QuickPanelSettings& settings);

} // namespace tt::settings::quickpanel
