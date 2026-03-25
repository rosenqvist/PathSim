#include "StatsPanel.hpp"

#include <imgui.h>

namespace pathsim::stats_panel {

// Stats moved to the menu bar.
// Kept so  main.cpp doesn't need changes
void draw(Playback& /*playback*/, Grid& /*grid*/, AlgoHistory& /*history*/) {}

} // namespace pathsim::stats_panel