#pragma once

#include "algo/Playback.hpp"
#include "core/Types.hpp"

namespace pathsim::stats_panel {

void draw(Playback& playback, Grid& grid, AlgoHistory& history);

} // namespace pathsim::stats_panel