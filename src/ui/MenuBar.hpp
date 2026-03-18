#pragma once

#include "../algo/Playback.hpp"
#include "../core/Grid.hpp"
#include "GridRenderer.hpp"
#include "ViewSettings.hpp"

namespace pathsim::menu_bar {

void draw(Grid& grid, GridRenderer& renderer, Playback& playback, AlgoHistory& history,
          ViewSettings& view);

} // namespace pathsim::menu_bar