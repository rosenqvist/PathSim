#pragma once

#include "../algo/Playback.hpp"
#include "../core/Grid.hpp"
#include "GridRenderer.hpp"

namespace pathsim::menu_bar {

void draw(Grid& grid, GridRenderer& renderer, Playback& playback);

} // namespace pathsim::menu_bar
