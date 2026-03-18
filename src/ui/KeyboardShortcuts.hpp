#pragma once

#include "../algo/Playback.hpp"
#include "../core/Grid.hpp"
#include "GridRenderer.hpp"

namespace pathsim::keyboard_shortcuts {

void handle(Grid& grid, GridRenderer& renderer, Playback& playback);

} // namespace pathsim::keyboard_shortcuts