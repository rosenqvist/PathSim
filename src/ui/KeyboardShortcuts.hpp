#pragma once

#include "algo/Playback.hpp"
#include "core/Grid.hpp"
#include "GridRenderer.hpp"
#include "ViewSettings.hpp"

namespace pathsim::keyboard_shortcuts {

void handle(Grid& grid, GridRenderer& renderer, Playback& playback, ViewSettings& view);

} // namespace pathsim::keyboard_shortcuts