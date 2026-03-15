#pragma once

#include "../core/Grid.hpp"

#include <imgui.h>

namespace pathsim {

class GridRenderer {
  public:
    // Draw the grid in an imgui window. called each frame

    void draw(const Grid& grid);

  private:
    // Converts a cellstate to a display color
    static ImU32 cell_color(CellState state);
};

} // namespace pathsim