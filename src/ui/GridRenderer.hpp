#pragma once

#include "../core/Grid.hpp"

#include <imgui.h>

#include <cstdint>

namespace pathsim {

enum class EditTool : std::uint8_t { Wall, Start, End, Erase };

enum class DragTarget : std::uint8_t { None, Start, End };

class GridRenderer {
  public:
    // Draw the grid in an imgui window. called each frame
    void draw(const Grid& grid);
    void handle_input(Grid& grid, bool editing_enabled = true);

    void set_tool(EditTool tool);
    [[nodiscard]] EditTool active_tool() const;

  private:
    // Converts a cellstate to a display color
    static ImU32 cell_color(CellState state);

    // Converts screen coordinates to grid position. Used for dragging start/end
    [[nodiscard]] Vec2i screen_to_grid(ImVec2 screen_pos) const;

    EditTool active_tool_ = EditTool::Wall;
    DragTarget drag_target_ = DragTarget::None;
    float cell_w_{};
    float cell_h_{};
    ImVec2 grid_origin_ = {0.0F, 0.0F};
    bool is_hovered_{false};
};

} // namespace pathsim