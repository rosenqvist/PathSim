#pragma once

#include "../core/Grid.hpp"
#include "core/Types.hpp"

#include <imgui.h>

#include <cstdint>

namespace pathsim {

enum class EditTool : std::uint8_t { Wall, Start, End, Erase, Weight };

enum class DragTarget : std::uint8_t { None, Start, End };

class GridRenderer {
  public:
    // Draw the grid in an imgui window. called each frame
    void draw(const Grid& grid);
    void handle_input(Grid& grid, bool editing_enabled = true);

    void set_tool(EditTool tool);
    [[nodiscard]] EditTool active_tool() const;

    void set_weight_brush(int weight);
    [[nodiscard]] int weight_brush() const;

    [[nodiscard]] Vec2i hovered_cell() const;
    [[nodiscard]] bool has_hovered_cell() const;

  private:
    // Converts a cellstate to a display color
    static ImU32 cell_color(CellState state, int weight = 1);

    // Converts screen coordinates to grid position. Used for dragging start/end
    [[nodiscard]] Vec2i screen_to_grid(ImVec2 screen_pos) const;

    EditTool active_tool_ = EditTool::Wall;
    DragTarget drag_target_ = DragTarget::None;
    float cell_w_{};
    float cell_h_{};
    ImVec2 grid_origin_ = {0.0F, 0.0F};
    bool is_hovered_{false};

    int active_weight_ = 3;

    Vec2i hovered_cell_{.x = -1, .y = -1};
    bool has_hover_{false};
};

} // namespace pathsim