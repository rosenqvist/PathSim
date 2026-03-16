#include "GridRenderer.hpp"

#include <imgui.h>

namespace pathsim {

void GridRenderer::draw(const Grid& grid) {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar |
                             ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
    ImGui::Begin("Grid", nullptr, flags);
    ImGui::PopStyleVar();

    ImVec2 available = ImGui::GetContentRegionAvail();
    grid_origin_ = ImGui::GetCursorScreenPos();
    is_hovered_ = ImGui::IsWindowHovered();
    cell_w_ = available.x / static_cast<float>(grid.width());
    cell_h_ = available.y / static_cast<float>(grid.height());

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    for (int row = 0; row < grid.height(); ++row) {
        for (int col = 0; col < grid.width(); ++col) {
            Vec2i cell{.x = col, .y = row};

            float x_min = grid_origin_.x + (static_cast<float>(col) * cell_w_);
            float y_min = grid_origin_.y + (static_cast<float>(row) * cell_h_);
            float x_max = x_min + cell_w_;
            float y_max = y_min + cell_h_;

            ImVec2 top_left{x_min, y_min};
            ImVec2 bottom_right{x_max, y_max};

            draw_list->AddRectFilled(top_left, bottom_right, cell_color(grid.at(cell)));
            draw_list->AddRect(top_left, bottom_right, IM_COL32(40, 40, 40, 255));
        }
    }

    ImGui::Dummy(available);
    ImGui::End();
}

void GridRenderer::handle_input(Grid& grid, bool editing_enabled) {
    // Implementation for handling input
    if (!is_hovered_ || cell_w_ <= 0.0F || cell_h_ <= 0.0F) {
        return;
    }

    // prevent editing during playback
    if (!editing_enabled) {
        return;
    }

    Vec2i mouse_pos = screen_to_grid(ImGui::GetMousePos());

    if (!grid.is_valid(mouse_pos)) {
        return;
    }

    // Begin drag if mouse just clicked on start or end
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (mouse_pos == grid.start()) {
            drag_target_ = DragTarget::Start;
        } else if (mouse_pos == grid.end()) {
            drag_target_ = DragTarget::End;
        }
    }

    // Handle ongoing drag
    if (drag_target_ != DragTarget::None) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            if (drag_target_ == DragTarget::Start && mouse_pos != grid.end()) {
                grid.set_start(mouse_pos);
            } else if (drag_target_ == DragTarget::End && mouse_pos != grid.start()) {
                grid.set_end(mouse_pos);
            }
            return; // drag takes priority, skip normal tool handling
        }
        drag_target_ = DragTarget::None;
    }

    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        switch (active_tool_) {
        case EditTool::Wall:
            grid.set_wall(mouse_pos, true);
            break;
        case EditTool::Start:
            grid.set_start(mouse_pos);
            break;
        case EditTool::End:
            grid.set_end(mouse_pos);
            break;
        case EditTool::Erase:
            grid.set_wall(mouse_pos, false);
            break;
        }
    }

    // Right-click always erases regardless of active tool
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        grid.set_wall(mouse_pos, false);
    }
}

Vec2i GridRenderer::screen_to_grid(ImVec2 screen_pos) const {
    int x = static_cast<int>((screen_pos.x - grid_origin_.x) / cell_w_);
    int y = static_cast<int>((screen_pos.y - grid_origin_.y) / cell_h_);
    return {.x = x, .y = y};
}

void GridRenderer::set_tool(EditTool tool) {
    active_tool_ = tool;
}

EditTool GridRenderer::active_tool() const {
    return active_tool_;
}

ImU32 GridRenderer::cell_color(CellState state) {
    switch (state) {
    case CellState::Empty:
        return IM_COL32(30, 30, 30, 255);
    case CellState::Wall:
        return IM_COL32(180, 180, 180, 255);
    case CellState::Start:
        return IM_COL32(0, 200, 80, 255);
    case CellState::End:
        return IM_COL32(220, 50, 50, 255);
    case CellState::Visited:
        return IM_COL32(60, 100, 180, 255);
    case CellState::Frontier:
        return IM_COL32(100, 180, 240, 255);
    case CellState::Path:
        return IM_COL32(255, 220, 50, 255);
    }
    return IM_COL32(0, 0, 0, 255);
}

} // namespace pathsim