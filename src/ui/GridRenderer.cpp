#include "GridRenderer.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstdio>

namespace pathsim {

void GridRenderer::draw_weight_label(ImDrawList* draw_list, const Grid& grid, Vec2i cell,
                                     float x_min, float y_min) const {
    std::array<char, 4> label{};
    std::snprintf(label.data(), label.size(), "%d", grid.weight(cell));

    ImVec2 text_size = ImGui::CalcTextSize(label.data());
    float text_x = x_min + ((cell_w_ - text_size.x) * 0.5F);
    float text_y = y_min + ((cell_h_ - text_size.y) * 0.5F);

    draw_list->AddText(ImVec2{text_x, text_y}, IM_COL32(255, 255, 255, 220), label.data());
}

void GridRenderer::draw_impassable(ImDrawList* draw_list, float x_min, float y_min, float x_max,
                                   float y_max) const {
    ImU32 hatch_col = IM_COL32(200, 50, 50, 180);
    float spacing = cell_w_ * 0.25F;
    for (float offset = 0.0F; offset < cell_w_ + cell_h_; offset += spacing) {
        float lx0 = x_min + offset;
        float ly0 = y_min;
        float lx1 = x_min;
        float ly1 = y_min + offset;

        if (lx0 > x_max) {
            ly0 += (lx0 - x_max);
            lx0 = x_max;
        }
        if (ly1 > y_max) {
            lx1 += (ly1 - y_max);
            ly1 = y_max;
        }

        if (lx0 >= x_min && ly0 <= y_max && lx1 <= x_max && ly1 >= y_min) {
            draw_list->AddLine(ImVec2{lx0, ly0}, ImVec2{lx1, ly1}, hatch_col, 1.5F);
        }
    }
}

void GridRenderer::draw_waypoint(ImDrawList* draw_list, const Grid& grid, Vec2i cell, float x_min,
                                 float y_min) const {
    const auto& wps = grid.waypoints();
    for (std::size_t i = 0; i < wps.size(); ++i) {
        if (wps[i] == cell) {
            std::array<char, 4> wp_label{};
            std::snprintf(wp_label.data(), wp_label.size(), "%d", static_cast<int>(i + 1));
            ImVec2 text_size = ImGui::CalcTextSize(wp_label.data());
            float text_x = x_min + ((cell_w_ - text_size.x) * 0.5F);
            float text_y = y_min + ((cell_h_ - text_size.y) * 0.5F);
            draw_list->AddText(ImVec2{text_x, text_y}, IM_COL32(255, 255, 255, 255),
                               wp_label.data());
            break;
        }
    }
}

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

            draw_list->AddRectFilled({x_min, y_min}, {x_max, y_max},
                                     cell_color(grid.at(cell), grid.weight(cell)));
            draw_list->AddRect({x_min, y_min}, {x_max, y_max}, IM_COL32(40, 40, 40, 255));

            if (grid.weight(cell) > 1 && grid.at(cell) == CellState::Empty) {
                draw_weight_label(draw_list, grid, cell, x_min, y_min);
            }
            if (grid.at(cell) == CellState::Impassable) {
                draw_impassable(draw_list, x_min, y_min, x_max, y_max);
            }
            if (grid.at(cell) == CellState::Waypoint) {
                draw_waypoint(draw_list, grid, cell, x_min, y_min);
            }
        }
    }

    // Hover highlight
    if (is_hovered_) {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        int hx = static_cast<int>((mouse_pos.x - grid_origin_.x) / cell_w_);
        int hy = static_cast<int>((mouse_pos.y - grid_origin_.y) / cell_h_);
        Vec2i hover_cell{.x = hx, .y = hy};

        has_hover_ = grid.is_valid(hover_cell);
        hovered_cell_ = hover_cell;

        if (has_hover_) {
            float x_min = grid_origin_.x + (static_cast<float>(hx) * cell_w_);
            float y_min = grid_origin_.y + (static_cast<float>(hy) * cell_h_);
            draw_list->AddRect(ImVec2{x_min, y_min}, ImVec2{x_min + cell_w_, y_min + cell_h_},
                               IM_COL32(255, 255, 255, 120), 0.0F, 0, 2.0F);
        }
    } else {
        has_hover_ = false;
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
        case EditTool::Weight:
            grid.set_weight(mouse_pos, active_weight_);
            break;
        case EditTool::Waypoint:
            grid.add_waypoint(mouse_pos);
            break;
        case EditTool::Impassable:
            grid.set_impassable(mouse_pos, true);
            break;
        }
    }

    // Right-click always erases regardless of active tool
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        grid.set_wall(mouse_pos, false);
        grid.set_weight(mouse_pos, 1);
        grid.remove_waypoint(mouse_pos);
        grid.set_impassable(mouse_pos, false);
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

void GridRenderer::set_weight_brush(int weight) {
    active_weight_ = std::clamp(weight, 1, 9);
}

int GridRenderer::weight_brush() const {
    return active_weight_;
}

Vec2i GridRenderer::hovered_cell() const {
    return hovered_cell_;
}

bool GridRenderer::has_hovered_cell() const {
    return has_hover_;
}

ImU32 GridRenderer::cell_color(CellState state, int weight) {
    switch (state) {
    case CellState::Empty: {
        if (weight <= 1) {
            return IM_COL32(30, 30, 30, 255);
        }
        float t = static_cast<float>(weight - 2) / 7.0F;
        auto r = static_cast<uint8_t>(80.0F + (t * 140.0F));
        auto g = static_cast<uint8_t>(90.0F + (t * 30.0F));
        auto b = static_cast<uint8_t>(120.0F - (t * 90.0F));
        return IM_COL32(r, g, b, 255);
    }
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
    case CellState::Waypoint:
        return IM_COL32(255, 160, 0, 255);
        break;
    case CellState::Impassable:
        return IM_COL32(50, 20, 20, 255);
        break;
    }
    return IM_COL32(0, 0, 0, 255);
}

} // namespace pathsim