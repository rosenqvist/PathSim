#include "GridRenderer.hpp"

#include <imgui.h>

namespace pathsim {

void GridRenderer::draw(const Grid& grid) {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
    ImGui::Begin("Grid", nullptr, flags);
    ImGui::PopStyleVar();

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 available = ImGui::GetContentRegionAvail();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float cell_w = available.x / static_cast<float>(grid.width());
    float cell_h = available.y / static_cast<float>(grid.height());

    for (int row = 0; row < grid.height(); ++row) {
        for (int col = 0; col < grid.width(); ++col) {
            Vec2i cell{.x = col, .y = row};

            float x_min = canvas_pos.x + (static_cast<float>(col) * cell_w);
            float y_min = canvas_pos.y + (static_cast<float>(row) * cell_h);
            float x_max = x_min + cell_w;
            float y_max = y_min + cell_h;

            ImVec2 top_left{x_min, y_min};
            ImVec2 bottom_right{x_max, y_max};

            draw_list->AddRectFilled(top_left, bottom_right, cell_color(grid.at(cell)));
            draw_list->AddRect(top_left, bottom_right, IM_COL32(40, 40, 40, 255));
        }
    }

    ImGui::Dummy(available);

    ImGui::End();
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