#include "MenuBar.hpp"

#include "../algo/AStar.hpp"
#include "../algo/BFS.hpp"
#include "../algo/Dijkstra.hpp"
#include "../algo/MazeGen.hpp"
#include "../algo/MultiPath.hpp"

#include <imgui.h>

#include <array>
#include <cstdio>

namespace pathsim::menu_bar {
namespace {

void draw_algorithm_menu(Playback& playback, Grid& grid) {
    if (!ImGui::BeginMenu("Algorithm")) {
        return;
    }

    auto state = playback.state();

    bool can_run = state == PlaybackState::Idle || state == PlaybackState::Finished;

    if (ImGui::MenuItem("Run BFS", nullptr, false, can_run)) {
        playback.start(find_path_with_waypoints(
                           grid, [](const Grid& g, Vec2i s, Vec2i e) { return bfs(g, s, e); }),
                       grid);
    }
    if (ImGui::MenuItem("Run Dijkstra", nullptr, false, can_run)) {
        playback.start(find_path_with_waypoints(
                           grid, [](const Grid& g, Vec2i s, Vec2i e) { return dijkstra(g, s, e); }),
                       grid);
    }
    if (ImGui::MenuItem("Run A*", nullptr, false, can_run)) {
        playback.start(find_path_with_waypoints(
                           grid, [](const Grid& g, Vec2i s, Vec2i e) { return a_star(g, s, e); }),
                       grid);
    }

    ImGui::Separator();

    if (ImGui::MenuItem("Pause", nullptr, false, state == PlaybackState::Playing)) {
        playback.pause();
    }
    if (ImGui::MenuItem("Resume", nullptr, false, state == PlaybackState::Paused)) {
        playback.resume();
    }
    if (ImGui::MenuItem("Reset", nullptr, false, state != PlaybackState::Idle)) {
        playback.reset(grid);
    }

    ImGui::Separator();

    int speed = playback.speed();
    if (ImGui::SliderInt("Speed", &speed, 1, 50)) {
        playback.set_speed(speed);
    }

    ImGui::EndMenu();
}

void draw_maze_menu(Playback& playback, Grid& grid) {
    if (!ImGui::BeginMenu("Maze")) {
        return;
    }

    auto state = playback.state();
    bool can_generate = state == PlaybackState::Idle || state == PlaybackState::Finished;

    if (ImGui::MenuItem("Maze", nullptr, false, can_generate)) {
        playback.reset(grid);
        generate(grid, GenerateMode::Maze);
    }
    if (ImGui::MenuItem("Terrain", nullptr, false, can_generate)) {
        playback.reset(grid);
        generate(grid, GenerateMode::Terrain);
    }
    if (ImGui::MenuItem("Maze + Terrain", nullptr, false, can_generate)) {
        playback.reset(grid);
        generate(grid, GenerateMode::MazeTerrain);
    }

    ImGui::EndMenu();
}

void draw_settings_menu(Grid& grid) {
    if (!ImGui::BeginMenu("Settings")) {
        return;
    }

    bool diag = grid.allow_diagonals();
    if (ImGui::Checkbox("Allow Diagonals", &diag)) {
        grid.set_allow_diagonals(diag);
    }

    ImGui::EndMenu();
}

void draw_active_tool(const GridRenderer& renderer) {
    const char* tool_name = nullptr;
    switch (renderer.active_tool()) {
    case EditTool::Wall:
        tool_name = "Wall";
        break;
    case EditTool::Start:
        tool_name = "Start";
        break;
    case EditTool::End:
        tool_name = "End";
        break;
    case EditTool::Erase:
        tool_name = "Erase";
        break;
    case EditTool::Weight: {
        int w = renderer.weight_brush();
        std::array<char, 8> num{};
        std::snprintf(num.data(), num.size(), "%d", w);

        const char* prefix = "Tool: Weight Brush (";
        const char* suffix = ")";

        float total_w = ImGui::CalcTextSize(prefix).x + ImGui::CalcTextSize(num.data()).x +
                        ImGui::CalcTextSize(suffix).x;
        float bar_w = ImGui::GetMainViewport()->Size.x;
        ImGui::SameLine((bar_w - total_w) * 0.5F);

        ImVec4 white(1.0F, 1.0F, 1.0F, 0.9F);
        float t = static_cast<float>(w - 2) / 7.0F;
        float r{};
        float g{};
        float b{};
        if (w <= 1) {
            r = 0.8F;
            g = 0.8F;
            b = 0.8F;
        } else {
            r = (150.0F + (t * 115.0F)) / 255.0F;
            g = (160.0F + (t * 20.0F)) / 255.0F;
            b = (170.0F - (t * 130.0F)) / 255.0F;
        }

        ImGui::TextColored(white, "%s", prefix);
        ImGui::SameLine(0.0F, 0.0F);
        ImGui::TextColored(ImVec4(r, g, b, 1.0F), "%s", num.data());
        ImGui::SameLine(0.0F, 0.0F);
        ImGui::TextColored(white, "%s", suffix);
        return;
    }
    case EditTool::Waypoint:
        tool_name = "Waypoint";
        break;
    case EditTool::Impassable:
        tool_name = "Impassable";
        break;
    case EditTool::OneWay: {
        const char* dir_name = "East";
        switch (renderer.direction_brush()) {
        case CellDirection::North:
            dir_name = "North";
            break;
        case CellDirection::South:
            dir_name = "South";
            break;
        case CellDirection::East:
            dir_name = "East";
            break;
        case CellDirection::West:
            dir_name = "West";
            break;
        case CellDirection::None:
            dir_name = "None";
            break;
        }
        std::array<char, 32> label{};
        std::snprintf(label.data(), label.size(), "Tool: One-Way (%s)", dir_name);
        float text_w = ImGui::CalcTextSize(label.data()).x;
        float bar_w = ImGui::GetMainViewport()->Size.x;
        ImGui::SameLine((bar_w - text_w) * 0.5F);
        ImGui::TextColored(ImVec4(1.0F, 1.0F, 1.0F, 0.9F), "%s", label.data());
        return;
    }
    }

    std::array<char, 32> label{};
    std::snprintf(label.data(), label.size(), "Tool: %s", tool_name);
    float text_w = ImGui::CalcTextSize(label.data()).x;
    float bar_w = ImGui::GetMainViewport()->Size.x;
    ImGui::SameLine((bar_w - text_w) * 0.5F);
    ImGui::TextColored(ImVec4(1.0F, 1.0F, 1.0F, 0.9F), "%s", label.data());
}

void draw_hover_info(const Grid& grid, const GridRenderer& renderer) {
    if (!renderer.has_hovered_cell()) {
        return;
    }

    Vec2i cell = renderer.hovered_cell();
    if (!grid.is_valid(cell)) {
        return;
    }

    std::array<char, 48> label{};
    int w = grid.weight(cell);
    if (w > 1) {
        std::snprintf(label.data(), label.size(), "(%d, %d)  weight: %d", cell.x, cell.y, w);
    } else {
        std::snprintf(label.data(), label.size(), "(%d, %d)", cell.x, cell.y);
    }

    float text_width = ImGui::CalcTextSize(label.data()).x;
    float bar_width = ImGui::GetMainViewport()->Size.x;
    ImGui::SameLine(bar_width - text_width - 20.0F);
    ImGui::TextColored(ImVec4(1.0F, 1.0F, 1.0F, 0.9F), "%s", label.data());
}

} // namespace

void draw_tools_menu(GridRenderer& renderer) {
    if (!ImGui::BeginMenu("Tools")) {
        return;
    }

    auto tool = renderer.active_tool();
    if (ImGui::MenuItem("Wall", nullptr, tool == EditTool::Wall)) {
        renderer.set_tool(EditTool::Wall);
    }
    if (ImGui::MenuItem("Start", nullptr, tool == EditTool::Start)) {
        renderer.set_tool(EditTool::Start);
    }
    if (ImGui::MenuItem("Impassable", nullptr, tool == EditTool::Impassable)) {
        renderer.set_tool(EditTool::Impassable);
    }
    if (ImGui::MenuItem("One-Way", nullptr, tool == EditTool::OneWay)) {
        renderer.set_tool(EditTool::OneWay);
    }
    if (ImGui::MenuItem("Waypoint", nullptr, tool == EditTool::Waypoint)) {
        renderer.set_tool(EditTool::Waypoint);
    }
    if (ImGui::MenuItem("End", nullptr, tool == EditTool::End)) {
        renderer.set_tool(EditTool::End);
    }
    if (ImGui::MenuItem("Erase", nullptr, tool == EditTool::Erase)) {
        renderer.set_tool(EditTool::Erase);
    }
    ImGui::Separator();

    if (ImGui::MenuItem("Weight Brush", nullptr, tool == EditTool::Weight)) {
        renderer.set_tool(EditTool::Weight);
    }

    if (tool == EditTool::Weight) {
        int weight = renderer.weight_brush();
        if (ImGui::SliderInt("Weight", &weight, 1, 9)) {
            renderer.set_weight_brush(weight);
        }
    }

    if (tool == EditTool::OneWay) {
        auto dir = renderer.direction_brush();
        if (ImGui::MenuItem("North", nullptr, dir == CellDirection::North)) {
            renderer.set_direction_brush(CellDirection::North);
        }
        if (ImGui::MenuItem("South", nullptr, dir == CellDirection::South)) {
            renderer.set_direction_brush(CellDirection::South);
        }
        if (ImGui::MenuItem("East", nullptr, dir == CellDirection::East)) {
            renderer.set_direction_brush(CellDirection::East);
        }
        if (ImGui::MenuItem("West", nullptr, dir == CellDirection::West)) {
            renderer.set_direction_brush(CellDirection::West);
        }
    }

    ImGui::EndMenu();
}

void draw(Grid& grid, GridRenderer& renderer, Playback& playback) {
    if (!ImGui::BeginMainMenuBar()) {
        return;
    }

    if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Clear Grid")) {
            playback.reset(grid);
            grid.clear();
        }
        ImGui::EndMenu();
    }

    draw_tools_menu(renderer);
    draw_algorithm_menu(playback, grid);
    draw_maze_menu(playback, grid);
    draw_settings_menu(grid);
    draw_active_tool(renderer);
    draw_hover_info(grid, renderer);

    ImGui::EndMainMenuBar();
}
} // namespace pathsim::menu_bar