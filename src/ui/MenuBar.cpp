#include "MenuBar.hpp"

#include "../algo/AlgoUtils.hpp"
#include "../algo/AStar.hpp"
#include "../algo/BFS.hpp"
#include "../algo/Dijkstra.hpp"
#include "../algo/MazeGen.hpp"
#include "../algo/MultiPath.hpp"

#include <imgui.h>

#include <array>
#include <chrono>
#include <cstdio>

namespace pathsim::menu_bar {
namespace {

void run_algorithm(Playback& playback, Grid& grid, AlgoHistory& history, const AlgoFunc& algo) {
    history.clear();

    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = find_path_with_waypoints(grid, algo);
    auto t1 = std::chrono::high_resolution_clock::now();

    result.compute_time_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    history[result.algorithm_name] = algo_utils::extract_stats(result);
    playback.start(std::move(result), grid);
}

void draw_algorithm_menu(Playback& playback, Grid& grid, AlgoHistory& history) {
    if (!ImGui::BeginMenu("Algorithm")) {
        return;
    }

    auto state = playback.state();
    bool can_run = state == PlaybackState::Idle || state == PlaybackState::Finished;

    if (ImGui::MenuItem("Run BFS", nullptr, false, can_run)) {
        run_algorithm(playback, grid, history,
                      [](const Grid& g, Vec2i s, Vec2i e) { return bfs(g, s, e); });
    }
    if (ImGui::MenuItem("Run Dijkstra", nullptr, false, can_run)) {
        run_algorithm(playback, grid, history,
                      [](const Grid& g, Vec2i s, Vec2i e) { return dijkstra(g, s, e); });
    }
    if (ImGui::MenuItem("Run A*", nullptr, false, can_run)) {
        run_algorithm(playback, grid, history,
                      [](const Grid& g, Vec2i s, Vec2i e) { return a_star(g, s, e); });
    }

    ImGui::Separator();

    if (ImGui::MenuItem("Run All", nullptr, false, can_run)) {
        auto run_and_record = [&](const AlgoFunc& algo) {
            auto t0 = std::chrono::high_resolution_clock::now();
            auto result = find_path_with_waypoints(grid, algo);
            auto t1 = std::chrono::high_resolution_clock::now();
            result.compute_time_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
            history[result.algorithm_name] = algo_utils::extract_stats(result);
            return result;
        };

        run_and_record([](const Grid& g, Vec2i s, Vec2i e) { return bfs(g, s, e); });
        run_and_record([](const Grid& g, Vec2i s, Vec2i e) { return dijkstra(g, s, e); });

        // A* runs last and becomes the displayed playback
        auto result =
            run_and_record([](const Grid& g, Vec2i s, Vec2i e) { return a_star(g, s, e); });
        playback.start(std::move(result), grid);
    }

    ImGui::EndMenu();
}

void draw_playback_menu(Playback& playback, Grid& grid) {
    if (!ImGui::BeginMenu("Playback")) {
        return;
    }

    auto state = playback.state();

    if (ImGui::MenuItem("Pause", "Space", false, state == PlaybackState::Playing)) {
        playback.pause();
    }
    if (ImGui::MenuItem("Resume", "Space", false, state == PlaybackState::Paused)) {
        playback.resume();
    }
    if (ImGui::MenuItem("Step Forward", ".", false, state == PlaybackState::Paused)) {
        playback.step_forward(grid);
    }
    if (ImGui::MenuItem("Reset", "R", false, state != PlaybackState::Idle)) {
        playback.reset(grid);
    }

    ImGui::Separator();

    int speed = playback.speed();
    if (ImGui::SliderInt("Speed", &speed, 1, 50)) {
        playback.set_speed(speed);
    }

    ImGui::Text("Progress: %d / %d", playback.current_step(), playback.total_steps());

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

void draw_view_menu(ViewSettings& view) {
    if (!ImGui::BeginMenu("View")) {
        return;
    }

    ImGui::Checkbox("Show Heatmap", &view.show_heatmap);

    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Dark blue = explored early, bright cyan = explored late.\n"
                          "Shows how each algorithm expands its search.");
    }

    ImGui::Checkbox("Show Path Direction", &view.show_path_direction);

    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Draw arrows along the path showing direction of travel.");
    }

    ImGui::EndMenu();
}

void draw_diagonal_status(const Grid& grid) {
    const char* label = grid.allow_diagonals() ? "Diagonal Movement: On" : "Diagonal Movement: Off";
    float text_w = ImGui::CalcTextSize(label).x;
    float bar_w = ImGui::GetMainViewport()->Size.x;
    ImGui::SameLine((bar_w * 0.3F) - (text_w * 0.5F));

    ImVec4 color =
        grid.allow_diagonals() ? ImVec4(0.3F, 0.8F, 0.4F, 0.9F) : ImVec4(0.6F, 0.6F, 0.6F, 0.5F);
    ImGui::TextColored(color, "%s", label);
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

void draw(Grid& grid, GridRenderer& renderer, Playback& playback, AlgoHistory& history,
          ViewSettings& view) {
    if (!ImGui::BeginMainMenuBar()) {
        return;
    }

    if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Clear Grid")) {
            playback.reset(grid);
            grid.clear();
            history.clear();
        }
        ImGui::EndMenu();
    }

    draw_tools_menu(renderer);
    draw_algorithm_menu(playback, grid, history);
    draw_playback_menu(playback, grid);
    draw_maze_menu(playback, grid);
    draw_view_menu(view);
    draw_settings_menu(grid);
    draw_diagonal_status(grid);
    draw_active_tool(renderer);
    draw_hover_info(grid, renderer);

    ImGui::EndMainMenuBar();
}
} // namespace pathsim::menu_bar