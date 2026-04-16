#include "MenuBar.hpp"

#include "algo/AlgoUtils.hpp"
#include "algo/AStar.hpp"
#include "algo/BFS.hpp"
#include "algo/Dijkstra.hpp"
#include "algo/MazeGen.hpp"
#include "algo/MultiPath.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <string>

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

void draw_settings_menu(Grid& grid, Playback& playback, AlgoHistory& history) {
    if (!ImGui::BeginMenu("Settings")) {
        return;
    }

    bool diag = grid.allow_diagonals();
    if (ImGui::Checkbox("Allow Diagonals", &diag)) {
        grid.set_allow_diagonals(diag);
    }

    bool ordered = grid.ordered_waypoints();
    if (ImGui::Checkbox("Ordered Waypoints", &ordered)) {
        grid.set_ordered_waypoints(ordered);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("When on, waypoints are visited in order (1 -> 2 -> 3).\n"
                          "When off, the algorithm finds the cheapest visit order.");
    }

    ImGui::Separator();

    static int pending_width = 40;
    static int pending_height = 30;
    static bool first_open = true;
    if (first_open) {
        pending_width = grid.width();
        pending_height = grid.height();
        first_open = false;
    }

    ImGui::Spacing();
    ImGui::SetNextItemWidth(100.0F);
    ImGui::InputScalar("Width", ImGuiDataType_S32, &pending_width);
    pending_width = std::clamp(pending_width, 5, 100);

    ImGui::Spacing();
    ImGui::SetNextItemWidth(100.0F);
    ImGui::InputScalar("Height", ImGuiDataType_S32, &pending_height);
    pending_height = std::clamp(pending_height, 5, 100);

    ImGui::Spacing();
    if (ImGui::Button("Resize Grid")) {
        playback.reset(grid);
        history.clear();
        grid.resize(pending_width, pending_height);
        pending_width = grid.width();
        pending_height = grid.height();
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

    ImGui::Checkbox("Show Cell Tooltips", &view.show_tooltips);

    ImGui::EndMenu();
}

void draw_diagonal_status(Grid& grid, float menus_end_x) {
    const char* label = grid.allow_diagonals() ? "Diagonal Movement: On" : "Diagonal Movement: Off";
    float pos_x = menus_end_x + 20.0F;
    ImGui::SameLine(pos_x);

    ImVec4 color =
        grid.allow_diagonals() ? ImVec4(0.3F, 0.8F, 0.4F, 0.9F) : ImVec4(0.6F, 0.6F, 0.6F, 0.5F);

    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1, 1, 1, 0.1F));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1, 1, 1, 0.2F));
    ImGui::PushStyleColor(ImGuiCol_Text, color);

    if (ImGui::SmallButton(label)) {
        grid.set_allow_diagonals(!grid.allow_diagonals());
    }

    ImGui::PopStyleColor(4);
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

void run_comparison(const Grid& grid, AlgoHistory& history, const AlgoFunc& algo) {
    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = find_path_with_waypoints(grid, algo);
    auto t1 = std::chrono::high_resolution_clock::now();

    result.compute_time_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    history[result.algorithm_name] = algo_utils::extract_stats(result);
}

void draw_stats_comparison(const PathResult& result, const AlgoHistory& history) {
    for (const auto& [name, stats] : history) {
        if (name == result.algorithm_name) {
            continue;
        }

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.7F, 0.7F, 0.9F, 1.0F), "%s", stats.algorithm_name.c_str());

        if (stats.compute_time_ms > 0.0F) {
            ImGui::Text("  Time: %.2f ms", static_cast<double>(stats.compute_time_ms));
        }

        ImGui::Text("  Nodes explored: %d", stats.nodes_visited);

        if (stats.max_frontier_size > 0) {
            ImGui::Text("  Peak frontier: %d", stats.max_frontier_size);
        }

        if (!result.path.empty() && stats.path_length > 0) {
            ImGui::Text("  Path cost: %.0f", static_cast<double>(stats.path_cost));

            float cost_diff = stats.path_cost - result.path_cost;
            if (cost_diff > 0.1F) {
                float pct = (cost_diff / result.path_cost) * 100.0F;
                ImGui::TextColored(ImVec4(1.0F, 0.5F, 0.3F, 1.0F), "  +%.0f%% more expensive",
                                   static_cast<double>(pct));
            } else if (cost_diff < -0.1F) {
                float pct = (-cost_diff / stats.path_cost) * 100.0F;
                ImGui::TextColored(ImVec4(0.3F, 1.0F, 0.5F, 1.0F), "  %.0f%% cheaper",
                                   static_cast<double>(pct));
            } else {
                ImGui::TextColored(ImVec4(0.7F, 0.7F, 0.7F, 1.0F), "  Same cost");
            }
        } else if (stats.path_length == 0) {
            ImGui::TextColored(ImVec4(1.0F, 0.3F, 0.3F, 1.0F), "  No path found");
        }
    }
}

void draw_copy_button(const PathResult& result, const AlgoHistory& history) {
    if (!ImGui::Button("Copy Stats")) {
        return;
    }

    std::string stats;
    stats += result.algorithm_name + "\n";
    stats += "Nodes explored: " + std::to_string(result.nodes_visited) + "\n";
    stats += "Peak frontier: " + std::to_string(result.max_frontier_size) + "\n";
    stats +=
        "Path length: " + std::to_string(static_cast<int>(result.path.size()) - 1) + " steps\n";
    stats += "Path cost: " + std::to_string(result.path_cost) + "\n";

    std::array<char, 32> time_buf{};
    std::snprintf(time_buf.data(), time_buf.size(), "%.2f",
                  static_cast<double>(result.compute_time_ms));
    stats += "Compute time: " + std::string(time_buf.data()) + " ms\n";

    for (const auto& [name, cmp] : history) {
        if (name == result.algorithm_name) {
            continue;
        }

        stats += "\n" + cmp.algorithm_name + "\n";
        stats += "  Nodes explored: " + std::to_string(cmp.nodes_visited) + "\n";
        stats += "  Peak frontier: " + std::to_string(cmp.max_frontier_size) + "\n";
        stats += "  Path length: " + std::to_string(std::max(0, cmp.path_length - 1)) + " steps\n";
        stats += "  Path cost: " + std::to_string(cmp.path_cost) + "\n";

        std::array<char, 32> cmp_time{};
        std::snprintf(cmp_time.data(), cmp_time.size(), "%.2f",
                      static_cast<double>(cmp.compute_time_ms));
        stats += "  Compute time: " + std::string(cmp_time.data()) + " ms\n";

        if (!result.path.empty() && cmp.path_length > 0) {
            float cost_diff = cmp.path_cost - result.path_cost;
            if (cost_diff > 0.1F) {
                float pct = (cost_diff / result.path_cost) * 100.0F;
                std::array<char, 32> pct_buf{};
                std::snprintf(pct_buf.data(), pct_buf.size(), "+%.0f%%", static_cast<double>(pct));
                stats += "  " + std::string(pct_buf.data()) + " more expensive\n";
            } else if (cost_diff < -0.1F) {
                float pct = (-cost_diff / cmp.path_cost) * 100.0F;
                std::array<char, 32> pct_buf{};
                std::snprintf(pct_buf.data(), pct_buf.size(), "%.0f%%", static_cast<double>(pct));
                stats += "  " + std::string(pct_buf.data()) + " cheaper\n";
            } else {
                stats += "  Same cost\n";
            }
        }
    }

    ImGui::SetClipboardText(stats.c_str());
}

void draw_stats_menu(Playback& playback, Grid& grid, AlgoHistory& history) {
    // Only show when an algorithm has been run
    if (playback.state() == PlaybackState::Idle) {
        return;
    }

    if (!ImGui::BeginMenu("Stats")) {
        return;
    }

    const auto& result = playback.result();

    ImGui::TextColored(ImVec4(0.4F, 0.8F, 1.0F, 1.0F), "Algorithm: %s",
                       result.algorithm_name.c_str());

    if (result.compute_time_ms > 0.0F) {
        ImGui::Text("Compute time: %.2f ms", static_cast<double>(result.compute_time_ms));
    }

    ImGui::Text("Progress: %d / %d", playback.current_step(), playback.total_steps());

    ImGui::Separator();

    if (playback.state() == PlaybackState::Finished) {
        ImGui::Text("Nodes explored: %d", result.nodes_visited);

        if (result.max_frontier_size > 0) {
            ImGui::Text("Peak frontier: %d", result.max_frontier_size);
        }

        if (result.path.empty()) {
            ImGui::TextColored(ImVec4(1.0F, 0.3F, 0.3F, 1.0F), "No path found");
        } else {
            ImGui::Text("Path length: %d steps", static_cast<int>(result.path.size()) - 1);
            ImGui::Text("Path cost: %.0f", static_cast<double>(result.path_cost));

            if (result.algorithm_name == "BFS") {
                ImGui::TextColored(ImVec4(1.0F, 0.8F, 0.3F, 1.0F),
                                   "BFS optimizes for fewest hops, not lowest cost");
            } else {
                ImGui::TextColored(ImVec4(0.3F, 0.8F, 0.4F, 1.0F),
                                   "%s optimizes for lowest total cost",
                                   result.algorithm_name.c_str());
            }
        }

        ImGui::Separator();

        // Compare buttons for the algorithms that haven't been compared yet
        struct AlgoEntry {
            const char* name;
            const char* button_label;
            AlgoFunc func;
        };

        std::array<AlgoEntry, 3> algos{{
            {.name = "BFS",
             .button_label = "Compare with BFS",
             .func = [](const Grid& g, Vec2i s, Vec2i e) { return bfs(g, s, e); }},
            {.name = "Dijkstra",
             .button_label = "Compare with Dijkstra",
             .func = [](const Grid& g, Vec2i s, Vec2i e) { return dijkstra(g, s, e); }},
            {.name = "A*",
             .button_label = "Compare with A*",
             .func = [](const Grid& g, Vec2i s, Vec2i e) { return a_star(g, s, e); }},
        }};

        for (const auto& algo : algos) {
            if (result.algorithm_name == algo.name) {
                continue;
            }
            if (history.contains(algo.name)) {
                continue;
            }
            if (ImGui::MenuItem(algo.button_label)) {
                run_comparison(grid, history, algo.func);
            }
        }

        draw_stats_comparison(result, history);

        ImGui::Separator();
        draw_copy_button(result, history);
    }

    ImGui::EndMenu();
}

} // namespace

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
        if (ImGui::MenuItem("Clear Path", nullptr, false,
                            playback.state() != PlaybackState::Idle)) {
            playback.reset(grid);
            history.clear();
        }
        ImGui::EndMenu();
    }

    draw_tools_menu(renderer);
    draw_algorithm_menu(playback, grid, history);
    draw_playback_menu(playback, grid);
    draw_maze_menu(playback, grid);
    draw_view_menu(view);
    draw_stats_menu(playback, grid, history);
    draw_settings_menu(grid, playback, history);
    float menus_end_x = ImGui::GetCursorPosX();
    draw_diagonal_status(grid, menus_end_x);
    draw_active_tool(renderer);
    draw_hover_info(grid, renderer);

    ImGui::EndMainMenuBar();

    // Help overlay
    if (view.show_help) {
        ImGuiViewport* vp = ImGui::GetMainViewport();
        ImVec2 center(vp->WorkPos.x + vp->WorkSize.x * 0.5F, vp->WorkPos.y + vp->WorkSize.y * 0.5F);
        ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5F, 0.5F));
        ImGui::SetNextWindowSize(ImVec2(360.0F, 0.0F));

        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08F, 0.08F, 0.10F, 0.94F));
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 8.0F);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20.0F, 16.0F));

        if (ImGui::Begin("##help", &view.show_help,
                         ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize |
                             ImGuiWindowFlags_NoSavedSettings)) {
            ImGui::TextColored(ImVec4(0.4F, 0.8F, 1.0F, 1.0F), "Keyboard Shortcuts");
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            auto row = [](const char* key, const char* desc) {
                ImGui::TextColored(ImVec4(1.0F, 0.8F, 0.3F, 1.0F), "%-12s", key);
                ImGui::SameLine(120.0F);
                ImGui::Text("%s", desc);
            };

            ImGui::TextColored(ImVec4(0.7F, 0.7F, 0.9F, 1.0F), "Tools");
            row("1", "Wall");
            row("2", "Erase");
            row("3", "Weight Brush");
            row("4", "Waypoint");
            row("5", "Impassable");
            ImGui::Spacing();

            ImGui::TextColored(ImVec4(0.7F, 0.7F, 0.9F, 1.0F), "Playback");
            row("Space", "Pause / Resume");
            row(".", "Step Forward");
            row("R", "Reset");
            ImGui::Spacing();

            ImGui::TextColored(ImVec4(0.7F, 0.7F, 0.9F, 1.0F), "Brushes");
            row("Left/Right", "Adjust weight (1-9)");
            row("Arrows", "Set one-way direction");
            row("Scroll", "Adjust weight");
            ImGui::Spacing();

            ImGui::TextColored(ImVec4(0.7F, 0.7F, 0.9F, 1.0F), "Mouse");
            row("Left Click", "Place with active tool");
            row("Right Click", "Erase any cell");
            row("Drag", "Move start / end");
            row("Middle Click", "Renumber waypoint");
            ImGui::Spacing();

            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.5F, 0.5F, 0.5F, 1.0F), "Press H to close");
        }
        ImGui::End();
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor();
    }
}
} // namespace pathsim::menu_bar