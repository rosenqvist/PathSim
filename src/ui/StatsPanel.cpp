#include "StatsPanel.hpp"

#include "algo/AlgoUtils.hpp"
#include "algo/AStar.hpp"
#include "algo/BFS.hpp"
#include "algo/Dijkstra.hpp"
#include "algo/MultiPath.hpp"

#include <imgui.h>

#include <array>
#include <chrono>
#include <cstdio>
#include <string>

namespace pathsim::stats_panel {
namespace {

void run_comparison(const Grid& grid, AlgoHistory& history, const AlgoFunc& algo) {
    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = find_path_with_waypoints(grid, algo);
    auto t1 = std::chrono::high_resolution_clock::now();

    result.compute_time_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    history[result.algorithm_name] = algo_utils::extract_stats(result);
}

void draw_controls(Playback& playback, Grid& grid) {
    auto state = playback.state();

    if (state == PlaybackState::Playing) {
        if (ImGui::Button("Pause [Space]")) {
            playback.pause();
        }
    } else if (state == PlaybackState::Paused) {
        if (ImGui::Button("Resume [Space]")) {
            playback.resume();
        }
        ImGui::SameLine();
        if (ImGui::Button("Step [.]")) {
            playback.step_forward(grid);
        }
    }

    if (state != PlaybackState::Idle) {
        if (state == PlaybackState::Playing || state == PlaybackState::Paused) {
            ImGui::SameLine();
        }
        if (ImGui::Button("Reset [R]")) {
            playback.reset(grid);
        }
    }

    if (state == PlaybackState::Playing || state == PlaybackState::Paused) {
        int speed = playback.speed();
        if (ImGui::SliderInt("Speed", &speed, 1, 50)) {
            playback.set_speed(speed);
        }
    }

    ImGui::Text("Progress: %d / %d", playback.current_step(), playback.total_steps());
}

void draw_compare_buttons(const PathResult& current, const Grid& grid, AlgoHistory& history) {
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
        if (current.algorithm_name == algo.name) {
            continue;
        }

        // Already have results for this algorithm
        if (history.contains(algo.name)) {
            continue;
        }

        if (ImGui::Button(algo.button_label)) {
            run_comparison(grid, history, algo.func);
        }
        ImGui::SameLine();
    }
    ImGui::NewLine();
}

void draw_comparison(const PathResult& current, const AlgoHistory& history) {
    for (const auto& [name, stats] : history) {
        if (name == current.algorithm_name) {
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

        if (!current.path.empty() && stats.path_length > 0) {
            ImGui::Text("  Path cost: %.0f", static_cast<double>(stats.path_cost));

            float cost_diff = stats.path_cost - current.path_cost;
            if (cost_diff > 0.1F) {
                float pct = (cost_diff / current.path_cost) * 100.0F;
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
        stats += "  Path length: " + std::to_string(cmp.path_length - 1) + " steps\n";
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

} // namespace

void draw(Playback& playback, Grid& grid, AlgoHistory& history) {
    if (playback.state() == PlaybackState::Idle) {
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(10.0F, 40.0F), ImGuiCond_FirstUseEver);

    ImGui::Begin("Stats", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const auto& result = playback.result();

    ImGui::Text("Algorithm: %s", result.algorithm_name.c_str());

    if (result.compute_time_ms > 0.0F) {
        ImGui::Text("Compute time: %.2f ms", static_cast<double>(result.compute_time_ms));
    }

    ImGui::Separator();

    draw_controls(playback, grid);

    if (playback.state() == PlaybackState::Finished) {
        ImGui::Separator();

        ImGui::Text("Nodes explored: %d", result.nodes_visited);

        if (result.max_frontier_size > 0) {
            ImGui::Text("Peak frontier size: %d", result.max_frontier_size);
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

        draw_compare_buttons(result, grid, history);
        draw_comparison(result, history);

        ImGui::Separator();
        draw_copy_button(result, history);
    }

    ImGui::End();
}

} // namespace pathsim::stats_panel