#include "StatsPanel.hpp"

#include <imgui.h>

#include <string>
#include <string_view>

namespace pathsim::stats_panel {

void draw(const Playback& playback) {
    if (playback.state() == PlaybackState::Idle) {
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(10.0F, 40.0F), ImGuiCond_FirstUseEver);

    ImGui::Begin("Stats", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    const auto& result = playback.result();

    ImGui::Text("Algorithm: %s", result.algorithm_name);

    ImGui::Separator();

    ImGui::Text("Progress: %d / %d", playback.current_step(), playback.total_steps());
    ImGui::Text("Nodes explored: %d", result.nodes_visited);

    if (playback.state() == PlaybackState::Finished) {
        ImGui::Separator();

        if (result.path.empty()) {
            ImGui::TextColored(ImVec4(1.0F, 0.3F, 0.3F, 1.0F), "No path found");
        } else {
            ImGui::Text("Path length: %d steps", static_cast<int>(result.path.size()) - 1);
            ImGui::Text("Path cost: %.0f", static_cast<double>(result.path_cost));

            if (std::string_view(result.algorithm_name) == "BFS") {
                ImGui::TextColored(ImVec4(1.0F, 0.8F, 0.3F, 1.0F),
                                   "BFS optimizes for fewest hops, not lowest cost");
            } else {
                ImGui::TextColored(ImVec4(0.3F, 0.8F, 0.4F, 1.0F),
                                   "%s optimizes for lowest total cost", result.algorithm_name);
            }
        }

        ImGui::Separator();

        if (ImGui::Button("Copy Stats")) {
            std::string stats;
            stats += std::string(result.algorithm_name) + "\n";
            stats += "Nodes explored: " + std::to_string(result.nodes_visited) + "\n";
            stats += "Path length: " + std::to_string(static_cast<int>(result.path.size()) - 1) +
                     " steps\n";
            stats += "Path cost: " + std::to_string(result.path_cost) + "\n";
            ImGui::SetClipboardText(stats.c_str());
        }
    }

    ImGui::End();
}

} // namespace pathsim::stats_panel
