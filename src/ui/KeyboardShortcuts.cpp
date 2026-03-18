#include "KeyboardShortcuts.hpp"

#include <imgui.h>

namespace pathsim::keyboard_shortcuts {

void handle(Grid& grid, GridRenderer& renderer, Playback& playback) {
    if (ImGui::GetIO().WantCaptureKeyboard) {
        return;
    }

    // Playback controls
    if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
        if (playback.state() == PlaybackState::Playing) {
            playback.pause();
        } else if (playback.state() == PlaybackState::Paused) {
            playback.resume();
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_R)) {
        if (playback.state() != PlaybackState::Idle) {
            playback.reset(grid);
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Period)) {
        playback.step_forward(grid);
    }

    // Tool selection
    if (ImGui::IsKeyPressed(ImGuiKey_1)) {
        renderer.set_tool(EditTool::Wall);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_2)) {
        renderer.set_tool(EditTool::Erase);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_3)) {
        renderer.set_tool(EditTool::Weight);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_4)) {
        renderer.set_tool(EditTool::Waypoint);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_5)) {
        renderer.set_tool(EditTool::Impassable);
    }

    // Arrow keys for one-way directions
    if (renderer.active_tool() == EditTool::OneWay) {
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
            renderer.set_direction_brush(CellDirection::North);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
            renderer.set_direction_brush(CellDirection::South);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) {
            renderer.set_direction_brush(CellDirection::East);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) {
            renderer.set_direction_brush(CellDirection::West);
        }
    }
}

} // namespace pathsim::keyboard_shortcuts