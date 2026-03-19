#include "KeyboardShortcuts.hpp"

#include <imgui.h>

namespace pathsim::keyboard_shortcuts {
namespace {

void handle_playback(Grid& grid, Playback& playback) {
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
}

void handle_tool_selection(GridRenderer& renderer) {
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
}

void handle_direction_keys(GridRenderer& renderer) {
    if (renderer.active_tool() != EditTool::OneWay) {
        return;
    }

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

void handle_weight_scroll(GridRenderer& renderer) {
    if (renderer.active_tool() != EditTool::Weight) {
        return;
    }

    // Arrow keys to adjust weight brush (works on both desktop and web)
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) {
        renderer.set_weight_brush(renderer.weight_brush() + 1);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) {
        renderer.set_weight_brush(renderer.weight_brush() - 1);
    }

    // Scroll wheel as an additional input (desktop only)
    float wheel = ImGui::GetIO().MouseWheel;
    if (wheel > 0.0F) {
        renderer.set_weight_brush(renderer.weight_brush() + 1);
    } else if (wheel < 0.0F) {
        renderer.set_weight_brush(renderer.weight_brush() - 1);
    }
}

} // namespace

void handle(Grid& grid, GridRenderer& renderer, Playback& playback) {
    if (ImGui::GetIO().WantCaptureKeyboard) {
        return;
    }

    handle_playback(grid, playback);
    handle_tool_selection(renderer);
    handle_direction_keys(renderer);
    handle_weight_scroll(renderer);
}

} // namespace pathsim::keyboard_shortcuts