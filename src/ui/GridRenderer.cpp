#include "GridRenderer.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace pathsim {

void GridRenderer::draw_direction_indicator(ImDrawList* draw_list, CellDirection dir, float x_min,
                                            float y_min, float x_max, float y_max) const {
    ImU32 clear = IM_COL32(80, 150, 220, 0);
    ImU32 tint = IM_COL32(80, 150, 220, 65);

    ImU32 tl = clear;
    ImU32 tr = clear;
    ImU32 br = clear;
    ImU32 bl = clear;

    switch (dir) {
    case CellDirection::North:
        tl = tint;
        tr = tint;
        break;
    case CellDirection::South:
        bl = tint;
        br = tint;
        break;
    case CellDirection::East:
        tr = tint;
        br = tint;
        break;
    case CellDirection::West:
        tl = tint;
        bl = tint;
        break;
    case CellDirection::None:
        return;
    }

    draw_list->AddRectFilledMultiColor(ImVec2{x_min, y_min}, ImVec2{x_max, y_max}, tl, tr, br, bl);

    ImU32 stripe = IM_COL32(90, 170, 255, 160);
    float thickness = std::max(2.0F, std::min(cell_w_, cell_h_) * 0.07F);

    switch (dir) {
    case CellDirection::North:
        draw_list->AddLine(ImVec2{x_min, y_min}, ImVec2{x_max, y_min}, stripe, thickness);
        break;
    case CellDirection::South:
        draw_list->AddLine(ImVec2{x_min, y_max}, ImVec2{x_max, y_max}, stripe, thickness);
        break;
    case CellDirection::East:
        draw_list->AddLine(ImVec2{x_max, y_min}, ImVec2{x_max, y_max}, stripe, thickness);
        break;
    case CellDirection::West:
        draw_list->AddLine(ImVec2{x_min, y_min}, ImVec2{x_min, y_max}, stripe, thickness);
        break;
    case CellDirection::None:
        break;
    }
}

void GridRenderer::draw_direction_badge(ImDrawList* draw_list, CellDirection dir,
                                        ImVec2 top_right) const {
    // adjust size of direction badge 0.35 is a happy medium
    float size = std::min(cell_w_, cell_h_) * 0.35F;
    float padding = size * 0.15F;

    float cx = top_right.x - padding - (size * 0.5F);
    float cy = top_right.y + padding + (size * 0.5F);
    float half = size * 0.5F;

    // Rounded dark background
    draw_list->AddRectFilled(ImVec2{cx - half, cy - half}, ImVec2{cx + half, cy + half},
                             IM_COL32(10, 10, 15, 210), 3.0F);
    draw_list->AddRect(ImVec2{cx - half, cy - half}, ImVec2{cx + half, cy + half},
                       IM_COL32(90, 170, 255, 100), 3.0F, 0, 1.0F);

    // Clean chevron arrow inside
    float arm = size * 0.28F;
    ImU32 col = IM_COL32(180, 210, 255, 240);
    float line_w = std::max(1.5F, size * 0.12F);

    switch (dir) {
    case CellDirection::North:
        draw_list->AddLine(ImVec2{cx - arm, cy + (arm * 0.4F)}, ImVec2{cx, cy - (arm * 0.6F)}, col,
                           line_w);
        draw_list->AddLine(ImVec2{cx, cy - (arm * 0.6F)}, ImVec2{cx + arm, cy + (arm * 0.4F)}, col,
                           line_w);
        break;
    case CellDirection::South:
        draw_list->AddLine(ImVec2{cx - arm, cy - (arm * 0.4F)}, ImVec2{cx, cy + (arm * 0.6F)}, col,
                           line_w);
        draw_list->AddLine(ImVec2{cx, cy + (arm * 0.6F)}, ImVec2{cx + arm, cy - (arm * 0.4F)}, col,
                           line_w);
        break;
    case CellDirection::East:
        draw_list->AddLine(ImVec2{cx - (arm * 0.4F), cy - arm}, ImVec2{cx + (arm * 0.6F), cy}, col,
                           line_w);
        draw_list->AddLine(ImVec2{cx + (arm * 0.6F), cy}, ImVec2{cx - (arm * 0.4F), cy + arm}, col,
                           line_w);
        break;
    case CellDirection::West:
        draw_list->AddLine(ImVec2{cx + (arm * 0.4F), cy - arm}, ImVec2{cx - (arm * 0.6F), cy}, col,
                           line_w);
        draw_list->AddLine(ImVec2{cx - (arm * 0.6F), cy}, ImVec2{cx + (arm * 0.4F), cy + arm}, col,
                           line_w);
        break;
    case CellDirection::None:
        return;
    }
}

void GridRenderer::draw_cell_tooltip(const Grid& grid, Vec2i cell) {
    CellState state = grid.at(cell);
    int w = grid.weight(cell);
    CellDirection dir = grid.direction(cell);

    // Figure out if there's anything worth showing beyond coordinates
    bool has_state_info = state != CellState::Empty;
    bool has_weight = w > 1;
    bool has_direction = dir != CellDirection::None;

    if (!has_state_info && !has_weight && !has_direction) {
        return;
    }

    ImGui::BeginTooltip();
    ImGui::Text("(%d, %d)", cell.x, cell.y);
    ImGui::Separator();

    switch (state) {
    case CellState::Wall:
        ImGui::TextColored(ImVec4(0.7F, 0.7F, 0.7F, 1.0F), "Wall");
        break;
    case CellState::Start:
        ImGui::TextColored(ImVec4(0.0F, 0.8F, 0.3F, 1.0F), "Start");
        break;
    case CellState::End:
        ImGui::TextColored(ImVec4(0.9F, 0.2F, 0.2F, 1.0F), "End");
        break;
    case CellState::Waypoint: {
        const auto& wps = grid.waypoints();
        for (std::size_t i = 0; i < wps.size(); ++i) {
            if (wps[i] == cell) {
                if (grid.ordered_waypoints()) {
                    ImGui::TextColored(ImVec4(1.0F, 0.6F, 0.0F, 1.0F), "Waypoint #%d",
                                       static_cast<int>(i + 1));
                    ImGui::TextColored(ImVec4(0.6F, 0.6F, 0.6F, 0.8F), "Middle-click to renumber");
                } else {
                    ImGui::TextColored(ImVec4(1.0F, 0.6F, 0.0F, 1.0F), "Waypoint (unordered)");
                }
                break;
            }
        }
        break;
    }
    case CellState::Impassable:
        ImGui::TextColored(ImVec4(0.8F, 0.2F, 0.2F, 1.0F), "Impassable");
        break;
    case CellState::Visited:
        ImGui::TextColored(ImVec4(0.2F, 0.4F, 0.7F, 1.0F), "Visited");
        break;
    case CellState::Frontier:
        ImGui::TextColored(ImVec4(0.4F, 0.7F, 0.9F, 1.0F), "Frontier");
        break;
    case CellState::Path:
        ImGui::TextColored(ImVec4(1.0F, 0.9F, 0.2F, 1.0F), "Path");
        break;
    case CellState::Empty:
        break;
    }

    if (has_weight) {
        ImGui::Text("Weight: %d", w);
    }

    if (has_direction) {
        const char* dir_name = "None";
        switch (dir) {
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
            break;
        }
        ImGui::Text("Direction: %s", dir_name);
    }

    ImGui::EndTooltip();
}

void GridRenderer::draw_hover(const Grid& grid, const ViewSettings& view) {
    if (!is_hovered_) {
        has_hover_ = false;
        return;
    }

    ImVec2 mouse_pos = ImGui::GetMousePos();
    int hx = static_cast<int>((mouse_pos.x - grid_origin_.x) / cell_w_);
    int hy = static_cast<int>((mouse_pos.y - grid_origin_.y) / cell_h_);
    Vec2i hover_cell{.x = hx, .y = hy};

    has_hover_ = grid.is_valid(hover_cell);
    hovered_cell_ = hover_cell;

    if (has_hover_) {
        float x_min = grid_origin_.x + (static_cast<float>(hx) * cell_w_);
        float y_min = grid_origin_.y + (static_cast<float>(hy) * cell_h_);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRect(ImVec2{x_min, y_min}, ImVec2{x_min + cell_w_, y_min + cell_h_},
                           IM_COL32(255, 255, 255, 120), 0.0F, 0, 2.0F);

        if (view.show_tooltips && !ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
            !ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
            draw_cell_tooltip(grid, hover_cell);
        }
    }
}

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
        if (wps[i] != cell) {
            continue;
        }

        bool ordered = grid.ordered_waypoints();

        if (cell_w_ < 20.0F || cell_h_ < 20.0F) {
            if (ordered) {
                std::array<char, 4> label{};
                std::snprintf(label.data(), label.size(), "%d", static_cast<int>(i + 1));
                ImVec2 text_size = ImGui::CalcTextSize(label.data());
                float text_x = x_min + ((cell_w_ - text_size.x) * 0.5F);
                float text_y = y_min + ((cell_h_ - text_size.y) * 0.5F);
                draw_list->AddText(ImVec2{text_x, text_y}, IM_COL32(255, 255, 255, 255),
                                   label.data());
            } else {
                // Unordered should be a  small diamond
                float cx = x_min + (cell_w_ * 0.5F);
                float cy = y_min + (cell_h_ * 0.5F);
                float r = std::min(cell_w_, cell_h_) * 0.25F;
                draw_list->AddQuadFilled(ImVec2{cx, cy - r}, ImVec2{cx + r, cy}, ImVec2{cx, cy + r},
                                         ImVec2{cx - r, cy}, IM_COL32(255, 255, 255, 230));
            }
            break;
        }

        if (ordered) {
            // Numbered flag marker
            std::array<char, 4> label{};
            std::snprintf(label.data(), label.size(), "%d", static_cast<int>(i + 1));
            ImVec2 text_size = ImGui::CalcTextSize(label.data());

            // Total content width is: pole + gap + flag width + gap + number
            float flag_w = cell_w_ * 0.20F;
            float flag_h = cell_h_ * 0.25F;
            float gap = cell_w_ * 0.05F;
            float pole_thickness = 2.0F;
            float content_w = pole_thickness + gap + flag_w + gap + text_size.x;

            // Make sure everything is centered horizontally
            float start_x = x_min + ((cell_w_ - content_w) * 0.5F);
            float cy = y_min + (cell_h_ * 0.5F);

            // Add Flag pole
            float pole_x = start_x;
            float pole_top = cy - (cell_h_ * 0.28F);
            float pole_bottom = cy + (cell_h_ * 0.24F);
            draw_list->AddLine(ImVec2{pole_x, pole_top}, ImVec2{pole_x, pole_bottom},
                               IM_COL32(255, 255, 255, 220), pole_thickness);

            // Add Flag triangle
            float flag_left = pole_x + (pole_thickness * 0.5F);
            ImVec2 flag_top{flag_left, pole_top};
            ImVec2 flag_right{flag_left + flag_w, pole_top + (flag_h * 0.5F)};
            ImVec2 flag_bottom{flag_left, pole_top + flag_h};
            draw_list->AddTriangleFilled(flag_top, flag_right, flag_bottom,
                                         IM_COL32(255, 255, 255, 230));

            // Add number after the flag
            float text_x = flag_left + flag_w + gap;
            float text_y = cy - (text_size.y * 0.5F);
            draw_list->AddText(ImVec2{text_x, text_y}, IM_COL32(255, 255, 255, 255), label.data());
        } else {
            // Unordered is a diamond marker with no number
            float cx = x_min + (cell_w_ * 0.5F);
            float cy = y_min + (cell_h_ * 0.5F);
            float r = std::min(cell_w_, cell_h_) * 0.28F;

            draw_list->AddQuadFilled(ImVec2{cx, cy - r}, ImVec2{cx + r, cy}, ImVec2{cx, cy + r},
                                     ImVec2{cx - r, cy}, IM_COL32(255, 255, 255, 230));
            draw_list->AddQuad(ImVec2{cx, cy - r}, ImVec2{cx + r, cy}, ImVec2{cx, cy + r},
                               ImVec2{cx - r, cy}, IM_COL32(255, 160, 0, 180), 1.5F);
        }

        break;
    }
}

void GridRenderer::draw_start_marker(ImDrawList* draw_list, ImVec2 top_left,
                                     ImVec2 bottom_right) const {
    float cx = (top_left.x + bottom_right.x) * 0.5F;
    float cy = (top_left.y + bottom_right.y) * 0.5F;
    float radius = std::min(cell_w_, cell_h_) * 0.35F;

    draw_list->AddCircleFilled(ImVec2{cx, cy}, radius, IM_COL32(0, 200, 80, 255));
    draw_list->AddCircle(ImVec2{cx, cy}, radius, IM_COL32(255, 255, 255, 180), 0, 2.0F);

    const char* label = "S";
    ImVec2 text_size = ImGui::CalcTextSize(label);
    draw_list->AddText(ImVec2{cx - (text_size.x * 0.5F), cy - (text_size.y * 0.5F)},
                       IM_COL32(255, 255, 255, 240), label);
}

void GridRenderer::draw_end_marker(ImDrawList* draw_list, ImVec2 top_left,
                                   ImVec2 bottom_right) const {
    float cx = (top_left.x + bottom_right.x) * 0.5F;
    float cy = (top_left.y + bottom_right.y) * 0.5F;
    float radius = std::min(cell_w_, cell_h_) * 0.35F;

    draw_list->AddCircleFilled(ImVec2{cx, cy}, radius, IM_COL32(220, 50, 50, 255));
    draw_list->AddCircle(ImVec2{cx, cy}, radius, IM_COL32(255, 255, 255, 180), 0, 2.0F);

    const char* label = "E";
    ImVec2 text_size = ImGui::CalcTextSize(label);
    draw_list->AddText(ImVec2{cx - (text_size.x * 0.5F), cy - (text_size.y * 0.5F)},
                       IM_COL32(255, 255, 255, 240), label);
}

void GridRenderer::draw_cell_overlays(ImDrawList* draw_list, const Grid& grid, Vec2i cell,
                                      float x_min, float y_min, float x_max, float y_max) const {
    CellState state = grid.at(cell);

    if (state == CellState::Start) {
        draw_start_marker(draw_list, {x_min, y_min}, {x_max, y_max});
    }
    if (state == CellState::End) {
        draw_end_marker(draw_list, {x_min, y_min}, {x_max, y_max});
    }
    if (grid.weight(cell) > 1 && state == CellState::Empty) {
        draw_weight_label(draw_list, grid, cell, x_min, y_min);
    }
    if (state == CellState::Impassable) {
        draw_impassable(draw_list, x_min, y_min, x_max, y_max);
    }
    if (state == CellState::Waypoint) {
        draw_waypoint(draw_list, grid, cell, x_min, y_min);
    }
    if (grid.direction(cell) != CellDirection::None) {
        draw_direction_indicator(draw_list, grid.direction(cell), x_min, y_min, x_max, y_max);
        bool has_content = state == CellState::Waypoint || state == CellState::Start ||
                           state == CellState::End ||
                           (state == CellState::Empty && grid.weight(cell) > 1);
        if (has_content) {
            draw_direction_badge(draw_list, grid.direction(cell), ImVec2{x_max, y_min});
        } else {
            draw_direction_arrow(draw_list, grid.direction(cell), ImVec2{x_min, y_min},
                                 ImVec2{x_max, y_max});
        }
    }
}

void GridRenderer::draw(Grid& grid, const ViewSettings& view, const PathResult* finished_result) {
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

    // Build heatmap lookup if enabled and playback is finished
    std::vector<int> visit_order(
        static_cast<std::size_t>(grid.width()) * static_cast<std::size_t>(grid.height()), -1);
    int total_visited = 0;

    if (view.show_heatmap && finished_result != nullptr) {
        for (const auto& step : finished_result->steps) {
            if (step.new_state == CellState::Visited) {
                int idx = (step.position.y * grid.width()) + step.position.x;
                visit_order[static_cast<std::size_t>(idx)] = total_visited;
                ++total_visited;
            }
        }
    }

    for (int row = 0; row < grid.height(); ++row) {
        for (int col = 0; col < grid.width(); ++col) {
            Vec2i cell{.x = col, .y = row};

            float x_min = grid_origin_.x + (static_cast<float>(col) * cell_w_);
            float y_min = grid_origin_.y + (static_cast<float>(row) * cell_h_);
            float x_max = x_min + cell_w_;
            float y_max = y_min + cell_h_;

            // Heatmap overrides cell color for visited cells
            int idx = (row * grid.width()) + col;
            ImU32 fill = 0;
            if (total_visited > 0 && visit_order[static_cast<std::size_t>(idx)] >= 0) {
                float t = static_cast<float>(visit_order[static_cast<std::size_t>(idx)]) /
                          static_cast<float>(std::max(1, total_visited - 1));
                fill = heatmap_color(t);
            } else {
                fill = cell_color(grid.at(cell), grid.weight(cell));
            }

            draw_list->AddRectFilled({x_min, y_min}, {x_max, y_max}, fill);
            draw_list->AddRect({x_min, y_min}, {x_max, y_max}, IM_COL32(38, 38, 45, 200));

            if (grid.at(cell) == CellState::Wall) {
                draw_list->AddLine({x_min + 1.0F, y_min}, {x_max - 1.0F, y_min},
                                   IM_COL32(210, 210, 210, 60), 1.0F);
                draw_list->AddLine({x_min, y_min + 1.0F}, {x_min, y_max - 1.0F},
                                   IM_COL32(210, 210, 210, 40), 1.0F);
                draw_list->AddLine({x_min + 1.0F, y_max}, {x_max - 1.0F, y_max},
                                   IM_COL32(80, 80, 80, 60), 1.0F);
                draw_list->AddLine({x_max, y_min + 1.0F}, {x_max, y_max - 1.0F},
                                   IM_COL32(80, 80, 80, 40), 1.0F);
            }

            draw_cell_overlays(draw_list, grid, cell, x_min, y_min, x_max, y_max);
        }
    }

    // Path overlay
    if (finished_result != nullptr && !finished_result->path.empty()) {
        draw_path_overlay(draw_list, finished_result->path, view.show_path_direction);
    }

    // Mouse Hover highlight
    draw_hover(grid, view);
    // renumber waypoints
    draw_waypoint_renumber_popup(grid);

    ImGui::Dummy(available);
    ImGui::End();
}

void GridRenderer::draw_path_dots(ImDrawList* draw_list, const std::vector<ImVec2>& points,
                                  float thickness) {
    float spacing = thickness * 2.8F;
    float max_radius = thickness * 0.4F;
    float speed = spacing * 1.0F;
    float time_offset = std::fmod(static_cast<float>(ImGui::GetTime()) * speed, spacing);

    // Number of fading trail dots behind each main dot
    constexpr int kTrailCount = 3;
    float trail_gap = spacing * 0.15F;

    float accumulated = time_offset;

    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        float dx = points[i + 1].x - points[i].x;
        float dy = points[i + 1].y - points[i].y;
        float seg_len = std::sqrt((dx * dx) + (dy * dy));
        if (seg_len < 0.001F) {
            continue;
        }

        float nx = dx / seg_len;
        float ny = dy / seg_len;

        while (accumulated < seg_len) {
            // Pulse dot that grows and brightens then shrinks and dims
            float frac = (accumulated / spacing) - std::floor(accumulated / spacing);
            float pulse = 0.5F + (0.5F * std::sin(frac * 3.14159F));
            float radius = max_radius * (0.5F + (0.5F * pulse));
            auto alpha = static_cast<uint8_t>(140.0F + (pulse * 115.0F));

            // Main dot
            float x = points[i].x + (nx * accumulated);
            float y = points[i].y + (ny * accumulated);
            draw_list->AddCircleFilled(ImVec2{x, y}, radius, IM_COL32(255, 255, 255, alpha));

            // Trail dots behind the main dot (fade out further back)
            for (int t = 1; t <= kTrailCount; ++t) {
                float trail_dist = static_cast<float>(t) * trail_gap;
                float tx = x - (nx * trail_dist);
                float ty = y - (ny * trail_dist);

                float fade = 1.0F - (static_cast<float>(t) / static_cast<float>(kTrailCount + 1));
                float tr = radius * fade * 0.8F;
                auto ta = static_cast<uint8_t>(static_cast<float>(alpha) * fade * 0.5F);

                if (tr > 0.5F && ta > 10) {
                    draw_list->AddCircleFilled(ImVec2{tx, ty}, tr, IM_COL32(255, 240, 180, ta));
                }
            }

            accumulated += spacing;
        }
        accumulated -= seg_len;
    }
}

void GridRenderer::draw_path_arrowheads(ImDrawList* draw_list, const std::vector<ImVec2>& points,
                                        float thickness) {
    ImU32 arrow_col = IM_COL32(255, 255, 255, 230);
    int spacing = std::max(3, static_cast<int>(points.size()) / 10);
    float arrow_size = thickness * 2.5F;

    for (std::size_t i = spacing; i + 1 < points.size(); i += static_cast<std::size_t>(spacing)) {
        ImVec2 prev = points[i - 1];
        ImVec2 curr = points[i];

        // Direction vector
        float dx = curr.x - prev.x;
        float dy = curr.y - prev.y;
        float len = std::sqrt((dx * dx) + (dy * dy));

        if (len < 0.001F) {
            continue;
        }

        dx /= len;
        dy /= len;

        // Perpendicular
        float px = -dy;
        float py = dx;

        // Triangle tip ahead of current point and two wings behind
        ImVec2 tip{curr.x + (dx * arrow_size * 0.5F), curr.y + (dy * arrow_size * 0.5F)};
        ImVec2 left{curr.x - (dx * arrow_size * 0.5F) + (px * arrow_size * 0.5F),
                    curr.y - (dy * arrow_size * 0.5F) + (py * arrow_size * 0.5F)};
        ImVec2 right{curr.x - (dx * arrow_size * 0.5F) - (px * arrow_size * 0.5F),
                     curr.y - (dy * arrow_size * 0.5F) - (py * arrow_size * 0.5F)};

        draw_list->AddTriangleFilled(tip, left, right, arrow_col);
    }
}

void GridRenderer::draw_path_overlay(ImDrawList* draw_list, const std::vector<Vec2i>& path,
                                     bool show_arrows) const {
    if (path.size() < 2 || cell_w_ <= 0.0F || cell_h_ <= 0.0F) {
        return;
    }

    float half_w = cell_w_ * 0.5F;
    float half_h = cell_h_ * 0.5F;
    float thickness = std::min(cell_w_, cell_h_) * 0.2F;

    // Breathing glow which makes the outer glow pulse gently
    float breath = 0.5F + (0.5F * std::sin(static_cast<float>(ImGui::GetTime()) * 2.0F));
    auto glow_alpha = static_cast<uint8_t>(25.0F + (breath * 25.0F));
    ImU32 glow_col = IM_COL32(255, 200, 30, glow_alpha);

    ImU32 outline_col = IM_COL32(40, 40, 40, 160);
    ImU32 line_col = IM_COL32(255, 220, 50, 220);
    ImU32 highlight_col = IM_COL32(255, 255, 200, 90);

    std::vector<ImVec2> points;
    points.reserve(path.size());
    for (const auto& cell : path) {
        float cx = grid_origin_.x + (static_cast<float>(cell.x) * cell_w_) + half_w;
        float cy = grid_origin_.y + (static_cast<float>(cell.y) * cell_h_) + half_h;
        points.emplace_back(cx, cy);
    }

    // Layer 1: Breathing glow
    draw_list->AddPolyline(points.data(), static_cast<int>(points.size()), glow_col, 0,
                           thickness * 3.5F);

    // Layer 2: Dark outline
    draw_list->AddPolyline(points.data(), static_cast<int>(points.size()), outline_col, 0,
                           thickness + 2.0F);

    // Layer 3: Main gold line
    draw_list->AddPolyline(points.data(), static_cast<int>(points.size()), line_col, 0, thickness);

    // Layer 4: Thin bright inner highlight for 3d effect
    draw_list->AddPolyline(points.data(), static_cast<int>(points.size()), highlight_col, 0,
                           thickness * 0.3F);

    // Animated dots with trailing fade
    draw_path_dots(draw_list, points, thickness);

    // Static arrowheads
    if (show_arrows && points.size() > 2) {
        draw_path_arrowheads(draw_list, points, thickness);
    }

    // Colored end caps matching start (green) and end (red) nodes
    float cap_radius = thickness * 1.0F;

    ImU32 start_cap = IM_COL32(0, 200, 80, 240);
    ImU32 end_cap = IM_COL32(220, 50, 50, 240);
    ImU32 cap_ring = IM_COL32(255, 255, 255, 160);

    draw_list->AddCircleFilled(points.front(), cap_radius, start_cap);
    draw_list->AddCircle(points.front(), cap_radius, cap_ring, 0, 1.5F);

    draw_list->AddCircleFilled(points.back(), cap_radius, end_cap);
    draw_list->AddCircle(points.back(), cap_radius, cap_ring, 0, 1.5F);
}

bool GridRenderer::handle_drag(Grid& grid, Vec2i mouse_pos) {
    // End drag if mouse released even when cursor is off-grid
    if (drag_target_ != DragTarget::None && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        drag_target_ = DragTarget::None;
        prev_drag_pos_ = {.x = -1, .y = -1};
        saved_drag_cell_ = {};
    }

    if (!grid.is_valid(mouse_pos)) {
        return false;
    }

    // Begin drag if mouse just clicked on start or end
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (mouse_pos == grid.start()) {
            drag_target_ = DragTarget::Start;
            prev_drag_pos_ = mouse_pos;
            saved_drag_cell_ = {};
        } else if (mouse_pos == grid.end()) {
            drag_target_ = DragTarget::End;
            prev_drag_pos_ = mouse_pos;
            saved_drag_cell_ = {};
        }
    }

    if (drag_target_ == DragTarget::None) {
        return false;
    }

    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        return false;
    }

    // Handle ongoing drag with cell preservation
    bool is_start = drag_target_ == DragTarget::Start;
    Vec2i other = is_start ? grid.end() : grid.start();

    if (mouse_pos != other && mouse_pos != prev_drag_pos_) {
        // 1. We save the destination before we decide to overwrite it
        CellData dest_backup = grid.cell_data(mouse_pos);

        // 2. Then we move start/end to new position
        if (is_start) {
            grid.set_start(mouse_pos);
        } else {
            grid.set_end(mouse_pos);
        }

        // 3. Restore previous position AFTER set_start/set_end
        //    so it overwrites the Empty that was just written
        if (prev_drag_pos_.x >= 0 && grid.is_valid(prev_drag_pos_)) {
            grid.restore_cell(prev_drag_pos_, saved_drag_cell_);
        }

        saved_drag_cell_ = dest_backup;
        prev_drag_pos_ = mouse_pos;
    }

    return true;
}

void GridRenderer::handle_waypoint_renumber(Grid& grid, Vec2i mouse_pos) {
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle) && grid.is_valid(mouse_pos) &&
        grid.at(mouse_pos) == CellState::Waypoint) {
        const auto& wps = grid.waypoints();
        for (int i = 0; i < static_cast<int>(wps.size()); ++i) {
            if (wps[i] == mouse_pos) {
                renumber_waypoint_index_ = i;
                renumber_open_requested_ = true;
                break;
            }
        }
    }
}

void GridRenderer::draw_waypoint_renumber_popup(Grid& grid) {
    if (renumber_open_requested_) {
        ImGui::OpenPopup("RenumberWaypoint");
        renumber_open_requested_ = false;
    }

    if (!ImGui::BeginPopup("RenumberWaypoint")) {
        renumber_waypoint_index_ = -1;
        return;
    }

    int wp_count = static_cast<int>(grid.waypoints().size());
    ImGui::Text("Move to position:");
    ImGui::Separator();

    for (int i = 0; i < wp_count; ++i) {
        std::array<char, 16> label{};
        std::snprintf(label.data(), label.size(), "#%d", i + 1);

        if (i == renumber_waypoint_index_) {
            ImGui::TextColored(ImVec4(1.0F, 0.6F, 0.0F, 1.0F), "%s (current)", label.data());
        } else if (ImGui::MenuItem(label.data())) {
            grid.move_waypoint_to_index(renumber_waypoint_index_, i);
            renumber_waypoint_index_ = -1;
            ImGui::CloseCurrentPopup();
        }
    }

    ImGui::EndPopup();
}

void GridRenderer::handle_input(Grid& grid, bool editing_enabled) {
    // Always check for drag release even when cursor is off-grid
    if (handle_drag(grid, screen_to_grid(ImGui::GetMousePos()))) {
        if (!is_hovered_) {
            return;
        }
    }

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

    // Middle-click renumbers waypoints
    handle_waypoint_renumber(grid, mouse_pos);

    // Drag takes priority over normal tool handling
    if (drag_target_ != DragTarget::None) {
        return;
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
        case EditTool::OneWay:
            grid.set_direction(mouse_pos, active_direction_);
            break;
        }
    }

    // Right-click will always erase regardless of which tool is active
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        grid.set_wall(mouse_pos, false);
        grid.set_weight(mouse_pos, 1);
        grid.remove_waypoint(mouse_pos);
        grid.set_impassable(mouse_pos, false);
        grid.set_direction(mouse_pos, CellDirection::None);
    }
}

void GridRenderer::draw_direction_arrow(ImDrawList* draw_list, CellDirection dir, ImVec2 top_left,
                                        ImVec2 bottom_right) {
    float cx = (top_left.x + bottom_right.x) * 0.5F;
    float cy = (top_left.y + bottom_right.y) * 0.5F;
    float hw = (bottom_right.x - top_left.x) * 0.3F;
    float hh = (bottom_right.y - top_left.y) * 0.3F;

    ImU32 col = IM_COL32(255, 255, 255, 200);

    ImVec2 tip;
    ImVec2 left;
    ImVec2 right;

    switch (dir) {
    case CellDirection::North:
        tip = {cx, cy - hh};
        left = {cx - hw, cy + hh};
        right = {cx + hw, cy + hh};
        break;
    case CellDirection::South:
        tip = {cx, cy + hh};
        left = {cx + hw, cy - hh};
        right = {cx - hw, cy - hh};
        break;
    case CellDirection::East:
        tip = {cx + hw, cy};
        left = {cx - hw, cy - hh};
        right = {cx - hw, cy + hh};
        break;
    case CellDirection::West:
        tip = {cx - hw, cy};
        left = {cx + hw, cy + hh};
        right = {cx + hw, cy - hh};
        break;
    case CellDirection::None:
        return;
    }

    draw_list->AddTriangleFilled(tip, left, right, col);
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

void GridRenderer::set_direction_brush(CellDirection dir) {
    active_direction_ = dir;
}

CellDirection GridRenderer::direction_brush() const {
    return active_direction_;
}

ImU32 GridRenderer::heatmap_color(float t) {
    // Dark blue -> cyan -> bright white-cyan
    float r{};
    float g{};
    float b{};

    if (t < 0.5F) {
        float s = t * 2.0F;
        r = 20.0F + (s * 0.0F);
        g = 20.0F + (s * 160.0F);
        b = 80.0F + (s * 140.0F);
    } else {
        float s = (t - 0.5F) * 2.0F;
        r = 20.0F + (s * 160.0F);
        g = 180.0F + (s * 60.0F);
        b = 220.0F + (s * 35.0F);
    }

    return IM_COL32(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b), 255);
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
        return IM_COL32(80, 70, 30, 255);
    case CellState::Waypoint:
        return IM_COL32(255, 160, 0, 255);
    case CellState::Impassable:
        return IM_COL32(50, 20, 20, 255);
    }
    return IM_COL32(0, 0, 0, 255);
}

} // namespace pathsim