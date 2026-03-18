#include "Grid.hpp"

#include "Types.hpp"

#include <algorithm>
#include <array>
#include <cassert>

namespace pathsim {
Grid::Grid(int width, int height)
    : width_(width), height_(height),
      cells_(static_cast<std::size_t>(width * height), CellState::Empty),
      walls_(static_cast<std::size_t>(width * height), 0),
      weights_(static_cast<std::size_t>(width * height), 1), start_{.x = 0, .y = 0},
      end_{.x = width - 1, .y = height - 1} {
    assert(width > 0);
    assert(height > 0);

    cells_[index_at(start_)] = CellState::Start;
    cells_[index_at(end_)] = CellState::End;
}

int Grid::width() const {
    return width_;
}

int Grid::height() const {
    return height_;
}

CellState Grid::at(Vec2i pos) const {
    return cells_[index_at(pos)];
}

void Grid::set(Vec2i pos, CellState state) {
    cells_[index_at(pos)] = state;
}

bool Grid::is_wall(Vec2i pos) const {
    return walls_[index_at(pos)] != 0U || cells_[index_at(pos)] == CellState::Impassable;
}

void Grid::set_wall(Vec2i pos, bool wall) {
    assert(is_valid(pos));

    // prevent walling over start or end
    if (pos == start_ || pos == end_) {
        return;
    }

    walls_[index_at(pos)] = static_cast<unsigned char>(wall);
    cells_[index_at(pos)] = wall ? CellState::Wall : CellState::Empty;

    if (wall) {
        weights_[index_at(pos)] = 1;
    }
}

void Grid::set_impassable(Vec2i pos, bool impassable) {
    assert(is_valid(pos));

    if (pos == start_ || pos == end_) {
        return;
    }

    if (impassable) {
        walls_[index_at(pos)] = 0U;
        cells_[index_at(pos)] = CellState::Impassable;
        weights_[index_at(pos)] = 1;
    } else if (cells_[index_at(pos)] == CellState::Impassable) {
        cells_[index_at(pos)] = CellState::Empty;
    }
}

void Grid::add_waypoint(Vec2i pos) {
    if (!is_valid(pos) || pos == start_ || pos == end_ || is_wall(pos)) {
        return;
    }

    // prevent duplicates
    for (const auto& wp : waypoints_) {
        if (wp == pos) {
            return;
        }
    }

    waypoints_.push_back(pos);
    cells_[index_at(pos)] = CellState::Waypoint;
}

void Grid::remove_waypoint(Vec2i pos) {
    auto it = std::ranges::find(waypoints_, pos);
    if (it != waypoints_.end()) {
        cells_[index_at(pos)] = CellState::Empty;
        waypoints_.erase(it);
    }
}

const std::vector<Vec2i>& Grid::waypoints() const {
    return waypoints_;
}

void Grid::clear_waypoints() {
    for (const auto& wp : waypoints_) {
        if (is_valid(wp)) {
            cells_[index_at(wp)] = CellState::Empty;
        }
    }
    waypoints_.clear();
}

int Grid::weight(Vec2i pos) const {
    assert(is_valid(pos));
    return weights_[index_at(pos)];
}

void Grid::set_weight(Vec2i pos, int weight) {
    assert(is_valid(pos));

    if (pos == start_ || pos == end_) {
        return;
    }

    weights_[index_at(pos)] = static_cast<uint8_t>(std::clamp(weight, 1, 9));
}

float Grid::move_cost(Vec2i pos) const {
    assert(is_valid(pos));
    return static_cast<float>(weights_[index_at(pos)]);
}

Vec2i Grid::start() const {
    return start_;
}

Vec2i Grid::end() const {
    return end_;
}

void Grid::set_start(Vec2i pos) {
    assert(is_valid(pos));

    // clear old start and place a new one, remove wall if present
    cells_[index_at(start_)] = CellState::Empty;
    start_ = pos;
    walls_[index_at(pos)] = 0U;
    cells_[index_at(pos)] = CellState::Start;
}

void Grid::set_end(Vec2i pos) {
    assert(is_valid(pos));

    // clear old end and place a new one, remove wall if present
    cells_[index_at(end_)] = CellState::Empty;
    end_ = pos;
    walls_[index_at(pos)] = 0U;
    cells_[index_at(pos)] = CellState::End;
}

Neighbors Grid::neighbors(Vec2i pos) const {
    assert(is_valid(pos));

    constexpr std::array<Vec2i, 4> kDirections{
        Vec2i{.x = 1, .y = 0},
        Vec2i{.x = 0, .y = 1},
        Vec2i{.x = -1, .y = 0},
        Vec2i{.x = 0, .y = -1},

    };

    Neighbors result{};

    for (const auto& dir : kDirections) {
        Vec2i neighbor{.x = pos.x + dir.x, .y = pos.y + dir.y};

        if (is_valid(neighbor) && !is_wall(neighbor)) {
            result.data.at(result.count) = neighbor;
            ++result.count;
        }
    }

    return result;
}

bool Grid::is_valid(Vec2i pos) const {
    return pos.x >= 0 && pos.x < width_ && pos.y >= 0 && pos.y < height_;
}

void Grid::reset_path_state() {
    for (auto& cell : cells_) {
        if (cell == CellState::Visited || cell == CellState::Frontier || cell == CellState::Path) {
            cell = CellState::Empty;
        }
    }

    // restore waypoint cells that may have been overwritten by the path state
    for (const auto& wp : waypoints_) {
        cells_[index_at(wp)] = CellState::Waypoint;
    }
}

void Grid::clear() {
    std::ranges::fill(cells_, CellState::Empty);
    std::ranges::fill(walls_, 0);
    std::ranges::fill(weights_, 1);
    clear_waypoints();

    start_ = {.x = 0, .y = 0};
    end_ = {.x = width_ - 1, .y = height_ - 1};
    cells_[index_at(start_)] = CellState::Start;
    cells_[index_at(end_)] = CellState::End;
}

int Grid::index_at(Vec2i pos) const {
    assert(is_valid(pos));
    return (pos.y * width_) + pos.x;
}

} // namespace pathsim