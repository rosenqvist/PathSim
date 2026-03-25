#include "Grid.hpp"

#include "Types.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <numbers>

namespace pathsim {
Grid::Grid(int width, int height)
    : width_(width), height_(height),
      cells_(static_cast<std::size_t>(width * height), CellState::Empty),
      walls_(static_cast<std::size_t>(width * height), 0),
      weights_(static_cast<std::size_t>(width * height), 1), start_{.x = 0, .y = 0},
      end_{.x = width - 1, .y = height - 1},
      directions_(static_cast<std::size_t>(width * height), CellDirection::None) {
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

    if (pos == start_ || pos == end_) {
        return;
    }

    walls_[index_at(pos)] = static_cast<unsigned char>(wall);
    cells_[index_at(pos)] = wall ? CellState::Wall : CellState::Empty;

    if (wall) {
        weights_[index_at(pos)] = 1;
        directions_[index_at(pos)] = CellDirection::None;

        // Remove from waypoints if present (cell state already set to Wall above)
        auto it = std::ranges::find(waypoints_, pos);
        if (it != waypoints_.end()) {
            waypoints_.erase(it);
        }
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
        directions_[index_at(pos)] = CellDirection::None;

        // Remove from waypoints if present (cell state already set to Impassable above)
        auto it = std::ranges::find(waypoints_, pos);
        if (it != waypoints_.end()) {
            waypoints_.erase(it);
        }
    } else {
        if (cells_[index_at(pos)] == CellState::Impassable) {
            cells_[index_at(pos)] = CellState::Empty;
        }
    }
}

void Grid::add_waypoint(Vec2i pos) {
    // Bounds-check and return instead of assert should be called directly from users input
    if (!is_valid(pos) || pos == start_ || pos == end_ || is_wall(pos)) {
        return;
    }

    // prevent duplicates
    if (std::ranges::any_of(waypoints_, [&pos](const Vec2i& wp) { return wp == pos; })) {
        return;
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

CellData Grid::cell_data(Vec2i pos) const {
    assert(is_valid(pos));
    auto idx = index_at(pos);
    return {
        .state = cells_[idx],
        .wall = walls_[idx],
        .weight = weights_[idx],
        .direction = directions_[idx],
    };
}

void Grid::restore_cell(Vec2i pos, const CellData& data) {
    assert(is_valid(pos));
    auto idx = index_at(pos);
    cells_[idx] = data.state;
    walls_[idx] = data.wall;
    weights_[idx] = data.weight;
    directions_[idx] = data.direction;

    // If restoring a waypoint we need to make sure it's back in the list
    if (data.state == CellState::Waypoint) {
        bool already =
            std::ranges::any_of(waypoints_, [&pos](const Vec2i& wp) { return wp == pos; });
        if (!already) {
            waypoints_.push_back(pos);
        }
    }
}

CellDirection Grid::direction(Vec2i pos) const {
    assert(is_valid(pos));
    return directions_[index_at(pos)];
}

void Grid::set_direction(Vec2i pos, CellDirection dir) {
    assert(is_valid(pos));
    // is_wall() covers both Wall and Impassable cells
    if (is_wall(pos)) {
        return;
    }
    directions_[index_at(pos)] = dir;
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

float Grid::move_cost(Vec2i from, Vec2i to) const {
    assert(is_valid(from));
    assert(is_valid(to));

    auto dest_weight = static_cast<float>(weights_[index_at(to)]);
    bool diagonal = (from.x != to.x) && (from.y != to.y);

    if (diagonal) {
        return dest_weight * std::numbers::sqrt2_v<float>;
    }
    return dest_weight;
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
    weights_[index_at(pos)] = 1;
    directions_[index_at(pos)] = CellDirection::None;

    auto it = std::ranges::find(waypoints_, pos);
    if (it != waypoints_.end()) {
        waypoints_.erase(it);
    }
}

void Grid::set_end(Vec2i pos) {
    assert(is_valid(pos));

    // clear old end and place a new one, remove wall if present
    cells_[index_at(end_)] = CellState::Empty;
    end_ = pos;
    walls_[index_at(pos)] = 0U;
    cells_[index_at(pos)] = CellState::End;
    weights_[index_at(pos)] = 1;
    directions_[index_at(pos)] = CellDirection::None;

    auto it = std::ranges::find(waypoints_, pos);
    if (it != waypoints_.end()) {
        waypoints_.erase(it);
    }
}

Neighbors Grid::neighbors(Vec2i pos) const {
    assert(is_valid(pos));

    constexpr std::array<Vec2i, 4> kCardinal{
        Vec2i{.x = 1, .y = 0},
        Vec2i{.x = 0, .y = 1},
        Vec2i{.x = -1, .y = 0},
        Vec2i{.x = 0, .y = -1},
    };

    constexpr std::array<CellDirection, 4> kMoveDir{
        CellDirection::East,
        CellDirection::South,
        CellDirection::West,
        CellDirection::North,
    };

    constexpr std::array<Vec2i, 4> kDiagonal{
        Vec2i{.x = 1, .y = 1},
        Vec2i{.x = -1, .y = 1},
        Vec2i{.x = -1, .y = -1},
        Vec2i{.x = 1, .y = -1},
    };

    Neighbors result{};

    // Cardinal directions
    for (std::size_t i = 0; i < kCardinal.size(); ++i) {
        Vec2i neighbor{.x = pos.x + kCardinal.at(i).x, .y = pos.y + kCardinal.at(i).y};

        if (!is_valid(neighbor) || is_wall(neighbor)) {
            continue;
        }

        CellDirection our_dir = directions_.at(index_at(pos));
        if (our_dir != CellDirection::None && our_dir != kMoveDir.at(i)) {
            continue;
        }

        CellDirection their_dir = directions_.at(index_at(neighbor));
        if (their_dir != CellDirection::None && their_dir != kMoveDir.at(i)) {
            continue;
        }

        result.data.at(result.count) = neighbor;
        ++result.count;
    }

    if (!allow_diagonals_) {
        return result;
    }

    // If current cell has a direction constraint, no diagonal exits
    if (directions_.at(index_at(pos)) != CellDirection::None) {
        return result;
    }

    for (const auto& dir : kDiagonal) {
        Vec2i neighbor{.x = pos.x + dir.x, .y = pos.y + dir.y};

        if (!is_valid(neighbor) || is_wall(neighbor)) {
            continue;
        }

        // Can't diagonally enter a one-way cell
        if (directions_.at(index_at(neighbor)) != CellDirection::None) {
            continue;
        }

        Vec2i adj_x{.x = pos.x + dir.x, .y = pos.y};
        Vec2i adj_y{.x = pos.x, .y = pos.y + dir.y};

        if (is_wall(adj_x) || is_wall(adj_y)) {
            continue;
        }

        result.data.at(result.count) = neighbor;
        ++result.count;
    }

    return result;
}

void Grid::set_allow_diagonals(bool allow) {
    allow_diagonals_ = allow;
}

bool Grid::allow_diagonals() const {
    return allow_diagonals_;
}

bool Grid::is_valid(Vec2i pos) const {
    return pos.x >= 0 && pos.x < width_ && pos.y >= 0 && pos.y < height_;
}

void Grid::reset_path_state() {
    std::ranges::replace_if(
        cells_,
        [](CellState s) {
            return s == CellState::Visited || s == CellState::Frontier || s == CellState::Path;
        },
        CellState::Empty);

    // restore waypoint cells that may have been overwritten by the path state
    for (const auto& wp : waypoints_) {
        cells_[index_at(wp)] = CellState::Waypoint;
    }
}

void Grid::resize(int width, int height) {
    if (width <= 0 || height <= 0) {
        return;
    }

    width_ = width;
    height_ = height;

    auto total = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    cells_.assign(total, CellState::Empty);
    walls_.assign(total, 0);
    weights_.assign(total, 1);
    directions_.assign(total, CellDirection::None);
    waypoints_.clear();

    start_ = {.x = 0, .y = 0};
    end_ = {.x = width - 1, .y = height - 1};
    cells_[index_at(start_)] = CellState::Start;
    cells_[index_at(end_)] = CellState::End;
}

void Grid::clear() {
    std::ranges::fill(cells_, CellState::Empty);
    std::ranges::fill(walls_, 0);
    std::ranges::fill(weights_, 1);
    std::ranges::fill(directions_, CellDirection::None);
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