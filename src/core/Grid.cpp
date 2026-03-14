#include "Grid.hpp"

#include "Types.hpp"

#include <algorithm>
#include <array>
#include <cassert>

namespace pathsim {
Grid::Grid(int width, int height)
    : width_(width), height_(height),
      cells_(static_cast<std::size_t>(width * height), CellState::Empty),
      walls_(static_cast<std::size_t>(width * height), 0), start_{.x = 0, .y = 0},
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
    return walls_[index_at(pos)] != 0U;
}

void Grid::set_wall(Vec2i pos, bool wall) {
    assert(is_valid(pos));

    // prevent walling over start or end
    if (pos == start_ || pos == end_) {
        return;
    }

    walls_[index_at(pos)] = static_cast<unsigned char>(wall);
    cells_[index_at(pos)] = wall ? CellState::Wall : CellState::Empty;
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
}

void Grid::clear() {
    std::ranges::fill(cells_, CellState::Empty);
    std::ranges::fill(walls_, 0);

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