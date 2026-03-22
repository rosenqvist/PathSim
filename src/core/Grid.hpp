#pragma once

#include "Types.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <vector>

namespace pathsim {

// Stack-allocated container for neighbor positions returned by Grid::neighbors().
//
// Holds up to 8 neighbors (4 cardinal + 4 diagonal) in a fixed-size array.
// Only the first `count` elements are valid. begin() and end() are defined
// to iterate exactly the valid range, so this will work in range-for loops:
//
//   for (const auto& neighbor : grid.neighbors(pos)) { ... }
//
// Using a fixed array avoids heap allocation on every neighbor lookup, which
// matters because this function is called for every node the algorithms would eventually visit.
struct Neighbors {
    std::array<Vec2i, 8> data{};
    int count{};

    [[nodiscard]] auto begin() const { return data.begin(); }
    [[nodiscard]] auto end() const { return data.begin() + count; }
};

class Grid {
  public:
    Grid(int width, int height);

    // Dimensions
    [[nodiscard]] int width() const;
    [[nodiscard]] int height() const;

    // cell access for renderer and algo playback
    [[nodiscard]] CellState at(Vec2i pos) const;
    void set(Vec2i pos, CellState state);

    // wall operations, separate bitfields for fast pathfinding checks
    [[nodiscard]] bool is_wall(Vec2i pos) const;
    void set_wall(Vec2i pos, bool wall);
    void set_impassable(Vec2i pos, bool impassable);
    [[nodiscard]] int weight(Vec2i pos) const;
    void set_weight(Vec2i pos, int weight);

    // cell directions for one-way cells
    [[nodiscard]] CellDirection direction(Vec2i pos) const;
    void set_direction(Vec2i pos, CellDirection dir);

    // start & end, enforcing exactly one of each
    [[nodiscard]] Vec2i start() const;
    [[nodiscard]] Vec2i end() const;
    void set_start(Vec2i pos);
    void set_end(Vec2i pos);

    // returns in-bounds non-wall neighbors (up to 8 with diagonals)
    [[nodiscard]] Neighbors neighbors(Vec2i pos) const;
    void set_allow_diagonals(bool allow);
    [[nodiscard]] bool allow_diagonals() const;

    // Returns sqrt(2) * weight for diagonal moves, weight for cardinal
    // Cost of moving from one cell to an adjacent cell
    [[nodiscard]] float move_cost(Vec2i from, Vec2i to) const;

    // Bounds check
    [[nodiscard]] bool is_valid(Vec2i pos) const;

    void add_waypoint(Vec2i pos);
    void remove_waypoint(Vec2i pos);
    [[nodiscard]] const std::vector<Vec2i>& waypoints() const;
    void clear_waypoints();

    // Resizes the grid, clears all state
    void resize(int width, int height);

    // Clears Visited/Frontier/Path cells back to Empty. Keeps walls/start/end.
    void reset_path_state();
    // full reset
    void clear();

  private:
    int width_{};
    int height_{};
    std::vector<CellState> cells_;
    std::vector<uint8_t> walls_;
    std::vector<uint8_t> weights_;
    Vec2i start_{};
    Vec2i end_{};
    std::vector<Vec2i> waypoints_;
    bool allow_diagonals_{false};
    std::vector<CellDirection> directions_;

    // unchecked in release, asserted in debug, all access needs to go through here
    [[nodiscard]] int index_at(Vec2i pos) const;
};

} // namespace pathsim