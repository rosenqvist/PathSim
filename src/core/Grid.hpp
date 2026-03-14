#pragma once

#include "Types.hpp"

#include <array>
#include <cassert>
#include <vector>

namespace pathsim {

struct Neighbors {
    std::array<Vec2i, 4> data{};
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

    // start & end, enforcing exactly one of each
    [[nodiscard]] Vec2i start() const;
    [[nodiscard]] Vec2i end() const;
    void set_start(Vec2i pos);
    void set_end(Vec2i pos);

    // returns in-bounds non wall neighbors (4 directional)
    [[nodiscard]] Neighbors neighbors(Vec2i pos) const;

    // Bounds check
    [[nodiscard]] bool is_valid(Vec2i pos) const;

    // Clears Visited/Frontier/Path cells back to Empty. Keeps walls/start/end.
    void reset_path_state();

    // full reset
    void clear();

  private:
    int width_{};
    int height_{};
    std::vector<CellState> cells_;
    std::vector<uint8_t> walls_;
    Vec2i start_{};
    Vec2i end_{};

    // unchecked in release, asserted in debug, all access needs to go through here
    [[nodiscard]] int index_at(Vec2i pos) const;
};

} // namespace pathsim