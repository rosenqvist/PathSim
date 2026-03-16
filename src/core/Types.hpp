#pragma once

#include <cstdint>
#include <vector>

namespace pathsim {

// 2d points, these are for grid coordinates
struct Vec2i {
    int x{};
    int y{};

    bool operator==(const Vec2i& other) const = default;
    bool operator!=(const Vec2i& other) const = default;
};

// every cell in the grid can be one of these states
enum class CellState : uint8_t {
    Empty,
    Wall,
    Start,
    End,
    Visited,
    Frontier,
    Path,
};

// for recording algo steps
struct AlgoStep {
    Vec2i position{};
    CellState new_state{};
};

// this is what every algorithm should return when finished
// steps is the full recording of the path animation
// path is the final route
// stats track how many nodes were explored, total cost
struct PathResult {
    std::vector<AlgoStep> steps;
    std::vector<Vec2i> path;
    int nodes_visited{};
    float path_cost{};
    const char* algorithm_name = "None";
};

} // namespace pathsim