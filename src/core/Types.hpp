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
    Impassable,
    Start,
    End,
    Visited,
    Frontier,
    Path,
    Waypoint,
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

// hash for Vec2i so it can be used as an unordered_map key
struct Vec2iHash {
    std::size_t operator()(const Vec2i& v) const {
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 16);
    }
};

} // namespace pathsim