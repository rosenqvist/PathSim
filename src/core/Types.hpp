#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace pathsim {

struct Vec2i {
    int x{};
    int y{};

    bool operator==(const Vec2i& other) const = default;
    bool operator!=(const Vec2i& other) const = default;
};

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

enum class CellDirection : uint8_t {
    None,
    North,
    South,
    East,
    West,
};

struct AlgoStep {
    Vec2i position{};
    CellState new_state{};
};

struct PathResult {
    std::vector<AlgoStep> steps;
    std::vector<Vec2i> path;
    int nodes_visited{};
    int max_frontier_size{};
    float path_cost{};
    float compute_time_ms{};
    const char* algorithm_name = "None";
};

// lightweight snapshot for comparing algorithms without having to store full step recordings
struct AlgoStats {
    int nodes_visited{};
    int max_frontier_size{};
    int path_length{};
    float path_cost{};
    float compute_time_ms{};
    const char* algorithm_name = "None";
};

using AlgoHistory = std::unordered_map<std::string, AlgoStats>;

struct Vec2iHash {
    std::size_t operator()(const Vec2i& v) const {
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 16);
    }
};

} // namespace pathsim