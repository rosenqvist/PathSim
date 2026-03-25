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

// Snapshot of a single cell's full state which is used to save or restore cells
// when dragging start or end across the grid without destroying existing content.
struct CellData {
    CellState state{CellState::Empty};
    uint8_t wall{0};
    uint8_t weight{1};
    CellDirection direction{CellDirection::None};
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
    std::string algorithm_name = "None";
};

// lightweight snapshot for comparing algorithms without having to store full step recordings
struct AlgoStats {
    int nodes_visited{};
    int max_frontier_size{};
    int path_length{};
    float path_cost{};
    float compute_time_ms{};
    std::string algorithm_name = "None";
};

using AlgoHistory = std::unordered_map<std::string, AlgoStats>;

struct Vec2iHash {
    // Knuth multiplicative hash (got this from art of computer programming book) to spread y-values
    // across buckets and reduce collisions from the simple XOR with x
    std::size_t operator()(const Vec2i& v) const {
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) * 2654435761U);
    }
};

} // namespace pathsim