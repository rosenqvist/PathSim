#pragma once

#include "core/Grid.hpp"

#include <cstdint>

namespace pathsim {

enum class GenerateMode : std::uint8_t {
    Maze,        // generates walls only, goal is to test heuristic efficiency (A* vs BFS/Dijkstra)
    Terrain,     // generate weights only, test cost-aware routing efficiency (BFS vs Dijkstra/A*)
    MazeTerrain, // generate both maze and terrain, goal is to make all three algos diverge
};

struct GenerateConfig {
    float passage_rate{0.15F};
    float weight_coverage{0.3F};
    int min_weight{2};
    int max_weight{9};
};

// Populates the grid with generated content. Clears existing state first
// Start and end positions are preserved and is guaranteed to be reachable
void generate(Grid& grid, GenerateMode mode, GenerateConfig config = {});

} // namespace pathsim