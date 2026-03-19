#pragma once

#include "core/Grid.hpp"
#include "core/Types.hpp"

#include <unordered_map>

namespace pathsim::algo_utils {

// Reconstructs the path from came_from map, computes cost from the grid,
// and records Path steps. Used by all three algorithms.
void record_path(PathResult& result, const std::unordered_map<Vec2i, Vec2i, Vec2iHash>& came_from,
                 const Grid& grid, Vec2i start, Vec2i end);

// Extracts lightweight stats from a full PathResult
AlgoStats extract_stats(const PathResult& result);

} // namespace pathsim::algo_utils