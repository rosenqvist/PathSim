#pragma once

#include "core/Types.hpp"

#include <unordered_map>

namespace pathsim::algo_utils {

// Reconstructs the path from came_from map and records Path steps
// Used by Dijkstra and A* which both track cost_so_far
void record_path(PathResult& result, const std::unordered_map<Vec2i, Vec2i, Vec2iHash>& came_from,
                 const std::unordered_map<Vec2i, float, Vec2iHash>& cost_so_far, Vec2i start,
                 Vec2i end);

// Extracts lightweight stats from a full PathResult
AlgoStats extract_stats(const PathResult& result);

} // namespace pathsim::algo_utils