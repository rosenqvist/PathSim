#pragma once

#include "core/Grid.hpp"
#include "core/Types.hpp"

#include <vector>

namespace pathsim::algo_utils {

// Sentinel value for flat came_from vectors. A position of (-1, -1) means
// "this cell has no parent yet" since no valid grid coordinate is negative.
inline constexpr Vec2i kNoParent{.x = -1, .y = -1};

// Converts a 2D grid position to a flat index: y * width + x.
// Same layout the Grid class uses internally for cells_, walls_, etc.
inline int flat_index(Vec2i pos, int width) {
    return (pos.y * width) + pos.x;
}

// Reconstructs the path by walking came_from[] backwards from end to start,
// computes the total path cost from the grid's move_cost(), and appends
// Path steps to the result. Used by all three algorithms after they find
// the goal.
//
// came_from is a flat vector indexed by flat_index(). Each entry holds the
// parent position that led to this cell during the search. The start cell
// points to itself (came_from[start] == start).
void record_path(PathResult& result, const std::vector<Vec2i>& came_from, const Grid& grid,
                 Vec2i start, Vec2i end);

// Extracts lightweight stats from a full PathResult for the comparison panel.
// This avoids storing the entire step recording when we only need summary numbers.
AlgoStats extract_stats(const PathResult& result);

} // namespace pathsim::algo_utils