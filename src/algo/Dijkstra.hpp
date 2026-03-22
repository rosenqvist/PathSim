#pragma once

#include "core/Grid.hpp"
#include "core/Types.hpp"

namespace pathsim {

// Dijkstra's algorithm. Uses a min-heap priority queue ordered by accumulated
// cost to always expand the cheapest reachable node next. Guarantees the
// optimal (lowest cost) path on weighted grids. Explores uniformly in all
// directions since it has no heuristic guiding it toward the goal.
//
// Does not modify the grid. Returns a full step recording for playback.
PathResult dijkstra(const Grid& grid);

// Overload for pathfinding between arbitrary start/end points.
// Used by the waypoint system to solve individual segments.
PathResult dijkstra(const Grid& grid, Vec2i start, Vec2i end);

} // namespace pathsim