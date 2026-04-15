#pragma once

#include "core/Grid.hpp"
#include "core/Types.hpp"

namespace pathsim {

// Breadth-first search. Explores cells layer by layer using a FIFO queue,
// guaranteeing the shortest path by hop count. Ignores cell weights entirely,
// so it may return an expensive path on weighted grids. Useful as a baseline
// to show the difference between "fewest steps" and "lowest cost".
//
// Does not modify the grid. Returns a full step recording for playback.
[[nodiscard]] PathResult bfs(const Grid& grid);

// Overload for pathfinding between arbitrary start/end points.
// Used by the waypoint system to solve individual segments.
[[nodiscard]] PathResult bfs(const Grid& grid, Vec2i start, Vec2i end);

} // namespace pathsim