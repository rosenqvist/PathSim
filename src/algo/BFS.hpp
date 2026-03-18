#pragma once

#include "../core/Grid.hpp"
#include "../core/Types.hpp"

namespace pathsim {
// Runs breadth-first search from grid.start() to grid.end().
// Returns a full recording of the search for playback,
// plus the shortest path if one exists.
// Does not modify the grid.
PathResult bfs(const Grid& grid);

// overload for pathfinding between arbitrary points
PathResult bfs(const Grid& grid, Vec2i start, Vec2i end);
} // namespace pathsim