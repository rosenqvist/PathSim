#pragma once

#include "../core/Grid.hpp"
#include "../core/Types.hpp"

namespace pathsim {

// Runs Dijkstra's algorithm from grid.start() to grid.end().
// Uniform cost on an unweighted grid, demonstrates priority-queue structure.
// Does not modify the grid.
PathResult dijkstra(const Grid& grid);

// overload for pathfinding between arbitrary points
PathResult dijkstra(const Grid& grid, Vec2i start, Vec2i end);
} // namespace pathsim