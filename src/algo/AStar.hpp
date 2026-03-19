#pragma once

#include "core/Grid.hpp"
#include "core/Types.hpp"

namespace pathsim {

// Runs A* search from grid.start() to grid.end().
// Uses Manhattan distance as the heuristic.
// Does not modify the grid.
PathResult a_star(const Grid& grid);

// overload for pathfinding between arbitrary points
PathResult a_star(const Grid& grid, Vec2i start, Vec2i end);
} // namespace pathsim