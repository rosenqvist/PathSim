#pragma once

#include "core/Grid.hpp"
#include "core/Types.hpp"

namespace pathsim {

// A* search. Extends Dijkstra's algorithm with a heuristic that estimates
// remaining cost to the goal (Manhattan distance for cardinal movement,
// octile distance for diagonal). Orders the priority queue by f = g + h
// which biases expansion toward the goal, visiting far fewer nodes than
// Dijkstra while still guaranteeing the optimal path.
//
// Does not modify the grid. Returns a full step recording for playback.
PathResult a_star(const Grid& grid);

// Overload for pathfinding between arbitrary start/end points.
// Used by the waypoint system to solve individual segments.
PathResult a_star(const Grid& grid, Vec2i start, Vec2i end);

} // namespace pathsim