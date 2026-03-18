#pragma once

#include "../core/Grid.hpp"
#include "../core/Types.hpp"

#include <functional>

namespace pathsim {

using AlgoFunc = std::function<PathResult(const Grid&, Vec2i, Vec2i)>;

// Runs the given algorithm across all segments
// Returns a combined PathResult with all steps and the full path
PathResult find_path_with_waypoints(const Grid& grid, const AlgoFunc& algo);

} // namespace pathsim