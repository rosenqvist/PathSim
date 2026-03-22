#include "AlgoUtils.hpp"

#include <algorithm>

namespace pathsim::algo_utils {

void record_path(PathResult& result, const std::vector<Vec2i>& came_from, const Grid& grid,
                 Vec2i start, Vec2i end) {

    int w = grid.width();

    // Walk backwards from end to start using the parent pointers.
    // Each cell's parent was recorded during the search, so following
    // came_from[idx] traces the shortest path in reverse.
    std::vector<Vec2i> path;
    Vec2i current = end;

    while (current != start) {
        path.push_back(current);
        current = came_from[flat_index(current, w)];
    }
    path.push_back(start);

    // The path was built end-to-start so reverse it to get start-to-end order
    std::ranges::reverse(path);

    // Accumulate the actual movement cost along the path using the grid's
    // weight system. This lets us show the true cost even for BFS, which
    // doesn't optimize for cost but still incurs it.
    float cost{0.0F};
    for (std::size_t i = 1; i < path.size(); ++i) {
        cost += grid.move_cost(path[i - 1], path[i]);
    }
    result.path_cost = cost;

    // Record Path steps for the playback system. Skip start and end so the
    // renderer keeps showing their distinct markers instead of overwriting
    // them with the path color.
    for (const auto& pos : path) {
        if (pos != start && pos != end) {
            result.steps.push_back({.position = pos, .new_state = CellState::Path});
        }
    }

    result.path = std::move(path);
}

AlgoStats extract_stats(const PathResult& result) {
    return {
        .nodes_visited = result.nodes_visited,
        .max_frontier_size = result.max_frontier_size,
        .path_length = static_cast<int>(result.path.size()),
        .path_cost = result.path_cost,
        .compute_time_ms = result.compute_time_ms,
        .algorithm_name = result.algorithm_name,
    };
}

} // namespace pathsim::algo_utils