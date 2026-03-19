#include "AlgoUtils.hpp"

#include <algorithm>

namespace pathsim::algo_utils {

void record_path(PathResult& result, const std::unordered_map<Vec2i, Vec2i, Vec2iHash>& came_from,
                 const Grid& grid, Vec2i start, Vec2i end) {

    std::vector<Vec2i> path;
    Vec2i current = end;

    while (current != start) {
        path.push_back(current);
        current = came_from.at(current);
    }
    path.push_back(start);

    std::ranges::reverse(path);

    float cost{0.0F};
    for (std::size_t i = 1; i < path.size(); ++i) {
        cost += grid.move_cost(path[i - 1], path[i]);
    }
    result.path_cost = cost;

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