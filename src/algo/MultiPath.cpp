#include "MultiPath.hpp"

#include <algorithm>

namespace pathsim {

PathResult find_path_with_waypoints(const Grid& grid, const AlgoFunc& algo) {
    const auto& waypoints = grid.waypoints();

    std::vector<Vec2i> stops;
    stops.push_back(grid.start());
    for (const auto& wp : waypoints) {
        stops.push_back(wp);
    }
    stops.push_back(grid.end());

    PathResult combined;
    combined.algorithm_name = "None";

    std::vector<AlgoStep> exploration_steps;
    std::vector<AlgoStep> path_steps;

    for (std::size_t i = 0; i + 1 < stops.size(); ++i) {
        PathResult segment = algo(grid, stops[i], stops[i + 1]);

        if (segment.path.empty()) {
            combined.algorithm_name = segment.algorithm_name;
            combined.steps = std::move(exploration_steps);
            combined.path.clear();
            combined.path_cost = 0.0F;
            return combined;
        }

        combined.algorithm_name = segment.algorithm_name;
        combined.nodes_visited += segment.nodes_visited;
        combined.path_cost += segment.path_cost;

        combined.max_frontier_size =
            std::max(segment.max_frontier_size, combined.max_frontier_size);

        for (const auto& step : segment.steps) {
            if (step.new_state == CellState::Path) {
                path_steps.push_back(step);
            } else {
                exploration_steps.push_back(step);
            }
        }

        if (combined.path.empty()) {
            for (const auto& pos : segment.path) {
                combined.path.push_back(pos);
            }
        } else {
            for (std::size_t j = 1; j < segment.path.size(); ++j) {
                combined.path.push_back(segment.path[j]);
            }
        }
    }

    // All exploration first then path
    combined.steps = std::move(exploration_steps);
    for (const auto& step : path_steps) {
        combined.steps.push_back(step);
    }

    return combined;
}

} // namespace pathsim