#include "BFS.hpp"

#include "AlgoUtils.hpp"

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <vector>

namespace pathsim {

PathResult bfs(const Grid& grid, Vec2i start, Vec2i end) {
    PathResult result;
    result.algorithm_name = "BFS";

    std::queue<Vec2i> frontier;
    std::unordered_map<Vec2i, Vec2i, Vec2iHash> came_from;

    frontier.push(start);
    came_from[start] = start;

    while (!frontier.empty()) {
        Vec2i current = frontier.front();
        frontier.pop();

        // we record this cell as visited (skip start/end to prevent them from being overwritten)
        if (current != start && current != end) {
            result.steps.push_back({.position = current, .new_state = CellState::Visited});
        }
        ++result.nodes_visited;

        // if goal is found, reconstruct and record the path
        if (current == end) {
            algo_utils::record_path(result, came_from, grid, start, end);
            return result;
        }

        // expand neighbors
        for (const auto& neighbor : grid.neighbors(current)) {
            if (!came_from.contains(neighbor)) {
                came_from[neighbor] = current;
                frontier.push(neighbor);

                if (neighbor != end) {
                    result.steps.push_back(
                        {.position = neighbor, .new_state = CellState::Frontier});
                }
            }
        }

        int frontier_size = static_cast<int>(frontier.size());
        result.max_frontier_size = std::max(frontier_size, result.max_frontier_size);
    }

    // no path found we return an empty result
    return result;
}

PathResult bfs(const Grid& grid) {
    return bfs(grid, grid.start(), grid.end());
}

} // namespace pathsim