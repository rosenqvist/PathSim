#include "Dijkstra.hpp"

#include "AlgoUtils.hpp"
#include "core/Types.hpp"

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <vector>

namespace pathsim {
namespace {

struct Node {
    Vec2i pos{};
    float cost{};

    bool operator>(const Node& other) const { return cost > other.cost; }
};

} // namespace

PathResult dijkstra(const Grid& grid, Vec2i start, Vec2i end) {
    PathResult result;
    result.algorithm_name = "Dijkstra";

    std::priority_queue<Node, std::vector<Node>, std::greater<>> frontier;
    std::unordered_map<Vec2i, Vec2i, Vec2iHash> came_from;
    std::unordered_map<Vec2i, float, Vec2iHash> cost_so_far;

    frontier.push({.pos = start, .cost = 0.0F});
    came_from[start] = start;
    cost_so_far[start] = 0.0F;

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        if (cost_so_far.contains(current.pos) && current.cost > cost_so_far[current.pos]) {
            continue;
        }

        if (current.pos != start && current.pos != end) {
            result.steps.push_back({.position = current.pos, .new_state = CellState::Visited});
        }
        ++result.nodes_visited;

        if (current.pos == end) {
            algo_utils::record_path(result, came_from, cost_so_far, start, end);
            return result;
        }

        for (const auto& neighbor : grid.neighbors(current.pos)) {
            float new_cost = cost_so_far[current.pos] + grid.move_cost(current.pos, neighbor);

            if (!cost_so_far.contains(neighbor) || new_cost < cost_so_far[neighbor]) {
                cost_so_far[neighbor] = new_cost;
                came_from[neighbor] = current.pos;
                frontier.push({.pos = neighbor, .cost = new_cost});

                if (neighbor != end) {
                    result.steps.push_back(
                        {.position = neighbor, .new_state = CellState::Frontier});
                }
            }
        }

        int frontier_size = static_cast<int>(frontier.size());
        result.max_frontier_size = std::max(frontier_size, result.max_frontier_size);
    }

    return result;
}

PathResult dijkstra(const Grid& grid) {
    return dijkstra(grid, grid.start(), grid.end());
}

} // namespace pathsim