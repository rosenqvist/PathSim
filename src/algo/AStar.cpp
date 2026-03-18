#include "AStar.hpp"

#include "AlgoUtils.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <queue>
#include <unordered_map>
#include <vector>

namespace pathsim {
namespace {

struct Node {
    Vec2i pos{};
    float priority{};
    float g_cost{};

    bool operator>(const Node& other) const {
        if (priority != other.priority) {
            return priority > other.priority;
        }
        return g_cost < other.g_cost;
    }
};

float heuristic(Vec2i a, Vec2i b, bool diagonals) {
    int dx = std::abs(a.x - b.x);
    int dy = std::abs(a.y - b.y);

    if (diagonals) {
        int mn = std::min(dx, dy);
        int mx = std::max(dx, dy);
        return (static_cast<float>(mn) * std::numbers::sqrt2_v<float>)+static_cast<float>(mx - mn);
    }
    return static_cast<float>(dx + dy);
}
} // namespace

PathResult a_star(const Grid& grid, Vec2i start, Vec2i end) {
    PathResult result;
    result.algorithm_name = "A*";

    std::priority_queue<Node, std::vector<Node>, std::greater<>> frontier;
    std::unordered_map<Vec2i, Vec2i, Vec2iHash> came_from;
    std::unordered_map<Vec2i, float, Vec2iHash> cost_so_far;

    frontier.push({.pos = start, .priority = 0.0F, .g_cost = 0.0F});
    came_from[start] = start;
    cost_so_far[start] = 0.0F;

    bool diag = grid.allow_diagonals();

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        if (cost_so_far.contains(current.pos) && current.g_cost > cost_so_far[current.pos]) {
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

                float priority = new_cost + heuristic(neighbor, end, diag);
                frontier.push({.pos = neighbor, .priority = priority, .g_cost = new_cost});

                if (neighbor != end) {
                    result.steps.push_back(
                        {.position = neighbor, .new_state = CellState::Frontier});
                }
            }
        }
    }

    return result;
}

PathResult a_star(const Grid& grid) {
    return a_star(grid, grid.start(), grid.end());
}

} // namespace pathsim