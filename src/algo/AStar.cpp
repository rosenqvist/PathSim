#include "AStar.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace pathsim {
namespace {

struct Vec2iHash {
    std::size_t operator()(const Vec2i& v) const {
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 16);
    }
};

struct Node {
    Vec2i pos{};
    float priority{};
    float g_cost{};

    bool operator>(const Node& other) const {
        if (priority != other.priority) {
            return priority > other.priority;
        }
        // Tie-break: prefer higher g-cost (closer to goal)
        return g_cost < other.g_cost;
    }
};

float manhattan_distance(Vec2i a, Vec2i b) {
    return static_cast<float>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
}

void record_path(PathResult& result, const std::unordered_map<Vec2i, Vec2i, Vec2iHash>& came_from,
                 const std::unordered_map<Vec2i, float, Vec2iHash>& cost_so_far, Vec2i start,
                 Vec2i end) {

    std::vector<Vec2i> path;
    Vec2i current = end;

    while (current != start) {
        path.push_back(current);
        current = came_from.at(current);
    }
    path.push_back(start);

    std::ranges::reverse(path);

    result.path_cost = cost_so_far.at(end);

    for (const auto& pos : path) {
        if (pos != start && pos != end) {
            result.steps.push_back({.position = pos, .new_state = CellState::Path});
        }
    }

    result.path = std::move(path);
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

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();

        if (current.pos != start && current.pos != end) {
            result.steps.push_back({.position = current.pos, .new_state = CellState::Visited});
        }
        ++result.nodes_visited;

        if (current.pos == end) {
            record_path(result, came_from, cost_so_far, start, end);
            return result;
        }

        for (const auto& neighbor : grid.neighbors(current.pos)) {
            float new_cost = cost_so_far[current.pos] + grid.move_cost(neighbor);

            if (!cost_so_far.contains(neighbor) || new_cost < cost_so_far[neighbor]) {
                cost_so_far[neighbor] = new_cost;
                came_from[neighbor] = current.pos;

                float priority = new_cost + manhattan_distance(neighbor, end);
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