#include "Dijkstra.hpp"

#include "algo/Dijkstra.hpp"
#include "core/Types.hpp"

#include <algorithm>
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
    float cost{};

    bool operator>(const Node& other) const { return cost > other.cost; }
};

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
            record_path(result, came_from, cost_so_far, start, end);
            return result;
        }

        for (const auto& neighbor : grid.neighbors(current.pos)) {
            float new_cost = cost_so_far[current.pos] + grid.move_cost(neighbor);

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
    }

    return result;
}

PathResult dijkstra(const Grid& grid) {
    return dijkstra(grid, grid.start(), grid.end());
}

} // namespace pathsim