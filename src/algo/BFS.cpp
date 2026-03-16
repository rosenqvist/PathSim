#include "BFS.hpp"

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <vector>

namespace pathsim {
namespace {

// hash for Vec2i so it can be used as an unordered_map key
struct Vec2iHash {
    std::size_t operator()(const Vec2i& v) const {
        // combine x & y into a single hash with bit shifting
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 16);
    }
};

std::vector<Vec2i> reconstruct_path(const std::unordered_map<Vec2i, Vec2i, Vec2iHash>& came_from,
                                    Vec2i start, Vec2i end) {

    std::vector<Vec2i> path;
    Vec2i current = end;

    while (current != start) {
        path.push_back(current);
        current = came_from.at(current);
    }
    path.push_back(start);

    std::ranges::reverse(path);
    return path;
}
} // namespace

PathResult bfs(const Grid& grid) {
    PathResult result;

    Vec2i start = grid.start();
    Vec2i end = grid.end();

    std::queue<Vec2i> frontier;
    std::unordered_map<Vec2i, Vec2i, Vec2iHash> came_from;

    frontier.push(start);
    came_from[start] = start; // start points to itself as a sentinel value

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
            std::vector<Vec2i> path = reconstruct_path(came_from, start, end);
            result.path_cost = static_cast<float>(path.size() - 1);

            for (const auto& pos : path) {
                if (pos != start && pos != end) {
                    result.steps.push_back({.position = pos, .new_state = CellState::Path});
                }
            }

            result.path = std::move(path);
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
    }

    // no path found we return an empty result
    return result;
}

} // namespace pathsim