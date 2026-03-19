#include "MazeGen.hpp"

#include <algorithm>
#include <array>
#include <random>
#include <stack>
#include <vector>

namespace pathsim {
namespace {

void carve_maze(Grid& grid, float passage_rate, std::mt19937& rng) {
    int w = grid.width();
    int h = grid.height();

    // Fill everything with walls
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            grid.set_wall({.x = x, .y = y}, true);
        }
    }

    // Track carved (passage) cells
    std::vector<uint8_t> carved(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
    auto idx = [w](Vec2i p) {
        return (static_cast<std::size_t>(p.y) * static_cast<std::size_t>(w)) +
               static_cast<std::size_t>(p.x);
    };

    Vec2i origin{.x = 0, .y = 0};
    grid.set_wall(origin, false);
    carved[idx(origin)] = 1;

    std::stack<Vec2i> stack;
    stack.push(origin);

    // Step 2 cells at a time: nodes at even coords, walls between them
    constexpr std::array<Vec2i, 4> kStep2{
        Vec2i{.x = 2, .y = 0},
        Vec2i{.x = 0, .y = 2},
        Vec2i{.x = -2, .y = 0},
        Vec2i{.x = 0, .y = -2},
    };

    while (!stack.empty()) {
        Vec2i current = stack.top();

        // Collect uncarved neighbors 2 steps away
        std::array<Vec2i, 4> unvisited{};
        int count = 0;

        for (const auto& dir : kStep2) {
            Vec2i next{.x = current.x + dir.x, .y = current.y + dir.y};
            if (grid.is_valid(next) && carved[idx(next)] == 0) {
                unvisited.at(count) = next;
                ++count;
            }
        }

        if (count == 0) {
            stack.pop();
            continue;
        }

        // Pick a random unvisited neighbor
        std::uniform_int_distribution<int> dist(0, count - 1);
        Vec2i chosen = unvisited.at(dist(rng));

        // Carve the wall cell between current and chosen
        Vec2i between{.x = (current.x + chosen.x) / 2, .y = (current.y + chosen.y) / 2};
        grid.set_wall(between, false);
        grid.set_wall(chosen, false);
        carved[idx(between)] = 1;
        carved[idx(chosen)] = 1;

        stack.push(chosen);
    }

    Vec2i end_pos = grid.end();
    Vec2i nearest_node{.x = (end_pos.x / 2) * 2, .y = (end_pos.y / 2) * 2};

    // Clear the small rectangle between the maze node and the end position
    for (int y = nearest_node.y; y <= end_pos.y; ++y) {
        for (int x = nearest_node.x; x <= end_pos.x; ++x) {
            grid.set_wall({.x = x, .y = y}, false);
        }
    }

    // Remove extra walls to create alternate routes
    std::vector<Vec2i> walls;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            Vec2i pos{.x = x, .y = y};
            if (grid.is_wall(pos)) {
                walls.push_back(pos);
            }
        }
    }

    std::ranges::shuffle(walls, rng);
    int to_remove = static_cast<int>(passage_rate * static_cast<float>(walls.size()));
    for (int i = 0; i < to_remove; ++i) {
        grid.set_wall(walls.at(static_cast<std::size_t>(i)), false);
    }
}

void add_terrain(Grid& grid, const GenerateConfig& config, std::mt19937& rng) {
    {
        int w = grid.width();
        int h = grid.height();

        // Count empty cells to know how many to cover
        int total_empty = 0;
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                Vec2i pos{.x = x, .y = y};
                if (!grid.is_wall(pos) && pos != grid.start() && pos != grid.end()) {
                    ++total_empty;
                }
            }
        }

        int target = static_cast<int>(config.weight_coverage * static_cast<float>(total_empty));
        int placed = 0;

        std::uniform_int_distribution<int> x_dist(0, w - 1);
        std::uniform_int_distribution<int> y_dist(0, h - 1);
        std::uniform_int_distribution<int> size_dist(3, 8);
        std::uniform_int_distribution<int> weight_dist(config.min_weight, config.max_weight);

        // Place rectangular weight clusters until we hit coverage target
        // prevent potential inifinte loop
        int attempts = 0;
        int max_attempts = target * 10;
        while (placed < target && attempts < max_attempts) {
            ++attempts;
            int cx = x_dist(rng);
            int cy = y_dist(rng);
            int rw = size_dist(rng);
            int rh = size_dist(rng);
            int weight = weight_dist(rng);

            for (int y = cy; y < std::min(cy + rh, h); ++y) {
                for (int x = cx; x < std::min(cx + rw, w); ++x) {
                    Vec2i pos{.x = x, .y = y};
                    if (!grid.is_wall(pos) && pos != grid.start() && pos != grid.end()) {
                        if (grid.weight(pos) == 1) {
                            grid.set_weight(pos, weight);
                            ++placed;
                        }
                    }
                }
            }
        }
    }
}
} // namespace

void generate(Grid& grid, GenerateMode mode, GenerateConfig config) {
    grid.clear();

    std::random_device rd;
    std::mt19937 rng(rd());

    if (mode == GenerateMode::Maze || mode == GenerateMode::MazeTerrain) {
        carve_maze(grid, config.passage_rate, rng);
    }

    if (mode == GenerateMode::Terrain || mode == GenerateMode::MazeTerrain) {
        add_terrain(grid, config, rng);
    }
}
} // namespace pathsim