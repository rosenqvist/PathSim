#include "algo/BFS.hpp"
#include "algo/MazeGen.hpp"
#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Generated maze is always solvable", "[mazegen]") {
    pathsim::Grid grid(40, 30);

    for (int i = 0; i < 20; ++i) {
        pathsim::generate(grid, pathsim::GenerateMode::Maze);
        pathsim::PathResult result = pathsim::bfs(grid);
        REQUIRE_FALSE(result.path.empty());
    }
}

TEST_CASE("Generated terrain has weighted cells", "[mazegen]") {
    pathsim::Grid grid(40, 30);
    pathsim::generate(grid, pathsim::GenerateMode::Terrain);

    int weighted = 0;
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            if (grid.weight({.x = x, .y = y}) > 1) {
                ++weighted;
            }
        }
    }

    REQUIRE(weighted > 0);
}

TEST_CASE("Maze + Terrain has both walls and weights", "[mazegen]") {
    pathsim::Grid grid(40, 30);
    pathsim::generate(grid, pathsim::GenerateMode::MazeTerrain);

    int walls = 0;
    int weighted = 0;
    for (int y = 0; y < grid.height(); ++y) {
        for (int x = 0; x < grid.width(); ++x) {
            pathsim::Vec2i pos{.x = x, .y = y};
            if (grid.is_wall(pos)) {
                ++walls;
            }
            if (grid.weight(pos) > 1) {
                ++weighted;
            }
        }
    }

    REQUIRE(walls > 0);
    REQUIRE(weighted > 0);
}

TEST_CASE("Maze + Terrain is always solvable", "[mazegen]") {
    pathsim::Grid grid(40, 30);

    for (int i = 0; i < 20; ++i) {
        pathsim::generate(grid, pathsim::GenerateMode::MazeTerrain);
        pathsim::PathResult result = pathsim::bfs(grid);
        REQUIRE_FALSE(result.path.empty());
    }
}

TEST_CASE("Start and end are never walled after generation", "[mazegen]") {
    pathsim::Grid grid(40, 30);

    pathsim::generate(grid, pathsim::GenerateMode::Maze);
    REQUIRE_FALSE(grid.is_wall(grid.start()));
    REQUIRE_FALSE(grid.is_wall(grid.end()));

    pathsim::generate(grid, pathsim::GenerateMode::MazeTerrain);
    REQUIRE_FALSE(grid.is_wall(grid.start()));
    REQUIRE_FALSE(grid.is_wall(grid.end()));
}