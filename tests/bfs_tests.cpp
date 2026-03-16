#include "../src/algo/BFS.hpp"
#include "../src/core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("BFS finds a path on an open grid", "[bfs]") {
    pathsim::Grid grid(5, 5);

    pathsim::PathResult result = pathsim::bfs(grid);

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == grid.start());
    REQUIRE(result.path.back() == grid.end());
}

TEST_CASE("BFS finds the shortest path on an open grid", "[bfs]") {
    pathsim::Grid grid(5, 5);

    pathsim::PathResult result = pathsim::bfs(grid);

    // Manhattan distance on a 5x5 grid from (0,0) to (4,4) is 8
    REQUIRE(result.path.size() == 9);
    REQUIRE(result.path_cost == 8.0F);
}

TEST_CASE("BFS returns empty path when completely walled off", "[bfs]") {
    pathsim::Grid grid(5, 5);

    // Wall off row 2 completely, blocking all paths
    for (int col = 0; col < 5; ++col) {
        grid.set_wall({.x = col, .y = 2}, true);
    }

    pathsim::PathResult result = pathsim::bfs(grid);

    REQUIRE(result.path.empty());
}

TEST_CASE("BFS navigates around walls", "[bfs]") {
    pathsim::Grid grid(5, 5);

    // Wall off row 1, gap only at far right
    for (int col = 0; col < 4; ++col) {
        grid.set_wall({.x = col, .y = 1}, true);
    }
    // Wall off row 3, gap only at far left
    for (int col = 1; col < 5; ++col) {
        grid.set_wall({.x = col, .y = 3}, true);
    }

    pathsim::PathResult result = pathsim::bfs(grid);

    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.front() == grid.start());
    REQUIRE(result.path.back() == grid.end());
    REQUIRE(result.path.size() > 9);
}

TEST_CASE("BFS records visited nodes", "[bfs]") {
    pathsim::Grid grid(5, 5);

    pathsim::PathResult result = pathsim::bfs(grid);

    REQUIRE(result.nodes_visited > 0);
    REQUIRE_FALSE(result.steps.empty());
}

TEST_CASE("BFS ignores weights and finds fewest hops", "[bfs]") {
    pathsim::Grid grid(10, 3);

    // Top row: all weight 1 (cheap but longer)
    // Middle row: weight 9 (expensive but direct)
    // Start at (0,1), end at (9,1) — direct path crosses weight 9
    grid.set_start({.x = 0, .y = 1});
    grid.set_end({.x = 9, .y = 1});

    for (int col = 1; col < 9; ++col) {
        grid.set_weight({.x = col, .y = 1}, 9);
    }

    pathsim::PathResult result = pathsim::bfs(grid);

    REQUIRE_FALSE(result.path.empty());
    // BFS takes the direct route: 9 hops
    REQUIRE(result.path.size() == 10);
}