#include "../src/algo/BFS.hpp"
#include "../src/core/Grid.hpp"
#include "algo/Dijkstra.hpp"

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

// todo: improve this test case even further
// should prove that bfs does go through expensive cells
// should show consequences of ignoring weighted cells
TEST_CASE("BFS ignores weights and finds fewest hops", "[bfs]") {
    pathsim::Grid grid(10, 3);

    grid.set_start({.x = 0, .y = 1});
    grid.set_end({.x = 9, .y = 1});

    // Middle row is expensive
    for (int col = 1; col < 9; ++col) {
        grid.set_weight({.x = col, .y = 1}, 9);
    }

    pathsim::PathResult result = pathsim::bfs(grid);

    REQUIRE_FALSE(result.path.empty());

    // BFS takes the direct 9-hop route through expensive cells
    REQUIRE(result.path.size() == 10);

    // Verify BFS actually traversed the expensive middle row
    for (int col = 1; col < 9; ++col) {
        pathsim::Vec2i expected{.x = col, .y = 1};
        REQUIRE(std::ranges::find(result.path, expected) != result.path.end());
    }

    // BFS path costs more than the optimal weighted path
    pathsim::PathResult dijkstra_result = pathsim::dijkstra(grid);
    REQUIRE(result.path_cost > dijkstra_result.path_cost);

    // But BFS used fewer or equal hops
    REQUIRE(result.path.size() <= dijkstra_result.path.size());
}