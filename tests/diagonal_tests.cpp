#include "algo/AStar.hpp"
#include "algo/BFS.hpp"
#include "algo/Dijkstra.hpp"
#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Diagonals disabled returns max 4 neighbors", "[diagonal]") {
    pathsim::Grid grid(10, 10);
    grid.set_allow_diagonals(false);

    auto n = grid.neighbors({.x = 5, .y = 5});
    REQUIRE(n.count <= 4);
}

TEST_CASE("Diagonals enabled returns up to 8 neighbors", "[diagonal]") {
    pathsim::Grid grid(10, 10);
    grid.set_allow_diagonals(true);

    auto n = grid.neighbors({.x = 5, .y = 5});
    REQUIRE(n.count == 8);
}

TEST_CASE("Corner cells have correct neighbor count with diagonals", "[diagonal]") {
    pathsim::Grid grid(10, 10);
    grid.set_allow_diagonals(true);

    auto corner = grid.neighbors({.x = 0, .y = 0});
    REQUIRE(corner.count == 3);

    auto edge = grid.neighbors({.x = 5, .y = 0});
    REQUIRE(edge.count == 5);
}

TEST_CASE("Diagonal movement blocked by corner walls", "[diagonal]") {
    pathsim::Grid grid(10, 10);
    grid.set_allow_diagonals(true);

    // Wall east and south of (5,5) and diagonal to (6,6) should be blocked
    grid.set_wall({.x = 6, .y = 5}, true);
    grid.set_wall({.x = 5, .y = 6}, true);

    auto n = grid.neighbors({.x = 5, .y = 5});

    bool found_diagonal = false;
    for (const auto& neighbor : n) {
        if (neighbor.x == 6 && neighbor.y == 6) {
            found_diagonal = true;
        }
    }
    REQUIRE_FALSE(found_diagonal);
}

TEST_CASE("Diagonal movement allowed when one adjacent wall", "[diagonal]") {
    pathsim::Grid grid(10, 10);
    grid.set_allow_diagonals(true);

    // Only wall east of (5,5) — diagonal to (6,6) blocked, but (4,6) allowed
    grid.set_wall({.x = 6, .y = 5}, true);

    auto n = grid.neighbors({.x = 5, .y = 5});

    bool found_46 = false;
    bool found_66 = false;
    for (const auto& neighbor : n) {
        if (neighbor.x == 4 && neighbor.y == 6) {
            found_46 = true;
        }
        if (neighbor.x == 6 && neighbor.y == 6) {
            found_66 = true;
        }
    }
    REQUIRE(found_46);
    REQUIRE_FALSE(found_66);
}

TEST_CASE("Diagonal path is shorter than cardinal-only path", "[diagonal]") {
    pathsim::Grid grid(10, 10);

    auto cardinal = pathsim::bfs(grid);

    grid.set_allow_diagonals(true);
    auto diagonal = pathsim::bfs(grid);

    REQUIRE_FALSE(cardinal.path.empty());
    REQUIRE_FALSE(diagonal.path.empty());
    REQUIRE(diagonal.path.size() < cardinal.path.size());
}

TEST_CASE("Diagonal move cost should use sqrt(2) factor", "[diagonal]") {
    pathsim::Grid grid(10, 10);

    pathsim::Vec2i from{.x = 5, .y = 5};
    pathsim::Vec2i cardinal{.x = 6, .y = 5};
    pathsim::Vec2i diagonal{.x = 6, .y = 6};

    float cardinal_cost = grid.move_cost(from, cardinal);
    float diagonal_cost = grid.move_cost(from, diagonal);

    REQUIRE(cardinal_cost == 1.0F);
    REQUIRE(diagonal_cost > 1.4F);
    REQUIRE(diagonal_cost < 1.5F);
}

TEST_CASE("All algorithms should find a path with diagonals enabled", "[diagonal]") {
    pathsim::Grid grid(10, 10);
    grid.set_allow_diagonals(true);

    auto bfs_result = pathsim::bfs(grid);
    auto dij_result = pathsim::dijkstra(grid);
    auto astar_result = pathsim::a_star(grid);

    REQUIRE_FALSE(bfs_result.path.empty());
    REQUIRE_FALSE(dij_result.path.empty());
    REQUIRE_FALSE(astar_result.path.empty());
}

TEST_CASE("Dijkstra and A* find same cost with diagonals", "[diagonal]") {
    pathsim::Grid grid(20, 20);
    grid.set_allow_diagonals(true);

    // Add some weights to make it interesting
    grid.set_weight({.x = 5, .y = 5}, 5);
    grid.set_weight({.x = 6, .y = 6}, 5);
    grid.set_weight({.x = 7, .y = 7}, 5);

    auto dij_result = pathsim::dijkstra(grid);
    auto astar_result = pathsim::a_star(grid);

    REQUIRE_FALSE(dij_result.path.empty());
    REQUIRE_FALSE(astar_result.path.empty());
    REQUIRE(dij_result.path_cost == astar_result.path_cost);
}