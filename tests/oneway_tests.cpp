#include "algo/BFS.hpp"
#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("One-way cell blocks wrong direction entry", "[oneway]") {
    pathsim::Grid grid(5, 5);

    // Cell (2,2) only allows east entry/exit
    grid.set_direction({.x = 2, .y = 2}, pathsim::CellDirection::East);

    auto n = grid.neighbors({.x = 2, .y = 1});

    // Moving south from (2,1) to (2,2) should be blocked
    bool found = false;
    for (const auto& neighbor : n) {
        if (neighbor.x == 2 && neighbor.y == 2) {
            found = true;
        }
    }
    REQUIRE_FALSE(found);
}

TEST_CASE("One-way cell allows correct direction entry", "[oneway]") {
    pathsim::Grid grid(5, 5);

    // Cell (2,2) only allows east
    grid.set_direction({.x = 2, .y = 2}, pathsim::CellDirection::East);

    // Moving east from (1,2) to (2,2) should work
    auto n = grid.neighbors({.x = 1, .y = 2});

    bool found = false;
    for (const auto& neighbor : n) {
        if (neighbor.x == 2 && neighbor.y == 2) {
            found = true;
        }
    }
    REQUIRE(found);
}

TEST_CASE("One-way cell blocks wrong direction exit", "[oneway]") {
    pathsim::Grid grid(5, 5);

    // Cell (2,2) only allows east
    grid.set_direction({.x = 2, .y = 2}, pathsim::CellDirection::East);

    auto n = grid.neighbors({.x = 2, .y = 2});

    // Should only be able to go east to (3,2)
    REQUIRE(n.count == 1);
    REQUIRE(n.data[0].x == 3);
    REQUIRE(n.data[0].y == 2);
}

TEST_CASE("No direction constraint allows all movement", "[oneway]") {
    pathsim::Grid grid(5, 5);

    auto n = grid.neighbors({.x = 2, .y = 2});
    REQUIRE(n.count == 4);
}

TEST_CASE("Set direction on start and end is allowed", "[oneway]") {
    pathsim::Grid grid(5, 5);

    grid.set_direction(grid.start(), pathsim::CellDirection::East);
    grid.set_direction(grid.end(), pathsim::CellDirection::West);

    REQUIRE(grid.direction(grid.start()) == pathsim::CellDirection::East);
    REQUIRE(grid.direction(grid.end()) == pathsim::CellDirection::West);
}

TEST_CASE("One-way cell forces detour", "[oneway]") {
    pathsim::Grid grid(3, 3);

    // Center cell is one-way east and blocks north/south passage through it
    grid.set_direction({.x = 1, .y = 1}, pathsim::CellDirection::East);

    // Start top-center & end bottom-center
    grid.set_start({.x = 1, .y = 0});
    grid.set_end({.x = 1, .y = 2});

    auto result = pathsim::bfs(grid);

    // Can't go straight down through (1,1) has to detour around
    REQUIRE_FALSE(result.path.empty());
    REQUIRE(result.path.size() > 3);
}

TEST_CASE("Setting wall clears direction", "[oneway]") {
    pathsim::Grid grid(5, 5);
    pathsim::Vec2i pos{.x = 2, .y = 2};

    grid.set_direction(pos, pathsim::CellDirection::East);
    grid.set_wall(pos, true);

    REQUIRE(grid.direction(pos) == pathsim::CellDirection::None);
}

TEST_CASE("Clear grid resets all directions", "[oneway]") {
    pathsim::Grid grid(5, 5);

    grid.set_direction({.x = 1, .y = 1}, pathsim::CellDirection::North);
    grid.set_direction({.x = 2, .y = 2}, pathsim::CellDirection::South);
    grid.clear();

    REQUIRE(grid.direction({.x = 1, .y = 1}) == pathsim::CellDirection::None);
    REQUIRE(grid.direction({.x = 2, .y = 2}) == pathsim::CellDirection::None);
}