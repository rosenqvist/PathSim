#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Adding a waypoint sets cell state", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i wp{.x = 5, .y = 5};

    grid.add_waypoint(wp);

    REQUIRE(grid.at(wp) == pathsim::CellState::Waypoint);
    REQUIRE(grid.waypoints().size() == 1);
    REQUIRE(grid.waypoints()[0] == wp);
}

TEST_CASE("Cannot add waypoint on start or end", "[waypoint]") {
    pathsim::Grid grid(10, 10);

    grid.add_waypoint(grid.start());
    grid.add_waypoint(grid.end());

    REQUIRE(grid.waypoints().empty());
}

TEST_CASE("Cannot add waypoint on wall", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i pos{.x = 3, .y = 3};
    grid.set_wall(pos, true);

    grid.add_waypoint(pos);

    REQUIRE(grid.waypoints().empty());
}

TEST_CASE("Cannot add duplicate waypoint", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i wp{.x = 5, .y = 5};

    grid.add_waypoint(wp);
    grid.add_waypoint(wp);

    REQUIRE(grid.waypoints().size() == 1);
}

TEST_CASE("Remove waypoint restores cell to empty", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i wp{.x = 5, .y = 5};

    grid.add_waypoint(wp);
    grid.remove_waypoint(wp);

    REQUIRE(grid.at(wp) == pathsim::CellState::Empty);
    REQUIRE(grid.waypoints().empty());
}

TEST_CASE("Clear grid removes all waypoints", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    grid.add_waypoint({.x = 2, .y = 2});
    grid.add_waypoint({.x = 5, .y = 5});
    grid.add_waypoint({.x = 7, .y = 7});

    grid.clear();

    REQUIRE(grid.waypoints().empty());
}

TEST_CASE("Reset path state preserves waypoints", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i wp{.x = 5, .y = 5};
    grid.add_waypoint(wp);

    // Simulate algo overwriting the waypoint cell
    grid.set(wp, pathsim::CellState::Visited);
    grid.reset_path_state();

    REQUIRE(grid.at(wp) == pathsim::CellState::Waypoint);
    REQUIRE(grid.waypoints().size() == 1);
}

TEST_CASE("Waypoints maintain insertion order", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i wp1{.x = 2, .y = 2};
    pathsim::Vec2i wp2{.x = 7, .y = 7};
    pathsim::Vec2i wp3{.x = 4, .y = 4};

    grid.add_waypoint(wp1);
    grid.add_waypoint(wp2);
    grid.add_waypoint(wp3);

    REQUIRE(grid.waypoints()[0] == wp1);
    REQUIRE(grid.waypoints()[1] == wp2);
    REQUIRE(grid.waypoints()[2] == wp3);
}

TEST_CASE("Moving start onto waypoint removes it", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i wp{.x = 3, .y = 3};

    grid.add_waypoint(wp);
    REQUIRE(grid.waypoints().size() == 1);

    grid.set_start(wp);

    REQUIRE(grid.waypoints().empty());
    REQUIRE(grid.at(wp) == pathsim::CellState::Start);
}

TEST_CASE("Moving end onto waypoint removes it", "[waypoint]") {
    pathsim::Grid grid(10, 10);
    pathsim::Vec2i wp{.x = 5, .y = 5};

    grid.add_waypoint(wp);
    REQUIRE(grid.waypoints().size() == 1);

    grid.set_end(wp);

    REQUIRE(grid.waypoints().empty());
    REQUIRE(grid.at(wp) == pathsim::CellState::End);
}