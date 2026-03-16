#include "algo/BFS.hpp"
#include "algo/Playback.hpp"
#include "core/Grid.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("Playback starts in Idle state", "[playback]") {
    pathsim::Playback playback;

    REQUIRE(playback.state() == pathsim::PlaybackState::Idle);
}

TEST_CASE("Playback transitions to Playing on start", "[playback]") {
    pathsim::Grid grid(5, 5);
    pathsim::Playback playback;

    playback.start(pathsim::bfs(grid), grid);

    REQUIRE(playback.state() == pathsim::PlaybackState::Playing);
    REQUIRE(playback.current_step() == 0);
    REQUIRE(playback.total_steps() > 0);
}

TEST_CASE("Playback reaches Finished after enough updates", "[playback]") {
    pathsim::Grid grid(5, 5);
    pathsim::Playback playback;

    playback.start(pathsim::bfs(grid), grid);
    playback.set_speed(playback.total_steps()); // process everything in one frame

    playback.update(grid);

    REQUIRE(playback.state() == pathsim::PlaybackState::Finished);
    REQUIRE(playback.current_step() == playback.total_steps());
}

TEST_CASE("Playback pause and resume", "[playback]") {
    pathsim::Grid grid(5, 5);
    pathsim::Playback playback;

    playback.start(pathsim::bfs(grid), grid);

    playback.pause();
    REQUIRE(playback.state() == pathsim::PlaybackState::Paused);

    int step_before = playback.current_step();
    playback.update(grid); // should not advance while paused
    REQUIRE(playback.current_step() == step_before);

    playback.resume();
    REQUIRE(playback.state() == pathsim::PlaybackState::Playing);
}

TEST_CASE("Playback reset returns to Idle", "[playback]") {
    pathsim::Grid grid(5, 5);
    pathsim::Playback playback;

    playback.start(pathsim::bfs(grid), grid);
    playback.update(grid);
    playback.reset(grid);

    REQUIRE(playback.state() == pathsim::PlaybackState::Idle);
    REQUIRE(playback.current_step() == 0);
}

TEST_CASE("Playback applies steps to the grid", "[playback]") {
    pathsim::Grid grid(5, 5);
    pathsim::Playback playback;

    playback.start(pathsim::bfs(grid), grid);
    playback.set_speed(1);
    playback.update(grid);

    // After one step, at least one cell should no longer be Empty
    bool found_changed = false;
    for (int row = 0; row < grid.height(); ++row) {
        for (int col = 0; col < grid.width(); ++col) {
            pathsim::CellState state = grid.at({.x = col, .y = row});
            if (state == pathsim::CellState::Visited || state == pathsim::CellState::Frontier) {
                found_changed = true;
            }
        }
    }
    REQUIRE(found_changed);
}