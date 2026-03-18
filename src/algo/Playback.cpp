#include "Playback.hpp"

#include "core/Types.hpp"

#include <algorithm>
#include <utility>

namespace pathsim {

void Playback::start(PathResult result, Grid& grid) {
    grid.reset_path_state();
    result_ = std::move(result);
    step_index_ = 0;
    state_ = PlaybackState::Playing;
}

void Playback::update(Grid& grid) {
    if (state_ != PlaybackState::Playing) {
        return;
    }

    int steps_remaining = steps_per_frame_;
    int total = static_cast<int>(result_.steps.size());

    while (steps_remaining > 0 && step_index_ < total) {
        const auto& step = result_.steps[static_cast<std::size_t>(step_index_)];
        grid.set(step.position, step.new_state);
        ++step_index_;
        --steps_remaining;
    }

    if (step_index_ >= total) {
        // Clear exploration cells and keep only the path
        for (const auto& step : result_.steps) {
            if (step.new_state == CellState::Visited || step.new_state == CellState::Frontier) {
                if (grid.at(step.position) != CellState::Path) {
                    grid.set(step.position, CellState::Empty);
                }
            }
        }

        // Restore waypoints that may have been overwritten during exploration
        for (const auto& wp : grid.waypoints()) {
            if (grid.at(wp) != CellState::Path) {
                grid.set(wp, CellState::Waypoint);
            }
        }
        // Restore start and end
        grid.set(grid.start(), CellState::Start);
        grid.set(grid.end(), CellState::End);

        state_ = PlaybackState::Finished;
        state_ = PlaybackState::Finished;
    }
}

void Playback::pause() {
    if (state_ == PlaybackState::Playing) {
        state_ = PlaybackState::Paused;
    }
}

void Playback::resume() {
    if (state_ == PlaybackState::Paused) {
        state_ = PlaybackState::Playing;
    }
}

void Playback::reset(Grid& grid) {
    grid.reset_path_state();
    result_ = {};
    step_index_ = 0;
    state_ = PlaybackState::Idle;
}

void Playback::set_speed(int steps_per_frame) {
    steps_per_frame_ = std::max(1, steps_per_frame);
}

int Playback::speed() const {
    return steps_per_frame_;
}

PlaybackState Playback::state() const {
    return state_;
}

int Playback::current_step() const {
    return step_index_;
}

int Playback::total_steps() const {
    return static_cast<int>(result_.steps.size());
}

const PathResult& Playback::result() const {
    return result_;
}

} // namespace pathsim