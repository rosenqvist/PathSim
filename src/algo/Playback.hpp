#pragma once

#include "../core/Grid.hpp"
#include "../core/Types.hpp"

#include <cstdint>

namespace pathsim {

enum class PlaybackState : std::uint8_t { Idle, Playing, Paused, Finished };

class Playback {
  public:
    // load a result and begin playback, clear any previous path state on the grid
    void start(PathResult result, Grid& grid);

    // called every frame while playing, apply the next batch of steps to the grid
    void update(Grid& grid);

    void pause();
    void resume();

    // stop playback and clear path state from the grid
    void reset(Grid& grid);

    void set_speed(int steps_per_frame);
    [[nodiscard]] int speed() const;

    [[nodiscard]] PlaybackState state() const;

    // how far through the recording we are (progress indicator)
    [[nodiscard]] int current_step() const;
    [[nodiscard]] int total_steps() const;

    // access the final result (display stats after playback)
    [[nodiscard]] const PathResult& result() const;

  private:
    PathResult result_{};
    PlaybackState state_ = PlaybackState::Idle;
    int step_index_{};
    int steps_per_frame_ = 1;
};

} // namespace pathsim