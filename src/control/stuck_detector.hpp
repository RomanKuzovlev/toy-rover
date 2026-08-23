#pragma once

#include <chrono>
#include <deque>
#include <optional>

#include "mapping/occupancy_grid.hpp"

namespace toy_rover::control
{

  class StuckDetector
  {
  public:
    StuckDetector(std::chrono::nanoseconds detection_window, int cell_tolerance);

    // Returns true once per stationary episode. The detector re-arms after the
    // rover moves outside the configured cell tolerance.
    [[nodiscard]] bool update(
        std::chrono::nanoseconds timestamp,
        mapping::GridIndex cell);

  private:
    struct PositionSample
    {
      std::chrono::nanoseconds timestamp;
      mapping::GridIndex cell;
    };

    [[nodiscard]] bool within_tolerance(
        mapping::GridIndex lhs,
        mapping::GridIndex rhs) const noexcept;
    void reset_history();

    std::chrono::nanoseconds detection_window_;
    int cell_tolerance_;
    std::deque<PositionSample> position_history_;
    std::optional<mapping::GridIndex> latched_cell_;
    std::optional<std::chrono::nanoseconds> last_timestamp_;
  };

} // namespace toy_rover::control
