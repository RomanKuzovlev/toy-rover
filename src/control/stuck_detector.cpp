#include "control/stuck_detector.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace toy_rover::control
{

  StuckDetector::StuckDetector(
      std::chrono::nanoseconds detection_window,
      int cell_tolerance)
      : detection_window_(detection_window),
        cell_tolerance_(cell_tolerance)
  {
    if (detection_window_ <= std::chrono::nanoseconds::zero())
    {
      throw std::invalid_argument("stuck detection window must be positive");
    }
    if (cell_tolerance_ < 0)
    {
      throw std::invalid_argument("stuck cell tolerance must be non-negative");
    }
  }

  bool StuckDetector::update(
      std::chrono::nanoseconds timestamp,
      mapping::GridIndex cell)
  {
    if (last_timestamp_ && timestamp < *last_timestamp_)
    {
      reset_history();
      latched_cell_.reset();
    }
    last_timestamp_ = timestamp;

    if (latched_cell_)
    {
      if (within_tolerance(cell, *latched_cell_))
      {
        return false;
      }

      // Movement outside the stationary area begins a fresh detection window.
      reset_history();
      latched_cell_.reset();
    }

    position_history_.push_back(PositionSample{timestamp, cell});

    // Preserve the newest sample at or before the start of the time window so
    // small odometry timing jitter cannot prevent the window from filling.
    while (position_history_.size() > 1 &&
           timestamp - position_history_[1].timestamp >= detection_window_)
    {
      position_history_.pop_front();
    }

    if (timestamp - position_history_.front().timestamp < detection_window_)
    {
      return false;
    }

    const bool stayed_in_area = std::all_of(
        position_history_.begin(),
        position_history_.end(),
        [this, cell](const PositionSample &sample)
        {
          return within_tolerance(sample.cell, cell);
        });

    if (!stayed_in_area)
    {
      return false;
    }

    latched_cell_ = cell;
    reset_history();
    return true;
  }

  bool StuckDetector::within_tolerance(
      mapping::GridIndex lhs,
      mapping::GridIndex rhs) const noexcept
  {
    return std::abs(lhs.x - rhs.x) <= cell_tolerance_ &&
           std::abs(lhs.y - rhs.y) <= cell_tolerance_;
  }

  void StuckDetector::reset_history()
  {
    position_history_.clear();
  }

} // namespace toy_rover::control
