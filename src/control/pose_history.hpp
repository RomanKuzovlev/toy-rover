#pragma once

#include <chrono>
#include <deque>
#include <optional>

#include "control/pure_pursuit.hpp"

namespace toy_rover::control
{

  class PoseHistory
  {
  public:
    explicit PoseHistory(std::chrono::nanoseconds retention);

    void add(std::chrono::nanoseconds timestamp, Pose2D pose);

    [[nodiscard]] std::optional<Pose2D> interpolate(
        std::chrono::nanoseconds timestamp) const;
    [[nodiscard]] std::optional<std::chrono::nanoseconds> latest_timestamp() const noexcept;

  private:
    struct TimedPose
    {
      std::chrono::nanoseconds timestamp;
      Pose2D pose;
    };

    std::chrono::nanoseconds retention_;
    std::deque<TimedPose> poses_;
  };

} // namespace toy_rover::control
