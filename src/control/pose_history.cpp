#include "control/pose_history.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace toy_rover::control
{

  PoseHistory::PoseHistory(std::chrono::nanoseconds retention)
      : retention_(retention)
  {
    if (retention_ <= std::chrono::nanoseconds::zero())
    {
      throw std::invalid_argument("pose history retention must be positive");
    }
  }

  void PoseHistory::add(std::chrono::nanoseconds timestamp, Pose2D pose)
  {
    if (!poses_.empty() && timestamp < poses_.back().timestamp)
    {
      poses_.clear();
    }

    if (!poses_.empty() && timestamp == poses_.back().timestamp)
    {
      poses_.back().pose = pose;
      return;
    }

    poses_.push_back(TimedPose{timestamp, pose});
    while (poses_.size() > 1 &&
           poses_.back().timestamp - poses_.front().timestamp > retention_)
    {
      poses_.pop_front();
    }
  }

  std::optional<Pose2D> PoseHistory::interpolate(
      std::chrono::nanoseconds timestamp) const
  {
    if (poses_.empty() || timestamp < poses_.front().timestamp ||
        timestamp > poses_.back().timestamp)
    {
      return std::nullopt;
    }

    const auto upper = std::lower_bound(
        poses_.begin(),
        poses_.end(),
        timestamp,
        [](const TimedPose &sample, std::chrono::nanoseconds requested)
        {
          return sample.timestamp < requested;
        });

    if (upper == poses_.begin() || upper->timestamp == timestamp)
    {
      return upper->pose;
    }

    const auto lower = std::prev(upper);
    const double interval_ns =
        static_cast<double>((upper->timestamp - lower->timestamp).count());
    const double alpha =
        static_cast<double>((timestamp - lower->timestamp).count()) / interval_ns;
    const double yaw_delta = std::atan2(
        std::sin(upper->pose.yaw_rad - lower->pose.yaw_rad),
        std::cos(upper->pose.yaw_rad - lower->pose.yaw_rad));

    return Pose2D{
        lower->pose.x + alpha * (upper->pose.x - lower->pose.x),
        lower->pose.y + alpha * (upper->pose.y - lower->pose.y),
        std::atan2(
            std::sin(lower->pose.yaw_rad + alpha * yaw_delta),
            std::cos(lower->pose.yaw_rad + alpha * yaw_delta)),
    };
  }

  std::optional<std::chrono::nanoseconds> PoseHistory::latest_timestamp() const noexcept
  {
    if (poses_.empty())
    {
      return std::nullopt;
    }
    return poses_.back().timestamp;
  }

} // namespace toy_rover::control
