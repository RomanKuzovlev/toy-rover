#include "control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace toy_rover::control
{

  PurePursuit::PurePursuit(double lookahead_m, double nominal_speed_mps)
      : lookahead_m_(lookahead_m), nominal_speed_mps_(nominal_speed_mps)
  {
    if (lookahead_m_ <= 0.0)
    {
      throw std::invalid_argument("lookahead_m must be positive");
    }
  }

  Twist2D PurePursuit::compute_command(const Pose2D &pose, const std::vector<Point2D> &path) const
  {
    if (path.empty())
    {
      return {};
    }

    const double goal_dx = path.back().x - pose.x;
    const double goal_dy = path.back().y - pose.y;
    constexpr double goal_tolerance_m = 0.15;
    if (std::hypot(goal_dx, goal_dy) <= goal_tolerance_m)
    {
      return {};
    }

    // The planner has already simplified the path into collision-checked
    // visible segments. Hold the immediate next waypoint instead of skipping
    // over a nearby corner: skipping rounds the turn through inflated cells.
    const Point2D target = path.size() > 1 ? path[1] : path.back();

    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double target_angle = std::atan2(dy, dx);
    const double heading_error = std::atan2(
        std::sin(target_angle - pose.yaw_rad),
        std::cos(target_angle - pose.yaw_rad));

    // Do not drive a forward arc when the path is substantially behind the
    // rover. Turn first, then advance with speed reduced by heading error.
    constexpr double rotate_in_place_threshold_rad = 0.8;
    const double waypoint_speed_scale =
        std::min(1.0, std::hypot(dx, dy) / lookahead_m_);
    const double linear_speed =
        std::abs(heading_error) >= rotate_in_place_threshold_rad
            ? 0.0
            : nominal_speed_mps_ * std::max(0.25, std::cos(heading_error)) *
                  waypoint_speed_scale;

    return {linear_speed, 2.0 * heading_error};
  }

} // namespace toy_rover::control
