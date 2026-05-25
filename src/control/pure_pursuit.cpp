#include "control/pure_pursuit.hpp"

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

    Point2D target = path.back();
    for (const auto &point : path)
    {
      const double dx = point.x - pose.x;
      const double dy = point.y - pose.y;
      if (std::hypot(dx, dy) >= lookahead_m_)
      {
        target = point;
        break;
      }
    }

    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    const double target_angle = std::atan2(dy, dx);
    const double heading_error = std::atan2(
        std::sin(target_angle - pose.yaw_rad),
        std::cos(target_angle - pose.yaw_rad));

    return {nominal_speed_mps_, 2.0 * heading_error};
  }

} // namespace toy_rover::control
