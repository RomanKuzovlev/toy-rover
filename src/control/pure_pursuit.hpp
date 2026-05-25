#pragma once

#include <vector>

namespace toy_rover::control
{

  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw_rad{0.0};
  };

  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  struct Twist2D
  {
    double linear_mps{0.0};
    double angular_radps{0.0};
  };

  class PurePursuit
  {
  public:
    PurePursuit(double lookahead_m, double nominal_speed_mps);

    [[nodiscard]] Twist2D compute_command(const Pose2D &pose, const std::vector<Point2D> &path) const;

  private:
    double lookahead_m_;
    double nominal_speed_mps_;
  };

} // namespace toy_rover::control
