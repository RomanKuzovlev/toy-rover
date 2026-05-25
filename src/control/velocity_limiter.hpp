#pragma once

#include "control/pure_pursuit.hpp"

namespace toy_rover::control
{

  class VelocityLimiter
  {
  public:
    VelocityLimiter(double max_linear_mps, double max_angular_radps);

    [[nodiscard]] Twist2D limit(Twist2D command) const;

  private:
    double max_linear_mps_;
    double max_angular_radps_;
  };

} // namespace toy_rover::control
