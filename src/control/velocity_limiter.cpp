#include "control/velocity_limiter.hpp"

#include <algorithm>
#include <stdexcept>

namespace toy_rover::control
{

  VelocityLimiter::VelocityLimiter(double max_linear_mps, double max_angular_radps)
      : max_linear_mps_(max_linear_mps), max_angular_radps_(max_angular_radps)
  {
    if (max_linear_mps_ < 0.0 || max_angular_radps_ < 0.0)
    {
      throw std::invalid_argument("velocity limits must be non-negative");
    }
  }

  Twist2D VelocityLimiter::limit(Twist2D command) const
  {
    command.linear_mps = std::clamp(command.linear_mps, -max_linear_mps_, max_linear_mps_);
    command.angular_radps = std::clamp(command.angular_radps, -max_angular_radps_, max_angular_radps_);
    return command;
  }

} // namespace toy_rover::control
