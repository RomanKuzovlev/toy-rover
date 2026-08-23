#include "control/free_space_escape.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace toy_rover::control
{
  namespace
  {
    double usable_range(float range, double range_min_m, double range_max_m)
    {
      if (std::isinf(range))
      {
        return range_max_m;
      }
      if (!std::isfinite(range) || range < range_min_m)
      {
        return 0.0;
      }
      return std::min(static_cast<double>(range), range_max_m);
    }

    double corridor_clearance(
        const std::vector<float> &ranges,
        std::size_t center,
        std::size_t half_window,
        double range_min_m,
        double range_max_m)
    {
      double clearance = range_max_m;
      const std::size_t count = ranges.size();
      for (std::size_t offset = 0; offset <= 2 * half_window; ++offset)
      {
        const std::size_t wrapped =
            (center + count + offset - half_window) % count;
        clearance = std::min(
            clearance, usable_range(ranges[wrapped], range_min_m, range_max_m));
      }
      return clearance;
    }
  } // namespace

  std::optional<double> choose_free_space_heading(
      const std::vector<float> &ranges,
      double angle_min_rad,
      double angle_increment_rad,
      double range_min_m,
      double range_max_m,
      double corridor_half_angle_rad,
      double turn_penalty_m_per_rad)
  {
    if (ranges.empty() || angle_increment_rad <= 0.0 || range_max_m <= range_min_m)
    {
      return std::nullopt;
    }

    const auto half_window = static_cast<std::size_t>(std::ceil(
        corridor_half_angle_rad / angle_increment_rad));
    double best_score = -std::numeric_limits<double>::infinity();
    std::optional<double> best_heading;

    for (std::size_t i = 0; i < ranges.size(); ++i)
    {
      const double heading = angle_min_rad + static_cast<double>(i) * angle_increment_rad;
      const double wrapped_heading = std::atan2(std::sin(heading), std::cos(heading));
      const double clearance = corridor_clearance(
          ranges, i, half_window, range_min_m, range_max_m);
      const double score = clearance - turn_penalty_m_per_rad * std::abs(wrapped_heading);
      if (score > best_score)
      {
        best_score = score;
        best_heading = wrapped_heading;
      }
    }

    return best_heading;
  }

  double forward_clearance(
      const std::vector<float> &ranges,
      double angle_min_rad,
      double angle_increment_rad,
      double range_min_m,
      double range_max_m,
      double corridor_half_angle_rad)
  {
    if (ranges.empty() || angle_increment_rad <= 0.0)
    {
      return 0.0;
    }
    const auto center = static_cast<std::size_t>(std::clamp(
        std::llround(-angle_min_rad / angle_increment_rad),
        0LL,
        static_cast<long long>(ranges.size() - 1)));
    const auto half_window = static_cast<std::size_t>(std::ceil(
        corridor_half_angle_rad / angle_increment_rad));
    return corridor_clearance(
        ranges, center, half_window, range_min_m, range_max_m);
  }
} // namespace toy_rover::control
