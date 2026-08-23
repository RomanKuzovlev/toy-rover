#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace toy_rover::control
{
  // Chooses the center heading of the widest-clear lidar corridor. Infinite
  // readings are treated as fully open up to range_max.
  [[nodiscard]] std::optional<double> choose_free_space_heading(
      const std::vector<float> &ranges,
      double angle_min_rad,
      double angle_increment_rad,
      double range_min_m,
      double range_max_m,
      double corridor_half_angle_rad = 0.25,
      double turn_penalty_m_per_rad = 0.05);

  [[nodiscard]] double forward_clearance(
      const std::vector<float> &ranges,
      double angle_min_rad,
      double angle_increment_rad,
      double range_min_m,
      double range_max_m,
      double corridor_half_angle_rad = 0.25);
} // namespace toy_rover::control
