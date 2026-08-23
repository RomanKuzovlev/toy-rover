#pragma once

#include "mapping/occupancy_grid.hpp"
#include "planning/astar.hpp"

namespace toy_rover::planning
{
  [[nodiscard]] bool has_line_of_sight(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex from,
      mapping::GridIndex to);

  [[nodiscard]] Path simplify_path(
      const mapping::OccupancyGrid &grid,
      const Path &path);
} // namespace toy_rover::planning
