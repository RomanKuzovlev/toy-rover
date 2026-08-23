#pragma once

#include <optional>

#include "mapping/occupancy_grid.hpp"
#include "planning/astar.hpp"

namespace toy_rover::planning
{
  // Finds the shortest route from an already-occupied navigation cell to a
  // normally free cell. Movement is constrained by the tighter recovery grid.
  [[nodiscard]] std::optional<Path> find_path_out_of_inflation(
      const mapping::OccupancyGrid &recovery_grid,
      const mapping::OccupancyGrid &navigation_grid,
      mapping::GridIndex start);
} // namespace toy_rover::planning
