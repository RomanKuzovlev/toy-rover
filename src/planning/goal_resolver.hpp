#pragma once

#include <optional>
#include <vector>

#include "mapping/occupancy_grid.hpp"
#include "planning/astar.hpp"

namespace toy_rover::planning
{
  [[nodiscard]] std::optional<mapping::GridIndex> find_nearest_free_cell(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex requested,
      double max_distance_m);

  [[nodiscard]] std::vector<mapping::GridIndex> find_free_cells_nearest_first(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex requested,
      double max_distance_m);

  struct ResolvedGoal
  {
    mapping::GridIndex cell;
    Path path;
  };

  [[nodiscard]] std::optional<ResolvedGoal> resolve_reachable_goal(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex start,
      mapping::GridIndex requested,
      double max_distance_m);
} // namespace toy_rover::planning
