#pragma once

#include <optional>
#include <vector>

#include "mapping/occupancy_grid.hpp"

namespace toy_rover::planning
{

  using Path = std::vector<mapping::GridIndex>;

  class AStar
  {
  public:
    [[nodiscard]] std::optional<Path> plan(
        const mapping::OccupancyGrid &grid,
        mapping::GridIndex start,
        mapping::GridIndex goal) const;

  private:
    [[nodiscard]] bool traversable(const mapping::OccupancyGrid &grid, mapping::GridIndex index) const;
  };
} // namespace toy_rover::planning
