#pragma once

#include "mapping/occupancy_grid.hpp"

namespace toy_rover::mapping
{

  // Expands occupied cells by a circular robot clearance radius. Unknown and
  // free cells inside that radius become occupied in the planning grid.
  void inflate_occupied_cells(OccupancyGrid &grid, double inflation_radius_m);

} // namespace toy_rover::mapping
