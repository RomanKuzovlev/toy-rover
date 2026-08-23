#include "mapping/obstacle_inflation.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace toy_rover::mapping
{

  void inflate_occupied_cells(OccupancyGrid &grid, double inflation_radius_m)
  {
    if (inflation_radius_m < 0.0)
    {
      throw std::invalid_argument("obstacle inflation radius must be non-negative");
    }
    if (inflation_radius_m == 0.0)
    {
      return;
    }

    std::vector<GridIndex> occupied_cells;
    for (int y = 0; y < grid.height(); ++y)
    {
      for (int x = 0; x < grid.width(); ++x)
      {
        const GridIndex cell{x, y};
        if (grid.at(cell) == Cell::Occupied)
        {
          occupied_cells.push_back(cell);
        }
      }
    }

    const double resolution_m = grid.resolution_m();
    const int cell_radius = static_cast<int>(std::ceil(inflation_radius_m / resolution_m));
    const double squared_radius = inflation_radius_m * inflation_radius_m;

    for (const auto occupied : occupied_cells)
    {
      for (int dy = -cell_radius; dy <= cell_radius; ++dy)
      {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx)
        {
          const double offset_x_m = static_cast<double>(dx) * resolution_m;
          const double offset_y_m = static_cast<double>(dy) * resolution_m;
          if (offset_x_m * offset_x_m + offset_y_m * offset_y_m > squared_radius)
          {
            continue;
          }

          const GridIndex inflated{occupied.x + dx, occupied.y + dy};
          if (grid.in_bounds(inflated))
          {
            grid.set(inflated, Cell::Occupied);
          }
        }
      }
    }
  }

} // namespace toy_rover::mapping
