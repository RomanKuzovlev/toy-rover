#include "planning/path_simplifier.hpp"

#include <cstdlib>

namespace toy_rover::planning
{
  namespace
  {
    bool traversable(
        const mapping::OccupancyGrid &grid,
        mapping::GridIndex cell)
    {
      return grid.in_bounds(cell) && grid.at(cell) != mapping::Cell::Occupied;
    }
  } // namespace

  bool has_line_of_sight(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex from,
      mapping::GridIndex to)
  {
    if (!grid.in_bounds(from) || !grid.in_bounds(to))
    {
      return false;
    }

    int x = from.x;
    int y = from.y;
    const int dx = std::abs(to.x - from.x);
    const int dy = std::abs(to.y - from.y);
    const int step_x = to.x > from.x ? 1 : (to.x < from.x ? -1 : 0);
    const int step_y = to.y > from.y ? 1 : (to.y < from.y ? -1 : 0);
    int crossed_x = 0;
    int crossed_y = 0;

    while (crossed_x < dx || crossed_y < dy)
    {
      const int x_crossing = (1 + 2 * crossed_x) * dy;
      const int y_crossing = (1 + 2 * crossed_y) * dx;

      if (x_crossing == y_crossing)
      {
        // The segment crosses a cell corner. Both side cells must be free so
        // smoothing cannot squeeze diagonally between touching obstacles.
        const mapping::GridIndex horizontal{x + step_x, y};
        const mapping::GridIndex vertical{x, y + step_y};
        const mapping::GridIndex diagonal{x + step_x, y + step_y};
        if (!traversable(grid, horizontal) ||
            !traversable(grid, vertical) ||
            !traversable(grid, diagonal))
        {
          return false;
        }
        x += step_x;
        y += step_y;
        ++crossed_x;
        ++crossed_y;
      }
      else if (x_crossing < y_crossing)
      {
        x += step_x;
        ++crossed_x;
        if (!traversable(grid, {x, y}))
        {
          return false;
        }
      }
      else
      {
        y += step_y;
        ++crossed_y;
        if (!traversable(grid, {x, y}))
        {
          return false;
        }
      }
    }

    return true;
  }

  Path simplify_path(const mapping::OccupancyGrid &grid, const Path &path)
  {
    if (path.size() <= 2)
    {
      return path;
    }

    Path simplified;
    simplified.reserve(path.size());
    std::size_t anchor = 0;
    simplified.push_back(path.front());

    while (anchor + 1 < path.size())
    {
      std::size_t next = path.size() - 1;
      while (next > anchor + 1 && !has_line_of_sight(grid, path[anchor], path[next]))
      {
        --next;
      }
      simplified.push_back(path[next]);
      anchor = next;
    }

    return simplified;
  }
} // namespace toy_rover::planning
