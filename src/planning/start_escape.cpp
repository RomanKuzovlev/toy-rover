#include "planning/start_escape.hpp"

#include <algorithm>
#include <array>
#include <queue>
#include <vector>

namespace toy_rover::planning
{
  std::optional<Path> find_path_out_of_inflation(
      const mapping::OccupancyGrid &recovery_grid,
      const mapping::OccupancyGrid &navigation_grid,
      mapping::GridIndex start)
  {
    if (recovery_grid.width() != navigation_grid.width() ||
        recovery_grid.height() != navigation_grid.height() ||
        !recovery_grid.in_bounds(start))
    {
      return std::nullopt;
    }
    if (navigation_grid.at(start) != mapping::Cell::Occupied)
    {
      return Path{start};
    }

    constexpr std::array<mapping::GridIndex, 8> offsets{{
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    }};
    constexpr int unvisited = -2;
    const int start_key = static_cast<int>(recovery_grid.linear_index(start));
    std::vector<int> parent(recovery_grid.cells().size(), unvisited);
    std::queue<mapping::GridIndex> open;
    parent[static_cast<std::size_t>(start_key)] = -1;
    open.push(start);

    while (!open.empty())
    {
      const auto current = open.front();
      open.pop();

      if (!(current == start) &&
          navigation_grid.at(current) != mapping::Cell::Occupied)
      {
        Path path;
        int cursor = static_cast<int>(recovery_grid.linear_index(current));
        while (cursor >= 0)
        {
          path.push_back({
              cursor % recovery_grid.width(),
              cursor / recovery_grid.width(),
          });
          cursor = parent[static_cast<std::size_t>(cursor)];
        }
        std::reverse(path.begin(), path.end());
        return path;
      }

      for (const auto offset : offsets)
      {
        const mapping::GridIndex next{current.x + offset.x, current.y + offset.y};
        if (!recovery_grid.in_bounds(next) ||
            recovery_grid.at(next) == mapping::Cell::Occupied)
        {
          continue;
        }

        const bool diagonal = offset.x != 0 && offset.y != 0;
        if (diagonal)
        {
          const mapping::GridIndex horizontal{current.x + offset.x, current.y};
          const mapping::GridIndex vertical{current.x, current.y + offset.y};
          if (!recovery_grid.in_bounds(horizontal) ||
              !recovery_grid.in_bounds(vertical) ||
              recovery_grid.at(horizontal) == mapping::Cell::Occupied ||
              recovery_grid.at(vertical) == mapping::Cell::Occupied)
          {
            continue;
          }
        }

        const auto next_key = recovery_grid.linear_index(next);
        if (parent[next_key] != unvisited)
        {
          continue;
        }
        parent[next_key] = static_cast<int>(recovery_grid.linear_index(current));
        open.push(next);
      }
    }

    return std::nullopt;
  }
} // namespace toy_rover::planning
