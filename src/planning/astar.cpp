#include "planning/astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>

namespace toy_rover::planning
{

  namespace
  {
    constexpr double diagonal_cost = 1.4142135623730951;

    struct Node
    {
      mapping::GridIndex index;
      double f_score{0.0};
    };

    struct NodeGreater
    {
      bool operator()(const Node &lhs, const Node &rhs) const
      {
        return lhs.f_score > rhs.f_score;
      }
    };

    int key(const mapping::OccupancyGrid &grid, mapping::GridIndex index)
    {
      return index.y * grid.width() + index.x;
    }

    struct Neighbor
    {
      mapping::GridIndex index;
      double cost;
      bool diagonal;
    };

    double heuristic(mapping::GridIndex a, mapping::GridIndex b)
    {
      const double dx = std::abs(a.x - b.x);
      const double dy = std::abs(a.y - b.y);
      return std::max(dx, dy) + (diagonal_cost - 1.0) * std::min(dx, dy);
    }

    std::vector<Neighbor> neighbors(mapping::GridIndex index)
    {
      return {
          {{index.x + 1, index.y}, 1.0, false},
          {{index.x - 1, index.y}, 1.0, false},
          {{index.x, index.y + 1}, 1.0, false},
          {{index.x, index.y - 1}, 1.0, false},
          {{index.x + 1, index.y + 1}, diagonal_cost, true},
          {{index.x + 1, index.y - 1}, diagonal_cost, true},
          {{index.x - 1, index.y + 1}, diagonal_cost, true},
          {{index.x - 1, index.y - 1}, diagonal_cost, true},
      };
    }
  } // namespace

  std::optional<Path> AStar::plan(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex start,
      mapping::GridIndex goal) const
  {
    if (!grid.in_bounds(start) || !grid.in_bounds(goal) ||
        grid.at(goal) == mapping::Cell::Occupied)
    {
      return std::nullopt;
    }

    // The robot can already be inside the conservative inflation halo because
    // of grid quantization or tracking error. Seed the search there so it can
    // move outward, but keep all neighboring occupied cells non-traversable.
    // This does not permit a route to enter or cross inflated obstacles.

    std::priority_queue<Node, std::vector<Node>, NodeGreater> open;
    std::unordered_map<int, mapping::GridIndex> came_from;
    std::unordered_map<int, double> g_score;

    const int start_key = key(grid, start);

    g_score[start_key] = 0.0;
    open.push({start, heuristic(start, goal)});

    while (!open.empty())
    {
      const auto current = open.top().index;
      open.pop();

      if (current == goal)
      {
        Path path;
        auto cursor = goal;
        path.push_back(cursor);

        while (!(cursor == start))
        {
          cursor = came_from.at(key(grid, cursor));
          path.push_back(cursor);
        }

        std::reverse(path.begin(), path.end());
        return path;
      }

      const double current_g = g_score.at(key(grid, current));
      for (const auto &neighbor : neighbors(current))
      {
        const auto next = neighbor.index;
        if (!traversable(grid, next))
        {
          continue;
        }

        if (neighbor.diagonal)
        {
          const mapping::GridIndex horizontal{next.x, current.y};
          const mapping::GridIndex vertical{current.x, next.y};
          if (!traversable(grid, horizontal) || !traversable(grid, vertical))
          {
            continue;
          }
        }

        const double tentative_g = current_g + neighbor.cost;
        const int next_key = key(grid, next);
        const auto known = g_score.find(next_key);
        if (known != g_score.end() && tentative_g >= known->second)
        {
          continue;
        }

        came_from[next_key] = current;
        g_score[next_key] = tentative_g;
        open.push({next, tentative_g + heuristic(next, goal)});
      }
    }

    return std::nullopt;
  }

  bool AStar::traversable(const mapping::OccupancyGrid &grid, mapping::GridIndex index) const
  {
    return grid.in_bounds(index) && grid.at(index) != mapping::Cell::Occupied;
  }

} // namespace toy_rover::planning
