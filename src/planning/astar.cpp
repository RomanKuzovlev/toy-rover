#include "planning/astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>

enum class PlanFailure
{
  StartNotTraversable,
  GoalNotTraversable,
  NoPath
};

namespace toy_rover::planning
{

  namespace
  {
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

    double heuristic(mapping::GridIndex a, mapping::GridIndex b)
    {
      return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    std::vector<mapping::GridIndex> neighbors(mapping::GridIndex index)
    {
      return {
          {index.x + 1, index.y},
          {index.x - 1, index.y},
          {index.x, index.y + 1},
          {index.x, index.y - 1},
      };
    }
  } // namespace

  std::optional<Path> AStar::plan(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex start,
      mapping::GridIndex goal) const
  {
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
      for (const auto next : neighbors(current))
      {
        if (!traversable(grid, next))
        {
          continue;
        }

        const double tentative_g = current_g + 1.0;
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
