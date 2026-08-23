#include "planning/goal_resolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace toy_rover::planning
{
  std::optional<mapping::GridIndex> find_nearest_free_cell(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex requested,
      double max_distance_m)
  {
    const auto candidates = find_free_cells_nearest_first(
        grid, requested, max_distance_m);
    if (candidates.empty())
    {
      return std::nullopt;
    }
    return candidates.front();
  }

  std::vector<mapping::GridIndex> find_free_cells_nearest_first(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex requested,
      double max_distance_m)
  {
    if (max_distance_m < 0.0)
    {
      throw std::invalid_argument("maximum goal adjustment distance must be non-negative");
    }
    if (!grid.in_bounds(requested))
    {
      return {};
    }
    if (grid.at(requested) != mapping::Cell::Occupied)
    {
      return {requested};
    }

    struct Candidate
    {
      mapping::GridIndex cell;
      int squared_cell_distance;
    };

    const double resolution_m = grid.resolution_m();
    const int cell_radius = static_cast<int>(std::ceil(max_distance_m / resolution_m));
    const double maximum_squared_cells =
        max_distance_m * max_distance_m / (resolution_m * resolution_m);
    std::vector<Candidate> candidates;

    for (int dy = -cell_radius; dy <= cell_radius; ++dy)
    {
      for (int dx = -cell_radius; dx <= cell_radius; ++dx)
      {
        const int squared_cells = dx * dx + dy * dy;
        if (static_cast<double>(squared_cells) > maximum_squared_cells)
        {
          continue;
        }

        const mapping::GridIndex candidate{requested.x + dx, requested.y + dy};
        if (!grid.in_bounds(candidate) ||
            grid.at(candidate) == mapping::Cell::Occupied)
        {
          continue;
        }

        candidates.push_back({candidate, squared_cells});
      }
    }

    std::sort(
        candidates.begin(), candidates.end(),
        [](const Candidate &lhs, const Candidate &rhs)
        {
          return lhs.squared_cell_distance < rhs.squared_cell_distance;
        });

    std::vector<mapping::GridIndex> cells;
    cells.reserve(candidates.size());
    for (const auto &candidate : candidates)
    {
      cells.push_back(candidate.cell);
    }
    return cells;
  }

  std::optional<ResolvedGoal> resolve_reachable_goal(
      const mapping::OccupancyGrid &grid,
      mapping::GridIndex start,
      mapping::GridIndex requested,
      double max_distance_m)
  {
    const AStar planner;
    std::optional<ResolvedGoal> best;
    double best_score = std::numeric_limits<double>::infinity();
    double reachable_distance_limit_m = std::numeric_limits<double>::infinity();
    for (const auto candidate : find_free_cells_nearest_first(
             grid, requested, max_distance_m))
    {
      const double goal_distance_m = grid.resolution_m() * std::hypot(
          candidate.x - requested.x,
          candidate.y - requested.y);
      if (goal_distance_m > reachable_distance_limit_m)
      {
        break;
      }

      auto path = planner.plan(grid, start, candidate);
      if (path)
      {
        if (!best)
        {
          // Compare candidates in a narrow band around the closest reachable
          // point. This avoids choosing the opposite side of a thin wall for
          // a millimetre of goal-distance improvement.
          reachable_distance_limit_m = goal_distance_m + 0.10;
        }
        const double route_length_m =
            grid.resolution_m() * static_cast<double>(path->size() - 1);
        const double score = goal_distance_m + 0.10 * route_length_m;
        if (score < best_score)
        {
          best_score = score;
          best = ResolvedGoal{candidate, std::move(*path)};
        }
      }
    }
    return best;
  }
} // namespace toy_rover::planning
