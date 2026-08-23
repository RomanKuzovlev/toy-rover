#include "mapping/obstacle_history.hpp"

#include <stdexcept>
#include <utility>

namespace toy_rover::mapping
{

  ObstacleHistory::ObstacleHistory(std::chrono::nanoseconds retention)
      : retention_(retention)
  {
    if (retention_ <= std::chrono::nanoseconds::zero())
    {
      throw std::invalid_argument("obstacle retention must be positive");
    }
  }

  void ObstacleHistory::update(
      std::chrono::nanoseconds timestamp,
      std::vector<GridIndex> occupied_cells,
      OccupancyGrid &grid)
  {
    // A backwards timestamp normally means simulation time was reset. Old
    // observations then belong to a different timeline and must not survive.
    if (!batches_.empty() && timestamp < batches_.back().timestamp)
    {
      batches_.clear();
    }

    batches_.push_back(Batch{timestamp, std::move(occupied_cells)});

    while (!batches_.empty() && timestamp - batches_.front().timestamp > retention_)
    {
      batches_.pop_front();
    }

    rebuild(grid);
  }

  std::size_t ObstacleHistory::batch_count() const noexcept
  {
    return batches_.size();
  }

  void ObstacleHistory::rebuild(OccupancyGrid &grid) const
  {
    grid.fill(Cell::Unknown);
    for (const auto &batch : batches_)
    {
      for (const auto cell : batch.occupied_cells)
      {
        if (grid.in_bounds(cell))
        {
          grid.set(cell, Cell::Occupied);
        }
      }
    }
  }

} // namespace toy_rover::mapping
