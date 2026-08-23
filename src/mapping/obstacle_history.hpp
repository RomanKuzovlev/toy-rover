#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <vector>

#include "mapping/occupancy_grid.hpp"

namespace toy_rover::mapping
{

  class ObstacleHistory
  {
  public:
    explicit ObstacleHistory(std::chrono::nanoseconds retention);

    void update(
        std::chrono::nanoseconds timestamp,
        std::vector<GridIndex> occupied_cells,
        OccupancyGrid &grid);

    [[nodiscard]] std::size_t batch_count() const noexcept;

  private:
    struct Batch
    {
      std::chrono::nanoseconds timestamp;
      std::vector<GridIndex> occupied_cells;
    };

    void rebuild(OccupancyGrid &grid) const;

    std::chrono::nanoseconds retention_;
    std::deque<Batch> batches_;
  };

} // namespace toy_rover::mapping
