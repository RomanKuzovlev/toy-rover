#include <stdexcept>

#include <gtest/gtest.h>

#include "mapping/obstacle_inflation.hpp"

namespace
{
  using toy_rover::mapping::Cell;
  using toy_rover::mapping::OccupancyGrid;
  using toy_rover::mapping::inflate_occupied_cells;

  TEST(ObstacleInflation, ExpandsOccupiedCellByMetricRadius)
  {
    OccupancyGrid grid(9, 9, 0.1);
    grid.set({4, 4}, Cell::Occupied);

    inflate_occupied_cells(grid, 0.21);

    EXPECT_EQ(grid.at({6, 4}), Cell::Occupied);
    EXPECT_EQ(grid.at({5, 5}), Cell::Occupied);
    EXPECT_EQ(grid.at({6, 6}), Cell::Unknown);
    EXPECT_EQ(grid.at({7, 4}), Cell::Unknown);
  }

  TEST(ObstacleInflation, ClipsInflationAtGridBoundary)
  {
    OccupancyGrid grid(3, 3, 0.1);
    grid.set({0, 0}, Cell::Occupied);

    EXPECT_NO_THROW(inflate_occupied_cells(grid, 0.2));
    EXPECT_EQ(grid.at({1, 0}), Cell::Occupied);
    EXPECT_EQ(grid.at({0, 1}), Cell::Occupied);
  }

  TEST(ObstacleInflation, RejectsNegativeRadius)
  {
    OccupancyGrid grid(3, 3, 0.1);

    EXPECT_THROW(inflate_occupied_cells(grid, -0.1), std::invalid_argument);
  }
} // namespace
