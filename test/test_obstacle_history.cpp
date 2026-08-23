#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "mapping/obstacle_history.hpp"

namespace
{
  using namespace std::chrono_literals;
  using toy_rover::mapping::Cell;
  using toy_rover::mapping::GridIndex;
  using toy_rover::mapping::ObstacleHistory;
  using toy_rover::mapping::OccupancyGrid;

  TEST(ObstacleHistory, ExpiresCellsOutsideRetentionWindow)
  {
    OccupancyGrid grid(4, 4, 0.1);
    ObstacleHistory history(3s);

    history.update(1s, {{1, 1}}, grid);
    history.update(5s, {}, grid);

    EXPECT_EQ(grid.at({1, 1}), Cell::Unknown);
    EXPECT_EQ(history.batch_count(), 1U);
  }

  TEST(ObstacleHistory, KeepsCellSupportedByNewerObservation)
  {
    OccupancyGrid grid(4, 4, 0.1);
    ObstacleHistory history(3s);

    history.update(1s, {{1, 1}}, grid);
    history.update(3s, {{1, 1}}, grid);
    history.update(5s, {}, grid);

    EXPECT_EQ(grid.at({1, 1}), Cell::Occupied);
    EXPECT_EQ(history.batch_count(), 2U);
  }

  TEST(ObstacleHistory, DropsOldTimelineWhenTimestampMovesBackwards)
  {
    OccupancyGrid grid(4, 4, 0.1);
    ObstacleHistory history(3s);

    history.update(10s, {{1, 1}}, grid);
    history.update(1s, {{2, 2}}, grid);

    EXPECT_EQ(grid.at({1, 1}), Cell::Unknown);
    EXPECT_EQ(grid.at({2, 2}), Cell::Occupied);
    EXPECT_EQ(history.batch_count(), 1U);
  }
} // namespace
