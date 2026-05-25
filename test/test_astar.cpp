#include "mapping/occupancy_grid.hpp"
#include "planning/astar.hpp"

#include <gtest/gtest.h>

using toy_rover::mapping::Cell;
using toy_rover::mapping::GridIndex;
using toy_rover::mapping::OccupancyGrid;
using toy_rover::planning::AStar;

TEST(AStar, FindsPathAroundObstacle) {
  OccupancyGrid grid(5, 5, 0.1);
  grid.set({1, 0}, Cell::Occupied);
  grid.set({1, 1}, Cell::Occupied);
  grid.set({1, 2}, Cell::Occupied);
  grid.set({1, 3}, Cell::Occupied);

  const auto path = AStar{}.plan(grid, {0, 0}, {4, 0});

  ASSERT_TRUE(path.has_value());
  ASSERT_FALSE(path->empty());
  EXPECT_EQ(path->front(), (GridIndex{0, 0}));
  EXPECT_EQ(path->back(), (GridIndex{4, 0}));

  for (const auto& step : *path) {
    EXPECT_NE(grid.at(step), Cell::Occupied);
  }
}

TEST(AStar, ReturnsNoPathWhenGoalBlocked) {
  OccupancyGrid grid(3, 3, 0.1);
  grid.set({2, 2}, Cell::Occupied);

  const auto path = AStar{}.plan(grid, {0, 0}, {2, 2});

  EXPECT_FALSE(path.has_value());
}
