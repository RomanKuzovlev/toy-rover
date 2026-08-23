#include "mapping/occupancy_grid.hpp"
#include "planning/astar.hpp"
#include "planning/path_simplifier.hpp"

#include <gtest/gtest.h>
#include <iterator>

using toy_rover::mapping::Cell;
using toy_rover::mapping::GridIndex;
using toy_rover::mapping::OccupancyGrid;
using toy_rover::planning::AStar;
using toy_rover::planning::Path;
using toy_rover::planning::has_line_of_sight;
using toy_rover::planning::simplify_path;

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

TEST(AStar, CanExitAnOccupiedStartCell) {
  OccupancyGrid grid(4, 3, 0.1);
  grid.set({0, 1}, Cell::Occupied);
  grid.set({0, 0}, Cell::Occupied);
  grid.set({0, 2}, Cell::Occupied);

  const auto path = AStar{}.plan(grid, {0, 1}, {3, 1});

  ASSERT_TRUE(path.has_value());
  ASSERT_GE(path->size(), 2U);
  EXPECT_EQ(path->front(), (GridIndex{0, 1}));
  for (auto step = std::next(path->begin()); step != path->end(); ++step) {
    EXPECT_NE(grid.at(*step), Cell::Occupied);
  }
}

TEST(AStar, UsesDiagonalMovesInOpenSpace) {
  OccupancyGrid grid(4, 4, 0.05);

  const auto path = AStar{}.plan(grid, {0, 0}, {3, 3});

  ASSERT_TRUE(path.has_value());
  ASSERT_EQ(path->size(), 4U);
  EXPECT_EQ((*path)[1], (GridIndex{1, 1}));
  EXPECT_EQ((*path)[2], (GridIndex{2, 2}));
}

TEST(AStar, DoesNotCutDiagonallyBetweenObstacles) {
  OccupancyGrid grid(2, 2, 0.05);
  grid.set({1, 0}, Cell::Occupied);
  grid.set({0, 1}, Cell::Occupied);

  EXPECT_FALSE(AStar{}.plan(grid, {0, 0}, {1, 1}).has_value());
}

TEST(PathSimplifier, CollapsesAVisiblePathToItsEndpoints) {
  OccupancyGrid grid(5, 5, 0.05);
  const Path path{{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}};

  const auto simplified = simplify_path(grid, path);

  ASSERT_EQ(simplified.size(), 2U);
  EXPECT_EQ(simplified.front(), path.front());
  EXPECT_EQ(simplified.back(), path.back());
}

TEST(PathSimplifier, RetainsASafeTurnAroundAnObstacle) {
  OccupancyGrid grid(5, 5, 0.05);
  grid.set({2, 2}, Cell::Occupied);
  const Path path{{0, 2}, {1, 1}, {2, 0}, {3, 1}, {4, 2}};

  const auto simplified = simplify_path(grid, path);

  ASSERT_GT(simplified.size(), 2U);
  for (std::size_t i = 1; i < simplified.size(); ++i) {
    EXPECT_TRUE(has_line_of_sight(grid, simplified[i - 1], simplified[i]));
  }
}

TEST(PathSimplifier, DoesNotPassThroughACellCorner) {
  OccupancyGrid grid(2, 2, 0.05);
  grid.set({1, 0}, Cell::Occupied);

  EXPECT_FALSE(has_line_of_sight(grid, {0, 0}, {1, 1}));
}

TEST(AStar, ReturnsNoPathWhenStartIsOutsideGrid) {
  OccupancyGrid grid(3, 3, 0.1);

  EXPECT_FALSE(AStar{}.plan(grid, {-1, 0}, {2, 2}).has_value());
}
