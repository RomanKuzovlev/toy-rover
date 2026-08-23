#include "mapping/occupancy_grid.hpp"
#include "planning/astar.hpp"
#include "planning/goal_resolver.hpp"
#include "planning/path_simplifier.hpp"
#include "planning/start_escape.hpp"

#include <gtest/gtest.h>
#include <iterator>

using toy_rover::mapping::Cell;
using toy_rover::mapping::GridIndex;
using toy_rover::mapping::OccupancyGrid;
using toy_rover::planning::AStar;
using toy_rover::planning::Path;
using toy_rover::planning::has_line_of_sight;
using toy_rover::planning::simplify_path;
using toy_rover::planning::find_path_out_of_inflation;
using toy_rover::planning::find_nearest_free_cell;
using toy_rover::planning::resolve_reachable_goal;

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

TEST(StartEscape, CrossesOnlyTheTighterRecoveryLayer) {
  OccupancyGrid navigation_grid(7, 5, 0.05);
  OccupancyGrid recovery_grid(7, 5, 0.05);
  for (int y = 0; y < 5; ++y) {
    navigation_grid.set({0, y}, Cell::Occupied);
    navigation_grid.set({1, y}, Cell::Occupied);
    navigation_grid.set({2, y}, Cell::Occupied);
    recovery_grid.set({0, y}, Cell::Occupied);
  }

  const auto escape = find_path_out_of_inflation(
      recovery_grid, navigation_grid, {1, 2});

  ASSERT_TRUE(escape.has_value());
  EXPECT_EQ(escape->front(), (GridIndex{1, 2}));
  EXPECT_EQ(escape->back().x, 3);
  EXPECT_NE(navigation_grid.at(escape->back()), Cell::Occupied);
  for (auto step = std::next(escape->begin()); step != escape->end(); ++step) {
    EXPECT_NE(recovery_grid.at(*step), Cell::Occupied);
  }
}

TEST(StartEscape, RefusesToCrossTheRecoveryFootprint) {
  OccupancyGrid navigation_grid(3, 3, 0.05);
  OccupancyGrid recovery_grid(3, 3, 0.05);
  navigation_grid.fill(Cell::Occupied);
  recovery_grid.fill(Cell::Occupied);

  EXPECT_FALSE(find_path_out_of_inflation(
      recovery_grid, navigation_grid, {1, 1}).has_value());
}

TEST(GoalResolver, KeepsAFreeRequestedGoal) {
  OccupancyGrid grid(3, 3, 0.05);

  EXPECT_EQ(find_nearest_free_cell(grid, {1, 1}, 1.0), (GridIndex{1, 1}));
}

TEST(GoalResolver, MovesAnOccupiedGoalToTheNearestFreeCell) {
  OccupancyGrid grid(5, 5, 0.05);
  grid.fill(Cell::Occupied);
  grid.set({3, 2}, Cell::Free);
  grid.set({4, 2}, Cell::Free);

  EXPECT_EQ(find_nearest_free_cell(grid, {2, 2}, 0.2), (GridIndex{3, 2}));
}

TEST(GoalResolver, RespectsMaximumAdjustmentDistance) {
  OccupancyGrid grid(5, 5, 0.05);
  grid.fill(Cell::Occupied);
  grid.set({4, 2}, Cell::Free);

  EXPECT_FALSE(find_nearest_free_cell(grid, {2, 2}, 0.09).has_value());
}

TEST(GoalResolver, SelectsTheReachableSideOfAnOccupiedGoal) {
  OccupancyGrid grid(7, 5, 0.05);
  for (int y = 0; y < 5; ++y) {
    grid.set({3, y}, Cell::Occupied);
  }

  const auto resolved = resolve_reachable_goal(grid, {0, 2}, {3, 2}, 0.2);

  ASSERT_TRUE(resolved.has_value());
  EXPECT_LT(resolved->cell.x, 3);
  EXPECT_EQ(resolved->path.front(), (GridIndex{0, 2}));
  EXPECT_EQ(resolved->path.back(), resolved->cell);
}

TEST(AStar, ReturnsNoPathWhenStartIsOutsideGrid) {
  OccupancyGrid grid(3, 3, 0.1);

  EXPECT_FALSE(AStar{}.plan(grid, {-1, 0}, {2, 2}).has_value());
}
