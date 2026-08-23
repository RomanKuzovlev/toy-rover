#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "control/pure_pursuit.hpp"

namespace
{
  using toy_rover::control::Point2D;
  using toy_rover::control::Pose2D;
  using toy_rover::control::PurePursuit;

  TEST(PurePursuit, StopsWithinGoalTolerance)
  {
    const auto command = PurePursuit(0.35, 0.2).compute_command(
        Pose2D{1.0, 1.0, 0.0},
        std::vector<Point2D>{{1.1, 1.0}});

    EXPECT_DOUBLE_EQ(command.linear_mps, 0.0);
    EXPECT_DOUBLE_EQ(command.angular_radps, 0.0);
  }

  TEST(PurePursuit, RotatesBeforeDrivingTowardPointBehind)
  {
    const auto command = PurePursuit(0.35, 0.2).compute_command(
        Pose2D{0.0, 0.0, 0.0},
        std::vector<Point2D>{{-1.0, 0.0}});

    EXPECT_DOUBLE_EQ(command.linear_mps, 0.0);
    EXPECT_GT(std::abs(command.angular_radps), 1.0);
  }

  TEST(PurePursuit, DrivesTowardAlignedPoint)
  {
    const auto command = PurePursuit(0.35, 0.2).compute_command(
        Pose2D{0.0, 0.0, 0.0},
        std::vector<Point2D>{{1.0, 0.0}});

    EXPECT_DOUBLE_EQ(command.linear_mps, 0.2);
    EXPECT_DOUBLE_EQ(command.angular_radps, 0.0);
  }
} // namespace
