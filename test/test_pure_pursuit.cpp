#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "control/free_space_escape.hpp"
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

  TEST(PurePursuit, DoesNotSkipANearbyCornerWaypoint)
  {
    const auto command = PurePursuit(0.35, 0.2).compute_command(
        Pose2D{0.0, 0.0, 0.0},
        std::vector<Point2D>{{0.0, 0.0}, {0.1, 0.0}, {0.1, 1.0}});

    EXPECT_GT(command.linear_mps, 0.0);
    EXPECT_LT(command.linear_mps, 0.2);
    EXPECT_DOUBLE_EQ(command.angular_radps, 0.0);
  }

  TEST(FreeSpaceEscape, ChoosesTheCenterOfAWideOpenCorridor)
  {
    constexpr std::size_t sample_count = 36;
    constexpr double increment = 2.0 * std::numbers::pi / sample_count;
    std::vector<float> ranges(sample_count, 1.0F);
    const std::size_t open_center = 27; // +pi/2 from an angle_min of -pi
    for (int offset = -2; offset <= 2; ++offset)
    {
      ranges[open_center + offset] = 5.0F;
    }

    const auto heading = toy_rover::control::choose_free_space_heading(
        ranges, -std::numbers::pi, increment, 0.08, 12.0, 0.2, 0.0);

    ASSERT_TRUE(heading.has_value());
    EXPECT_NEAR(*heading, std::numbers::pi / 2.0, increment);
  }

  TEST(FreeSpaceEscape, ReportsTheNarrowestForwardReading)
  {
    constexpr std::size_t sample_count = 36;
    constexpr double increment = 2.0 * std::numbers::pi / sample_count;
    std::vector<float> ranges(sample_count, 5.0F);
    ranges[18] = 0.7F;

    const double clearance = toy_rover::control::forward_clearance(
        ranges, -std::numbers::pi, increment, 0.08, 12.0, 0.1);

    EXPECT_NEAR(clearance, 0.7, 1e-6);
  }
} // namespace
