#include <chrono>
#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "control/pose_history.hpp"
#include "control/timestamp_gate.hpp"

namespace
{
  using namespace std::chrono_literals;
  using toy_rover::control::Pose2D;
  using toy_rover::control::PoseHistory;
  using toy_rover::control::TimestampGate;

  TEST(PoseHistory, InterpolatesPoseAtRequestedTimestamp)
  {
    PoseHistory history(2s);
    history.add(1s, Pose2D{1.0, 2.0, 0.0});
    history.add(2s, Pose2D{3.0, 6.0, 1.0});

    const auto pose = history.interpolate(1500ms);

    ASSERT_TRUE(pose.has_value());
    EXPECT_DOUBLE_EQ(pose->x, 2.0);
    EXPECT_DOUBLE_EQ(pose->y, 4.0);
    EXPECT_DOUBLE_EQ(pose->yaw_rad, 0.5);
  }

  TEST(PoseHistory, InterpolatesYawAcrossWraparound)
  {
    PoseHistory history(2s);
    history.add(1s, Pose2D{0.0, 0.0, 179.0 * std::numbers::pi / 180.0});
    history.add(2s, Pose2D{0.0, 0.0, -179.0 * std::numbers::pi / 180.0});

    const auto pose = history.interpolate(1500ms);

    ASSERT_TRUE(pose.has_value());
    EXPECT_NEAR(std::abs(pose->yaw_rad), std::numbers::pi, 1e-12);
  }

  TEST(PoseHistory, RefusesToExtrapolate)
  {
    PoseHistory history(2s);
    history.add(1s, Pose2D{});
    history.add(2s, Pose2D{});

    EXPECT_FALSE(history.interpolate(500ms).has_value());
    EXPECT_FALSE(history.interpolate(2500ms).has_value());
  }

  TEST(PoseHistory, ClearsOldTimelineWhenTimestampMovesBackwards)
  {
    PoseHistory history(2s);
    history.add(10s, Pose2D{10.0, 0.0, 0.0});
    history.add(1s, Pose2D{1.0, 0.0, 0.0});

    EXPECT_FALSE(history.interpolate(10s).has_value());
    const auto pose = history.interpolate(1s);
    ASSERT_TRUE(pose.has_value());
    EXPECT_DOUBLE_EQ(pose->x, 1.0);
  }

  TEST(TimestampGate, DropsDuplicateAndOlderMessages)
  {
    TimestampGate gate;

    EXPECT_TRUE(gate.accept(1s, 1s));
    EXPECT_TRUE(gate.accept(2s, 2s));
    EXPECT_FALSE(gate.accept(2s, 3s));
    EXPECT_FALSE(gate.accept(1500ms, 3s));
    EXPECT_TRUE(gate.accept(3s, 3s));
  }

  TEST(TimestampGate, AcceptsANewSequenceAfterClockReset)
  {
    TimestampGate gate;

    EXPECT_TRUE(gate.accept(10s, 10s));
    EXPECT_TRUE(gate.accept(1s, 1s));
    EXPECT_TRUE(gate.accept(2s, 2s));
  }
} // namespace
