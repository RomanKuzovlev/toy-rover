#include <chrono>

#include <gtest/gtest.h>

#include "control/stuck_detector.hpp"

namespace
{
  using namespace std::chrono_literals;
  using toy_rover::control::StuckDetector;

  TEST(StuckDetector, EmitsOnceWhenPositionStaysWithinTolerance)
  {
    StuckDetector detector(2s, 1);

    EXPECT_FALSE(detector.update(0s, {10, 10}));
    EXPECT_FALSE(detector.update(1s, {11, 9}));
    EXPECT_TRUE(detector.update(2s, {10, 10}));
    EXPECT_FALSE(detector.update(3s, {10, 10}));
  }

  TEST(StuckDetector, DoesNotEmitWhenWindowContainsMeaningfulMovement)
  {
    StuckDetector detector(2s, 1);

    EXPECT_FALSE(detector.update(0s, {10, 10}));
    EXPECT_FALSE(detector.update(1s, {12, 10}));
    EXPECT_FALSE(detector.update(2s, {10, 10}));
    EXPECT_FALSE(detector.update(3s, {10, 10}));
    EXPECT_TRUE(detector.update(4s, {10, 10}));
  }

  TEST(StuckDetector, RearmsAfterLeavingLatchedArea)
  {
    StuckDetector detector(2s, 1);

    EXPECT_FALSE(detector.update(0s, {10, 10}));
    EXPECT_TRUE(detector.update(2s, {10, 10}));
    EXPECT_FALSE(detector.update(3s, {12, 10}));
    EXPECT_FALSE(detector.update(4s, {12, 10}));
    EXPECT_TRUE(detector.update(5s, {12, 10}));
  }
} // namespace
