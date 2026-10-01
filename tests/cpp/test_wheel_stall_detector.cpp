// WheelStallDetector: ADR 0014, "Fault detectors", WHEEL_STALL.

#include "safety/wheel_stall_detector.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace recon::core {
namespace {

constexpr float kLimit = 0.3F;              // wheel_duty_limit default
constexpr uint32_t kStallUs = 500000;       // wheel_stall_ms default, in us
constexpr uint32_t kTickUs = 1000;          // 1 kHz motor loop

WheelStallDetector::Sample sample(float duty, float speed_rad_s, bool valid = true) {
  return {duty, kLimit, speed_rad_s, valid};
}

/// Feeds the same sample every tick from `start_us` for `ticks` ticks.
/// \return The first tick index (0-based) that tripped, or -1.
int run(WheelStallDetector& d, const WheelStallDetector::Sample& s, int ticks,
        uint32_t start_us = 0) {
  for (int i = 0; i < ticks; ++i) {
    if (d.update(s, start_us + static_cast<uint32_t>(i) * kTickUs, kStallUs)) {
      return i;
    }
  }
  return -1;
}

TEST(WheelStallDetector, SaturatedAndStillTripsAtExactlyTheStallTime) {
  WheelStallDetector d;
  // Condition starts at tick 0; 500 ms later is tick 500.
  EXPECT_EQ(run(d, sample(kLimit, 0.0F), 1000), 500);
}

TEST(WheelStallDetector, NegativeSaturationTripsToo) {
  WheelStallDetector d;
  EXPECT_EQ(run(d, sample(-kLimit, 0.5F), 1000), 500);
}

TEST(WheelStallDetector, UnsaturatedNeverTrips) {
  WheelStallDetector d;
  EXPECT_EQ(run(d, sample(0.29F, 0.0F), 2000), -1) << "a hard push below the limit is fine";
}

TEST(WheelStallDetector, SaturatedButTurningTheRightWayNeverTrips) {
  // ADR 0014: full stick on carpet saturates, but the wheel turns at ~5 rad/s.
  WheelStallDetector d;
  EXPECT_EQ(run(d, sample(kLimit, 5.0F), 2000), -1);
  WheelStallDetector back;
  EXPECT_EQ(run(back, sample(-kLimit, -5.0F), 2000), -1);
}

TEST(WheelStallDetector, SpeedJustAboveTheThresholdDoesNotCountAsStill) {
  WheelStallDetector d;
  EXPECT_EQ(run(d, sample(kLimit, kStallSpeed_rad_s), 2000), -1);
}

TEST(WheelStallDetector, SaturatedAndTurningAgainstTheDutyTrips) {
  // A reversed encoder: positive duty, wheel reads fast backwards.
  WheelStallDetector d;
  EXPECT_EQ(run(d, sample(kLimit, -9.0F), 1000), 500);
}

TEST(WheelStallDetector, ABreakInTheConditionRestartsTheTimer) {
  WheelStallDetector d;
  EXPECT_EQ(run(d, sample(kLimit, 0.0F), 400, 0), -1);
  EXPECT_FALSE(d.update(sample(0.1F, 0.0F), 400 * kTickUs, kStallUs));  // unsaturated once
  // Restarts at tick 401: trips 500 ticks later, not 100.
  EXPECT_EQ(run(d, sample(kLimit, 0.0F), 1000, 401 * kTickUs), 500);
}

TEST(WheelStallDetector, AnInvalidSpeedIsNotEvidenceOfAStall) {
  WheelStallDetector d;
  EXPECT_EQ(run(d, sample(kLimit, 0.0F, /*valid=*/false), 2000), -1);
}

TEST(WheelStallDetector, ResetForgetsAPartialStall) {
  WheelStallDetector d;
  EXPECT_EQ(run(d, sample(kLimit, 0.0F), 400, 0), -1);
  d.reset();
  EXPECT_EQ(run(d, sample(kLimit, 0.0F), 1000, 400 * kTickUs), 500);
}

TEST(WheelStallDetector, TimingIsCorrectAcrossTheClockWrap) {
  WheelStallDetector d;
  const uint32_t start = 0xFFFFFFFFU - 200000U;  // wraps 200 ms in
  EXPECT_EQ(run(d, sample(kLimit, 0.0F), 1000, start), 500);
}

TEST(WheelStallDetector, KeepsReportingWhileTheConditionHolds) {
  WheelStallDetector d;
  ASSERT_EQ(run(d, sample(kLimit, 0.0F), 1000), 500);
  EXPECT_TRUE(d.update(sample(kLimit, 0.0F), 501 * kTickUs, kStallUs));
}

}  // namespace
}  // namespace recon::core
