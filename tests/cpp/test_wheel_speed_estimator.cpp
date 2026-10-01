// WheelSpeedEstimator: ADR 0013 "Speed measurement".

#include "control/wheel_speed_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

namespace recon::core {
namespace {

constexpr float kCpr = 1400.0F;
constexpr float kTwoPi = 6.2831853F;
constexpr uint32_t kPeriodUs = 1000;

/// rad/s represented by `counts` over `window` 1 ms samples.
float speed_of(int counts, int window) {
  return static_cast<float>(counts) / kCpr * kTwoPi / (static_cast<float>(window) * 1e-3F);
}

TEST(WheelSpeedEstimator, FirstSampleIsInvalid) {
  WheelSpeedEstimator est(kCpr, 10);
  const auto r = est.update(123, 0);
  EXPECT_FALSE(r.valid);
  EXPECT_FALSE(r.fault);
  EXPECT_EQ(r.speed_rad_s, 0.0F);
}

TEST(WheelSpeedEstimator, ExactSpeedForAConstantCountRate) {
  WheelSpeedEstimator est(kCpr, 10);
  WheelSpeedEstimator::Result r{};
  for (uint32_t k = 0; k <= 20; ++k) {
    r = est.update(3 * k, k * kPeriodUs);  // 3 counts per ms
  }
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.speed_rad_s, speed_of(30, 10), 1e-4F);
}

TEST(WheelSpeedEstimator, ResolutionIsOneCountPerWindow) {
  // ADR 0013: 10 samples -> 0.449 rad/s steps at 1400 counts/rev.
  EXPECT_NEAR(speed_of(1, 10), 0.4488F, 1e-4F);
  WheelSpeedEstimator est(kCpr, 10);
  for (uint32_t k = 0; k < 10; ++k) {
    est.update(0, k * kPeriodUs);
  }
  const auto r = est.update(1, 10 * kPeriodUs);
  EXPECT_NEAR(r.speed_rad_s, speed_of(1, 10), 1e-4F);
}

TEST(WheelSpeedEstimator, WarmUpUsesTheSamplesAvailable) {
  WheelSpeedEstimator est(kCpr, 10);
  est.update(0, 0);
  const auto r = est.update(2, kPeriodUs);  // one interval so far
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.speed_rad_s, speed_of(2, 1), 1e-3F);
}

TEST(WheelSpeedEstimator, UsesMeasuredTimeNotTheNominalPeriod) {
  WheelSpeedEstimator est(kCpr, 2);
  est.update(0, 0);
  est.update(3, 1000);
  const auto r = est.update(6, 3000);  // the second interval ran 2 ms late
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.speed_rad_s, speed_of(6, 3), 1e-3F) << "6 counts over 3 ms, not 2 ms";
}

TEST(WheelSpeedEstimator, ForwardThroughTheCounterWrap) {
  WheelSpeedEstimator est(kCpr, 10);
  WheelSpeedEstimator::Result r{};
  const uint32_t start = 0xFFFFFFF0U;
  for (uint32_t k = 0; k <= 20; ++k) {
    r = est.update(start + 3 * k, k * kPeriodUs);  // crosses 2^32 at k = 6
  }
  EXPECT_NEAR(r.speed_rad_s, speed_of(30, 10), 1e-4F);
}

TEST(WheelSpeedEstimator, BackwardThroughZero) {
  WheelSpeedEstimator est(kCpr, 10);
  WheelSpeedEstimator::Result r{};
  for (uint32_t k = 0; k <= 20; ++k) {
    r = est.update(5U - 2U * k, k * kPeriodUs);  // unsigned: wraps below 0
  }
  EXPECT_NEAR(r.speed_rad_s, speed_of(-20, 10), 1e-4F);
}

TEST(WheelSpeedEstimator, AcrossTheTimestampWrap) {
  WheelSpeedEstimator est(kCpr, 10);
  WheelSpeedEstimator::Result r{};
  const uint32_t t0 = 0xFFFFFFFFU - 5000U;
  for (uint32_t k = 0; k <= 20; ++k) {
    r = est.update(3 * k, t0 + k * kPeriodUs);
  }
  EXPECT_NEAR(r.speed_rad_s, speed_of(30, 10), 1e-4F);
}

TEST(WheelSpeedEstimator, AnImpossibleJumpFaultsAndRestartsHistory) {
  WheelSpeedEstimator est(kCpr, 10);
  for (uint32_t k = 0; k <= 10; ++k) {
    est.update(3 * k, k * kPeriodUs);
  }
  // 100 rad/s over 1 ms is 22.3 counts; a 500-count jump is a glitch, not motion.
  const auto bad = est.update(30 + 500, 11 * kPeriodUs);
  EXPECT_TRUE(bad.fault);
  EXPECT_FALSE(bad.valid);
  const auto next = est.update(30 + 500 + 3, 12 * kPeriodUs);
  EXPECT_FALSE(next.fault);
  ASSERT_TRUE(next.valid) << "recovers from the new baseline";
  EXPECT_NEAR(next.speed_rad_s, speed_of(3, 1), 1e-3F);
}

TEST(WheelSpeedEstimator, TheFastestPlausibleSpeedIsNotAFault) {
  WheelSpeedEstimator est(kCpr, 10);
  est.update(0, 0);
  const auto counts_per_ms = static_cast<uint32_t>(
      kMaxPlausibleWheelSpeed_rad_s / kTwoPi * kCpr * 1e-3F);  // 22
  EXPECT_FALSE(est.update(counts_per_ms, kPeriodUs).fault);
}

TEST(WheelSpeedEstimator, NoTimeElapsedIsInvalidNotInfinite) {
  WheelSpeedEstimator est(kCpr, 10);
  est.update(0, 500);
  const auto r = est.update(0, 500);
  EXPECT_FALSE(r.valid);
  EXPECT_TRUE(std::isfinite(r.speed_rad_s));
}

TEST(WheelSpeedEstimator, WindowIsClampedAndChangingItClearsHistory) {
  WheelSpeedEstimator est(kCpr, 0);
  EXPECT_EQ(est.window_samples(), 1);
  est.set_window_samples(200);
  EXPECT_EQ(est.window_samples(), WheelSpeedEstimator::kMaxWindowSamples);
  est.update(0, 0);
  est.update(3, kPeriodUs);
  est.set_window_samples(5);
  EXPECT_FALSE(est.update(6, 2 * kPeriodUs).valid) << "history cleared";
}

TEST(WheelSpeedEstimator, ResetForgetsEverything) {
  WheelSpeedEstimator est(kCpr, 10);
  est.update(0, 0);
  est.update(3, kPeriodUs);
  est.reset();
  EXPECT_FALSE(est.update(6, 2 * kPeriodUs).valid);
}

}  // namespace
}  // namespace recon::core
