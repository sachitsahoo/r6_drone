// diff_drive and RateLimiter: ADR 0013 "Kinematics, saturation and acceleration limit".

#include "control/diff_drive.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "control/geometry.hpp"
#include "control/rate_limiter.hpp"

namespace recon::core {
namespace {

constexpr float kNoLimit = 1e6F;

TEST(DiffDrive, StraightAheadDrivesBothWheelsEqually) {
  const auto r = diff_drive(0.105F, 0.0F, kNoLimit);
  EXPECT_FLOAT_EQ(r.left_rad_s, 0.105F / kWheelRadius_m);  // 2 rad/s
  EXPECT_FLOAT_EQ(r.right_rad_s, r.left_rad_s);
}

TEST(DiffDrive, PositiveYawRateTurnsLeft) {
  // REP-103: +w is counter-clockwise from above, so the right wheel runs faster.
  const auto r = diff_drive(0.0F, 1.0F, kNoLimit);
  const float rim_m_s = 1.0F * kTrackWidth_m / 2.0F;
  EXPECT_FLOAT_EQ(r.right_rad_s, rim_m_s / kWheelRadius_m);
  EXPECT_FLOAT_EQ(r.left_rad_s, -r.right_rad_s) << "turning in place";
}

TEST(DiffDrive, SaturationScalesBothWheelsAndKeepsTheCurvature) {
  const float v = 1.0F;
  const float w = 2.0F;
  const auto free = diff_drive(v, w, kNoLimit);
  const float limit = 10.0F;
  const auto sat = diff_drive(v, w, limit);
  EXPECT_FLOAT_EQ(std::fmax(std::fabs(sat.left_rad_s), std::fabs(sat.right_rad_s)), limit);
  EXPECT_NEAR(sat.left_rad_s / sat.right_rad_s, free.left_rad_s / free.right_rad_s, 1e-5F)
      << "same ratio = same turning radius";
}

TEST(DiffDrive, WithinTheLimitIsUnchanged) {
  const auto a = diff_drive(0.2F, 0.5F, kNoLimit);
  const auto b = diff_drive(0.2F, 0.5F, 100.0F);
  EXPECT_FLOAT_EQ(a.left_rad_s, b.left_rad_s);
  EXPECT_FLOAT_EQ(a.right_rad_s, b.right_rad_s);
}

TEST(DiffDrive, ZeroAndNanCommandsGiveZero) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const auto& r : {diff_drive(0.0F, 0.0F, 10.0F), diff_drive(nan, nan, 10.0F)}) {
    EXPECT_EQ(r.left_rad_s, 0.0F);
    EXPECT_EQ(r.right_rad_s, 0.0F);
  }
}

TEST(DiffDrive, AZeroLimitStopsBothWheels) {
  const auto r = diff_drive(1.0F, 1.0F, 0.0F);
  EXPECT_EQ(r.left_rad_s, 0.0F);
  EXPECT_EQ(r.right_rad_s, 0.0F);
}

TEST(RateLimiter, MovesByAtMostTheStep) {
  RateLimiter lim;
  EXPECT_FLOAT_EQ(lim.step(10.0F, 0.5F), 0.5F);
  EXPECT_FLOAT_EQ(lim.step(10.0F, 0.5F), 1.0F);
  EXPECT_FLOAT_EQ(lim.step(-10.0F, 0.25F), 0.75F);
}

TEST(RateLimiter, LandsExactlyOnTheTarget) {
  RateLimiter lim;
  lim.reset(0.9F);
  EXPECT_FLOAT_EQ(lim.step(1.0F, 0.5F), 1.0F);
  EXPECT_FLOAT_EQ(lim.step(1.0F, 0.5F), 1.0F);
}

TEST(RateLimiter, NegativeStepHoldsAndResetJumps) {
  RateLimiter lim;
  EXPECT_FLOAT_EQ(lim.step(5.0F, -1.0F), 0.0F);
  lim.reset(3.0F);
  EXPECT_FLOAT_EQ(lim.value(), 3.0F);
}

}  // namespace
}  // namespace recon::core
