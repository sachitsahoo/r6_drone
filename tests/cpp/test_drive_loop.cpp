// DriveLoop against the HAL fakes: wiring, ARMED gating, rate limiting, fault fallback.
// Closed-loop behaviour on a plant is tested in test_sil_drive.cpp.

#include "control/drive_loop.hpp"

#include <gtest/gtest.h>

#include <cstdint>

#include "control/geometry.hpp"
#include "hal_fakes.hpp"

namespace recon::core {
namespace {

using test::FakeClock;
using test::FakeWheelEncoder;
using test::FakeWheelMotor;

constexpr uint32_t kPeriodUs = 1000;
constexpr float kCpr = 1400.0F;

struct Rig {
  FakeClock clock;
  FakeWheelEncoder left_enc{kCpr};
  FakeWheelEncoder right_enc{kCpr};
  FakeWheelMotor left_motor;
  FakeWheelMotor right_motor;
  DriveLoop loop{clock, left_enc, left_motor, right_enc, right_motor};

  void tick(bool armed) {
    clock.advance_us(kPeriodUs);
    loop.step(armed);
  }
};

TEST(DriveLoop, NeverWritesTheMotorsWhenNotArmed) {
  Rig rig;
  rig.loop.set_command(0.3F, 0.0F);
  for (int i = 0; i < 100; ++i) {
    rig.tick(false);
  }
  EXPECT_EQ(rig.left_motor.set_duty_calls(), 0);
  EXPECT_EQ(rig.right_motor.set_duty_calls(), 0);
  EXPECT_EQ(rig.loop.left().duty, 0.0F);
  EXPECT_EQ(rig.loop.left().ref_rad_s, 0.0F) << "limiter held at zero while disarmed";
}

TEST(DriveLoop, EstimatorsKeepRunningWhileDisarmed) {
  Rig rig;
  for (uint32_t k = 0; k < 20; ++k) {
    rig.left_enc.set_count(3 * k);
    rig.tick(false);
  }
  EXPECT_TRUE(rig.loop.left().speed_valid);
  EXPECT_GT(rig.loop.left().speed_rad_s, 0.0F) << "a rolling robot is measured before arming";
}

TEST(DriveLoop, ZeroCommandWhileArmedActivelyHoldsZero) {
  Rig rig;
  rig.tick(true);
  rig.tick(true);
  EXPECT_GT(rig.left_motor.set_duty_calls(), 0) << "ADR 0013 Q4: hold, do not coast";
  EXPECT_FALSE(rig.left_motor.stopped());
  EXPECT_FLOAT_EQ(rig.left_motor.last_duty(), 0.0F);
}

TEST(DriveLoop, ReferenceRampsAtTheAccelerationLimit) {
  Rig rig;
  rig.loop.set_command(1.0F, 0.0F);
  constexpr int kSteps = 50;
  for (int i = 0; i < kSteps; ++i) {
    rig.tick(true);
  }
  const DriveConfig c;
  const float per_step = c.accel_limit_m_s2 / kWheelRadius_m * (kPeriodUs * 1e-6F);
  // The first step after construction has no previous timestamp, so it integrates dt = 0:
  // kSteps calls make kSteps - 1 ramp increments.
  EXPECT_NEAR(rig.loop.left().ref_rad_s, per_step * (kSteps - 1), 1e-3F);
  EXPECT_GT(rig.left_motor.last_duty(), 0.0F);
}

TEST(DriveLoop, TurningInPlaceDrivesTheWheelsOppositely) {
  Rig rig;
  rig.loop.set_command(0.0F, 1.0F);
  for (int i = 0; i < 200; ++i) {
    rig.tick(true);
  }
  EXPECT_FLOAT_EQ(rig.loop.left().ref_rad_s, -rig.loop.right().ref_rad_s);
  EXPECT_LT(rig.left_motor.last_duty(), 0.0F);
  EXPECT_GT(rig.right_motor.last_duty(), 0.0F);
}

TEST(DriveLoop, DisarmingResetsIntegratorsAndLimiters) {
  Rig rig;
  rig.loop.set_command(0.3F, 0.0F);
  for (int i = 0; i < 300; ++i) {
    rig.tick(true);  // encoders never move: integrators wind toward the limit
  }
  ASSERT_GT(rig.loop.left().integrator, 0.0F);
  rig.tick(false);
  EXPECT_EQ(rig.loop.left().integrator, 0.0F);
  EXPECT_EQ(rig.loop.left().ref_rad_s, 0.0F);
  rig.tick(true);
  EXPECT_LT(rig.loop.left().ref_rad_s, 0.05F) << "re-arming ramps up from zero again";
}

TEST(DriveLoop, DutyNeverExceedsTheConfiguredLimit) {
  Rig rig;
  rig.loop.set_command(5.0F, 0.0F);
  for (int i = 0; i < 2000; ++i) {
    rig.tick(true);
    ASSERT_LE(rig.left_motor.last_duty(), rig.loop.config().gains.duty_limit);
  }
}

TEST(DriveLoop, AnEncoderFaultFallsBackToFeedforwardOnly) {
  Rig rig;
  rig.loop.set_command(0.1F, 0.0F);
  for (int i = 0; i < 200; ++i) {
    rig.tick(true);
  }
  const float integrator = rig.loop.left().integrator;
  rig.left_enc.set_count(100000);  // impossible jump
  rig.tick(true);
  EXPECT_TRUE(rig.loop.left().encoder_fault);
  EXPECT_FALSE(rig.loop.left().speed_valid);
  EXPECT_FLOAT_EQ(rig.loop.left().integrator, integrator) << "no integrating on a bad measurement";
  const float ff = rig.loop.config().gains.kff * rig.loop.left().ref_rad_s;
  EXPECT_FLOAT_EQ(rig.left_motor.last_duty(), ff);
  EXPECT_FALSE(rig.loop.right().encoder_fault) << "wheels are independent";
}

TEST(DriveLoop, AStallIsIntegratedOverAtMostTheMaxStep) {
  Rig rig;
  rig.loop.set_command(1.0F, 0.0F);
  rig.tick(true);
  const float ref_before = rig.loop.left().ref_rad_s;
  rig.clock.advance_us(500000);  // a 0.5 s stall
  rig.loop.step(true);
  const DriveConfig c;
  const float max_ramp = c.accel_limit_m_s2 / kWheelRadius_m * kMaxDriveStepDt_s;
  EXPECT_NEAR(rig.loop.left().ref_rad_s - ref_before, max_ramp, 1e-4F);
}

TEST(DriveLoop, ConfigDefaultsComeFromTheSchema) {
  const DriveConfig c;
  EXPECT_FLOAT_EQ(c.max_wheel_speed_rad_s, 9.0F);
  EXPECT_FLOAT_EQ(c.accel_limit_m_s2, 1.0F);
  EXPECT_EQ(c.speed_window_samples, 10);
}

TEST(DriveLoop, MeasuresItsOwnExecutionTime) {
  Rig rig;
  rig.tick(true);
  // FakeClock does not advance inside step(), so the measured time is exactly zero here.
  // The point is that the measurement path runs; real timings come from the target.
  EXPECT_EQ(rig.loop.last_exec_us(), 0U);
  EXPECT_EQ(rig.loop.max_exec_us(), 0U);
}

}  // namespace
}  // namespace recon::core
