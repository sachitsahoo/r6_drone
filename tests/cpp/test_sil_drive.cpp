// SIL: the ADR 0013 wheel velocity loop closed on the simulated wheel plant.
//
// Every bound here is derived in docs/theory/wheel-velocity-loop.md, not read off a run.
// The plant is a PLACEHOLDER (sim/sim/wheel_plant.hpp): these tests prove the loop's
// structure and logic, not its performance on hardware.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "control/drive_loop.hpp"
#include "control/geometry.hpp"
#include "sim/sim_world.hpp"

namespace recon::sim {
namespace {

using core::DriveConfig;
using core::DriveLoop;
using core::kWheelRadius_m;

constexpr uint32_t kPeriodUs = 1000;  // 1 kHz motor loop (CLAUDE.md)

struct DriveRig {
  explicit DriveRig(const SimWorld::Config& world_config = SimWorld::Config{},
               const DriveConfig& drive_config = DriveConfig{})
      : world(world_config),
        loop(world.clock, world.left_wheel, world.left_wheel, world.right_wheel,
             world.right_wheel, drive_config) {}

  void advance(double seconds, bool armed = true) {
    const auto steps = static_cast<int>(seconds * 1e6 / kPeriodUs + 0.5);
    for (int i = 0; i < steps; ++i) {
      world.step(kPeriodUs);
      loop.step(armed);
    }
  }

  SimWorld world;
  DriveLoop loop;
};

/// A config with the acceleration limit at its maximum, so a step tests the loop itself.
DriveConfig fast_ramp() {
  DriveConfig c;
  c.accel_limit_m_s2 = 10.0F;  // params.yaml upper bound
  return c;
}

/// Mean true speed over `seconds`, which averages out the one-count limit cycle.
double mean_speed(DriveRig& run, double seconds, bool left = true) {
  const auto steps = static_cast<int>(seconds * 1e6 / kPeriodUs + 0.5);
  double sum = 0.0;
  for (int i = 0; i < steps; ++i) {
    run.advance(kPeriodUs * 1e-6);
    sum += left ? run.world.left_wheel.speed_rad_s() : run.world.right_wheel.speed_rad_s();
  }
  return sum / steps;
}

// Theory §3.3: with kff = 0 the closed loop is first order, tau_cl = 20 ms, so the ideal 5%
// settle is 3*tau_cl = 60 ms with no overshoot. The 100 ms / 5% bounds leave room for the
// 5 ms window delay and quantisation.
TEST(SilDrive, StepSettlesWithin5PercentIn100msWithLittleOvershoot) {
  DriveRig run(SimWorld::Config{}, fast_ramp());
  constexpr float kSpeed_m_s = 0.2F;
  const double ref = kSpeed_m_s / kWheelRadius_m;  // 3.81 rad/s
  run.loop.set_command(kSpeed_m_s, 0.0F);

  double peak = 0.0;
  double settle_time_s = -1.0;
  for (int k = 1; k <= 400; ++k) {
    run.advance(1e-3);
    const double w = run.world.left_wheel.speed_rad_s();
    peak = std::max(peak, w);
    const bool inside = std::fabs(w - ref) <= 0.05 * ref;
    if (!inside) {
      settle_time_s = -1.0;
    } else if (settle_time_s < 0.0) {
      settle_time_s = k * 1e-3;
    }
  }
  ASSERT_GE(settle_time_s, 0.0) << "never settled";
  EXPECT_LT(settle_time_s, 0.100);
  EXPECT_LT(peak, 1.05 * ref) << "overshoot under 5%";
}

TEST(SilDrive, TracksTheDefaultAccelerationRamp) {
  DriveRig run;
  run.loop.set_command(0.2F, 0.0F);
  run.advance(0.1);  // halfway up a 0.2 s ramp at 1 m/s^2
  // Theory §4: a first-order closed loop (tau_cl = 20 ms) lags a ramp of slope a by
  // a*tau_cl*(1 - e^{-t/tau_cl}) = 1.0 * 0.02 * (1 - e^-5) = 0.020 m/s at t = 0.1 s.
  const double rim_m_s = run.world.left_wheel.speed_rad_s() * kWheelRadius_m;
  EXPECT_NEAR(rim_m_s, 0.1 - 0.020, 0.01) << "within one encoder count's worth of the lag";
}

TEST(SilDrive, TurnInPlaceGivesEqualAndOppositeWheelSpeeds) {
  DriveRig run(SimWorld::Config{}, fast_ramp());
  run.loop.set_command(0.0F, 1.0F);
  run.advance(0.3);
  const double left = mean_speed(run, 0.1, true);
  const double right = mean_speed(run, 0.1, false);
  const double expected = 1.0 * core::kTrackWidth_m / 2.0 / kWheelRadius_m;
  EXPECT_NEAR(right, expected, 0.02 * expected);
  EXPECT_NEAR(left, -expected, 0.02 * expected);
}

TEST(SilDrive, CarpetLoadIsRejectedWithTheSameGains) {
  // ADR 0013, "One gain set for every surface": extra constant load plus extra viscous drag.
  constexpr float kLoadDuty = 0.05F;
  constexpr float kExtraDrag = 0.5F;
  SimWorld::Config carpet;
  carpet.left.load_duty = kLoadDuty;
  carpet.left.extra_viscous_drag = kExtraDrag;
  DriveRig on_carpet(carpet, fast_ramp());
  DriveRig on_tile(SimWorld::Config{}, fast_ramp());
  for (DriveRig* rig : {&on_carpet, &on_tile}) {
    rig->loop.set_command(0.2F, 0.0F);
    rig->advance(1.0);
  }
  const double ref = 0.2 / kWheelRadius_m;
  EXPECT_NEAR(mean_speed(on_carpet, 0.2), ref, 0.01 * ref) << "integral action: zero steady error";

  // Theory §5: the surface signature is the integrator difference at equal speed,
  // d + c*w/K = 0.05 + 0.5 * 3.81 / 31.4 = 0.111 duty.
  const WheelPlantParams p;
  const double expected = kLoadDuty + kExtraDrag * ref / p.no_load_speed_rad_s;
  const double signature = on_carpet.loop.left().integrator - on_tile.loop.left().integrator;
  EXPECT_NEAR(signature, expected, 0.1 * expected);
}

TEST(SilDrive, ZeroCommandHoldsTheRobotOnASlope) {
  // ADR 0013 Q4. A constant load with nothing commanded is a slope trying to roll the wheel.
  SimWorld::Config slope;
  slope.left.load_duty = 0.05F;
  DriveRig run(slope);
  run.advance(1.0);
  EXPECT_NEAR(mean_speed(run, 0.2), 0.0, 0.05);
  EXPECT_NEAR(run.loop.left().integrator, 0.05F, 0.01F) << "holding duty equals the slope";
}

TEST(SilDrive, NotArmedNeverMovesTheWheels) {
  DriveRig run;
  run.loop.set_command(0.3F, 0.5F);
  run.advance(1.0, /*armed=*/false);
  EXPECT_EQ(run.world.left_wheel.speed_rad_s(), 0.0);
  EXPECT_EQ(run.world.right_wheel.speed_rad_s(), 0.0);
}

/// Plant mismatch: the gains were derived for K = 31.4, tau = 0.05. Each case changes the
/// plant by 50%, and the loop must still settle to the reference without oscillating.
class SilDrivePlantMismatch : public ::testing::TestWithParam<std::pair<float, float>> {};

TEST_P(SilDrivePlantMismatch, StaysStableAndSettles) {
  const auto [k_scale, tau_scale] = GetParam();
  SimWorld::Config cfg;
  cfg.left.no_load_speed_rad_s *= k_scale;
  cfg.left.drive_tau_s *= tau_scale;
  DriveRig run(cfg, fast_ramp());
  run.loop.set_command(0.2F, 0.0F);
  const double ref = 0.2 / kWheelRadius_m;

  double peak = 0.0;
  for (int k = 0; k < 500; ++k) {
    run.advance(1e-3);
    peak = std::max(peak, run.world.left_wheel.speed_rad_s());
  }
  EXPECT_LT(peak, 1.3 * ref) << "bounded overshoot";
  EXPECT_NEAR(mean_speed(run, 0.2), ref, 0.02 * ref) << "still converges";
}

INSTANTIATE_TEST_SUITE_P(PlusMinus50Percent, SilDrivePlantMismatch,
                         ::testing::Values(std::make_pair(0.5F, 1.0F), std::make_pair(1.5F, 1.0F),
                                           std::make_pair(1.0F, 0.5F), std::make_pair(1.0F, 1.5F),
                                           std::make_pair(0.5F, 1.5F), std::make_pair(1.5F, 0.5F)));

}  // namespace
}  // namespace recon::sim
