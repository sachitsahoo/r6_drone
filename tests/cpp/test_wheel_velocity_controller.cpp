// WheelVelocityController: ADR 0013 "Controller".

#include "control/wheel_velocity_controller.hpp"

#include <gtest/gtest.h>

namespace recon::core {
namespace {

constexpr float kDt = 1e-3F;

WheelVelocityGains gains(float kff, float kp, float ki, float limit) {
  WheelVelocityGains g;
  g.kff = kff;
  g.kp = kp;
  g.ki = ki;
  g.duty_limit = limit;
  return g;
}

TEST(WheelVelocityController, DefaultsComeFromTheSchema) {
  const WheelVelocityGains g;
  EXPECT_FLOAT_EQ(g.kff, 0.0F) << "ADR 0013 amendment: feedforward off by default";
  EXPECT_FLOAT_EQ(g.kp, 0.0796F);
  EXPECT_FLOAT_EQ(g.ki, 1.59F);
  EXPECT_FLOAT_EQ(g.duty_limit, 0.3F);
}

TEST(WheelVelocityController, ZeroErrorOutputsFeedforwardOnly) {
  WheelVelocityController c(gains(0.03F, 0.1F, 2.0F, 1.0F));
  EXPECT_FLOAT_EQ(c.update(5.0F, 5.0F, kDt), 0.15F);
  EXPECT_FLOAT_EQ(c.integrator(), 0.0F);
}

TEST(WheelVelocityController, ProportionalActsOnError) {
  WheelVelocityController c(gains(0.0F, 0.1F, 0.0F, 1.0F));
  EXPECT_FLOAT_EQ(c.update(2.0F, 1.0F, kDt), 0.1F);
}

TEST(WheelVelocityController, IntegratorAccumulatesInDutyUnits) {
  WheelVelocityController c(gains(0.0F, 0.0F, 2.0F, 1.0F));
  for (int i = 0; i < 100; ++i) {
    c.update(1.0F, 0.0F, kDt);  // e = 1 rad/s for 0.1 s
  }
  EXPECT_NEAR(c.integrator(), 2.0F * 1.0F * 0.1F, 1e-5F);
}

TEST(WheelVelocityController, OutputSaturatesAtTheLimitWithoutScaling) {
  WheelVelocityController c(gains(0.1F, 0.0F, 0.0F, 0.3F));
  EXPECT_FLOAT_EQ(c.update(10.0F, 10.0F, kDt), 0.3F);
  EXPECT_TRUE(c.saturated());
  EXPECT_FLOAT_EQ(c.update(-10.0F, -10.0F, kDt), -0.3F);
  EXPECT_FLOAT_EQ(c.update(1.0F, 1.0F, kDt), 0.1F) << "inside the limit: unchanged, not scaled";
  EXPECT_FALSE(c.saturated());
}

TEST(WheelVelocityController, IntegratorFreezesWhileSaturatedInTheErrorsDirection) {
  WheelVelocityController c(gains(0.0F, 0.0F, 2.0F, 0.3F));
  for (int i = 0; i < 1000; ++i) {
    c.update(100.0F, 0.0F, kDt);  // a stalled wheel: error never shrinks
  }
  EXPECT_LE(c.integrator(), 0.3F) << "no windup past the limit";
}

TEST(WheelVelocityController, IntegratorUnwindsWhenTheErrorReverses) {
  WheelVelocityController c(gains(0.0F, 0.0F, 2.0F, 0.3F));
  for (int i = 0; i < 1000; ++i) {
    c.update(100.0F, 0.0F, kDt);
  }
  const float wound = c.integrator();
  c.update(0.0F, 1.0F, kDt);  // now over speed
  EXPECT_LT(c.integrator(), wound) << "saturated, but this error pulls it back: integrate";
}

TEST(WheelVelocityController, ChangingKiDoesNotStepTheOutput) {
  WheelVelocityController c(gains(0.0F, 0.0F, 2.0F, 1.0F));
  for (int i = 0; i < 100; ++i) {
    c.update(1.0F, 0.0F, kDt);
  }
  const float before = c.update(1.0F, 1.0F, kDt);  // e = 0: output is the integrator
  c.set_gains(gains(0.0F, 0.0F, 20.0F, 1.0F));
  const float after = c.update(1.0F, 1.0F, kDt);
  EXPECT_FLOAT_EQ(after, before) << "bumpless: stored I is duty, not integral(e)";
}

TEST(WheelVelocityController, ZeroKiFreezesTheIntegrator) {
  WheelVelocityController c(gains(0.0F, 0.0F, 2.0F, 1.0F));
  for (int i = 0; i < 100; ++i) {
    c.update(1.0F, 0.0F, kDt);
  }
  const float held = c.integrator();
  c.set_gains(gains(0.0F, 0.0F, 0.0F, 1.0F));
  c.update(1.0F, 0.0F, kDt);
  EXPECT_FLOAT_EQ(c.integrator(), held);
}

TEST(WheelVelocityController, LoweringTheLimitBoundsAStoredIntegrator) {
  WheelVelocityController c(gains(0.0F, 0.0F, 2.0F, 1.0F));
  for (int i = 0; i < 1000; ++i) {
    c.update(1.0F, 0.0F, kDt);
  }
  c.set_gains(gains(0.0F, 0.0F, 2.0F, 0.2F));
  EXPECT_LE(c.update(1.0F, 1.0F, kDt), 0.2F);
  EXPECT_LE(c.integrator(), 0.2F);
}

TEST(WheelVelocityController, ResetZeroesTheIntegrator) {
  WheelVelocityController c(gains(0.0F, 0.0F, 2.0F, 1.0F));
  c.update(1.0F, 0.0F, 0.1F);
  c.reset();
  EXPECT_EQ(c.integrator(), 0.0F);
}

TEST(WheelVelocityController, FeedforwardOnlyIsClampedAndLeavesTheIntegrator) {
  WheelVelocityController c(gains(0.1F, 0.0F, 2.0F, 0.3F));
  c.update(1.0F, 0.0F, 0.1F);
  const float held = c.integrator();
  EXPECT_FLOAT_EQ(c.feedforward_only(10.0F), 0.3F);
  EXPECT_FLOAT_EQ(c.feedforward_only(1.0F), 0.1F);
  EXPECT_FLOAT_EQ(c.integrator(), held);
}

}  // namespace
}  // namespace recon::core
