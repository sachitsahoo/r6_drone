// Unit tests for the simulated HAL in sim/. Each component is driven through its hal:: base
// where it has one, exactly as core will drive it.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "sim/sim_clock.hpp"
#include "sim/sim_serial_link.hpp"
#include "sim/sim_world.hpp"
#include "sim/static_sensors.hpp"
#include "sim/wheel_plant.hpp"
#include "time/timestamp.hpp"

namespace recon::sim {
namespace {

constexpr double kTwoPi = 6.283185307179586;

// 460 800 baud, 10 bits per byte: one byte every 1e7 / 460800 = 21.70 us.
constexpr uint32_t kByteTimeFloorUs = 21;
constexpr uint32_t kByteTimeCeilUs = 22;

std::vector<uint8_t> drain(hal::SerialPort& port) {
  std::vector<uint8_t> out;
  uint8_t buf[64];
  size_t n = 0;
  while ((n = port.read(buf, sizeof buf)) > 0) {
    out.insert(out.end(), buf, buf + n);
  }
  return out;
}

// ------------------------------------------------------------------------- SimClock

TEST(SimClock, StartsWhereToldAndWrapsAtTwoToThe32) {
  SimClock sim(0xFFFFFF00U);
  const hal::Clock& clock = sim;
  const uint32_t before = clock.now_us();
  sim.advance_us(0x200U);
  EXPECT_EQ(clock.now_us(), 0x100U);
  EXPECT_EQ(core::elapsed_us(before, clock.now_us()), 0x200U);
}

// --------------------------------------------------------------------- SimSerialLink

TEST(SimSerialLink, AByteTakesOneUartByteTimeToArrive) {
  SimSerialLink link;
  const uint8_t byte = 0xA5;
  ASSERT_EQ(link.mcu().write(&byte, 1), 1U);
  link.step(kByteTimeFloorUs);
  EXPECT_EQ(link.host().rx_pending(), 0U) << "21 us is less than one 21.7 us byte time";
  link.step(kByteTimeCeilUs - kByteTimeFloorUs);
  ASSERT_EQ(link.host().rx_pending(), 1U);
  EXPECT_EQ(drain(link.host()), std::vector<uint8_t>{0xA5});
}

TEST(SimSerialLink, ThroughputDoesNotDependOnStepSize) {
  SimSerialLink coarse;
  SimSerialLink fine;
  const std::vector<uint8_t> block(200, 0x55);
  coarse.mcu().write(block.data(), block.size());
  fine.mcu().write(block.data(), block.size());

  constexpr uint32_t kSpanUs = 1000;  // 46.08 byte times
  coarse.step(kSpanUs);
  for (uint32_t i = 0; i < kSpanUs; ++i) {
    fine.step(1);
  }
  EXPECT_EQ(coarse.host().rx_pending(), 46U);
  EXPECT_EQ(fine.host().rx_pending(), 46U);
}

TEST(SimSerialLink, DeliversInOrderInBothDirections) {
  SimSerialLink link;
  std::vector<uint8_t> up(100);
  std::vector<uint8_t> down(100);
  for (size_t i = 0; i < up.size(); ++i) {
    up[i] = static_cast<uint8_t>(i);
    down[i] = static_cast<uint8_t>(255 - i);
  }
  link.mcu().write(up.data(), up.size());
  link.host().write(down.data(), down.size());
  link.step(10000);
  EXPECT_EQ(drain(link.host()), up);
  EXPECT_EQ(drain(link.mcu()), down);
}

TEST(SimSerialLink, AnIdleLineDoesNotBankCredit) {
  SimSerialLink link;
  link.step(1000000);  // a second with nothing to send
  const uint8_t bytes[10] = {};
  link.mcu().write(bytes, sizeof bytes);
  link.step(0);
  EXPECT_EQ(link.host().rx_pending(), 0U) << "idle time must not let a burst skip the line rate";
}

TEST(SimSerialLink, WriteReportsPartialAcceptanceWhenTxIsFull) {
  SimSerialLink link;
  hal::SerialPort& port = link.mcu();
  const std::vector<uint8_t> block(kSimTxBufferBytes + 40, 0);
  EXPECT_EQ(port.write(block.data(), block.size()), kSimTxBufferBytes);
  EXPECT_EQ(port.write(block.data(), 1), 0U);
}

TEST(SimSerialLink, AnUnreadReceiverOverflowsAndCountsTheLoss) {
  SimSerialLink link;
  const std::vector<uint8_t> block(kSimRxBufferBytes, 0x11);
  link.mcu().write(block.data(), block.size());
  link.step(1000000);
  ASSERT_EQ(link.host().rx_pending(), kSimRxBufferBytes);
  EXPECT_EQ(link.host().rx_overflow_count(), 0U);

  const uint8_t extra[10] = {};
  link.mcu().write(extra, sizeof extra);
  link.step(1000000);
  const hal::SerialPort& host = link.host();
  EXPECT_EQ(host.rx_overflow_count(), 10U);
  EXPECT_EQ(link.host().rx_pending(), kSimRxBufferBytes) << "old bytes kept, new ones lost";
}

TEST(SimSerialLink, ErrorRateOneFlipsExactlyOneBitInEveryByte) {
  SimSerialLink::Config config;
  config.byte_error_rate = 1.0F;
  SimSerialLink link(config);
  const std::vector<uint8_t> sent(100, 0x00);
  link.mcu().write(sent.data(), sent.size());
  link.step(100000);
  const auto got = drain(link.host());
  ASSERT_EQ(got.size(), sent.size());
  for (uint8_t b : got) {
    EXPECT_EQ(__builtin_popcount(b), 1) << "one flipped bit from 0x00";
  }
  EXPECT_EQ(link.corrupted_bytes(), 100U);
}

TEST(SimSerialLink, ErrorsAreDeterministicForAGivenSeed) {
  SimSerialLink::Config config;
  config.byte_error_rate = 0.1F;
  config.seed = 1234;
  SimSerialLink a(config);
  SimSerialLink b(config);
  const std::vector<uint8_t> sent(250, 0x5A);
  a.mcu().write(sent.data(), sent.size());
  b.mcu().write(sent.data(), sent.size());
  a.step(100000);
  b.step(100000);
  EXPECT_GT(a.corrupted_bytes(), 0U);
  EXPECT_EQ(drain(a.host()), drain(b.host()));
}

TEST(SimSerialLink, ZeroSeedStillProducesErrors) {
  SimSerialLink::Config config;
  config.byte_error_rate = 0.5F;
  config.seed = 0;  // xorshift's fixed point; must be replaced, not used
  SimSerialLink link(config);
  const std::vector<uint8_t> sent(200, 0);
  link.mcu().write(sent.data(), sent.size());
  link.step(100000);
  EXPECT_GT(link.corrupted_bytes(), 0U);
  EXPECT_LT(link.corrupted_bytes(), 200U);
}

// -------------------------------------------------------------------------- SimWheel

WheelPlantParams test_params() {
  WheelPlantParams p;
  p.output_limit = 1.0F;  // most plant tests want the full range
  return p;
}

TEST(SimWheel, ANeverCommandedWheelStaysPut) {
  SimWheel wheel(test_params());
  wheel.step(1.0);
  EXPECT_EQ(wheel.speed_rad_s(), 0.0);
  EXPECT_EQ(static_cast<const hal::WheelEncoder&>(wheel).count(), 0U);
}

TEST(SimWheel, ReachesDutyTimesNoLoadSpeedInSteadyState) {
  const auto p = test_params();
  SimWheel wheel(p);
  hal::WheelMotor& motor = wheel;
  motor.set_duty(0.5F);
  wheel.step(20 * p.drive_tau_s);  // e^-20: indistinguishable from steady state
  EXPECT_NEAR(wheel.speed_rad_s(), 0.5 * p.no_load_speed_rad_s, 1e-6);
}

TEST(SimWheel, FollowsTheFirstOrderStepResponse) {
  const auto p = test_params();
  SimWheel wheel(p);
  wheel.set_duty(1.0F);
  wheel.step(p.drive_tau_s);  // one time constant
  EXPECT_NEAR(wheel.speed_rad_s(), p.no_load_speed_rad_s * (1.0 - std::exp(-1.0)), 1e-4);
}

TEST(SimWheel, ResultDoesNotDependOnStepSize) {
  const auto p = test_params();
  SimWheel coarse(p);
  SimWheel fine(p);
  coarse.set_duty(0.7F);
  fine.set_duty(0.7F);
  coarse.step(0.1);
  for (int i = 0; i < 1000; ++i) {
    fine.step(1e-4);
  }
  EXPECT_NEAR(coarse.speed_rad_s(), fine.speed_rad_s(), 1e-9);
  EXPECT_NEAR(static_cast<double>(coarse.count()), static_cast<double>(fine.count()), 1.0);
}

TEST(SimWheel, EncoderCountsMatchTheIntegratedAngle) {
  const auto p = test_params();
  SimWheel wheel(p);
  const hal::WheelEncoder& encoder = wheel;
  wheel.set_duty(1.0F);
  constexpr double kT = 0.3;
  wheel.step(kT);
  // From rest: angle(T) = w*T - w*tau*(1 - e^{-T/tau}).
  const double w = p.no_load_speed_rad_s;
  const double tau = p.drive_tau_s;
  const double angle = w * kT - w * tau * (1.0 - std::exp(-kT / tau));
  const double expected = std::floor(angle / kTwoPi * p.counts_per_rev);
  EXPECT_EQ(encoder.count(), static_cast<uint32_t>(expected));
  EXPECT_FLOAT_EQ(encoder.counts_per_rev(), p.counts_per_rev);
}

TEST(SimWheel, ClampsThenAppliesTheOutputLimit) {
  WheelPlantParams p;  // default limit: the low placeholder
  SimWheel wheel(p);
  hal::WheelMotor& motor = wheel;
  motor.set_duty(5.0F);
  EXPECT_FLOAT_EQ(wheel.applied_duty(), p.output_limit);
  motor.set_duty(-5.0F);
  EXPECT_FLOAT_EQ(wheel.applied_duty(), -p.output_limit);
  motor.set_duty(0.1F);
  EXPECT_FLOAT_EQ(wheel.applied_duty(), 0.1F) << "inside the limit passes through unchanged";
}

TEST(SimWheel, NanDutyCoastsInsteadOfDriving) {
  SimWheel wheel(test_params());
  wheel.set_duty(std::numeric_limits<float>::quiet_NaN());
  EXPECT_EQ(wheel.applied_duty(), 0.0F);
  wheel.step(1.0);
  EXPECT_EQ(wheel.speed_rad_s(), 0.0);
}

TEST(SimWheel, BrakeStopsFasterThanCoast) {
  const auto p = test_params();
  SimWheel coasting(p);
  SimWheel braking(p);
  for (SimWheel* w : {&coasting, &braking}) {
    w->set_duty(1.0F);
    w->step(1.0);
  }
  coasting.stop(hal::StopMode::kCoast);
  braking.stop(hal::StopMode::kBrake);
  coasting.step(0.1);
  braking.step(0.1);
  EXPECT_LT(braking.speed_rad_s(), 0.2 * coasting.speed_rad_s());
  EXPECT_GT(coasting.speed_rad_s(), 0.0) << "a coasting wheel keeps rolling";
}

TEST(SimWheel, RollingBackwardThroughZeroWrapsTheCounter) {
  auto p = test_params();
  p.initial_count = 5;
  SimWheel wheel(p);
  const hal::WheelEncoder& encoder = wheel;
  const uint32_t before = encoder.count();
  wheel.set_duty(-1.0F);
  wheel.step(0.05);
  const uint32_t after = encoder.count();
  EXPECT_GT(after, 0x80000000U) << "counter wrapped below zero";
  // Unsigned subtraction reinterpreted as signed recovers the true (negative) travel.
  const auto delta = static_cast<int32_t>(after - before);
  EXPECT_LT(delta, 0);
  EXPECT_GT(delta, -1000);
}

TEST(SimWheel, ForwardWrapAtTwoToThe32IsSeamless) {
  auto p = test_params();
  p.initial_count = 0xFFFFFFF0U;
  SimWheel wheel(p);
  wheel.set_duty(1.0F);
  wheel.step(0.1);
  const uint32_t after = wheel.count();
  EXPECT_LT(after, 0x1000U) << "counter wrapped past 2^32";
  EXPECT_GT(after - p.initial_count, 16U);
}

// ------------------------------------------------------------------- static sensors

TEST(SimImu, DefaultsToLevelAndStillAndStampsWithTheClock) {
  SimClock clock(1234);
  SimImu sim(clock);
  const hal::Imu& imu = sim;
  auto s = imu.latest();
  EXPECT_TRUE(s.valid);
  EXPECT_FLOAT_EQ(s.accel_m_s2[2], kStandardGravity_m_s2);
  EXPECT_FLOAT_EQ(s.gyro_rad_s[1], 0.0F);
  EXPECT_EQ(s.timestamp_us, 1234U);
  clock.advance_us(1000);
  EXPECT_EQ(imu.latest().timestamp_us, 2234U);
}

TEST(SimAbsoluteEncoder, WrapsAnyAngleIntoOneTurn) {
  SimClock clock;
  SimAbsoluteEncoder sim(clock);
  const hal::AbsoluteEncoder& enc = sim;
  sim.set_angle_rad(-0.1F);
  EXPECT_NEAR(enc.latest().angle_rad, kTwoPi - 0.1, 1e-5);
  sim.set_angle_rad(7.0F);
  EXPECT_NEAR(enc.latest().angle_rad, 7.0 - kTwoPi, 1e-5);
  sim.set_angle_rad(static_cast<float>(-4.0 * kTwoPi));
  const float a = enc.latest().angle_rad;
  EXPECT_GE(a, 0.0F);
  EXPECT_LT(a, static_cast<float>(kTwoPi));
}

TEST(SimPowerMonitor, DefaultsToNominal3s) {
  SimClock clock;
  SimPowerMonitor sim(clock);
  const hal::PowerMonitor& pm = sim;
  EXPECT_FLOAT_EQ(pm.latest().bus_voltage_V, kNominal3sVoltage_V);
  EXPECT_TRUE(pm.latest().valid);
}

TEST(SimPitchPowerStage, ClampsDutiesAndStartsDisabled) {
  SimClock clock;
  SimPitchPowerStage sim(clock);
  hal::PitchPowerStage& stage = sim;
  EXPECT_FALSE(sim.enabled());
  stage.set_phase_duties(-0.5F, 0.4F, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FLOAT_EQ(sim.duties()[0], 0.0F);
  EXPECT_FLOAT_EQ(sim.duties()[1], 0.4F);
  EXPECT_FLOAT_EQ(sim.duties()[2], 0.0F);
  stage.set_phase_duties(2.0F, 1.0F, 0.0F);
  EXPECT_FLOAT_EQ(sim.duties()[0], 1.0F);
  EXPECT_FALSE(stage.latest_currents().valid) << "no current sense by default";
}

// ------------------------------------------------------------------------- SimWorld

TEST(SimWorld, StepAdvancesClockLinkAndBothWheels) {
  SimWorld world;
  world.left_wheel.set_duty(0.2F);
  world.right_wheel.set_duty(-0.2F);
  const uint8_t b = 7;
  world.link.mcu().write(&b, 1);
  world.step(10000);
  EXPECT_EQ(world.clock.now_us(), 10000U);
  EXPECT_EQ(world.link.host().rx_pending(), 1U);
  EXPECT_GT(world.left_wheel.speed_rad_s(), 0.0);
  EXPECT_LT(world.right_wheel.speed_rad_s(), 0.0);
}

}  // namespace
}  // namespace recon::sim
