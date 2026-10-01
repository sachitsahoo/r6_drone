// The fakes are what every core unit test will stand on, so their behaviour is pinned here.
// Each test talks to the fake through its hal:: base, exactly as core will.

#include <gtest/gtest.h>

#include <cstdint>

#include "hal_fakes.hpp"
#include "time/timestamp.hpp"

namespace recon::test {
namespace {

TEST(FakeClock, AdvancesAndWrapsLikeTheRealClock) {
  FakeClock fake;
  const hal::Clock& clock = fake;
  fake.set_us(0xFFFFFF00U);
  const uint32_t before = clock.now_us();
  fake.advance_us(0x200U);  // crosses 2^32
  EXPECT_EQ(clock.now_us(), 0x100U);
  EXPECT_EQ(core::elapsed_us(before, clock.now_us()), 0x200U);
}

TEST(FakeSerialPort, WriteReportsPartialAcceptanceWhenFull) {
  FakeSerialPort fake;
  hal::SerialPort& port = fake;
  uint8_t block[FakeSerialPort::kCapacity] = {};
  EXPECT_EQ(port.write(block, sizeof block - 10), sizeof block - 10);
  EXPECT_EQ(port.write(block, 50), 10U) << "only the remaining space is accepted";
  EXPECT_EQ(port.write(block, 1), 0U);
}

TEST(FakeSerialPort, ReadDrainsInjectedBytesInOrder) {
  FakeSerialPort fake;
  hal::SerialPort& port = fake;
  const uint8_t in[] = {1, 2, 3, 4};
  fake.inject_rx(in, sizeof in);
  uint8_t out[3] = {};
  ASSERT_EQ(port.read(out, sizeof out), 3U);
  EXPECT_EQ(out[0], 1);
  EXPECT_EQ(out[2], 3);
  ASSERT_EQ(port.read(out, sizeof out), 1U);
  EXPECT_EQ(out[0], 4);
  EXPECT_EQ(port.read(out, sizeof out), 0U) << "never blocks when empty";
}

TEST(FakeSerialPort, OverflowIsCountedNotSilent) {
  FakeSerialPort fake;
  uint8_t block[FakeSerialPort::kCapacity + 5] = {};
  fake.inject_rx(block, sizeof block);
  EXPECT_EQ(static_cast<const hal::SerialPort&>(fake).rx_overflow_count(), 5U);
}

TEST(FakeImu, DefaultSampleIsInvalid) {
  // A fake that started "valid" would let a core test pass without ever providing data.
  FakeImu fake;
  EXPECT_FALSE(static_cast<const hal::Imu&>(fake).latest().valid);
}

TEST(FakeImu, ReturnsTheSampleItWasGiven) {
  FakeImu fake;
  hal::ImuSample s{};
  s.gyro_rad_s[1] = 0.5F;  // pitch rate, positive nose-down
  s.accel_m_s2[2] = 9.81F;
  s.timestamp_us = 1234;
  s.valid = true;
  fake.set(s);
  const hal::ImuSample got = static_cast<const hal::Imu&>(fake).latest();
  EXPECT_FLOAT_EQ(got.gyro_rad_s[1], 0.5F);
  EXPECT_EQ(got.timestamp_us, 1234U);
  EXPECT_TRUE(got.valid);
}

TEST(FakeWheelEncoder, CountDifferencesSurviveWrap) {
  // The contract core relies on (hard rule 7): unsigned subtraction of two counts gives the
  // true delta even when the counter wrapped between them.
  FakeWheelEncoder fake(1000.0F);
  const hal::WheelEncoder& enc = fake;
  fake.set_count(0xFFFFFFF0U);
  const uint32_t before = enc.count();
  fake.set_count(0x10U);
  EXPECT_EQ(enc.count() - before, 0x20U);
  EXPECT_FLOAT_EQ(enc.counts_per_rev(), 1000.0F);
}

TEST(FakeWheelMotor, RecordsCommandsWithoutClamping) {
  FakeWheelMotor fake;
  hal::WheelMotor& motor = fake;
  EXPECT_TRUE(fake.stopped()) << "never-commanded motor is stopped";
  motor.set_duty(1.7F);
  EXPECT_FLOAT_EQ(fake.last_duty(), 1.7F) << "clamping is the implementation's job, so core "
                                             "bugs that exceed range stay visible";
  motor.stop(hal::StopMode::kBrake);
  EXPECT_TRUE(fake.stopped());
  EXPECT_EQ(fake.last_stop_mode(), hal::StopMode::kBrake);
}

TEST(FakePitchPowerStage, StartsDisabledAndRecordsDuties) {
  FakePitchPowerStage fake;
  hal::PitchPowerStage& stage = fake;
  EXPECT_FALSE(fake.enabled()) << "power stage must start disabled";
  stage.set_enabled(true);
  stage.set_phase_duties(0.25F, 0.5F, 0.75F);
  EXPECT_TRUE(fake.enabled());
  EXPECT_FLOAT_EQ(fake.duties()[2], 0.75F);
  fake.set_fault(true);
  EXPECT_TRUE(stage.fault());
}

TEST(FakePitchPowerStage, CurrentsAreNeverValidWithoutCurrentSense) {
  hal::PhaseCurrents c{};
  c.a_A = 0.3F;
  c.valid = true;

  FakePitchPowerStage v1(false);
  v1.set_currents(c);
  EXPECT_FALSE(static_cast<hal::PitchPowerStage&>(v1).has_current_sense());
  EXPECT_FALSE(static_cast<hal::PitchPowerStage&>(v1).latest_currents().valid);

  FakePitchPowerStage v23(true);
  v23.set_currents(c);
  EXPECT_TRUE(static_cast<hal::PitchPowerStage&>(v23).latest_currents().valid);
}

TEST(FakePowerMonitor, ReturnsTheSampleItWasGiven) {
  FakePowerMonitor fake;
  hal::PowerSample s{};
  s.bus_voltage_V = 11.1F;
  s.current_A = 0.4F;
  s.valid = true;
  fake.set(s);
  EXPECT_FLOAT_EQ(static_cast<const hal::PowerMonitor&>(fake).latest().bus_voltage_V, 11.1F);
}

}  // namespace
}  // namespace recon::test
