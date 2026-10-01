// Software-in-the-loop: real core code (the frame codec) running against the simulated HAL.
//
// The robot side of each test talks only to hal:: interfaces, as firmware will. The
// operator side stands in for the Pi relay plus the laptop. No control law runs here: the
// motor-control, estimation and safety designs are owner-reviewed and not yet approved, so
// the wheel is driven open-loop with a fixed duty.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include "messages.hpp"
#include "protocol/frame.hpp"
#include "sim/sim_world.hpp"
#include "time/timestamp.hpp"

namespace recon::sim {
namespace {

using protocol::DecodedFrame;
using protocol::DriveCommand;
using protocol::FrameDecoder;
using protocol::MessageId;
using protocol::StateTelemetry;

/// World step for these tests. 100 us resolves a 21.7 us byte time to within five bytes,
/// which is fine for frame-level timing.
constexpr uint32_t kStepUs = 100;
/// Robot main-loop poll period: the 1 kHz motor loop rate (CLAUDE.md).
constexpr uint32_t kRobotLoopUs = 1000;
/// Operator command period: ~50 Hz (CLAUDE.md).
constexpr uint32_t kCommandPeriodUs = 20000;
/// Telemetry period: 100 Hz, the top of the 50-100 Hz range (CLAUDE.md).
constexpr uint32_t kTelemetryPeriodUs = 10000;
/// 105 mm wheel (ADR 0008).
constexpr double kWheelRadius_m = 0.0525;
constexpr double kTwoPi = 6.283185307179586;

/// Encodes and writes one frame. \return False if the TX buffer could not take all of it.
template <typename Msg>
bool send(hal::SerialPort& port, MessageId id, uint8_t seq, uint32_t t_us, const Msg& msg) {
  uint8_t payload[Msg::kPayloadBytes] = {};
  msg.encode(payload);
  uint8_t wire[protocol::kMaxWireFrameBytes] = {};
  size_t len = 0;
  if (!protocol::encode_frame(static_cast<uint8_t>(id), seq, t_us, payload, sizeof payload, wire,
                              sizeof wire, len)) {
    return false;
  }
  return port.write(wire, len) == len;
}

/// Drains `port` through `decoder`, appending every frame that validates.
void receive(hal::SerialPort& port, FrameDecoder& decoder, std::vector<DecodedFrame>& out) {
  uint8_t buf[64];
  size_t n = 0;
  DecodedFrame frame{};
  while ((n = port.read(buf, sizeof buf)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (decoder.push_byte(buf[i], frame)) {
        out.push_back(frame);
      }
    }
  }
}

TEST(SilLink, DriveCommandsAt50HzArriveIntactInOrderAndOnTime) {
  SimWorld world;
  hal::SerialPort& robot_port = world.link.mcu();
  hal::SerialPort& operator_port = world.link.host();
  const hal::Clock& clock = world.clock;
  FrameDecoder robot_decoder;

  std::vector<DecodedFrame> received;
  std::vector<uint32_t> received_at_us;
  constexpr int kCommands = 50;  // one second
  int sent = 0;

  for (uint32_t t = 0; t <= 1100000; t += kStepUs) {
    if (t % kCommandPeriodUs == 0 && sent < kCommands) {
      DriveCommand cmd{};
      cmd.cmd_linear_speed_m_s = 0.01F * static_cast<float>(sent);
      cmd.cmd_angular_rate_rad_s = -0.02F * static_cast<float>(sent);
      ASSERT_TRUE(send(operator_port, MessageId::DriveCommand, static_cast<uint8_t>(sent),
                       clock.now_us(), cmd));
      ++sent;
    }
    world.step(kStepUs);
    if (clock.now_us() % kRobotLoopUs == 0) {
      const size_t before = received.size();
      receive(robot_port, robot_decoder, received);
      received_at_us.insert(received_at_us.end(), received.size() - before, clock.now_us());
    }
  }

  ASSERT_EQ(received.size(), static_cast<size_t>(kCommands));
  for (int i = 0; i < kCommands; ++i) {
    const DecodedFrame& f = received[static_cast<size_t>(i)];
    ASSERT_EQ(f.message_id, static_cast<uint8_t>(MessageId::DriveCommand));
    EXPECT_EQ(f.seq, static_cast<uint8_t>(i));
    DriveCommand cmd{};
    ASSERT_TRUE(DriveCommand::decode(f.payload, f.payload_len, cmd));
    EXPECT_FLOAT_EQ(cmd.cmd_linear_speed_m_s, 0.01F * static_cast<float>(i));
    EXPECT_FLOAT_EQ(cmd.cmd_angular_rate_rad_s, -0.02F * static_cast<float>(i));
    // Latency budget: a DriveCommand frame is < 25 bytes = < 0.55 ms on the wire, plus up to
    // one robot loop period of polling delay. 2 ms leaves margin without hiding a stall.
    EXPECT_LE(core::elapsed_us(f.timestamp_us, received_at_us[static_cast<size_t>(i)]), 2000U);
  }
  const auto& stats = robot_decoder.stats();
  EXPECT_EQ(stats.frames_ok, static_cast<uint32_t>(kCommands));
  EXPECT_EQ(stats.crc_errors + stats.cobs_errors + stats.length_mismatch + stats.unknown_id +
                stats.version_mismatch + stats.desyncs,
            0U);
  EXPECT_EQ(robot_port.rx_overflow_count(), 0U);
}

TEST(SilLink, BitErrorsNeverProduceAWrongCommand) {
  SimWorld::Config config;
  config.link.byte_error_rate = 2e-3F;  // far worse than a healthy UART; exercises rejection
  config.link.seed = 42;
  SimWorld world(config);
  hal::SerialPort& robot_port = world.link.mcu();
  hal::SerialPort& operator_port = world.link.host();
  FrameDecoder robot_decoder;

  // Keyed by timestamp, which is unique per frame here (seq wraps after 256).
  std::map<uint32_t, DriveCommand> sent_by_ts;
  std::vector<DecodedFrame> received;
  constexpr int kCommands = 1000;  // 20 s at 50 Hz
  int sent = 0;

  for (uint32_t t = 0; sent < kCommands || t % kCommandPeriodUs != 0; t += kStepUs) {
    if (t % kCommandPeriodUs == 0 && sent < kCommands) {
      DriveCommand cmd{};
      cmd.cmd_linear_speed_m_s = 0.001F * static_cast<float>(sent);
      cmd.cmd_angular_rate_rad_s = 0.5F;
      ASSERT_TRUE(send(operator_port, MessageId::DriveCommand, static_cast<uint8_t>(sent),
                       world.clock.now_us(), cmd));
      sent_by_ts[world.clock.now_us()] = cmd;
      ++sent;
    }
    world.step(kStepUs);
    receive(robot_port, robot_decoder, received);
  }

  ASSERT_GT(world.link.corrupted_bytes(), 0U) << "the test must actually inject errors";
  ASSERT_LT(received.size(), static_cast<size_t>(kCommands)) << "some frames must be lost";
  // ~20 wire bytes per frame at 2e-3 per byte loses ~4% directly, plus at most one more per
  // corrupted delimiter (FrameDecoder resync guarantee). 85% is a loose floor.
  EXPECT_GT(received.size(), static_cast<size_t>(kCommands * 85 / 100));

  for (const DecodedFrame& f : received) {
    const auto it = sent_by_ts.find(f.timestamp_us);
    ASSERT_NE(it, sent_by_ts.end()) << "accepted a frame whose header was corrupted";
    DriveCommand cmd{};
    ASSERT_TRUE(DriveCommand::decode(f.payload, f.payload_len, cmd));
    EXPECT_EQ(cmd.cmd_linear_speed_m_s, it->second.cmd_linear_speed_m_s);
    EXPECT_EQ(cmd.cmd_angular_rate_rad_s, it->second.cmd_angular_rate_rad_s);
  }

  const auto& stats = robot_decoder.stats();
  EXPECT_EQ(stats.frames_ok, received.size());
  EXPECT_GT(stats.crc_errors + stats.cobs_errors + stats.length_mismatch + stats.unknown_id +
                stats.version_mismatch,
            0U)
      << "every lost frame is visible in a counter, never silently dropped";
}

TEST(SilLink, TelemetryBuiltFromTheSimulatedEncoderReachesTheOperatorAcrossAClockWrap) {
  SimWorld::Config config;
  config.start_us = 0xFFFFFFFFU - 200000U;  // wraps 0.2 s into a 1 s run
  SimWorld world(config);
  hal::SerialPort& robot_port = world.link.mcu();
  hal::SerialPort& operator_port = world.link.host();
  hal::WheelMotor& motor = world.left_wheel;
  const hal::WheelEncoder& encoder = world.left_wheel;
  const hal::Clock& clock = world.clock;
  FrameDecoder operator_decoder;

  constexpr float kDuty = 0.2F;  // within the default output limit
  motor.set_duty(kDuty);

  std::vector<DecodedFrame> received;
  uint32_t last_count = encoder.count();
  uint32_t last_telemetry_us = clock.now_us();
  uint8_t seq = 0;
  const uint32_t start_us = clock.now_us();

  while (core::elapsed_us(start_us, clock.now_us()) < 1000000U) {
    world.step(kStepUs);
    if (core::elapsed_us(last_telemetry_us, clock.now_us()) >= kTelemetryPeriodUs) {
      // Test scaffolding, not an estimator: a plain count difference over the period.
      const uint32_t now_count = encoder.count();
      const auto delta = static_cast<int32_t>(now_count - last_count);
      const double dt_s = core::elapsed_us(last_telemetry_us, clock.now_us()) * 1e-6;
      StateTelemetry tm{};
      tm.wheel_speed_left_m_s = static_cast<float>(delta / encoder.counts_per_rev() * kTwoPi /
                                                   dt_s * kWheelRadius_m);
      ASSERT_TRUE(send(robot_port, MessageId::StateTelemetry, seq++, clock.now_us(), tm));
      last_count = now_count;
      last_telemetry_us = clock.now_us();
    }
    receive(operator_port, operator_decoder, received);
  }

  ASSERT_GE(received.size(), 98U);  // 100 sent; the last may still be on the wire
  for (size_t i = 1; i < received.size(); ++i) {
    EXPECT_EQ(core::elapsed_us(received[i - 1].timestamp_us, received[i].timestamp_us),
              kTelemetryPeriodUs)
        << "timestamps stay evenly spaced across the 2^32 wrap, frame " << i;
  }
  EXPECT_LT(received.back().timestamp_us, start_us) << "the run did cross the wrap";

  StateTelemetry last{};
  ASSERT_TRUE(StateTelemetry::decode(received.back().payload, received.back().payload_len, last));
  const WheelPlantParams p;
  const double expected_m_s = kDuty * p.no_load_speed_rad_s * kWheelRadius_m;
  // One count per 10 ms period is 2*pi/1400/0.01*0.0525 = 0.024 m/s of quantisation.
  EXPECT_NEAR(last.wheel_speed_left_m_s, expected_m_s, 0.03);
}

}  // namespace
}  // namespace recon::sim
