// SIL: the ADR 0014 safety machine closed around the ADR 0013 wheel loop, on the simulated
// plant, with every operator frame crossing the simulated UART through the real codec.
//
// `Robot` below runs the REAL firmware glue (ADR 0015): `MainLoop::poll()` and
// `MotorLoop::tick()` from firmware/core/runtime/, on the simulated HAL, in the order the G474
// runs them (main-loop pass, then the 1 kHz timer interrupt). Only the operator is scripted.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

#include "control/drive_loop.hpp"
#include "control/geometry.hpp"
#include "messages.hpp"
#include "protocol/frame.hpp"
#include "runtime/main_loop.hpp"
#include "runtime/motor_loop.hpp"
#include "safety/check_in_monitor.hpp"
#include "safety/safety_supervisor.hpp"
#include "sim/sim_watchdog.hpp"
#include "sim/sim_world.hpp"
#include "time/timestamp.hpp"

namespace recon::sim {
namespace {

using core::CheckInMonitor;
using core::CheckInTask;
using core::DriveLoop;
using core::SafetyInputs;
using core::SafetyOutputs;
using core::SafetySupervisor;
using protocol::DecodedFrame;
using protocol::FaultCode;
using protocol::FrameDecoder;
using protocol::MessageId;
using protocol::NackReason;
using protocol::SafetyState;

constexpr uint32_t kPeriodUs = 1000;          // 1 kHz motor loop (CLAUDE.md)
constexpr uint32_t kCommandPeriodUs = 20000;  // 50 Hz operator commands (CLAUDE.md)
constexpr uint32_t kSafetyStateMagic = 0x41524D21;  // "ARM!", messages.yaml
constexpr uint32_t kEstopMagic = 0x45535450;        // "ESTP", messages.yaml

template <typename Msg>
void send(hal::SerialPort& port, MessageId id, uint8_t seq, uint32_t t_us, const Msg& msg) {
  uint8_t payload[Msg::kPayloadBytes > 0 ? Msg::kPayloadBytes : 1] = {};
  const size_t n = msg.encode(payload);
  uint8_t wire[protocol::kMaxWireFrameBytes] = {};
  size_t len = 0;
  ASSERT_TRUE(protocol::encode_frame(static_cast<uint8_t>(id), seq, t_us, payload, n, wire,
                                     sizeof wire, len));
  port.write(wire, len);  // a full buffer drops the frame, as a real UART would
}

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

/// One robot (MCU side) plus a scripted operator, stepped together in 1 ms ticks.
class Robot {
 public:
  explicit Robot(const SimWorld::Config& world_config = SimWorld::Config{},
                 bool boot_after_watchdog_reset = false)
      : world(world_config),
        watchdog(world.clock, core::kIwdgTimeout_ms * 1000U, boot_after_watchdog_reset),
        loop(world.clock, world.left_wheel, world.left_wheel, world.right_wheel,
             world.right_wheel),
        // Firmware reads the reset reason once, at boot, exactly like this.
        supervisor(core::SafetyConfig{}, watchdog.reset_was_watchdog()),
        check_ins(static_cast<uint32_t>(CheckInTask::kMotorLoop) |
                  static_cast<uint32_t>(CheckInTask::kMainLoop)),
        main_loop(world.clock, world.link.mcu(), watchdog, mailbox, reports, check_ins,
                  core::SafetyConfig{}.comms_timeout_ms * 1000U),
        motor_loop(world.clock, loop, supervisor, mailbox, reports, check_ins, world.left_wheel,
                   world.right_wheel, world.pitch_stage) {}

  // ------------------------------------------------------------ operator side
  /// While true, the operator sends Heartbeat + DriveCommand(cmd) every 20 ms.
  bool streaming = true;
  float cmd_linear_m_s = 0.0F;
  float cmd_angular_rad_s = 0.0F;

  void send_request(SafetyState state) {
    protocol::SafetyStateRequest req{};
    req.requested_state = state;
    req.magic = kSafetyStateMagic;
    send(world.link.host(), MessageId::SafetyStateRequest, op_seq_++, world.clock.now_us(), req);
  }
  void send_estop() {
    protocol::EstopRequest req{};
    req.magic = kEstopMagic;
    send(world.link.host(), MessageId::EstopRequest, op_seq_++, world.clock.now_us(), req);
  }

  std::vector<protocol::Fault> faults_seen;  ///< Fault frames that reached the operator.
  std::vector<protocol::Nack> nacks_seen;

  // ------------------------------------------------------------ robot side
  /// Simulates a hung motor-loop ISR: its code stops running, but the timers keep driving
  /// the PWM at the last duty (ADR 0014), and the main loop keeps trying to feed the IWDG.
  bool motor_loop_hung = false;

  void tick() {
    if (streaming && core::elapsed_us(last_stream_us_, world.clock.now_us()) >= kCommandPeriodUs) {
      last_stream_us_ = world.clock.now_us();
      send(world.link.host(), MessageId::Heartbeat, op_seq_++, last_stream_us_,
           protocol::Heartbeat{});
      protocol::DriveCommand cmd{};
      cmd.cmd_linear_speed_m_s = cmd_linear_m_s;
      cmd.cmd_angular_rate_rad_s = cmd_angular_rad_s;
      send(world.link.host(), MessageId::DriveCommand, op_seq_++, last_stream_us_, cmd);
    }

    world.step(kPeriodUs);
    const uint32_t drives_before = main_loop.stats().drive_commands;
    main_loop.poll();
    if (main_loop.stats().drive_commands != drives_before) {
      last_drive_rx_us = world.clock.now_us();
    }
    if (!motor_loop_hung) {
      motor_loop.tick();
    }

    std::vector<DecodedFrame> back;
    receive(world.link.host(), operator_decoder_, back);
    for (const DecodedFrame& f : back) {
      if (f.message_id == static_cast<uint8_t>(MessageId::Fault)) {
        protocol::Fault fault{};
        ASSERT_TRUE(protocol::Fault::decode(f.payload, f.payload_len, fault));
        faults_seen.push_back(fault);
      } else if (f.message_id == static_cast<uint8_t>(MessageId::Nack)) {
        protocol::Nack nack{};
        ASSERT_TRUE(protocol::Nack::decode(f.payload, f.payload_len, nack));
        nacks_seen.push_back(nack);
      }
    }
  }

  void run(double seconds) {
    const auto ticks = static_cast<int>(seconds * 1e6 / kPeriodUs + 0.5);
    for (int i = 0; i < ticks; ++i) {
      tick();
    }
  }

  /// Streams a centred stick for 50 ms, then asks to arm.
  void arm() {
    cmd_linear_m_s = 0.0F;
    cmd_angular_rad_s = 0.0F;
    run(0.05);
    send_request(SafetyState::ARMED);
    run(0.005);
    ASSERT_EQ(out().state, SafetyState::ARMED);
  }

  bool saw_fault(FaultCode code, uint32_t context) const {
    for (const protocol::Fault& f : faults_seen) {
      if (f.fault_code == code && f.context == context) {
        return true;
      }
    }
    return false;
  }

  const SafetyOutputs& out() const { return motor_loop.outputs(); }

  SimWorld world;
  SimWatchdog watchdog;
  DriveLoop loop;
  SafetySupervisor supervisor;
  core::SafetyMailbox mailbox;
  core::ReportQueue reports;
  CheckInMonitor check_ins;
  core::MainLoop main_loop;
  core::MotorLoop motor_loop;
  uint32_t last_drive_rx_us = 0;  ///< MCU time of the main-loop pass that took the last DriveCommand.

 private:
  FrameDecoder operator_decoder_;
  uint32_t last_stream_us_ = 0;
  uint8_t op_seq_ = 0;
};

TEST(SilSafety, ALinkCutCoastsAt200msBrakesAt1sAndNeverResumesByItself) {
  Robot robot;
  robot.arm();
  robot.cmd_linear_m_s = 0.2F;
  robot.run(1.0);
  ASSERT_EQ(robot.out().state, SafetyState::ARMED);
  const double cruise_rad_s = robot.world.left_wheel.speed_rad_s();
  ASSERT_GT(cruise_rad_s, 3.0);

  robot.world.link.set_cut(true);
  uint32_t fault_us = 0;
  uint32_t brake_us = 0;
  for (int i = 0; i < 2000 && brake_us == 0; ++i) {
    robot.tick();
    const uint32_t now = robot.world.clock.now_us();
    if (fault_us == 0 && robot.out().state == SafetyState::FAULT) {
      fault_us = now;
      EXPECT_FALSE(robot.world.left_wheel.braking()) << "a fault coasts first";
      EXPECT_TRUE(robot.world.pitch_stage.enabled()) << "Q6: stabilizer keeps running";
    }
    if (robot.world.left_wheel.braking()) {
      brake_us = now;
    }
  }
  ASSERT_NE(fault_us, 0U);
  ASSERT_NE(brake_us, 0U);
  // Measured from the MCU receive time of the last DriveCommand that got through.
  EXPECT_EQ(core::elapsed_us(robot.last_drive_rx_us, fault_us), 200000U);
  EXPECT_EQ(core::elapsed_us(robot.last_drive_rx_us, brake_us), 1000000U);
  EXPECT_EQ(robot.supervisor.fault_flags(), core::fault_bit(FaultCode::COMMS_TIMEOUT));
  // ADR 0014's table: 800 ms of coasting (tau 0.5 s) leaves about e^-1.6 = 20% of the speed.
  EXPECT_LT(robot.world.left_wheel.speed_rad_s(), 0.25 * cruise_rad_s);
  EXPECT_GT(robot.world.left_wheel.speed_rad_s(), 0.15 * cruise_rad_s);
  robot.run(0.5);
  EXPECT_LT(std::fabs(robot.world.left_wheel.speed_rad_s()), 0.01) << "braked to a stop";

  // The link returns with the stick still pushed: the robot must not move.
  robot.world.link.set_cut(false);
  robot.run(1.0);
  EXPECT_EQ(robot.out().state, SafetyState::FAULT);
  EXPECT_LT(std::fabs(robot.world.left_wheel.speed_rad_s()), 0.01);

  // Clear, then arm: two deliberate requests, with the stick centred for the second.
  robot.send_request(SafetyState::DISARMED);
  robot.run(0.01);
  EXPECT_EQ(robot.out().state, SafetyState::DISARMED);
  robot.arm();
}

TEST(SilSafety, AFrozenEncoderLatchesWheelStallAndTheRobotStops) {
  Robot robot;
  robot.arm();
  robot.cmd_linear_m_s = 0.2F;
  robot.run(0.5);
  robot.world.left_wheel.freeze_encoder(true);  // cable pulled while driving
  const uint32_t freeze_us = robot.world.clock.now_us();

  uint32_t trip_us = 0;
  for (int i = 0; i < 1500 && trip_us == 0; ++i) {
    robot.tick();
    if (robot.out().state == SafetyState::FAULT) {
      trip_us = robot.world.clock.now_us();
    }
  }
  ASSERT_NE(trip_us, 0U) << "an unplugged encoder must not run away forever";
  // ADR 0014: within 500 ms plus one speed window (10 ms) of the freeze, plus 5 ms for the
  // controller to reach the duty limit.
  const uint32_t delay_us = core::elapsed_us(freeze_us, trip_us);
  EXPECT_GE(delay_us, 500000U);
  EXPECT_LE(delay_us, 515000U);
  EXPECT_EQ(robot.supervisor.fault_flags(), core::fault_bit(FaultCode::WHEEL_STALL));

  robot.run(2.0);
  EXPECT_TRUE(robot.saw_fault(FaultCode::WHEEL_STALL, core::kLeftWheel));
  EXPECT_LT(std::fabs(robot.world.left_wheel.speed_rad_s()), 0.01) << "the robot stops";
  EXPECT_LT(std::fabs(robot.world.right_wheel.speed_rad_s()), 0.01);
}

TEST(SilSafety, AReversedEncoderLatchesWheelStall) {
  SimWorld::Config cfg;
  cfg.left.encoder_reversed = true;
  Robot robot(cfg);
  robot.arm();
  robot.cmd_linear_m_s = 0.2F;
  double peak_rad_s = 0.0;
  for (int i = 0; i < 1000 && robot.out().state == SafetyState::ARMED; ++i) {
    robot.tick();
    peak_rad_s = std::fmax(peak_rad_s, robot.world.left_wheel.speed_rad_s());
  }
  ASSERT_EQ(robot.out().state, SafetyState::FAULT);
  EXPECT_GT(peak_rad_s, 0.2 / core::kWheelRadius_m) << "positive feedback: it ran away";
  robot.run(0.05);
  EXPECT_TRUE(robot.saw_fault(FaultCode::WHEEL_STALL, core::kLeftWheel));
}

TEST(SilSafety, FullStickOnCarpetSaturatesWithoutAFault) {
  // ADR 0013's carpet plant: 0.48 duty needed against a 0.3 limit, so the loop saturates, but
  // the wheel still turns at ~5 rad/s the right way. WHEEL_STALL must stay quiet.
  SimWorld::Config carpet;
  for (WheelPlantParams* p : {&carpet.left, &carpet.right}) {
    p->load_duty = 0.05F;
    p->extra_viscous_drag = 0.5F;
  }
  Robot robot(carpet);
  robot.arm();
  robot.cmd_linear_m_s = 2.0F;  // DriveCommand range maximum
  robot.run(3.0);
  EXPECT_EQ(robot.out().state, SafetyState::ARMED);
  EXPECT_EQ(robot.supervisor.fault_flags(), 0);
  EXPECT_TRUE(robot.faults_seen.empty());
  EXPECT_FLOAT_EQ(robot.loop.left().duty, robot.loop.config().gains.duty_limit)
      << "the case must actually saturate, or it proves nothing";
  EXPECT_GT(robot.world.left_wheel.speed_rad_s(), 4.0);
}

TEST(SilSafety, BootAfterAWatchdogResetReportsItAndClears) {
  Robot robot(SimWorld::Config{}, /*boot_after_watchdog_reset=*/true);
  robot.run(0.1);
  EXPECT_EQ(robot.out().state, SafetyState::FAULT);
  ASSERT_EQ(robot.faults_seen.size(), 1U);
  EXPECT_EQ(robot.faults_seen[0].fault_code, FaultCode::WATCHDOG_RESET);

  robot.send_request(SafetyState::ARMED);
  robot.run(0.01);
  ASSERT_EQ(robot.nacks_seen.size(), 1U);
  EXPECT_EQ(robot.nacks_seen[0].reason, NackReason::NOT_DISARMED);

  robot.send_request(SafetyState::DISARMED);
  robot.run(0.01);
  EXPECT_EQ(robot.out().state, SafetyState::DISARMED);
  robot.arm();
}

TEST(SilSafety, ArmingWithTheStickDeflectedIsNackedOverTheLink) {
  Robot robot;
  robot.cmd_linear_m_s = 0.3F;
  robot.run(0.05);
  robot.send_request(SafetyState::ARMED);
  robot.run(0.01);
  EXPECT_EQ(robot.out().state, SafetyState::DISARMED);
  ASSERT_EQ(robot.nacks_seen.size(), 1U);
  EXPECT_EQ(robot.nacks_seen[0].reason, NackReason::ARM_INTERLOCK);
  EXPECT_EQ(robot.nacks_seen[0].rejected_message_id,
            static_cast<uint8_t>(MessageId::SafetyStateRequest));
  EXPECT_TRUE(robot.saw_fault(FaultCode::NONE,
                              static_cast<uint32_t>(core::ArmInterlock::kCommandNotZero)));
  EXPECT_EQ(robot.world.left_wheel.speed_rad_s(), 0.0);
}

TEST(SilSafety, EstopOverTheLinkBrakesWithinTwoMilliseconds) {
  Robot robot;
  robot.arm();
  robot.cmd_linear_m_s = 0.2F;
  robot.run(0.5);
  robot.send_estop();
  const uint32_t sent_us = robot.world.clock.now_us();
  while (!robot.world.left_wheel.braking()) {
    robot.tick();
    ASSERT_LT(core::elapsed_us(sent_us, robot.world.clock.now_us()), 10000U);
  }
  // ADR 0014: one main-loop iteration plus one motor tick; the frame is ~10 bytes (0.2 ms).
  EXPECT_LE(core::elapsed_us(sent_us, robot.world.clock.now_us()), 2000U);
  EXPECT_EQ(robot.out().state, SafetyState::ESTOP);
}

TEST(SilSafety, HealthyLoopsKeepTheIwdgFed) {
  Robot robot;
  robot.arm();
  robot.run(2.0);
  EXPECT_FALSE(robot.watchdog.expired());
  EXPECT_GT(robot.watchdog.feed_count(), 2000U) << "fed every tick with both check-ins";
}

TEST(SilSafety, AHungMotorLoopStarvesTheIwdgWithin50ms) {
  Robot robot;
  robot.arm();
  robot.cmd_linear_m_s = 0.2F;
  robot.run(0.5);
  const uint32_t hang_us = robot.world.clock.now_us();
  robot.motor_loop_hung = true;
  // The motor loop's last check-in (the tick before the hang) can still pay for one feed on
  // the next main-loop pass. After that the main loop alone can never feed.
  robot.tick();
  const uint32_t last_feed_us = robot.watchdog.last_feed_us();
  EXPECT_LE(core::elapsed_us(hang_us, last_feed_us), kPeriodUs);
  while (core::elapsed_us(last_feed_us, robot.world.clock.now_us()) < 49000U) {
    robot.tick();
    ASSERT_FALSE(robot.watchdog.expired());
  }
  EXPECT_GT(robot.world.left_wheel.speed_rad_s(), 3.0)
      << "and meanwhile the PWM keeps driving: why the timeout is short (ADR 0014)";
  robot.tick();  // 50 ms after the last feed: it would reset the chip
  EXPECT_TRUE(robot.watchdog.expired());
  EXPECT_EQ(robot.watchdog.last_feed_us(), last_feed_us) << "no feed without the motor loop";
}

}  // namespace
}  // namespace recon::sim
