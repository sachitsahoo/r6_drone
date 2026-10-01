// MainLoop and MotorLoop: the ADR 0015 glue, run on the simulated HAL.
//
// These test each half on its own. tests/cpp/test_sil_safety.cpp runs both together against
// the plant.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "control/drive_loop.hpp"
#include "messages.hpp"
#include "protocol/frame.hpp"
#include "runtime/main_loop.hpp"
#include "runtime/motor_loop.hpp"
#include "sim/sim_watchdog.hpp"
#include "sim/sim_world.hpp"

namespace recon::core {
namespace {

using protocol::DecodedFrame;
using protocol::FaultCode;
using protocol::MessageId;
using protocol::NackReason;
using protocol::SafetyState;

constexpr uint32_t kCommsTimeoutUs = 200000;
constexpr uint32_t kAllLoops = static_cast<uint32_t>(CheckInTask::kMotorLoop) |
                               static_cast<uint32_t>(CheckInTask::kMainLoop);

template <typename Msg>
void send(hal::SerialPort& port, MessageId id, uint8_t seq, uint32_t t_us, const Msg& msg) {
  uint8_t payload[Msg::kPayloadBytes > 0 ? Msg::kPayloadBytes : 1] = {};
  const size_t n = msg.encode(payload);
  uint8_t wire[protocol::kMaxWireFrameBytes] = {};
  size_t len = 0;
  ASSERT_TRUE(protocol::encode_frame(static_cast<uint8_t>(id), seq, t_us, payload, n, wire,
                                     sizeof wire, len));
  ASSERT_EQ(port.write(wire, len), len);
}

/// MainLoop on the MCU end of a simulated link, with the test as the operator.
struct MainRig {
  MainRig()
      : watchdog(world.clock, kIwdgTimeout_ms * 1000U),
        checks(kAllLoops),
        main(world.clock, world.link.mcu(), watchdog, mailbox, reports, checks, kCommsTimeoutUs) {}

  /// Lets the link carry what was sent, then runs one main-loop pass.
  void deliver_and_poll() {
    world.step(2000);  // 2 ms: room for a few frames at 460 800 baud
    main.poll();
    world.step(2000);  // and for the replies to reach the operator
  }

  /// Every frame that reached the operator since the last call.
  std::vector<DecodedFrame> replies() {
    std::vector<DecodedFrame> out;
    uint8_t buf[64];
    size_t n = 0;
    DecodedFrame f{};
    while ((n = world.link.host().read(buf, sizeof buf)) > 0) {
      for (size_t i = 0; i < n; ++i) {
        if (operator_decoder.push_byte(buf[i], f)) {
          out.push_back(f);
        }
      }
    }
    return out;
  }

  std::vector<protocol::Nack> nacks() {
    std::vector<protocol::Nack> out;
    for (const DecodedFrame& f : replies()) {
      protocol::Nack n{};
      if (f.message_id == static_cast<uint8_t>(MessageId::Nack) &&
          protocol::Nack::decode(f.payload, f.payload_len, n)) {
        out.push_back(n);
      }
    }
    return out;
  }

  sim::SimWorld world;
  sim::SimWatchdog watchdog;
  SafetyMailbox mailbox;
  ReportQueue reports;
  CheckInMonitor checks;
  MainLoop main;
  protocol::FrameDecoder operator_decoder;
};

protocol::SafetyStateRequest request(SafetyState state) {
  protocol::SafetyStateRequest r{};
  r.requested_state = state;
  r.magic = 0x41524D21;  // "ARM!", messages.yaml
  return r;
}

TEST(MainLoop, ADriveCommandReachesTheMailbox) {
  MainRig rig;
  protocol::DriveCommand cmd{};
  cmd.cmd_linear_speed_m_s = 0.25F;
  cmd.cmd_angular_rate_rad_s = -1.0F;
  send(rig.world.link.host(), MessageId::DriveCommand, 1, 1000, cmd);
  rig.deliver_and_poll();
  SafetyInputs in;
  rig.mailbox.take(in);
  EXPECT_TRUE(in.drive_command);
  EXPECT_FLOAT_EQ(in.drive_linear_m_s, 0.25F);
  EXPECT_FLOAT_EQ(in.drive_angular_rad_s, -1.0F);
  EXPECT_EQ(rig.main.stats().drive_commands, 1U);
}

TEST(MainLoop, AStaleDriveCommandIsNackedAndNeverPosted) {
  MainRig rig;
  protocol::DriveCommand cmd{};
  send(rig.world.link.host(), MessageId::DriveCommand, 1, 5000, cmd);
  rig.deliver_and_poll();
  SafetyInputs first;
  rig.mailbox.take(first);
  send(rig.world.link.host(), MessageId::DriveCommand, 2, 4000, cmd);  // older
  rig.deliver_and_poll();
  SafetyInputs in;
  rig.mailbox.take(in);
  EXPECT_FALSE(in.drive_command) << "a stale frame must not feed the comms watchdog";
  const auto nacks = rig.nacks();
  ASSERT_EQ(nacks.size(), 1U);
  EXPECT_EQ(nacks[0].reason, NackReason::STALE_TIMESTAMP);
  EXPECT_EQ(nacks[0].rejected_message_id, static_cast<uint8_t>(MessageId::DriveCommand));
  EXPECT_EQ(nacks[0].rejected_seq, 2);
  EXPECT_EQ(rig.main.stats().stale_rejects, 1U);
}

TEST(MainLoop, EstopAndRequestsArePosted) {
  MainRig rig;
  protocol::EstopRequest estop{};
  estop.magic = 0x45535450;  // "ESTP"
  send(rig.world.link.host(), MessageId::EstopRequest, 3, 1000, estop);
  send(rig.world.link.host(), MessageId::SafetyStateRequest, 4, 1000,
       request(SafetyState::DISARMED));
  rig.deliver_and_poll();
  SafetyInputs in;
  rig.mailbox.take(in);
  EXPECT_TRUE(in.estop);
  ASSERT_TRUE(in.request.present);
  EXPECT_EQ(in.request.requested_state, SafetyState::DISARMED);
  EXPECT_EQ(in.request.seq, 4);
}

TEST(MainLoop, AFullRequestRingNacksTheExtraRequest) {
  MainRig rig;
  for (uint8_t i = 0; i < kSafetyRequestSlots + 1; ++i) {
    send(rig.world.link.host(), MessageId::SafetyStateRequest, i, 1000U + i,
         request(SafetyState::ARMED));
  }
  rig.deliver_and_poll();
  const auto nacks = rig.nacks();
  ASSERT_EQ(nacks.size(), 1U);
  EXPECT_EQ(nacks[0].reason, NackReason::UNSPECIFIED);
  EXPECT_EQ(nacks[0].rejected_seq, kSafetyRequestSlots);
  EXPECT_EQ(rig.main.stats().request_overflows, 1U);
}

TEST(MainLoop, ABadMagicEstopIsNackedBadMagic) {
  MainRig rig;
  protocol::EstopRequest estop{};
  estop.magic = 0x12345678;
  send(rig.world.link.host(), MessageId::EstopRequest, 9, 1000, estop);
  rig.deliver_and_poll();
  SafetyInputs in;
  rig.mailbox.take(in);
  EXPECT_FALSE(in.estop);
  const auto nacks = rig.nacks();
  ASSERT_EQ(nacks.size(), 1U);
  EXPECT_EQ(nacks[0].reason, NackReason::BAD_MAGIC);
}

TEST(MainLoop, HeartbeatCountsAsAnotherValidFrame) {
  MainRig rig;
  send(rig.world.link.host(), MessageId::Heartbeat, 1, 1000, protocol::Heartbeat{});
  rig.deliver_and_poll();
  SafetyInputs in;
  rig.mailbox.take(in);
  EXPECT_TRUE(in.other_frame);
  EXPECT_FALSE(in.drive_command);
}

TEST(MainLoop, ParamFramesFeedTheLinkButAreNackedUntilTheParamTableExists) {
  MainRig rig;
  protocol::ParamGet get{};
  get.param_id = 0x0200;
  send(rig.world.link.host(), MessageId::ParamGet, 1, 1000, get);
  rig.deliver_and_poll();
  SafetyInputs in;
  rig.mailbox.take(in);
  EXPECT_TRUE(in.other_frame) << "ADR 0014: a param request is a valid frame";
  const auto nacks = rig.nacks();
  ASSERT_EQ(nacks.size(), 1U);
  EXPECT_EQ(nacks[0].reason, NackReason::UNKNOWN_PARAM_ID);
}

TEST(MainLoop, ARobotToOperatorMessageArrivingAtTheRobotIsNacked) {
  MainRig rig;
  protocol::Fault fault{};
  send(rig.world.link.host(), MessageId::Fault, 5, 1000, fault);
  rig.deliver_and_poll();
  SafetyInputs in;
  rig.mailbox.take(in);
  EXPECT_FALSE(in.other_frame) << "a wrong-direction frame does not prove an operator link";
  const auto nacks = rig.nacks();
  ASSERT_EQ(nacks.size(), 1U);
  EXPECT_EQ(nacks[0].reason, NackReason::UNKNOWN_MESSAGE_ID);
}

TEST(MainLoop, QueuedReportsBecomeFramesOnTheWire) {
  MainRig rig;
  OutboundReport fault;
  fault.kind = OutboundReport::Kind::kFault;
  fault.fault = {FaultCode::WHEEL_STALL, kLeftWheel};
  OutboundReport nack;
  nack.kind = OutboundReport::Kind::kNack;
  nack.nack = {true, static_cast<uint8_t>(MessageId::SafetyStateRequest), NackReason::ARM_INTERLOCK, 7};
  ASSERT_TRUE(rig.reports.push(fault));
  ASSERT_TRUE(rig.reports.push(nack));
  rig.deliver_and_poll();
  const auto frames = rig.replies();
  ASSERT_EQ(frames.size(), 2U);
  protocol::Fault f{};
  ASSERT_TRUE(protocol::Fault::decode(frames[0].payload, frames[0].payload_len, f));
  EXPECT_EQ(f.fault_code, FaultCode::WHEEL_STALL);
  EXPECT_EQ(f.context, kLeftWheel);
  protocol::Nack n{};
  ASSERT_TRUE(protocol::Nack::decode(frames[1].payload, frames[1].payload_len, n));
  EXPECT_EQ(n.reason, NackReason::ARM_INTERLOCK);
  EXPECT_EQ(n.rejected_seq, 7);
  EXPECT_EQ(frames[1].seq, static_cast<uint8_t>(frames[0].seq + 1)) << "robot seq increments";
}

TEST(MainLoop, TheBootReportIsSentOnceWithTheMarker) {
  MainRig rig;
  rig.main.queue_boot_report(0x0C);  // e.g. pin reset + brown-out
  rig.deliver_and_poll();
  rig.deliver_and_poll();
  const auto frames = rig.replies();
  ASSERT_EQ(frames.size(), 1U);
  protocol::Fault f{};
  ASSERT_TRUE(protocol::Fault::decode(frames[0].payload, frames[0].payload_len, f));
  EXPECT_EQ(f.fault_code, FaultCode::NONE);
  EXPECT_EQ(f.context, kBootReportMarker | 0x0CU);
}

TEST(MainLoop, AZeroBootReportSendsNothing) {
  MainRig rig;
  rig.main.queue_boot_report(0);
  rig.deliver_and_poll();
  EXPECT_TRUE(rig.replies().empty());
}

TEST(MainLoop, FeedsTheIwdgOnlyAfterTheMotorLoopChecksIn) {
  MainRig rig;
  rig.main.poll();
  EXPECT_EQ(rig.watchdog.feed_count(), 0U) << "the main loop alone cannot feed";
  rig.checks.check_in(CheckInTask::kMotorLoop);
  rig.main.poll();
  EXPECT_EQ(rig.watchdog.feed_count(), 1U);
  rig.main.poll();
  EXPECT_EQ(rig.watchdog.feed_count(), 1U) << "a fresh motor check-in is needed for each feed";
}

// ------------------------------------------------------------------------- MotorLoop

/// A clock that advances by a fixed step on every read, so a tick has a measurable duration.
class SteppingClock final : public hal::Clock {
 public:
  explicit SteppingClock(uint32_t step_us) : step_us_(step_us) {}
  uint32_t now_us() const override {
    const uint32_t t = t_us_;
    t_us_ += step_us_;
    return t;
  }

 private:
  uint32_t step_us_;
  mutable uint32_t t_us_ = 0;
};

struct MotorRig {
  explicit MotorRig(const hal::Clock& clk)
      : drive(clk, world.left_wheel, world.left_wheel, world.right_wheel, world.right_wheel),
        checks(kAllLoops),
        motor(clk, drive, supervisor, mailbox, reports, checks, world.left_wheel,
              world.right_wheel, world.pitch_stage) {}

  sim::SimWorld world;
  DriveLoop drive;
  SafetySupervisor supervisor;
  SafetyMailbox mailbox;
  ReportQueue reports;
  CheckInMonitor checks;
  MotorLoop motor;
};

TEST(MotorLoop, AnEstopInTheMailboxBrakesBothWheelsThatTick) {
  sim::SimClock clock(1000);
  MotorRig rig(clock);
  rig.mailbox.post_estop();
  rig.motor.tick();
  EXPECT_EQ(rig.motor.outputs().state, SafetyState::ESTOP);
  EXPECT_TRUE(rig.world.left_wheel.braking());
  EXPECT_TRUE(rig.world.right_wheel.braking());
  EXPECT_FALSE(rig.world.pitch_stage.enabled());
}

TEST(MotorLoop, ArmedDrivesAndDisarmedCoasts) {
  sim::SimClock clock(1000);
  MotorRig rig(clock);
  rig.mailbox.post_other_frame();
  rig.motor.tick();
  ASSERT_TRUE(rig.mailbox.post_request({true, SafetyState::ARMED, 1}));
  clock.advance_us(1000);
  rig.motor.tick();
  ASSERT_EQ(rig.motor.outputs().state, SafetyState::ARMED);
  EXPECT_TRUE(rig.world.pitch_stage.enabled());
  for (int i = 0; i < 100; ++i) {
    if (i % 20 == 0) {
      rig.mailbox.post_drive_command(0.2F, 0.0F);
    }
    clock.advance_us(1000);
    rig.world.step(1000);
    rig.motor.tick();
  }
  EXPECT_GT(rig.world.left_wheel.applied_duty(), 0.0F) << "the drive loop is driving";
  ASSERT_TRUE(rig.mailbox.post_request({true, SafetyState::DISARMED, 2}));
  clock.advance_us(1000);
  rig.motor.tick();
  EXPECT_EQ(rig.world.left_wheel.applied_duty(), 0.0F);
  EXPECT_FALSE(rig.world.left_wheel.braking()) << "DISARMED coasts";
}

TEST(MotorLoop, ANackIsQueuedForTheMainLoop) {
  sim::SimClock clock(1000);
  MotorRig rig(clock);
  ASSERT_TRUE(rig.mailbox.post_request({true, SafetyState::FAULT, 5}));
  rig.motor.tick();
  OutboundReport r;
  ASSERT_TRUE(rig.reports.pop(r));
  EXPECT_EQ(r.kind, OutboundReport::Kind::kNack);
  EXPECT_EQ(r.nack.reason, NackReason::RANGE_REJECT);
  EXPECT_EQ(r.nack.rejected_seq, 5);
}

TEST(MotorLoop, ChecksInEveryTick) {
  sim::SimClock clock(1000);
  MotorRig rig(clock);
  rig.checks.check_in(CheckInTask::kMainLoop);
  EXPECT_FALSE(rig.checks.should_feed());
  rig.motor.tick();
  EXPECT_TRUE(rig.checks.should_feed());
}

TEST(MotorLoop, MeasuresItsExecutionTimeAndCountsOverruns) {
  // Every clock read advances 400 us, so a tick spans several reads: well over 1000 us.
  SteppingClock slow(400);
  MotorRig rig(slow);
  rig.motor.tick();
  EXPECT_GT(rig.motor.last_exec_us(), kMotorLoopPeriod_us);
  EXPECT_EQ(rig.motor.overrun_count(), 1U);
  EXPECT_EQ(rig.motor.max_exec_us(), rig.motor.last_exec_us());

  SteppingClock fast(1);
  MotorRig quick(fast);
  quick.motor.tick();
  EXPECT_LT(quick.motor.last_exec_us(), kMotorLoopPeriod_us);
  EXPECT_EQ(quick.motor.overrun_count(), 0U);
}

}  // namespace
}  // namespace recon::core
