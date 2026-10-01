// SafetySupervisor: ADR 0014. Pure logic, so every test drives it with input structs.
//
// The transition-table test is the mechanical form of CLAUDE.md's "test every transition":
// every (state, event) pair must appear in kTable exactly once, so adding a state or an event
// without deciding what it does fails the build's tests rather than going unnoticed.

#include "safety/safety_supervisor.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace recon::core {
namespace {

using protocol::FaultCode;
using protocol::MessageId;
using protocol::NackReason;
using protocol::SafetyState;

constexpr uint32_t kTickUs = 1000;  // 1 kHz motor loop
constexpr float kLimit = protocol::param_defaults::wheel_duty_limit;

/// Drives one supervisor in 1 ms ticks and remembers everything it reported.
class Rig {
 public:
  explicit Rig(const SafetyConfig& config = SafetyConfig{}, bool watchdog_boot = false,
               uint32_t start_us = 1000000)
      : sup(config, watchdog_boot), now_us(start_us) {}

  /// Inputs for the current tick: no frames, idle wheels, no detector flags.
  SafetyInputs inputs() const {
    SafetyInputs in;
    in.now_us = now_us;
    in.left = {0.0F, kLimit, 0.0F, true};
    in.right = {0.0F, kLimit, 0.0F, true};
    return in;
  }

  SafetyOutputs tick(const SafetyInputs& in) {
    out = sup.step(in);
    for (size_t i = 0; i < out.fault_count; ++i) {
      reports.push_back(out.faults[i]);
    }
    if (out.nack.present) {
      nacks.push_back(out.nack);
    }
    now_us += kTickUs;
    return out;
  }
  SafetyOutputs tick() { return tick(inputs()); }

  SafetyOutputs heartbeat() {
    SafetyInputs in = inputs();
    in.other_frame = true;
    return tick(in);
  }
  SafetyOutputs drive(float linear_m_s, float angular_rad_s = 0.0F) {
    SafetyInputs in = inputs();
    in.drive_command = true;
    in.drive_linear_m_s = linear_m_s;
    in.drive_angular_rad_s = angular_rad_s;
    return tick(in);
  }
  SafetyOutputs request(SafetyState state, uint8_t seq = 7) {
    SafetyInputs in = inputs();
    in.request = {true, state, seq};
    return tick(in);
  }
  SafetyOutputs estop() {
    SafetyInputs in = inputs();
    in.estop = true;
    return tick(in);
  }
  /// Heartbeat, then an arm request one tick later.
  void arm() {
    heartbeat();
    request(SafetyState::ARMED);
    ASSERT_EQ(sup.state(), SafetyState::ARMED);
  }
  /// `ms` ticks. While ARMED, a zero DriveCommand every 20 ms keeps the link fed.
  void run_ms(int ms, bool feed = true) {
    for (int i = 0; i < ms; ++i) {
      if (feed && i % 20 == 0) {
        drive(0.0F);
      } else {
        tick();
      }
    }
  }
  void clear_history() {
    reports.clear();
    nacks.clear();
  }
  bool reported(FaultCode code) const {
    for (const FaultReport& r : reports) {
      if (r.code == code) {
        return true;
      }
    }
    return false;
  }

  SafetySupervisor sup;
  uint32_t now_us;
  SafetyOutputs out{};
  std::vector<FaultReport> reports;
  std::vector<NackReport> nacks;
};

// ----------------------------------------------------------------------- transition table

enum class Event {
  kRequestDisarm,
  kRequestArm,
  kRequestFault,
  kRequestEstop,  // SafetyStateRequest(ESTOP), not EstopRequest
  kEstop,         // EstopRequest
  kCommsTimeout,
  kEncoderFault,
  kWheelStall,
  kLoopOverrun,
  kDriverFault,
};
constexpr Event kAllEvents[] = {
    Event::kRequestDisarm, Event::kRequestArm,   Event::kRequestFault, Event::kRequestEstop,
    Event::kEstop,         Event::kCommsTimeout, Event::kEncoderFault, Event::kWheelStall,
    Event::kLoopOverrun,   Event::kDriverFault,
};
constexpr SafetyState kAllStates[] = {SafetyState::DISARMED, SafetyState::ARMED,
                                      SafetyState::FAULT, SafetyState::ESTOP};

constexpr NackReason kNoNack = NackReason::UNSPECIFIED;  // sentinel: no Nack expected
constexpr FaultCode kNoReport = FaultCode::NONE;         // sentinel: no Fault frame expected

struct Row {
  SafetyState from;
  Event event;
  SafetyState to;
  NackReason nack;
  FaultCode report;
};

// The FAULT row set starts from a watchdog-reset boot (WATCHDOG_RESET latched), so each
// detector event in FAULT is a NEW flag and must be reported.
constexpr Row kTable[] = {
    // DISARMED
    {SafetyState::DISARMED, Event::kRequestDisarm, SafetyState::DISARMED, kNoNack, kNoReport},
    {SafetyState::DISARMED, Event::kRequestArm, SafetyState::ARMED, kNoNack, kNoReport},
    {SafetyState::DISARMED, Event::kRequestFault, SafetyState::DISARMED, NackReason::RANGE_REJECT, kNoReport},
    {SafetyState::DISARMED, Event::kRequestEstop, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::DISARMED, Event::kEstop, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::DISARMED, Event::kCommsTimeout, SafetyState::DISARMED, kNoNack, kNoReport},
    {SafetyState::DISARMED, Event::kEncoderFault, SafetyState::FAULT, kNoNack, FaultCode::ENCODER_FAULT},
    {SafetyState::DISARMED, Event::kWheelStall, SafetyState::DISARMED, kNoNack, kNoReport},
    {SafetyState::DISARMED, Event::kLoopOverrun, SafetyState::FAULT, kNoNack, FaultCode::LOOP_OVERRUN},
    {SafetyState::DISARMED, Event::kDriverFault, SafetyState::FAULT, kNoNack, FaultCode::MOTOR_DRIVER_FAULT},
    // ARMED
    {SafetyState::ARMED, Event::kRequestDisarm, SafetyState::DISARMED, kNoNack, kNoReport},
    {SafetyState::ARMED, Event::kRequestArm, SafetyState::ARMED, NackReason::NOT_DISARMED, kNoReport},
    {SafetyState::ARMED, Event::kRequestFault, SafetyState::ARMED, NackReason::RANGE_REJECT, kNoReport},
    {SafetyState::ARMED, Event::kRequestEstop, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::ARMED, Event::kEstop, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::ARMED, Event::kCommsTimeout, SafetyState::FAULT, kNoNack, FaultCode::COMMS_TIMEOUT},
    {SafetyState::ARMED, Event::kEncoderFault, SafetyState::FAULT, kNoNack, FaultCode::ENCODER_FAULT},
    {SafetyState::ARMED, Event::kWheelStall, SafetyState::FAULT, kNoNack, FaultCode::WHEEL_STALL},
    {SafetyState::ARMED, Event::kLoopOverrun, SafetyState::FAULT, kNoNack, FaultCode::LOOP_OVERRUN},
    {SafetyState::ARMED, Event::kDriverFault, SafetyState::FAULT, kNoNack, FaultCode::MOTOR_DRIVER_FAULT},
    // FAULT
    {SafetyState::FAULT, Event::kRequestDisarm, SafetyState::DISARMED, kNoNack, kNoReport},
    {SafetyState::FAULT, Event::kRequestArm, SafetyState::FAULT, NackReason::NOT_DISARMED, kNoReport},
    {SafetyState::FAULT, Event::kRequestFault, SafetyState::FAULT, NackReason::RANGE_REJECT, kNoReport},
    {SafetyState::FAULT, Event::kRequestEstop, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::FAULT, Event::kEstop, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::FAULT, Event::kCommsTimeout, SafetyState::FAULT, kNoNack, kNoReport},
    {SafetyState::FAULT, Event::kEncoderFault, SafetyState::FAULT, kNoNack, FaultCode::ENCODER_FAULT},
    {SafetyState::FAULT, Event::kWheelStall, SafetyState::FAULT, kNoNack, kNoReport},
    {SafetyState::FAULT, Event::kLoopOverrun, SafetyState::FAULT, kNoNack, FaultCode::LOOP_OVERRUN},
    {SafetyState::FAULT, Event::kDriverFault, SafetyState::FAULT, kNoNack, FaultCode::MOTOR_DRIVER_FAULT},
    // ESTOP: detectors still latch and report (visibility), but never leave ESTOP.
    {SafetyState::ESTOP, Event::kRequestDisarm, SafetyState::DISARMED, kNoNack, kNoReport},
    {SafetyState::ESTOP, Event::kRequestArm, SafetyState::ESTOP, NackReason::NOT_DISARMED, kNoReport},
    {SafetyState::ESTOP, Event::kRequestFault, SafetyState::ESTOP, NackReason::RANGE_REJECT, kNoReport},
    {SafetyState::ESTOP, Event::kRequestEstop, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::ESTOP, Event::kEstop, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::ESTOP, Event::kCommsTimeout, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::ESTOP, Event::kEncoderFault, SafetyState::ESTOP, kNoNack, FaultCode::ENCODER_FAULT},
    {SafetyState::ESTOP, Event::kWheelStall, SafetyState::ESTOP, kNoNack, kNoReport},
    {SafetyState::ESTOP, Event::kLoopOverrun, SafetyState::ESTOP, kNoNack, FaultCode::LOOP_OVERRUN},
    {SafetyState::ESTOP, Event::kDriverFault, SafetyState::ESTOP, kNoNack, FaultCode::MOTOR_DRIVER_FAULT},
};

/// Builds a rig sitting in `state`, with a fresh link and an empty report history.
void enter_state(Rig& rig, SafetyState state) {
  switch (state) {
    case SafetyState::DISARMED:
      rig.heartbeat();
      break;
    case SafetyState::ARMED:
      rig.arm();
      break;
    case SafetyState::FAULT:
      rig = Rig(SafetyConfig{}, /*watchdog_boot=*/true);
      rig.heartbeat();
      break;
    case SafetyState::ESTOP:
      rig.heartbeat();
      rig.estop();
      break;
  }
  ASSERT_EQ(rig.sup.state(), state);
  rig.clear_history();
}

void apply(Rig& rig, Event event) {
  const SafetyConfig& cfg = rig.sup.config();
  switch (event) {
    case Event::kRequestDisarm:
      rig.request(SafetyState::DISARMED);
      break;
    case Event::kRequestArm:
      rig.request(SafetyState::ARMED);
      break;
    case Event::kRequestFault:
      rig.request(SafetyState::FAULT);
      break;
    case Event::kRequestEstop:
      rig.request(SafetyState::ESTOP);
      break;
    case Event::kEstop:
      rig.estop();
      break;
    case Event::kCommsTimeout:
      rig.run_ms(cfg.comms_timeout_ms + 1, /*feed=*/false);
      break;
    case Event::kEncoderFault: {
      SafetyInputs in = rig.inputs();
      in.encoder_fault_left = true;
      rig.tick(in);
      break;
    }
    case Event::kWheelStall:
      // Saturated and still for longer than the stall time, link kept fed.
      for (int i = 0; i <= cfg.wheel_stall_ms + 1; ++i) {
        SafetyInputs in = rig.inputs();
        in.left = {kLimit, kLimit, 0.0F, true};
        if (i % 20 == 0) {
          in.drive_command = true;
        }
        rig.tick(in);
      }
      break;
    case Event::kLoopOverrun:
      rig.now_us += cfg.loop_overrun_fault_us;  // next tick lands one gap of 5 ms + 1 tick
      rig.tick();
      break;
    case Event::kDriverFault: {
      SafetyInputs in = rig.inputs();
      in.driver_fault = true;
      rig.tick(in);
      break;
    }
  }
}

/// Outputs that follow from the state alone (ADR 0014, "States and what each one allows").
void expect_outputs_consistent(const SafetyOutputs& out) {
  EXPECT_EQ(out.drive_armed, out.state == SafetyState::ARMED) << "motors move only in ARMED";
  if (out.state == SafetyState::ESTOP) {
    EXPECT_EQ(out.wheel_stop, hal::StopMode::kBrake);
  }
  if (out.state == SafetyState::DISARMED) {
    EXPECT_EQ(out.wheel_stop, hal::StopMode::kCoast) << "DISARMED can be pushed by hand";
    EXPECT_EQ(out.fault_flags, 0) << "DISARMED never holds a latched flag";
  }
  const bool comms_only = out.fault_flags == fault_bit(FaultCode::COMMS_TIMEOUT);
  EXPECT_EQ(out.pitch_enabled, out.state == SafetyState::ARMED ||
                                   (out.state == SafetyState::FAULT && comms_only));
}

TEST(SafetyTransitionTable, EveryStateEventPairIsListedExactlyOnce) {
  for (SafetyState s : kAllStates) {
    for (Event e : kAllEvents) {
      int n = 0;
      for (const Row& r : kTable) {
        n += (r.from == s && r.event == e) ? 1 : 0;
      }
      EXPECT_EQ(n, 1) << "state " << static_cast<int>(s) << " event " << static_cast<int>(e)
                      << " must have exactly one row";
    }
  }
}

TEST(SafetyTransitionTable, EveryRowBehavesAsListed) {
  for (const Row& row : kTable) {
    SCOPED_TRACE(::testing::Message() << "from " << static_cast<int>(row.from) << " event "
                                      << static_cast<int>(row.event));
    Rig rig;
    enter_state(rig, row.from);
    apply(rig, row.event);

    EXPECT_EQ(rig.sup.state(), row.to);
    if (row.nack == kNoNack) {
      EXPECT_TRUE(rig.nacks.empty());
    } else {
      ASSERT_EQ(rig.nacks.size(), 1U);
      EXPECT_EQ(rig.nacks[0].reason, row.nack);
      EXPECT_EQ(rig.nacks[0].rejected_message_id,
                static_cast<uint8_t>(MessageId::SafetyStateRequest));
      EXPECT_EQ(rig.nacks[0].rejected_seq, 7);
    }
    if (row.report == kNoReport) {
      EXPECT_TRUE(rig.reports.empty());
    } else {
      ASSERT_EQ(rig.reports.size(), 1U) << "one Fault frame, on the rising edge only";
      EXPECT_EQ(rig.reports[0].code, row.report);
      EXPECT_NE(rig.sup.fault_flags() & fault_bit(row.report), 0);
    }
    expect_outputs_consistent(rig.out);
  }
}

// ----------------------------------------------------------------------------- boot

TEST(SafetyBoot, NormalBootIsDisarmedAndCoasting) {
  Rig rig;
  const SafetyOutputs out = rig.tick();
  EXPECT_EQ(out.state, SafetyState::DISARMED);
  EXPECT_EQ(out.fault_flags, 0);
  EXPECT_FALSE(out.drive_armed);
  EXPECT_EQ(out.wheel_stop, hal::StopMode::kCoast);
  EXPECT_FALSE(out.pitch_enabled);
  EXPECT_EQ(out.fault_count, 0U);
}

TEST(SafetyBoot, WatchdogResetBootsIntoFaultAndReportsItOnce) {
  Rig rig(SafetyConfig{}, /*watchdog_boot=*/true);
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
  rig.tick();
  rig.run_ms(100, false);
  ASSERT_EQ(rig.reports.size(), 1U);
  EXPECT_EQ(rig.reports[0].code, FaultCode::WATCHDOG_RESET);
  EXPECT_EQ(rig.sup.fault_flags(), fault_bit(FaultCode::WATCHDOG_RESET));
  EXPECT_FALSE(rig.out.pitch_enabled);
}

TEST(SafetyBoot, AWatchdogResetFaultClearsAndThenArmsWithTwoRequests) {
  Rig rig(SafetyConfig{}, /*watchdog_boot=*/true);
  rig.heartbeat();
  rig.request(SafetyState::DISARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
  EXPECT_EQ(rig.sup.fault_flags(), 0);
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
}

// ------------------------------------------------------------------------ arm interlocks
//
// Interlock 2 (no fault latched, ArmInterlock::kFaultLatched) has no test, because no input
// sequence reaches it: every latch moves DISARMED to FAULT, and both ways back into DISARMED
// (clearing FAULT or ESTOP) leave zero flags. expect_outputs_consistent() asserts that
// invariant on every row of the transition table. The check stays in the code as defence in
// depth, as ADR 0014 lists it, so a future path into DISARMED with a flag still cannot arm.

TEST(SafetyArmInterlock, RefusedWithTheStickDeflected) {
  Rig rig;
  rig.drive(0.2F);
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
  ASSERT_EQ(rig.nacks.size(), 1U);
  EXPECT_EQ(rig.nacks[0].reason, NackReason::ARM_INTERLOCK);
  ASSERT_EQ(rig.reports.size(), 1U);
  EXPECT_EQ(rig.reports[0].code, FaultCode::NONE) << "an interlock is not a fault";
  EXPECT_EQ(rig.reports[0].context, static_cast<uint32_t>(ArmInterlock::kCommandNotZero));
  EXPECT_EQ(rig.sup.fault_flags(), 0) << "a refused arm latches nothing";
}

TEST(SafetyArmInterlock, AngularDeflectionAloneIsRefused) {
  Rig rig;
  rig.drive(0.0F, 0.5F);
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
  ASSERT_EQ(rig.reports.size(), 1U);
  EXPECT_EQ(rig.reports[0].context, static_cast<uint32_t>(ArmInterlock::kCommandNotZero));
}

TEST(SafetyArmInterlock, AcceptedOnceTheStickIsCentred) {
  Rig rig;
  rig.drive(0.2F);
  rig.request(SafetyState::ARMED);
  rig.drive(0.0F);
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
}

TEST(SafetyArmInterlock, StickWithinTheDeadbandCountsAsCentred) {
  Rig rig;
  rig.drive(0.009F, -0.009F);  // under kArmZeroLinear_m_s / kArmZeroAngular_rad_s
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
}

TEST(SafetyArmInterlock, NoDriveCommandSinceBootCountsAsCentred) {
  Rig rig;
  rig.heartbeat();
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
}

TEST(SafetyArmInterlock, RefusedOutOfSilence) {
  // The request's own frame proves nothing about the link before it.
  Rig rig;
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
  ASSERT_EQ(rig.nacks.size(), 1U);
  EXPECT_EQ(rig.nacks[0].reason, NackReason::ARM_INTERLOCK);
  ASSERT_EQ(rig.reports.size(), 1U);
  EXPECT_EQ(rig.reports[0].context, static_cast<uint32_t>(ArmInterlock::kLinkStale));
}

TEST(SafetyArmInterlock, LinkFreshnessBoundaryIsTheCommsTimeout) {
  for (const int gap_ms : {199, 200, 201}) {
    SCOPED_TRACE(gap_ms);
    Rig rig;
    rig.heartbeat();  // at t0; the request below lands at t0 + gap_ms
    rig.run_ms(gap_ms - 1, /*feed=*/false);
    rig.request(SafetyState::ARMED);
    const bool fresh = gap_ms < 200;
    EXPECT_EQ(rig.sup.state(), fresh ? SafetyState::ARMED : SafetyState::DISARMED);
  }
}

TEST(SafetyArmInterlock, TheStaleLinkCheckSurvivesAClockWrapAfterALongIdle) {
  // Heartbeat once at t = 0, then idle for one full uint32 wrap (71.6 min) plus a little. A
  // naive elapsed_us would read that heartbeat as a few ms old. 4 ms ticks keep LOOP_OVERRUN
  // (> 5 ms) quiet; the clock is set explicitly so the 64-bit true time is known.
  Rig rig(SafetyConfig{}, false, /*start_us=*/0);
  rig.heartbeat();
  constexpr uint64_t kIdleTickUs = 4000;
  constexpr uint64_t kWrapUs = 1ULL << 32;
  uint64_t t = kTickUs;
  for (; t < kWrapUs + 50000; t += kIdleTickUs) {
    rig.now_us = static_cast<uint32_t>(t);
    rig.tick();
  }
  rig.now_us = static_cast<uint32_t>(t);
  ASSERT_LT(rig.now_us, 200000U) << "a naive check would call the link fresh here";
  ASSERT_EQ(rig.sup.state(), SafetyState::DISARMED) << "idle is not a fault";
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
  ASSERT_FALSE(rig.reports.empty());
  EXPECT_EQ(rig.reports.back().context, static_cast<uint32_t>(ArmInterlock::kLinkStale));
}

TEST(SafetyArmInterlock, NotDisarmedHasItsOwnReasonAndNoFaultFrame) {
  Rig rig;
  rig.arm();
  rig.clear_history();
  rig.request(SafetyState::ARMED);
  ASSERT_EQ(rig.nacks.size(), 1U);
  EXPECT_EQ(rig.nacks[0].reason, NackReason::NOT_DISARMED);
  EXPECT_TRUE(rig.reports.empty());
}

// ------------------------------------------------------------------------ comms watchdog

TEST(SafetyCommsWatchdog, TripsAtExactly200msAfterTheLastDriveCommand) {
  for (const int gap_ms : {199, 200, 201}) {
    SCOPED_TRACE(gap_ms);
    Rig rig;
    rig.arm();
    rig.drive(0.1F);  // last DriveCommand at t_last
    rig.run_ms(gap_ms - 1, /*feed=*/false);
    rig.tick();       // lands at t_last + gap_ms
    const bool timed_out = gap_ms >= 200;
    EXPECT_EQ(rig.sup.state(), timed_out ? SafetyState::FAULT : SafetyState::ARMED);
    EXPECT_EQ(rig.reported(FaultCode::COMMS_TIMEOUT), timed_out);
  }
}

TEST(SafetyCommsWatchdog, ArmingStartsTheTimerFromTheArmRequest) {
  Rig rig;
  rig.arm();  // the request tick is t_arm = now - 1 ms
  rig.run_ms(199, false);  // t_arm + 1 .. t_arm + 199
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
  rig.tick();  // t_arm + 200 ms
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
}

TEST(SafetyCommsWatchdog, HeartbeatDoesNotFeedItWhileArmed) {
  // ADR 0014: an operator app whose input thread died but whose heartbeat thread lives must
  // not keep the robot driving its last command.
  Rig rig;
  rig.arm();
  rig.drive(0.2F);
  for (int i = 0; i < 300; ++i) {
    if (i % 20 == 0) {
      rig.heartbeat();
    } else {
      rig.tick();
    }
  }
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
  EXPECT_TRUE(rig.reported(FaultCode::COMMS_TIMEOUT));
}

TEST(SafetyCommsWatchdog, DriveCommandsAt50HzKeepItFed) {
  Rig rig;
  rig.arm();
  rig.run_ms(5000);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
  EXPECT_TRUE(rig.reports.empty());
}

TEST(SafetyCommsWatchdog, AnyValidFrameFeedsItWhileDisarmed) {
  Rig rig;
  for (int i = 0; i < 1000; ++i) {
    if (i % 20 == 0) {
      rig.heartbeat();
    } else {
      rig.tick();
    }
  }
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED) << "Heartbeat kept the link fresh";
}

TEST(SafetyCommsWatchdog, ATimeoutWhileDisarmedIsNotAFault) {
  Rig rig;
  rig.heartbeat();
  rig.run_ms(2000, false);
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
  EXPECT_EQ(rig.sup.fault_flags(), 0);
  EXPECT_TRUE(rig.reports.empty());
}

TEST(SafetyCommsWatchdog, UsesReceiveTimeCorrectlyAcrossAClockWrap) {
  Rig rig(SafetyConfig{}, false, /*start_us=*/0xFFFFFFFFU - 100000U);
  rig.arm();
  rig.drive(0.1F);  // ~100 ms before the wrap
  rig.run_ms(198, false);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
  rig.tick();       // 199 ms after
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
  rig.tick();       // 200 ms after, past the wrap
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
}

TEST(SafetyCommsWatchdog, TheTimeoutIsAParameter) {
  SafetyConfig cfg;
  cfg.comms_timeout_ms = 500;
  Rig rig(cfg);
  rig.arm();
  rig.drive(0.0F);
  rig.run_ms(499, false);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
  rig.tick();
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
}

TEST(SafetyCommsWatchdog, ALinkThatReturnsDoesNotResumeDriving) {
  // ADR 0014 Q1 (a): the robot never starts moving on its own when Wi-Fi comes back.
  Rig rig;
  rig.arm();
  rig.run_ms(300, false);
  ASSERT_EQ(rig.sup.state(), SafetyState::FAULT);
  rig.run_ms(500);  // DriveCommands flowing again
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
  EXPECT_FALSE(rig.out.drive_armed);
}

// ------------------------------------------------------------------- stop escalation

TEST(SafetyEscalation, FaultCoastsThenBrakesAtExactlyTheDelay) {
  Rig rig;
  rig.arm();
  SafetyInputs in = rig.inputs();
  in.encoder_fault_left = true;
  const SafetyOutputs at_entry = rig.tick(in);  // FAULT entered at this tick
  ASSERT_EQ(at_entry.state, SafetyState::FAULT);
  EXPECT_FALSE(at_entry.drive_armed);
  EXPECT_EQ(at_entry.wheel_stop, hal::StopMode::kCoast);

  rig.run_ms(protocol::param_defaults::fault_brake_delay_ms - 1, false);
  EXPECT_EQ(rig.out.wheel_stop, hal::StopMode::kCoast) << "799 ms after entry";
  rig.tick();
  EXPECT_EQ(rig.out.wheel_stop, hal::StopMode::kBrake) << "800 ms after entry";
  rig.run_ms(2000, false);
  EXPECT_EQ(rig.out.wheel_stop, hal::StopMode::kBrake) << "and stays braked";
}

TEST(SafetyEscalation, ALinkLossBrakesOneSecondAfterTheLastCommand) {
  Rig rig;
  rig.arm();
  rig.drive(0.2F);  // t_last
  rig.run_ms(199, false);
  rig.tick();       // t_last + 200: FAULT, coast
  ASSERT_EQ(rig.out.state, SafetyState::FAULT);
  EXPECT_EQ(rig.out.wheel_stop, hal::StopMode::kCoast);
  rig.run_ms(799, false);  // t_last + 999
  EXPECT_EQ(rig.out.wheel_stop, hal::StopMode::kCoast);
  rig.tick();              // t_last + 1000
  EXPECT_EQ(rig.out.wheel_stop, hal::StopMode::kBrake);
}

TEST(SafetyEscalation, ZeroDelayBrakesImmediately) {
  SafetyConfig cfg;
  cfg.fault_brake_delay_ms = 0;
  Rig rig(cfg);
  rig.arm();
  SafetyInputs in = rig.inputs();
  in.driver_fault = true;
  EXPECT_EQ(rig.tick(in).wheel_stop, hal::StopMode::kBrake);
}

TEST(SafetyEscalation, AFurtherFaultDoesNotRestartTheBrakeTimer) {
  Rig rig;
  rig.arm();
  SafetyInputs in = rig.inputs();
  in.encoder_fault_left = true;
  rig.tick(in);
  rig.run_ms(500, false);
  in = rig.inputs();
  in.driver_fault = true;
  rig.tick(in);
  rig.run_ms(299, false);  // entry + 502 .. entry + 800: 800 ms after the FIRST fault
  EXPECT_EQ(rig.out.wheel_stop, hal::StopMode::kBrake);
}

TEST(SafetyEscalation, EstopBrakesAtOnceFromEveryState) {
  for (SafetyState s : kAllStates) {
    SCOPED_TRACE(static_cast<int>(s));
    Rig rig;
    enter_state(rig, s);
    const SafetyOutputs out = rig.estop();
    EXPECT_EQ(out.state, SafetyState::ESTOP);
    EXPECT_EQ(out.wheel_stop, hal::StopMode::kBrake);
    EXPECT_FALSE(out.drive_armed);
    EXPECT_FALSE(out.pitch_enabled);
  }
}

TEST(SafetyEscalation, EstopWinsOverAnArmRequestInTheSameTick) {
  Rig rig;
  rig.heartbeat();
  SafetyInputs in = rig.inputs();
  in.estop = true;
  in.request = {true, SafetyState::ARMED, 3};
  rig.tick(in);
  EXPECT_EQ(rig.sup.state(), SafetyState::ESTOP);
}

TEST(SafetyEscalation, LeavingFaultWhileBrakingReturnsToCoast) {
  Rig rig;
  rig.arm();
  rig.run_ms(1300, false);  // comms timeout, then brake
  ASSERT_EQ(rig.out.wheel_stop, hal::StopMode::kBrake);
  rig.heartbeat();
  rig.request(SafetyState::DISARMED);
  EXPECT_EQ(rig.out.state, SafetyState::DISARMED);
  EXPECT_EQ(rig.out.wheel_stop, hal::StopMode::kCoast);
}

TEST(SafetyEscalation, RefaultingAfterAClearRestartsTheCoastPhase) {
  Rig rig;
  rig.arm();
  rig.run_ms(1300, false);
  rig.heartbeat();
  rig.request(SafetyState::DISARMED);
  SafetyInputs in = rig.inputs();
  in.driver_fault = true;
  EXPECT_EQ(rig.tick(in).wheel_stop, hal::StopMode::kCoast);
}

// ------------------------------------------------------------------------- clearing

TEST(SafetyClear, AnActiveDriverFaultRefusesTheClear) {
  Rig rig;
  rig.arm();
  SafetyInputs in = rig.inputs();
  in.driver_fault = true;
  rig.tick(in);
  in = rig.inputs();
  in.driver_fault = true;  // nFAULT still asserted
  in.request = {true, SafetyState::DISARMED, 9};
  rig.tick(in);
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
  ASSERT_EQ(rig.nacks.size(), 1U);
  EXPECT_EQ(rig.nacks[0].reason, NackReason::FAULT_ACTIVE);
  EXPECT_EQ(rig.nacks[0].rejected_seq, 9);

  rig.request(SafetyState::DISARMED);  // nFAULT released
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
}

TEST(SafetyClear, AStillGlitchingEncoderRefusesTheClear) {
  Rig rig;
  rig.arm();
  SafetyInputs in = rig.inputs();
  in.encoder_fault_right = true;
  rig.tick(in);
  ASSERT_EQ(rig.reports.size(), 1U);
  EXPECT_EQ(rig.reports[0].context, static_cast<uint32_t>(kRightWheel));
  in = rig.inputs();
  in.encoder_fault_right = true;
  in.request = {true, SafetyState::DISARMED, 1};
  rig.tick(in);
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
  EXPECT_EQ(rig.nacks.back().reason, NackReason::FAULT_ACTIVE);
}

TEST(SafetyClear, ClearsInactiveFlagsAndKeepsActiveOnes) {
  Rig rig;
  rig.arm();
  rig.run_ms(250, false);  // COMMS_TIMEOUT
  SafetyInputs in = rig.inputs();
  in.driver_fault = true;
  rig.tick(in);
  ASSERT_EQ(rig.sup.fault_flags(), fault_bit(FaultCode::COMMS_TIMEOUT) |
                                       fault_bit(FaultCode::MOTOR_DRIVER_FAULT));
  in = rig.inputs();
  in.driver_fault = true;
  in.request = {true, SafetyState::DISARMED, 1};
  rig.tick(in);
  EXPECT_EQ(rig.sup.fault_flags(), fault_bit(FaultCode::MOTOR_DRIVER_FAULT))
      << "COMMS_TIMEOUT can always be cleared: the request proves the link";
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
}

TEST(SafetyClear, AFaultSeenOnlyOnceStaysLatchedUntilCleared) {
  Rig rig;
  rig.arm();
  SafetyInputs in = rig.inputs();
  in.driver_fault = true;
  rig.tick(in);
  rig.run_ms(3000, false);  // the condition is long gone
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
  EXPECT_NE(rig.sup.fault_flags() & fault_bit(FaultCode::MOTOR_DRIVER_FAULT), 0);
}

TEST(SafetyClear, AHeldConditionIsReportedOnceNotEveryTick) {
  Rig rig;
  rig.heartbeat();
  for (int i = 0; i < 100; ++i) {
    SafetyInputs in = rig.inputs();
    in.driver_fault = true;
    rig.tick(in);
  }
  EXPECT_EQ(rig.reports.size(), 1U);
}

TEST(SafetyClear, EstopWithAnActiveFaultRefusesToLeave) {
  Rig rig;
  rig.heartbeat();
  rig.estop();
  SafetyInputs in = rig.inputs();
  in.driver_fault = true;
  in.request = {true, SafetyState::DISARMED, 1};
  rig.tick(in);
  EXPECT_EQ(rig.sup.state(), SafetyState::ESTOP);
  EXPECT_EQ(rig.nacks.back().reason, NackReason::FAULT_ACTIVE);
  rig.request(SafetyState::DISARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
  EXPECT_EQ(rig.sup.fault_flags(), 0);
}

TEST(SafetyClear, ReArmingAfterAFaultTakesTwoRequests) {
  Rig rig;
  rig.arm();
  rig.run_ms(250, false);
  ASSERT_EQ(rig.sup.state(), SafetyState::FAULT);
  rig.heartbeat();
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT) << "FAULT never goes straight to ARMED";
  rig.request(SafetyState::DISARMED);
  rig.request(SafetyState::ARMED);
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
}

// ---------------------------------------------------------------------- detectors

TEST(SafetyLoopOverrun, AGapOfExactlyTheLimitIsTolerated) {
  Rig rig;
  rig.heartbeat();
  rig.now_us += protocol::param_defaults::loop_overrun_fault_us - kTickUs;  // gap = 5000 us
  rig.tick();
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
}

TEST(SafetyLoopOverrun, OneMicrosecondMoreTripsWithTheGapAsContext) {
  Rig rig;
  rig.heartbeat();
  rig.now_us += protocol::param_defaults::loop_overrun_fault_us - kTickUs + 1;  // 5001 us
  rig.tick();
  EXPECT_EQ(rig.sup.state(), SafetyState::FAULT);
  ASSERT_EQ(rig.reports.size(), 1U);
  EXPECT_EQ(rig.reports[0].code, FaultCode::LOOP_OVERRUN);
  EXPECT_EQ(rig.reports[0].context, 5001U);
}

TEST(SafetyLoopOverrun, TheFirstStepAfterBootHasNoGap) {
  Rig rig(SafetyConfig{}, false, /*start_us=*/123456789);
  rig.tick();
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
}

TEST(SafetyWheelStall, OnlyRunsWhileArmed) {
  Rig rig;
  rig.heartbeat();
  for (int i = 0; i < 2000; ++i) {
    SafetyInputs in = rig.inputs();
    in.left = {kLimit, kLimit, 0.0F, true};
    if (i % 20 == 0) {
      in.other_frame = true;
    }
    rig.tick(in);
  }
  EXPECT_EQ(rig.sup.state(), SafetyState::DISARMED);
}

TEST(SafetyWheelStall, ContextNamesTheWheel) {
  Rig rig;
  rig.arm();
  for (int i = 0; i < 600; ++i) {
    SafetyInputs in = rig.inputs();
    in.right = {-kLimit, kLimit, 0.0F, true};
    in.drive_command = i % 20 == 0;
    rig.tick(in);
  }
  ASSERT_EQ(rig.reports.size(), 1U);
  EXPECT_EQ(rig.reports[0].code, FaultCode::WHEEL_STALL);
  EXPECT_EQ(rig.reports[0].context, static_cast<uint32_t>(kRightWheel));
}

TEST(SafetyWheelStall, ATimerFromABriefArmedSpellDoesNotCarryOver) {
  Rig rig;
  rig.arm();
  for (int i = 0; i < 400; ++i) {  // 400 ms of stall, then disarm
    SafetyInputs in = rig.inputs();
    in.left = {kLimit, kLimit, 0.0F, true};
    in.drive_command = i % 20 == 0;
    rig.tick(in);
  }
  rig.request(SafetyState::DISARMED);
  rig.request(SafetyState::ARMED);
  for (int i = 0; i < 200; ++i) {  // 200 more: 600 total, but only 200 in this spell
    SafetyInputs in = rig.inputs();
    in.left = {kLimit, kLimit, 0.0F, true};
    in.drive_command = i % 20 == 0;
    rig.tick(in);
  }
  EXPECT_EQ(rig.sup.state(), SafetyState::ARMED);
}

// ------------------------------------------------------------------- pitch enable (Q6)

TEST(SafetyPitch, KeepsRunningThroughACommsTimeoutOnly) {
  Rig rig;
  rig.arm();
  EXPECT_TRUE(rig.out.pitch_enabled);
  rig.run_ms(250, false);
  ASSERT_EQ(rig.out.state, SafetyState::FAULT);
  EXPECT_TRUE(rig.out.pitch_enabled) << "Q6: the camera stays level through a lost link";
  SafetyInputs in = rig.inputs();
  in.encoder_fault_left = true;
  rig.tick(in);
  EXPECT_FALSE(rig.out.pitch_enabled) << "any second fault disables it";
}

TEST(SafetyPitch, DisabledInDisarmedAndOnEveryOtherFault) {
  Rig rig;
  rig.heartbeat();
  EXPECT_FALSE(rig.out.pitch_enabled);
  rig.arm();
  SafetyInputs in = rig.inputs();
  in.driver_fault = true;
  rig.tick(in);
  EXPECT_FALSE(rig.out.pitch_enabled);
}

TEST(SafetyConfigTest, DefaultsComeFromTheSchema) {
  const SafetyConfig c;
  EXPECT_EQ(c.comms_timeout_ms, 200);
  EXPECT_EQ(c.fault_brake_delay_ms, 800);
  EXPECT_EQ(c.wheel_stall_ms, 500);
  EXPECT_EQ(c.loop_overrun_fault_us, 5000);
}

TEST(SafetyFaultBit, CodeNIsBitNMinusOne) {
  EXPECT_EQ(fault_bit(FaultCode::COMMS_TIMEOUT), 0x0001);
  EXPECT_EQ(fault_bit(FaultCode::WHEEL_STALL), 0x0080);
  EXPECT_EQ(fault_bit(FaultCode::WATCHDOG_RESET), 0x0100);
}

}  // namespace
}  // namespace recon::core
