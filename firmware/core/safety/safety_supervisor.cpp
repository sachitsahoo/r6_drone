#include "safety/safety_supervisor.hpp"

#include <cmath>

#include "time/timestamp.hpp"

namespace recon::core {

using protocol::FaultCode;
using protocol::NackReason;
using protocol::SafetyState;

namespace {

constexpr uint32_t kUsPerMs = 1000;

}  // namespace

SafetySupervisor::SafetySupervisor(const SafetyConfig& config, bool boot_after_watchdog_reset)
    : config_(config),
      state_(boot_after_watchdog_reset ? SafetyState::FAULT : SafetyState::DISARMED) {
  if (boot_after_watchdog_reset) {
    // ADR 0014 Q5: a silent reset hides a crash. The report goes out on the first step,
    // because the constructor has no output to put it in.
    fault_flags_ = fault_bit(FaultCode::WATCHDOG_RESET);
    report_watchdog_reset_ = true;
  }
}

SafetyOutputs SafetySupervisor::step(const SafetyInputs& in) {
  SafetyOutputs out;
  const uint32_t now = in.now_us;

  if (report_watchdog_reset_) {
    add_report(out, FaultCode::WATCHDOG_RESET, 0);
    report_watchdog_reset_ = false;
  }

  // 1. E-stop first: nothing else in this tick can delay or undo it.
  if (in.estop) {
    enter(SafetyState::ESTOP);
  }

  // 2. Detectors. Each latches its flag; latch() moves ARMED or DISARMED to FAULT.
  overrun_this_step_ = false;
  if (have_last_step_) {
    const uint32_t gap_us = elapsed_us(last_step_us_, now);
    if (gap_us > config_.loop_overrun_fault_us) {
      overrun_this_step_ = true;
      latch(FaultCode::LOOP_OVERRUN, gap_us, out);
    }
  }
  last_step_us_ = now;
  have_last_step_ = true;

  if (in.encoder_fault_left || in.encoder_fault_right) {
    const uint32_t wheels = (in.encoder_fault_left ? kLeftWheel : 0U) |
                            (in.encoder_fault_right ? kRightWheel : 0U);
    latch(FaultCode::ENCODER_FAULT, wheels, out);
  }
  if (in.driver_fault) {
    latch(FaultCode::MOTOR_DRIVER_FAULT, 0, out);
  }
  if (state_ == SafetyState::ARMED) {
    const uint32_t stall_us = static_cast<uint32_t>(config_.wheel_stall_ms) * kUsPerMs;
    const bool left = stall_left_.update(in.left, now, stall_us);
    const bool right = stall_right_.update(in.right, now, stall_us);
    if (left || right) {
      latch(FaultCode::WHEEL_STALL, (left ? kLeftWheel : 0U) | (right ? kRightWheel : 0U), out);
    }
  }

  // 3. Comms watchdog. In ARMED only DriveCommand feeds it (ADR 0014): a live heartbeat
  // thread must not keep a robot driving whose input thread has died.
  if (in.drive_command) {
    last_drive_zero_ = std::fabs(in.drive_linear_m_s) < kArmZeroLinear_m_s &&
                       std::fabs(in.drive_angular_rad_s) < kArmZeroAngular_rad_s;
  }
  const bool feeds = state_ == SafetyState::ARMED ? in.drive_command
                                                  : (in.drive_command || in.other_frame);
  if (feeds) {
    feed_link(now);
  }
  const uint32_t silence_us = elapsed_us(last_feed_us_, now);
  if (link_fresh_ && silence_us >= static_cast<uint32_t>(config_.comms_timeout_ms) * kUsPerMs) {
    link_fresh_ = false;  // latched until the next feed, so a 71.6 min idle cannot wrap it
  }
  if (state_ == SafetyState::ARMED && !link_fresh_) {
    latch(FaultCode::COMMS_TIMEOUT, silence_us, out);
  }

  // 4. The operator request, judged on the state the detectors left.
  handle_request(in, out);
  if (in.request.present && state_ != SafetyState::ARMED) {
    feed_link(now);  // a valid request is a valid frame; in ARMED only DriveCommand counts
  }

  fill_outputs(now, out);
  return out;
}

void SafetySupervisor::latch(FaultCode code, uint32_t context, SafetyOutputs& out) {
  const uint16_t bit = fault_bit(code);
  if ((fault_flags_ & bit) == 0) {
    fault_flags_ = static_cast<uint16_t>(fault_flags_ | bit);
    add_report(out, code, context);  // rising edge only
  }
  // FAULT and ESTOP keep their state: a further fault is recorded, not acted on again.
  if (state_ == SafetyState::ARMED || state_ == SafetyState::DISARMED) {
    enter(SafetyState::FAULT);
  }
}

void SafetySupervisor::enter(SafetyState next) {
  if (next == SafetyState::FAULT && state_ != SafetyState::FAULT) {
    fault_timer_started_ = false;  // fill_outputs() starts it on this tick
  }
  if (next != SafetyState::ARMED) {
    stall_left_.reset();
    stall_right_.reset();
  }
  state_ = next;
}

void SafetySupervisor::handle_request(const SafetyInputs& in, SafetyOutputs& out) {
  const SafetyRequest& req = in.request;
  if (!req.present) {
    return;
  }
  switch (req.requested_state) {
    case SafetyState::ESTOP:
      // Stopping is never refused, whichever message asks for it.
      enter(SafetyState::ESTOP);
      return;
    case SafetyState::FAULT:
      // FAULT is entered by detectors only; the operator cannot request it.
      nack(out, req, NackReason::RANGE_REJECT);
      return;
    case SafetyState::ARMED: {
      if (state_ != SafetyState::DISARMED) {
        nack(out, req, NackReason::NOT_DISARMED);  // interlock 1
        return;
      }
      ArmInterlock failed{};
      bool ok = true;
      if (fault_flags_ != 0) {
        failed = ArmInterlock::kFaultLatched;
        ok = false;
      } else if (!link_fresh_) {
        failed = ArmInterlock::kLinkStale;
        ok = false;
      } else if (!last_drive_zero_) {
        failed = ArmInterlock::kCommandNotZero;
        ok = false;
      }
      if (!ok) {
        nack(out, req, NackReason::ARM_INTERLOCK);
        add_report(out, FaultCode::NONE, static_cast<uint32_t>(failed));
        return;
      }
      enter(SafetyState::ARMED);
      // The ARMED watchdog starts from the arm request: the operator has one timeout to get
      // the first DriveCommand through.
      feed_link(in.now_us);
      return;
    }
    case SafetyState::DISARMED:
      switch (state_) {
        case SafetyState::DISARMED:
          return;
        case SafetyState::ARMED:
          enter(SafetyState::DISARMED);
          return;
        case SafetyState::FAULT:
        case SafetyState::ESTOP:
          if (try_clear(in)) {
            enter(SafetyState::DISARMED);
          } else {
            nack(out, req, NackReason::FAULT_ACTIVE);
          }
          return;
      }
      return;
  }
}

bool SafetySupervisor::try_clear(const SafetyInputs& in) {
  // Conditions still present this tick. COMMS_TIMEOUT, WHEEL_STALL and WATCHDOG_RESET are
  // never "still active" here: the request proves the link, stall detection runs only in
  // ARMED, and a reset is a past event.
  uint16_t active = 0;
  if (in.encoder_fault_left || in.encoder_fault_right) {
    active = static_cast<uint16_t>(active | fault_bit(FaultCode::ENCODER_FAULT));
  }
  if (in.driver_fault) {
    active = static_cast<uint16_t>(active | fault_bit(FaultCode::MOTOR_DRIVER_FAULT));
  }
  if (overrun_this_step_) {
    active = static_cast<uint16_t>(active | fault_bit(FaultCode::LOOP_OVERRUN));
  }
  fault_flags_ = static_cast<uint16_t>(fault_flags_ & active);
  return fault_flags_ == 0;
}

void SafetySupervisor::feed_link(uint32_t now_us) {
  last_feed_us_ = now_us;
  link_fresh_ = true;
}

void SafetySupervisor::fill_outputs(uint32_t now_us, SafetyOutputs& out) {
  out.state = state_;
  out.fault_flags = fault_flags_;
  out.drive_armed = state_ == SafetyState::ARMED;
  switch (state_) {
    case SafetyState::ARMED:
      out.wheel_stop = hal::StopMode::kCoast;  // unused: the drive loop owns the motors
      out.pitch_enabled = true;
      break;
    case SafetyState::DISARMED:
      out.wheel_stop = hal::StopMode::kCoast;  // can be pushed and carried by hand
      out.pitch_enabled = false;
      break;
    case SafetyState::FAULT:
      if (!fault_timer_started_) {
        fault_timer_started_ = true;
        fault_since_us_ = now_us;
        fault_braking_ = false;
      }
      // Latched once reached, so the timer never has to measure past the 71.6 min wrap.
      if (!fault_braking_ &&
          elapsed_us(fault_since_us_, now_us) >=
              static_cast<uint32_t>(config_.fault_brake_delay_ms) * kUsPerMs) {
        fault_braking_ = true;
      }
      out.wheel_stop = fault_braking_ ? hal::StopMode::kBrake : hal::StopMode::kCoast;
      // ADR 0014 Q6: the one exception to "motors only move when ARMED".
      out.pitch_enabled = fault_flags_ == fault_bit(FaultCode::COMMS_TIMEOUT);
      break;
    case SafetyState::ESTOP:
      out.wheel_stop = hal::StopMode::kBrake;
      out.pitch_enabled = false;
      break;
  }
}

void SafetySupervisor::add_report(SafetyOutputs& out, FaultCode code, uint32_t context) {
  if (out.fault_count < kMaxFaultReportsPerTick) {
    out.faults[out.fault_count++] = FaultReport{code, context};
  }
}

void SafetySupervisor::nack(SafetyOutputs& out, const SafetyRequest& req, NackReason reason) {
  out.nack.present = true;
  out.nack.rejected_message_id = static_cast<uint8_t>(protocol::MessageId::SafetyStateRequest);
  out.nack.reason = reason;
  out.nack.rejected_seq = req.seq;
}

}  // namespace recon::core
