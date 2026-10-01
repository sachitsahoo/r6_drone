#pragma once

#include <cstddef>
#include <cstdint>

#include "hal/wheel_motor.hpp"  // hal::StopMode only: an enum, not a HAL call
#include "messages.hpp"         // generated: SafetyState, FaultCode, NackReason, param_defaults
#include "safety/wheel_stall_detector.hpp"

namespace recon::core {

/// Every tunable of the safety machine. Defaults come from params.yaml (block 0x0200).
struct SafetyConfig {
  uint16_t comms_timeout_ms = protocol::param_defaults::comms_timeout_ms;
  uint16_t fault_brake_delay_ms = protocol::param_defaults::fault_brake_delay_ms;
  uint16_t wheel_stall_ms = protocol::param_defaults::wheel_stall_ms;
  uint16_t loop_overrun_fault_us = protocol::param_defaults::loop_overrun_fault_us;
};

/// |linear| below this counts as a centred stick for the arm interlock, m/s (ADR 0014).
inline constexpr float kArmZeroLinear_m_s = 0.01F;
/// |angular| below this counts as a centred stick for the arm interlock, rad/s (ADR 0014).
inline constexpr float kArmZeroAngular_rad_s = 0.01F;

/// Which arm interlock failed. Sent as the `context` of a `Fault(NONE)` frame alongside
/// `Nack(ARM_INTERLOCK)`. Numbers match the list in ADR 0014, "Arm interlocks"; condition 1
/// (not DISARMED) has its own Nack reason and no Fault frame.
enum class ArmInterlock : uint32_t {
  kFaultLatched = 2,     ///< A fault flag is latched. Defensive: DISARMED never holds a flag.
  kLinkStale = 3,        ///< No valid frame (other than this request) within comms_timeout_ms.
  kCommandNotZero = 4,   ///< The last DriveCommand was not centred.
};

/// Bits of the `context` of a WHEEL_STALL or ENCODER_FAULT frame: which wheels tripped.
inline constexpr uint32_t kLeftWheel = 1U << 0;
inline constexpr uint32_t kRightWheel = 1U << 1;

/// The fault_flags bit for a code: code n is bit n-1 (ADR 0014).
constexpr uint16_t fault_bit(protocol::FaultCode code) {
  return static_cast<uint16_t>(1U << (static_cast<uint16_t>(code) - 1U));
}

/// An operator request carried by a valid SafetyStateRequest frame.
struct SafetyRequest {
  bool present = false;
  protocol::SafetyState requested_state = protocol::SafetyState::DISARMED;
  uint8_t seq = 0;  ///< The request frame's seq, echoed in any Nack.
};

/// Everything the supervisor sees in one motor-loop tick. Built by the glue from decoded,
/// validated frames (full decoder chain plus stale-timestamp rejection) and detector flags.
struct SafetyInputs {
  /// MCU receive time, `Clock::now_us()`. Never a frame's own timestamp.
  uint32_t now_us = 0;

  // --- frames received since the last tick (all passed validation) ---
  /// A valid EstopRequest. Kept apart from `request` so an e-stop is never queued behind one.
  bool estop = false;
  /// At most one SafetyStateRequest per tick; the glue holds any extra for the next tick.
  SafetyRequest request{};
  /// A valid DriveCommand arrived. Its values follow.
  bool drive_command = false;
  float drive_linear_m_s = 0.0F;
  float drive_angular_rad_s = 0.0F;
  /// Any OTHER valid operator-to-robot frame arrived (Heartbeat, Param*). Do not set this for
  /// the SafetyStateRequest in `request`; the supervisor counts that one itself.
  bool other_frame = false;

  // --- detectors, sampled after DriveLoop::step ---
  WheelStallDetector::Sample left{};
  WheelStallDetector::Sample right{};
  bool encoder_fault_left = false;   ///< Estimator plausibility trip this step.
  bool encoder_fault_right = false;
  bool driver_fault = false;         ///< PitchPowerStage::fault() (DRV8313 nFAULT), a level.
};

/// One Fault frame to send. `code == NONE` reports a failed arm interlock (see ArmInterlock).
struct FaultReport {
  protocol::FaultCode code;
  uint32_t context;
};

/// One Nack frame to send, answering the request in the same tick.
struct NackReport {
  bool present = false;
  uint8_t rejected_message_id = 0;
  protocol::NackReason reason = protocol::NackReason::UNSPECIFIED;
  uint8_t rejected_seq = 0;
};

/// Most Fault frames one tick can produce: one per FaultCode bit, plus one interlock report.
inline constexpr size_t kMaxFaultReportsPerTick = 17;

/// What the glue must do after a tick.
struct SafetyOutputs {
  protocol::SafetyState state = protocol::SafetyState::DISARMED;
  uint16_t fault_flags = 0;
  /// True only in ARMED: pass to `DriveLoop::step(armed)`.
  bool drive_armed = false;
  /// When `!drive_armed`, call `WheelMotor::stop(wheel_stop)` on both wheels.
  hal::StopMode wheel_stop = hal::StopMode::kCoast;
  /// `PitchPowerStage::set_enabled(pitch_enabled)`. ARMED, or FAULT with only COMMS_TIMEOUT.
  bool pitch_enabled = false;
  FaultReport faults[kMaxFaultReportsPerTick] = {};
  size_t fault_count = 0;
  NackReport nack{};
};

/// The safety state machine of ADR 0014: DISARMED -> ARMED -> FAULT / ESTOP, the comms
/// watchdog, fault latching and clearing, and the coast-then-brake escalation.
///
/// Pure logic. It never touches the HAL: inputs arrive as a struct, decisions leave as a
/// struct, so every transition can be tested from a table (tests/cpp/test_safety_supervisor.cpp).
///
/// Call `step()` once per motor-loop iteration (1 kHz), immediately BEFORE
/// `DriveLoop::step(outputs.drive_armed)`, feeding the detector samples from the previous
/// DriveLoop step. Because it is stepped by the motor loop, the gap between two of its own
/// steps is the motor-loop gap, which is how it detects LOOP_OVERRUN without a timer.
///
/// Order inside one tick, so a request and a fault in the same millisecond resolve safely:
/// e-stop, then detectors and the comms watchdog, then the operator request, then outputs.
///
/// Wraparound: every timer it keeps is bounded by a latch (stale link, brake reached), and it
/// is stepped every millisecond, so no measured interval ever approaches the 71.6 min wrap.
///
/// \note Not ISR-safe; owned by the motor loop. No allocation.
class SafetySupervisor {
 public:
  /// \param boot_after_watchdog_reset `hal::Watchdog::reset_was_watchdog()` at boot. True
  ///        boots into FAULT with WATCHDOG_RESET latched (ADR 0014 Q5), reported on the first
  ///        step.
  explicit SafetySupervisor(const SafetyConfig& config = SafetyConfig{},
                            bool boot_after_watchdog_reset = false);

  /// One tick. See the class comment for ordering.
  SafetyOutputs step(const SafetyInputs& in);

  /// New tunables, effective from the next step. Running timers keep their start times.
  void set_config(const SafetyConfig& config) { config_ = config; }

  protocol::SafetyState state() const { return state_; }
  uint16_t fault_flags() const { return fault_flags_; }
  const SafetyConfig& config() const { return config_; }

 private:
  void latch(protocol::FaultCode code, uint32_t context, SafetyOutputs& out);
  void enter(protocol::SafetyState next);
  void handle_request(const SafetyInputs& in, SafetyOutputs& out);
  bool try_clear(const SafetyInputs& in);
  void feed_link(uint32_t now_us);
  void fill_outputs(uint32_t now_us, SafetyOutputs& out);
  static void add_report(SafetyOutputs& out, protocol::FaultCode code, uint32_t context);
  static void nack(SafetyOutputs& out, const SafetyRequest& req, protocol::NackReason reason);

  SafetyConfig config_;
  protocol::SafetyState state_;
  uint16_t fault_flags_ = 0;
  bool report_watchdog_reset_ = false;

  // Comms watchdog. `link_fresh_` latches false once the timeout passes, so a long idle
  // DISARMED period can never wrap back into looking fresh.
  bool link_fresh_ = false;
  uint32_t last_feed_us_ = 0;

  bool last_drive_zero_ = true;  // "or none has arrived since boot" counts as zero

  // FAULT escalation: coast from `fault_since_us_`, brake once the delay has passed. The
  // timer starts in fill_outputs() on the tick FAULT is entered (or the first tick, after a
  // watchdog-reset boot, when the constructor had no time to start it from).
  bool fault_timer_started_ = false;
  uint32_t fault_since_us_ = 0;
  bool fault_braking_ = false;

  // LOOP_OVERRUN: gap between this step and the previous one.
  bool have_last_step_ = false;
  uint32_t last_step_us_ = 0;
  bool overrun_this_step_ = false;

  WheelStallDetector stall_left_;
  WheelStallDetector stall_right_;
};

}  // namespace recon::core
