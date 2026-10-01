#pragma once

#include <cstdint>

namespace recon::core {

/// Below this |speed| a saturated wheel counts as "not turning", rad/s.
/// ADR 0014: about 2 encoder counts per 10 ms window, safely above the estimator's
/// 0.449 rad/s resolution (1400 counts/rev, ADR 0013).
inline constexpr float kStallSpeed_rad_s = 1.0F;

/// |duty| within this of the duty limit counts as saturated. The controller clamps to
/// exactly the limit (std::fmin), so the tolerance only absorbs float rounding in a limit
/// that came over the protocol. Dimensionless duty.
inline constexpr float kSaturationTolerance = 1e-4F;

/// Detects WHEEL_STALL for one wheel. Design: ADR 0014, "Fault detectors".
///
/// Trips when the output has been saturated **and** the wheel is either not turning
/// (|speed| < kStallSpeed_rad_s) or turning against its duty, continuously for the stall time.
/// That catches an unplugged encoder (reads a valid 0, so the integrator winds to the limit)
/// and a reversed encoder (positive feedback), neither of which centring the stick can stop.
/// It deliberately ignores saturation alone: full stick on carpet saturates, but the wheel
/// still turns the right way at ~5 rad/s.
///
/// Pure logic: no HAL, no clock. Time comes in as `now_us`. Not ISR-safe (call from the one
/// loop that owns it).
class WheelStallDetector {
 public:
  /// One sample of the wheel, taken after `DriveLoop::step`.
  struct Sample {
    float duty;          ///< Duty the loop wrote, in [-1, 1].
    float duty_limit;    ///< Controller's |duty| ceiling (param wheel_duty_limit).
    float speed_rad_s;   ///< Estimated wheel speed. Positive = robot forward.
    bool speed_valid;    ///< Estimator validity. An invalid speed is not evidence of a stall.
  };

  /// Feeds one sample. \return True on the step the condition has held for `stall_us`.
  /// Keeps returning true while it continues to hold.
  /// \param stall_us Condition duration that trips, us (param wheel_stall_ms * 1000).
  bool update(const Sample& sample, uint32_t now_us, uint32_t stall_us);

  /// Forgets any partial stall. Call whenever the detector is inactive (outside ARMED).
  void reset() { active_ = false; }

 private:
  bool active_ = false;
  uint32_t since_us_ = 0;
};

}  // namespace recon::core
