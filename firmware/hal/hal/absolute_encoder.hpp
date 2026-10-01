#pragma once

#include <cstdint>

namespace recon::hal {

/// One reading of the pitch encoder.
struct AngleSample {
  /// Casing angle relative to the chassis, within one turn, in [0, 2*pi) rad. Positive
  /// direction is the casing rotating nose-down (positive about +y) relative to the chassis.
  float angle_rad;
  /// When the sample was taken, from `Clock::now_us()`.
  uint32_t timestamp_us;
  /// False on a magnet-field fault or a communication error. `angle_rad` is then unspecified.
  bool valid;
};

/// The single absolute encoder on the pitch axis (ADR 0011).
///
/// With direct drive the rotor angle *is* the casing-to-chassis angle, so this one sensor
/// serves both FOC commutation and the estimator's relative angle `phi` (ADR 0005). It reads
/// off-axis, because wheel A's shaft runs through the motor.
///
/// Reports angle within one turn only. Counting turns for the wire loop (ADR 0009) and
/// applying the calibrated zero offset are `core`'s job.
class AbsoluteEncoder {
 public:
  /// The most recent reading.
  /// \note ISR-safe: the FOC loop reads it from a timer interrupt.
  virtual AngleSample latest() const = 0;

 protected:
  ~AbsoluteEncoder() = default;  // see Clock
};

}  // namespace recon::hal
