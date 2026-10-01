#pragma once

#include <cstdint>

namespace recon::hal {

/// One N20 quadrature encoder.
///
/// A raw, wrapping counter rather than a velocity. Differentiating, filtering and handling
/// the wrap are `core`'s job, so the wrap handling is written and tested once (hard rule 7)
/// instead of once per implementation.
class WheelEncoder {
 public:
  /// Encoder counts since boot. Wraps at 2^32: take differences with unsigned subtraction.
  /// Positive is the wheel rolling the robot forward; the implementation flips the sign for
  /// the mirrored side.
  /// \note ISR-safe.
  virtual uint32_t count() const = 0;

  /// Counts per wheel revolution, gearbox included. Constant for a given build.
  /// \note ISR-safe.
  virtual float counts_per_rev() const = 0;

 protected:
  ~WheelEncoder() = default;  // see Clock
};

}  // namespace recon::hal
