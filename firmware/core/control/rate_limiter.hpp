#pragma once

namespace recon::core {

/// Moves an output toward a target by at most `max_step` per call.
///
/// Used on each wheel's speed reference to enforce the acceleration limit (ADR 0013):
/// `max_step = accel_limit / wheel_radius * dt`. Lands exactly on the target, never past it.
class RateLimiter {
 public:
  /// \param target   Desired value.
  /// \param max_step Largest change allowed this call, >= 0 (a negative value is treated as 0).
  /// \return The new output.
  float step(float target, float max_step);

  /// Jumps the output to `value` with no limiting.
  void reset(float value = 0.0F) { value_ = value; }

  float value() const { return value_; }

 private:
  float value_ = 0.0F;
};

}  // namespace recon::core
