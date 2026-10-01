#pragma once

#include "messages.hpp"  // generated: param_defaults

namespace recon::core {

/// Gains and limit for one wheel's velocity loop. Defaults come from params.yaml.
struct WheelVelocityGains {
  /// Feedforward, duty per rad/s of reference.
  float kff = protocol::param_defaults::wheel_kff;
  /// Proportional, duty per rad/s of error.
  float kp = protocol::param_defaults::wheel_kp;
  /// Integral, duty per rad of accumulated error.
  float ki = protocol::param_defaults::wheel_ki;
  /// Largest |duty| commanded. Saturates; does not scale (ADR 0013, question 3).
  float duty_limit = protocol::param_defaults::wheel_duty_limit;
};

/// PI plus feedforward for one wheel, with conditional-integration anti-windup.
///
/// Design: ADR 0013, "Controller". Theory: docs/theory/wheel-velocity-loop.md §3.
///
///     e = ref - meas
///     u = clamp(kff*ref + kp*e + I, +/-duty_limit)
///     I += ki*e*dt   unless u is saturated and e would push it further
///
/// The integrator is stored in duty units (not as the integral of e), so changing ki
/// changes only future accumulation and never steps the output. It is also bounded to
/// +/-duty_limit: a larger value could only ever be clipped.
///
/// No derivative term: speed comes from quantised counts, and its derivative is noise
/// (ADR 0013; docs/theory/wheel-velocity-loop.md §3.2).
///
/// Pure arithmetic: no HAL calls, no allocation. Call from one context only.
class WheelVelocityController {
 public:
  explicit WheelVelocityController(const WheelVelocityGains& gains = WheelVelocityGains{})
      : gains_(gains) {}

  /// One loop iteration.
  ///
  /// \param ref_rad_s  Wheel speed reference, rad/s, positive forward.
  /// \param meas_rad_s Measured wheel speed, rad/s. Must be a valid estimate.
  /// \param dt_s       Time since the previous call, s. >= 0.
  /// \return Duty in [-duty_limit, +duty_limit].
  float update(float ref_rad_s, float meas_rad_s, float dt_s);

  /// Feedforward only, integrator untouched. For iterations with no valid speed estimate:
  /// the loop stays bounded and open-loop rather than acting on a bad measurement.
  float feedforward_only(float ref_rad_s) const;

  /// Zeroes the integrator. Called whenever the robot is not ARMED.
  void reset() { integrator_ = 0.0F; saturated_ = false; }

  /// Takes effect on the next `update`; the integrator is kept (bumpless).
  void set_gains(const WheelVelocityGains& gains) { gains_ = gains; }

  const WheelVelocityGains& gains() const { return gains_; }
  /// Integrator state, duty. Its settled value is the surface's extra load (ADR 0013).
  float integrator() const { return integrator_; }
  /// Whether the last `update` hit the duty limit.
  bool saturated() const { return saturated_; }

 private:
  float clamp_to_limit(float u) const;

  WheelVelocityGains gains_;
  float integrator_ = 0.0F;
  bool saturated_ = false;
};

}  // namespace recon::core
