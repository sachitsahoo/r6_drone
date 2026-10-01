#include "control/wheel_velocity_controller.hpp"

#include <cmath>

namespace recon::core {

float WheelVelocityController::clamp_to_limit(float u) const {
  const float lim = gains_.duty_limit;
  return std::fmax(-lim, std::fmin(lim, u));
}

float WheelVelocityController::update(float ref_rad_s, float meas_rad_s, float dt_s) {
  // A lowered limit must also bound an integrator stored under the old one.
  integrator_ = clamp_to_limit(integrator_);

  const float error = ref_rad_s - meas_rad_s;
  const float unclamped = gains_.kff * ref_rad_s + gains_.kp * error + integrator_;
  const float output = clamp_to_limit(unclamped);
  saturated_ = output != unclamped;

  // Conditional integration: while saturated, integrate only if the error pulls the output
  // back toward the linear range. Otherwise the integrator would store duty that can never
  // be applied and must later unwind (windup).
  const bool pushes_further = (unclamped > 0.0F) == (error > 0.0F);
  if (!(saturated_ && pushes_further)) {
    integrator_ = clamp_to_limit(integrator_ + gains_.ki * error * dt_s);
  }
  return output;
}

float WheelVelocityController::feedforward_only(float ref_rad_s) const {
  return clamp_to_limit(gains_.kff * ref_rad_s);
}

}  // namespace recon::core
