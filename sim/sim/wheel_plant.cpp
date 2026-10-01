#include "sim/wheel_plant.hpp"

#include <cmath>

namespace recon::sim {

namespace {

constexpr double kTwoPi = 6.283185307179586;

}  // namespace

SimWheel::SimWheel(const WheelPlantParams& params) : params_(params) {}

void SimWheel::set_duty(float duty) {
  if (std::isnan(duty)) {
    duty = 0.0F;
  }
  // Clamp first, then the output limit, in that order per the hal::WheelMotor contract.
  const float clamped = std::fmax(-1.0F, std::fmin(1.0F, duty));
  const float limit = params_.output_limit;
  applied_duty_ = std::fmax(-limit, std::fmin(limit, clamped));
  mode_ = Mode::kDrive;
}

void SimWheel::stop(hal::StopMode mode) {
  applied_duty_ = 0.0F;
  mode_ = (mode == hal::StopMode::kBrake) ? Mode::kBrake : Mode::kCoast;
}

uint32_t SimWheel::count() const {
  // Truncate toward -infinity so a wheel rolling backwards through zero reads ..., 1, 0,
  // 2^32-1, ... as a real quadrature counter does. Converting a negative int64 to uint32 is
  // defined as modulo 2^32, which is exactly the hardware wrap.
  const auto whole = static_cast<int64_t>(std::floor(position_counts_));
  return params_.initial_count + static_cast<uint32_t>(whole);
}

void SimWheel::step(double dt_s) {
  double target_rad_s = 0.0;
  double tau_s = params_.coast_tau_s;
  switch (mode_) {
    case Mode::kDrive:
      target_rad_s = static_cast<double>(applied_duty_) * params_.no_load_speed_rad_s;
      tau_s = params_.drive_tau_s;
      break;
    case Mode::kCoast:
      tau_s = params_.coast_tau_s;
      break;
    case Mode::kBrake:
      tau_s = params_.brake_tau_s;
      break;
  }

  // Exact solution of dw/dt = (target - w)/tau over dt:
  //   w(dt)     = target + (w0 - target) e^{-dt/tau}
  //   angle(dt) = target*dt + (w0 - target) tau (1 - e^{-dt/tau})
  const double decay = std::exp(-dt_s / tau_s);
  const double error = speed_rad_s_ - target_rad_s;
  const double angle_rad = target_rad_s * dt_s + error * tau_s * (1.0 - decay);
  speed_rad_s_ = target_rad_s + error * decay;
  position_counts_ += angle_rad / kTwoPi * params_.counts_per_rev;
}

}  // namespace recon::sim
