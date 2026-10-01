#include "control/drive_loop.hpp"

#include <cmath>

#include "control/geometry.hpp"
#include "time/timestamp.hpp"

namespace recon::core {

namespace {

constexpr float kSPerUs = 1e-6F;

}  // namespace

DriveLoop::DriveLoop(const hal::Clock& clock, const hal::WheelEncoder& left_encoder,
                     hal::WheelMotor& left_motor, const hal::WheelEncoder& right_encoder,
                     hal::WheelMotor& right_motor, const DriveConfig& config)
    : clock_(clock),
      config_(config),
      left_{left_encoder, left_motor,
            WheelSpeedEstimator(left_encoder.counts_per_rev(), config.speed_window_samples),
            WheelVelocityController(config.gains), RateLimiter{}, WheelStatus{}},
      right_{right_encoder, right_motor,
             WheelSpeedEstimator(right_encoder.counts_per_rev(), config.speed_window_samples),
             WheelVelocityController(config.gains), RateLimiter{}, WheelStatus{}} {}

void DriveLoop::set_command(float linear_speed_m_s, float angular_rate_rad_s) {
  cmd_linear_m_s_ = linear_speed_m_s;
  cmd_angular_rad_s_ = angular_rate_rad_s;
}

void DriveLoop::set_config(const DriveConfig& config) {
  const bool window_changed = config.speed_window_samples != config_.speed_window_samples;
  config_ = config;
  for (Wheel* w : {&left_, &right_}) {
    w->controller.set_gains(config.gains);
    if (window_changed) {
      w->estimator.set_window_samples(config.speed_window_samples);
    }
  }
}

void DriveLoop::step(bool armed) {
  const uint32_t start_us = clock_.now_us();

  // Real elapsed time, capped: a stall must not be integrated in one jump.
  float dt_s = 0.0F;
  if (have_last_step_) {
    dt_s = std::fmin(static_cast<float>(elapsed_us(last_step_us_, start_us)) * kSPerUs,
                     kMaxDriveStepDt_s);
  }
  last_step_us_ = start_us;
  have_last_step_ = true;

  const WheelSpeedRefs refs =
      diff_drive(cmd_linear_m_s_, cmd_angular_rad_s_, config_.max_wheel_speed_rad_s);
  step_wheel(left_, refs.left_rad_s, armed, dt_s, start_us);
  step_wheel(right_, refs.right_rad_s, armed, dt_s, start_us);

  last_exec_us_ = elapsed_us(start_us, clock_.now_us());
  if (last_exec_us_ > max_exec_us_) {
    max_exec_us_ = last_exec_us_;
  }
}

void DriveLoop::step_wheel(Wheel& wheel, float target_rad_s, bool armed, float dt_s,
                           uint32_t now_us) {
  // Estimate even when disarmed, so the window is warm and faults are seen before arming.
  const WheelSpeedEstimator::Result est = wheel.estimator.update(wheel.encoder.count(), now_us);
  WheelStatus& s = wheel.status;
  s.speed_rad_s = est.speed_rad_s;
  s.speed_valid = est.valid;
  s.encoder_fault = est.fault;

  if (!armed) {
    // Integrator and ramp start from zero on the next arm (ADR 0013, integrator resets).
    // The motors are not written: stopping them is the safety state machine's job.
    wheel.controller.reset();
    wheel.limiter.reset(0.0F);
    s.ref_rad_s = 0.0F;
    s.duty = 0.0F;
    s.integrator = 0.0F;
    return;
  }

  const float max_step_rad_s = config_.accel_limit_m_s2 / kWheelRadius_m * dt_s;
  const float ref = wheel.limiter.step(target_rad_s, max_step_rad_s);
  // Without a valid speed, run open-loop on feedforward with the integrator frozen, rather
  // than acting on a measurement known to be bad. The fault flag goes to the safety machine.
  const float duty = est.valid ? wheel.controller.update(ref, est.speed_rad_s, dt_s)
                               : wheel.controller.feedforward_only(ref);
  wheel.motor.set_duty(duty);

  s.ref_rad_s = ref;
  s.duty = duty;
  s.integrator = wheel.controller.integrator();
}

}  // namespace recon::core
