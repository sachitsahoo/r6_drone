#pragma once

#include <cstdint>

#include "control/diff_drive.hpp"
#include "control/rate_limiter.hpp"
#include "control/wheel_speed_estimator.hpp"
#include "control/wheel_velocity_controller.hpp"
#include "hal/clock.hpp"
#include "hal/wheel_encoder.hpp"
#include "hal/wheel_motor.hpp"
#include "messages.hpp"  // generated: param_defaults

namespace recon::core {

/// Every tunable of the drive loop. Defaults come from params.yaml.
struct DriveConfig {
  WheelVelocityGains gains{};
  float max_wheel_speed_rad_s = protocol::param_defaults::wheel_max_speed_rad_s;
  float accel_limit_m_s2 = protocol::param_defaults::wheel_accel_limit_m_s2;
  uint8_t speed_window_samples = protocol::param_defaults::wheel_speed_window_samples;
};

/// Snapshot of one wheel after a step, for telemetry and tests.
struct WheelStatus {
  float ref_rad_s;      ///< Rate-limited reference the controller tracked.
  float speed_rad_s;    ///< Estimated speed (0 if not valid).
  float duty;           ///< Duty written to the motor (0 when not ARMED).
  float integrator;     ///< Controller integrator, duty.
  bool speed_valid;
  bool encoder_fault;   ///< The estimator's plausibility check tripped this step.
};

/// The wheel velocity loop for both wheels: kinematics, acceleration limit, speed estimate,
/// PI + feedforward. Design: ADR 0013.
///
/// Call `step()` at a fixed rate (1 kHz target, CLAUDE.md). Who calls it, a timer ISR or the
/// main loop, is the STM32 timer design's decision; the loop measures real elapsed time and
/// does not assume the period.
///
/// Outside ARMED the estimators keep running (so the window is warm on arming), but the
/// integrators and rate limiters are reset and the motors are NOT written: stopping them is
/// the safety state machine's job (ADR 0013, "Behaviour outside ARMED").
class DriveLoop {
 public:
  DriveLoop(const hal::Clock& clock, const hal::WheelEncoder& left_encoder,
            hal::WheelMotor& left_motor, const hal::WheelEncoder& right_encoder,
            hal::WheelMotor& right_motor, const DriveConfig& config = DriveConfig{});

  /// Latest body command (from DriveCommand). Held until replaced.
  void set_command(float linear_speed_m_s, float angular_rate_rad_s);

  /// One loop iteration. \param armed True only in the ARMED safety state.
  void step(bool armed);

  /// New tunables; the integrators are kept. Changing the window clears speed history.
  void set_config(const DriveConfig& config);

  const DriveConfig& config() const { return config_; }
  const WheelStatus& left() const { return left_.status; }
  const WheelStatus& right() const { return right_.status; }
  /// Execution time of the last `step()`, us (hard rule 4; reported as LoopTiming).
  uint32_t last_exec_us() const { return last_exec_us_; }
  /// Worst execution time since construction, us.
  uint32_t max_exec_us() const { return max_exec_us_; }

 private:
  struct Wheel {
    const hal::WheelEncoder& encoder;
    hal::WheelMotor& motor;
    WheelSpeedEstimator estimator;
    WheelVelocityController controller;
    RateLimiter limiter;
    WheelStatus status;
  };

  void step_wheel(Wheel& wheel, float target_rad_s, bool armed, float dt_s, uint32_t now_us);

  const hal::Clock& clock_;
  DriveConfig config_;
  Wheel left_;
  Wheel right_;
  float cmd_linear_m_s_ = 0.0F;
  float cmd_angular_rad_s_ = 0.0F;
  uint32_t last_step_us_ = 0;
  bool have_last_step_ = false;
  uint32_t last_exec_us_ = 0;
  uint32_t max_exec_us_ = 0;
};

/// Longest interval one step will integrate over, s. Initial guess: 10 loop periods. A
/// longer gap means the loop stalled; integrating the whole gap at once would kick the
/// integrator. The stall itself is LOOP_OVERRUN, the safety machine's concern.
inline constexpr float kMaxDriveStepDt_s = 0.01F;

}  // namespace recon::core
