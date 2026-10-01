#pragma once

#include <cstdint>

#include "hal/wheel_encoder.hpp"
#include "hal/wheel_motor.hpp"

namespace recon::sim {

/// Placeholder parameters for one N20 wheel.
///
/// EVERY VALUE HERE IS A PLACEHOLDER. The gear ratio is not chosen (docs/bom.md) and no
/// system identification has been done, so these exist only to give the simulator a plant
/// with the right *shape*: a first-order lag from duty to speed. Do not tune a controller
/// against these numbers and expect it to work on hardware.
struct WheelPlantParams {
  /// Wheel speed at duty +1 in steady state, rad/s. Initial guess: a 12 V N20 at ~300 rpm
  /// output, i.e. 300 * 2*pi / 60.
  float no_load_speed_rad_s = 31.4F;
  /// Time constant of the speed response while driven, s. Initial guess for a small
  /// gearmotor with a 105 mm wheel attached.
  float drive_tau_s = 0.05F;
  /// Spin-down time constant with the driver outputs off, s. Initial guess: friction only,
  /// so much slower than driven.
  float coast_tau_s = 0.5F;
  /// Spin-down time constant with the terminals shorted, s. Initial guess: back-EMF into a
  /// short brakes about as hard as the motor drives.
  float brake_tau_s = 0.05F;
  /// Encoder counts per wheel revolution, gearbox included. Initial guess: 7 pole-pair
  /// magnetic encoder x4 quadrature x 1:50 gearbox = 1400. Real value depends on gear ratio.
  float counts_per_rev = 1400.0F;
  /// Largest |duty| the motor will apply, after clamping to [-1, 1]. Mirrors the low
  /// default output limit required by CLAUDE.md (hardware-in-the-loop safety). Initial
  /// value 0.3 is a placeholder; the real per-build limit is set with the motor-control
  /// design.
  float output_limit = 0.3F;
  /// Encoder reading at construction. Set near 2^32 to put a counter wrap inside a test.
  uint32_t initial_count = 0;
  /// Constant load torque, expressed as the duty that would cancel it. Positive opposes
  /// forward motion. Acts in every mode, so with no drive it rolls the wheel backwards:
  /// it models a slope, or rolling resistance while moving forward. Coulomb friction, which
  /// can never reverse motion, is NOT modelled. Default 0: no load.
  float load_duty = 0.0F;
  /// Extra speed-proportional drag, relative to the plant's own (dimensionless, >= 0). It
  /// lowers the steady-state speed by 1/(1 + drag) and speeds up the response by the same
  /// factor. Default 0. Used for the ADR 0013 carpet case.
  float extra_viscous_drag = 0.0F;
};

/// One simulated wheel: a TB6612 channel driving an N20, with its quadrature encoder.
///
/// Implements both `hal::WheelMotor` and `hal::WheelEncoder` because they are two views of
/// the same physical shaft; `core` sees them as two separate interfaces, exactly as on the
/// robot. Positive is the robot moving forward on both, so no side is mirrored here.
///
/// Model: speed `w` follows a first-order lag toward a target, `dw/dt = (w_target - w)/tau`.
/// Driven, `w_target = duty * no_load_speed` with `tau = drive_tau`. Coasting or braking,
/// `w_target = 0` with the matching time constant. A load `d` and extra drag `c` make it
/// `tau dw/dt = K (u - d) - (1 + c) w`, i.e. target `K (u - d) / (1 + c)`, lag `tau / (1 + c)`. Stepped with the exact solution of that
/// ODE, so the result does not depend on the step size.
///
/// Not modelled: load torque, wheel slip, voltage sag, encoder quantisation noise beyond
/// integer counts, backlash. See sim/README.md.
class SimWheel final : public hal::WheelMotor, public hal::WheelEncoder {
 public:
  explicit SimWheel(const WheelPlantParams& params = WheelPlantParams{});

  // --- hal::WheelMotor ---
  /// Clamps to [-1, 1], then saturates at +/- output_limit. NaN is treated as 0, so a
  /// corrupted command coasts the wheel rather than driving it.
  void set_duty(float duty) override;
  void stop(hal::StopMode mode) override;

  // --- hal::WheelEncoder ---
  uint32_t count() const override;
  float counts_per_rev() const override { return params_.counts_per_rev; }

  // --- simulation ---
  /// Advances the plant by `dt_s` seconds. \pre dt_s >= 0.
  void step(double dt_s);

  /// True wheel speed, rad/s. Ground truth for tests; `core` must never see this.
  double speed_rad_s() const { return speed_rad_s_; }
  /// Duty actually applied after clamping and limiting, or 0 when stopped.
  float applied_duty() const { return applied_duty_; }

 private:
  enum class Mode : uint8_t { kDrive, kCoast, kBrake };

  WheelPlantParams params_;
  Mode mode_ = Mode::kCoast;  // a motor that has never been commanded is not driving
  float applied_duty_ = 0.0F;
  double speed_rad_s_ = 0.0;
  /// Shaft position in encoder counts since construction, unwrapped. Double, so it stays
  /// exact to well under one count for any test-length simulation (2^53 counts).
  double position_counts_ = 0.0;
};

}  // namespace recon::sim
