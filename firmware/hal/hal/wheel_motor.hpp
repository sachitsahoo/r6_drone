#pragma once

#include <cstdint>

namespace recon::hal {

/// How a wheel motor stops.
///
/// Owner decision (2026-10-01): the comms-watchdog default is `kCoast`, escalating to
/// `kBrake` if the link stays down long enough. The escalation delay is part of the safety
/// state machine design, which is owner-reviewed; this interface only provides both modes.
enum class StopMode : uint8_t {
  kCoast,  ///< Driver outputs off: the wheel spins freely. A thrown robot keeps rolling.
  kBrake,  ///< Motor terminals shorted: resists rolling, holds on a slope.
};

/// One channel of the TB6612FNG.
///
/// Knows nothing about speed or PID: the velocity loop is `core`'s.
class WheelMotor {
 public:
  /// Commands a PWM duty.
  ///
  /// \param duty In [-1, 1]; positive drives the robot forward. The implementation clamps
  ///             out-of-range values, then applies the per-build output limit, which is
  ///             low by default (CLAUDE.md, hardware-in-the-loop safety).
  /// \note Not ISR-safe: call from the motor loop only.
  virtual void set_duty(float duty) = 0;

  /// Stops the motor in the given mode. Takes effect immediately.
  /// \note ISR-safe, so a fault handler can stop the wheels.
  virtual void stop(StopMode mode) = 0;

 protected:
  ~WheelMotor() = default;  // see Clock
};

}  // namespace recon::hal
