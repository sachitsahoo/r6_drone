#pragma once

#include <cstdint>

namespace recon::hal {

/// Measured phase currents, from a current-sensing power stage.
struct PhaseCurrents {
  float a_A;
  float b_A;
  float c_A;
  /// When the currents were sampled, from `Clock::now_us()`.
  uint32_t timestamp_us;
  /// False if the stage has no current sensing or the reading is stale.
  bool valid;
};

/// The bought three-phase power stage for the pitch motor (ADR 0007): a SimpleFOCMini with a
/// DRV8313.
///
/// Takes three duty cycles and knows nothing about FOC. The electrical angle, inverse Park
/// transform and SVPWM are `core`'s, so the commutation maths is testable on a laptop.
class PitchPowerStage {
 public:
  /// Sets the three phase duty cycles.
  ///
  /// \param a, b, c Duty for each phase in [0, 1]. The implementation clamps out-of-range
  ///                values. Written to the timer's compare registers; takes effect at the
  ///                start of the next PWM period.
  /// \note ISR-safe: called from the FOC loop's timer interrupt.
  virtual void set_phase_duties(float a, float b, float c) = 0;

  /// Drives the stage's enable lines. Disabled means all three phases floating, so the
  /// casing turns freely.
  /// \note ISR-safe, so a fault handler can disable the stage.
  virtual void set_enabled(bool enabled) = 0;

  /// True while the DRV8313 reports a fault on nFAULT (overcurrent, overtemperature or
  /// undervoltage).
  /// \note ISR-safe.
  virtual bool fault() const = 0;

  /// Whether this stage measures phase current. The SimpleFOCMini v1 does not; v2.3 does.
  /// Lets the board be chosen at purchase time without changing `core`.
  virtual bool has_current_sense() const = 0;

  /// The most recent phase currents. `valid` is always false if `has_current_sense()` is.
  /// \note ISR-safe.
  virtual PhaseCurrents latest_currents() const = 0;

 protected:
  ~PitchPowerStage() = default;  // see Clock
};

}  // namespace recon::hal
