#pragma once

#include <cstdint>

#include "hal/pitch_power_stage.hpp"
#include "hal/wheel_encoder.hpp"
#include "hal/wheel_motor.hpp"

namespace recon::stm32 {

/// Encoder resolution the null wheel reports, counts/rev. Placeholder, same as the sim plant
/// (1400 = 7 pole pairs x 4 x 1:50); only needs to be positive for the speed estimator.
inline constexpr float kNullCountsPerRev = 1400.0F;

/// A wheel that is not there. **The first image drives no motor**: the TB6612 driver needs a
/// pin map and PWM timer design, which is the next owner-reviewed slice. Commands are
/// discarded and the encoder never moves, so arming with the stick pushed latches
/// WHEEL_STALL after 500 ms: a free check of that detector on real silicon.
class NullWheel final : public hal::WheelMotor, public hal::WheelEncoder {
 public:
  void set_duty(float /*duty*/) override {}
  void stop(hal::StopMode /*mode*/) override {}
  uint32_t count() const override { return 0; }
  float counts_per_rev() const override { return kNullCountsPerRev; }
};

/// A pitch stage that is not there. Never reports a fault, never drives anything.
class NullPitchStage final : public hal::PitchPowerStage {
 public:
  void set_phase_duties(float /*a*/, float /*b*/, float /*c*/) override {}
  void set_enabled(bool /*enabled*/) override {}
  bool fault() const override { return false; }
  bool has_current_sense() const override { return false; }
  hal::PhaseCurrents latest_currents() const override { return {0.0F, 0.0F, 0.0F, 0, false}; }
};

}  // namespace recon::stm32
