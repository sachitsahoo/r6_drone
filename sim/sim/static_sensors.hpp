#pragma once

// Sensors and actuators with no dynamics yet: each returns a settable sample, stamped with
// the simulation clock so `core` sees fresh data.
//
// They differ from tests/cpp/hal_fakes.hpp in two ways: they stamp samples with virtual
// time, and the actuator honours its interface's clamping contract. Dynamics arrive with the
// designs that need them -- pitch plant with the FOC design, IMU motion with the estimator.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "hal/absolute_encoder.hpp"
#include "hal/clock.hpp"
#include "hal/imu.hpp"
#include "hal/pitch_power_stage.hpp"
#include "hal/power_monitor.hpp"

namespace recon::sim {

/// Standard gravity, m/s^2 (CGPM 1901 conventional value).
inline constexpr float kStandardGravity_m_s2 = 9.80665F;

/// 3S LiPo nominal voltage, V: 3 cells x 3.7 V nominal (ADR 0012 picks 3S).
inline constexpr float kNominal3sVoltage_V = 11.1F;

/// Casing IMU. Defaults to level and still: zero rate, +1 g on z.
class SimImu final : public hal::Imu {
 public:
  explicit SimImu(const hal::Clock& clock) : clock_(clock) {}

  hal::ImuSample latest() const override {
    hal::ImuSample s = sample_;
    s.timestamp_us = clock_.now_us();
    return s;
  }

  /// Sets everything except the timestamp, which always comes from the clock.
  void set(const hal::ImuSample& sample) { sample_ = sample; }

 private:
  const hal::Clock& clock_;
  hal::ImuSample sample_{{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, kStandardGravity_m_s2}, 0, true};
};

/// Pitch encoder. Defaults to angle 0 (casing at the calibrated zero), valid.
class SimAbsoluteEncoder final : public hal::AbsoluteEncoder {
 public:
  explicit SimAbsoluteEncoder(const hal::Clock& clock) : clock_(clock) {}

  hal::AngleSample latest() const override { return {angle_rad_, clock_.now_us(), valid_}; }

  /// Sets the casing-to-chassis angle. Wrapped into [0, 2*pi) here, because the interface
  /// promises that range and a test should be able to pass any angle.
  void set_angle_rad(float angle_rad) {
    constexpr float kTwoPi = 6.2831855F;
    float wrapped = std::fmod(angle_rad, kTwoPi);
    if (wrapped < 0.0F) {
      wrapped += kTwoPi;
    }
    // fmod of a value just below a multiple of 2*pi can round up to exactly 2*pi after the
    // addition; fold that onto 0 to keep the half-open range.
    angle_rad_ = (wrapped >= kTwoPi) ? 0.0F : wrapped;
  }
  void set_valid(bool valid) { valid_ = valid; }

 private:
  const hal::Clock& clock_;
  float angle_rad_ = 0.0F;
  bool valid_ = true;
};

/// Battery monitor. Defaults to a 3S pack at nominal voltage, no load.
class SimPowerMonitor final : public hal::PowerMonitor {
 public:
  explicit SimPowerMonitor(const hal::Clock& clock) : clock_(clock) {}

  hal::PowerSample latest() const override { return {voltage_V_, current_A_, clock_.now_us(), valid_}; }

  void set(float bus_voltage_V, float current_A) {
    voltage_V_ = bus_voltage_V;
    current_A_ = current_A;
  }
  void set_valid(bool valid) { valid_ = valid; }

 private:
  const hal::Clock& clock_;
  float voltage_V_ = kNominal3sVoltage_V;
  float current_A_ = 0.0F;
  bool valid_ = true;
};

/// Pitch power stage. Records what `core` commands; no motor dynamics yet.
class SimPitchPowerStage final : public hal::PitchPowerStage {
 public:
  explicit SimPitchPowerStage(const hal::Clock& clock, bool has_current_sense = false)
      : clock_(clock), has_current_sense_(has_current_sense) {}

  /// Clamps each duty to [0, 1] per the interface; NaN becomes 0 (phase held low).
  void set_phase_duties(float a, float b, float c) override {
    duties_[0] = clamp_unit(a);
    duties_[1] = clamp_unit(b);
    duties_[2] = clamp_unit(c);
  }
  void set_enabled(bool enabled) override { enabled_ = enabled; }
  bool fault() const override { return fault_; }
  bool has_current_sense() const override { return has_current_sense_; }
  hal::PhaseCurrents latest_currents() const override {
    // No electrical model yet, so the currents are zero -- and invalid without sensing.
    return {0.0F, 0.0F, 0.0F, clock_.now_us(), has_current_sense_};
  }

  const float* duties() const { return duties_; }
  bool enabled() const { return enabled_; }
  void set_fault(bool fault) { fault_ = fault; }

 private:
  static float clamp_unit(float x) { return std::isnan(x) ? 0.0F : std::clamp(x, 0.0F, 1.0F); }

  const hal::Clock& clock_;
  bool has_current_sense_;
  float duties_[3] = {0.0F, 0.0F, 0.0F};
  bool enabled_ = false;  // a stage that has never been enabled leaves the casing free
  bool fault_ = false;
};

}  // namespace recon::sim
