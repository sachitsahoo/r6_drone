#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "messages.hpp"  // generated: param_max, for the window bound

namespace recon::core {

/// Wheel angular speed from a wrapping quadrature counter, by windowed count difference.
///
/// Design: ADR 0013, "Speed measurement". Theory: docs/theory/wheel-velocity-loop.md §2.
///
/// The speed is (counts now - counts N samples ago) over the measured time between them.
/// Differencing consecutive 1 kHz samples would quantise to one count per millisecond
/// (4.49 rad/s at 1400 counts/rev); spanning N samples divides that step by N, at the cost
/// of an N/2-sample delay.
///
/// Wrap: counts are differenced with unsigned subtraction reinterpreted as int32_t, which is
/// exact across the 2^32 wrap as long as the wheel turns fewer than 2^31 counts per window.
///
/// Not ISR-safe on its own: call `update` from one context only (the drive loop).
class WheelSpeedEstimator {
 public:
  /// Fixed history capacity, in samples. The window parameter cannot exceed it.
  static constexpr uint8_t kMaxWindowSamples = 50;
  static_assert(kMaxWindowSamples == protocol::param_max::wheel_speed_window_samples,
                "params.yaml range must match the estimator's buffer");

  struct Result {
    /// Positive = robot moving forward. 0 when `valid` is false.
    float speed_rad_s;
    /// False until two samples exist, after a fault, or if time has not advanced.
    bool valid;
    /// True on the sample that tripped the plausibility check (a count jump no real wheel
    /// could make). The history restarts from that sample. What to do about it belongs to
    /// the safety state machine (ENCODER_FAULT).
    bool fault;
  };

  /// \param counts_per_rev Encoder counts per wheel revolution, gearbox included. > 0.
  /// \param window_samples N, clamped to [1, kMaxWindowSamples].
  WheelSpeedEstimator(float counts_per_rev, uint8_t window_samples);

  /// Adds one sample and returns the speed estimate.
  ///
  /// \param count        Raw counter reading (`hal::WheelEncoder::count()`).
  /// \param timestamp_us When it was read (`hal::Clock::now_us()`). Wraps; handled.
  ///
  /// While the history holds fewer than N+1 samples, the estimate spans what is there:
  /// valid, but coarser, for the first N samples after construction, reset or a fault.
  Result update(uint32_t count, uint32_t timestamp_us);

  /// Changes N and clears the history (old spacing would mix two window lengths).
  void set_window_samples(uint8_t window_samples);

  /// Forgets every sample. The next `update` returns invalid.
  void reset();

  uint8_t window_samples() const { return window_; }

 private:
  struct Sample {
    uint32_t count;
    uint32_t timestamp_us;
  };

  void push(const Sample& s);
  const Sample& oldest() const { return history_[head_]; }
  const Sample& newest() const;

  float counts_per_rev_;
  uint8_t window_ = 1;
  std::array<Sample, kMaxWindowSamples + 1> history_{};
  size_t head_ = 0;  // index of the oldest sample
  size_t size_ = 0;
};

/// Fastest wheel speed the plausibility check accepts, rad/s. Initial guess: about 3x the
/// placeholder no-load speed (31.4 rad/s, sim/sim/wheel_plant.hpp). A wheel can be dragged
/// faster than its motor drives it, but not by that much. Revisit with the real gear ratio.
inline constexpr float kMaxPlausibleWheelSpeed_rad_s = 100.0F;

}  // namespace recon::core
