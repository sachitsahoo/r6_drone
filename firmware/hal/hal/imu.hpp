#pragma once

#include <cstdint>

namespace recon::hal {

/// One raw reading from the IMU, already in the project's casing frame.
///
/// Trivially copyable on purpose, so an ISR can hand it over by value.
struct ImuSample {
  /// Angular rate in the casing frame (x forward, y left, z up), rad/s. Index 1 is pitch
  /// rate, POSITIVE NOSE-DOWN (positive about +y).
  float gyro_rad_s[3];
  /// Specific force in the casing frame, m/s^2, including gravity: about +9.81 on z when the
  /// casing is level and still.
  float accel_m_s2[3];
  /// When the sample was taken, from `Clock::now_us()`.
  uint32_t timestamp_us;
  /// False when there is no fresh, trustworthy sample (bus error, data not ready, sensor
  /// fault). The values above are then unspecified and must not be used.
  bool valid;
};

/// The ICM-42688-P on the rotating casing (ADR 0005), 3.8 mm beside the axis (ADR 0010).
///
/// Unfiltered. Filtering and the offset correction for that 3.8 mm are estimation, which
/// belongs to `core`. The implementation owns the chip's mounting orientation: it rotates
/// and sign-flips the chip's axes so that `core` only ever sees the casing frame.
class Imu {
 public:
  /// The most recent sample.
  /// \note ISR-safe. Never blocks: returns the last completed reading, never starts one.
  virtual ImuSample latest() const = 0;

 protected:
  ~Imu() = default;  // see Clock
};

}  // namespace recon::hal
