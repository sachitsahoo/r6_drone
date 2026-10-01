#pragma once

#include <cstdint>

namespace recon::hal {

/// One reading of the battery line.
struct PowerSample {
  float bus_voltage_V;
  /// Positive while the battery is discharging.
  float current_A;
  /// When the sample was taken, from `Clock::now_us()`.
  uint32_t timestamp_us;
  /// False on an I2C error or a stale reading.
  bool valid;
};

/// The INA226 on the battery line (ADR 0012).
class PowerMonitor {
 public:
  /// The most recent reading.
  /// \note ISR-safe: returns the last completed reading. The I2C transfer that produces it
  ///       happens inside the implementation, never in this call.
  virtual PowerSample latest() const = 0;

 protected:
  ~PowerMonitor() = default;  // see Clock
};

}  // namespace recon::hal
