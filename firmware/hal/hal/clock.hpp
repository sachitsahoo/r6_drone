#pragma once

#include <cstdint>

namespace recon::hal {

/// Monotonic time since MCU boot. Approved design: firmware/hal/design-proposal.md.
///
/// Every timestamp in the HAL comes from this clock, so `core` can compare any two samples
/// with `core::elapsed_us`.
class Clock {
 public:
  /// Microseconds since boot.
  ///
  /// \return A `uint32_t` that wraps every ~71.6 minutes. Never compare two values with `<`;
  ///         subtract them with `core::elapsed_us`, which is correct across one wrap.
  /// \note ISR-safe. Never blocks.
  virtual uint32_t now_us() const = 0;

 protected:
  // Protected and non-virtual: HAL objects are statically allocated and never deleted through
  // a base pointer. A public virtual destructor would emit a deleting destructor that
  // references operator delete, which the no-heap build must not need (hard rule 2).
  ~Clock() = default;
};

}  // namespace recon::hal
