#pragma once

#include "hal/clock.hpp"

namespace recon::stm32 {

/// `hal::Clock` on TIM2, a 32-bit timer (RM0440: TIM2 and TIM5 are the G4's 32-bit timers)
/// counting at 1 MHz. Its counter IS the microsecond timestamp, so it wraps at exactly 2^32 us
/// as the HAL contract says, with no software extension and no interrupt.
class StmClock final : public hal::Clock {
 public:
  /// Enables, configures and starts TIM2. Call once, before anything reads the clock.
  void start();
  /// \note ISR-safe: one 32-bit register read.
  uint32_t now_us() const override;
};

}  // namespace recon::stm32
