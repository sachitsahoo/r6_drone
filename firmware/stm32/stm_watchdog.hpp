#pragma once

#include <cstdint>

#include "hal/watchdog.hpp"

namespace recon::stm32 {

/// `hal::Watchdog` on the IWDG. Design: ADR 0014 (50 ms, fed on check-in) and ADR 0015 §4.
class StmWatchdog final : public hal::Watchdog {
 public:
  /// Reads the RCC reset flags, keeps them, then clears them so the next boot sees a fresh
  /// cause. Call first thing in main(), before anything can trigger another reset.
  void capture_reset_cause();

  /// Freezes the IWDG while the core is halted by a debugger, then starts it at
  /// core::kIwdgTimeout_ms. Once started it cannot be stopped (RM0440). Call once, at the end
  /// of initialisation.
  void start();

  // --- hal::Watchdog ---
  void feed() override;
  bool reset_was_watchdog() const override { return was_watchdog_; }

  /// RCC_CSR bits 31..24 at boot (reset-cause flags), for the boot report (ADR 0015 Q4).
  uint8_t reset_flags() const { return reset_flags_; }

 private:
  bool was_watchdog_ = false;
  uint8_t reset_flags_ = 0;
};

}  // namespace recon::stm32
