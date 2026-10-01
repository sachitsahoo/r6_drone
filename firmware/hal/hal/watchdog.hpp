#pragma once

namespace recon::hal {

/// The independent hardware watchdog (the G474 IWDG). Design: ADR 0014, "Hardware watchdog".
///
/// The timeout is NOT configurable through this interface. It is the build constant
/// `core::kIwdgTimeout_ms`, fixed when the implementation starts the watchdog, because a
/// watchdog whose timeout can be changed at run time (let alone over the radio) is not a
/// watchdog. Once started it cannot be stopped; only a reset stops it.
///
/// Who calls `feed()` matters more than how: firmware feeds only when
/// `core::CheckInMonitor` says every periodic loop has checked in. Feeding from a timer
/// interrupt alone would keep a hung main loop alive forever.
class Watchdog {
 public:
  /// Restarts the countdown.
  /// \note ISR-safe and non-blocking, but call it from the main loop only (see above).
  virtual void feed() = 0;

  /// True if the most recent reset was caused by this watchdog expiring.
  ///
  /// Read once at boot to decide whether to start in FAULT(WATCHDOG_RESET) (ADR 0014 Q5).
  /// The STM32 implementation reads the RCC_CSR reset flags before anything clears them.
  /// \note ISR-safe. Constant for the life of the boot.
  virtual bool reset_was_watchdog() const = 0;

 protected:
  ~Watchdog() = default;  // see Clock
};

}  // namespace recon::hal
