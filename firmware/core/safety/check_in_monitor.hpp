#pragma once

#include <atomic>
#include <cstdint>

namespace recon::core {

/// IWDG timeout, ms. ADR 0014 Q5: at 0.47 m/s a hung core drives 24 mm before the reset,
/// the same distance as an immediate brake. A build constant, deliberately NOT a param.
/// [UNCLEAR] The LSI that clocks the IWDG is only about +/-10% accurate (check the G474
/// datasheet), so analysis must assume anywhere from 45 to 55 ms.
inline constexpr uint32_t kIwdgTimeout_ms = 50;

/// Periodic tasks that must check in before the IWDG is fed. One bit each.
enum class CheckInTask : uint32_t {
  kMotorLoop = 1U << 0,  ///< 1 kHz motor loop (DriveLoop + SafetySupervisor).
  kMainLoop = 1U << 1,   ///< Main loop: frame decode, telemetry, the IWDG feed itself.
  kPitchLoop = 1U << 2,  ///< Pitch stabilizer. Reserved: not required until it exists.
};

/// Decides when the hardware watchdog may be fed. Design: ADR 0014, "Hardware watchdog".
///
/// Every periodic task sets its bit with `check_in()`. The main loop calls `should_feed()`;
/// it returns true only when every required bit is set, and then clears them. So a stuck ISR
/// with a healthy main loop, or a stuck main loop with a healthy ISR, both starve the IWDG and
/// reset the chip.
///
/// Pure: it never touches the watchdog itself; the caller feeds on `true`.
///
/// \note `check_in()` is ISR-safe (one atomic OR, lock-free on Cortex-M4 via LDREX/STREX).
///       `should_feed()` must be called from one context only (the main loop).
class CheckInMonitor {
  // A lock-based atomic could deadlock when an ISR interrupts the main loop mid-update.
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "check_in() is called from ISRs, so the mask must be lock-free");

 public:
  /// \param required_mask OR of the `CheckInTask` bits that must all check in per feed.
  explicit CheckInMonitor(uint32_t required_mask) : required_(required_mask) {}

  /// Marks `task` as having run since the last feed.
  void check_in(CheckInTask task) {
    bits_.fetch_or(static_cast<uint32_t>(task), std::memory_order_relaxed);
  }

  /// \return True if every required task has checked in since the last true return; the
  ///         required bits are then cleared. False leaves the bits as they are, so they keep
  ///         accumulating toward the next feed.
  bool should_feed();

  uint32_t required_mask() const { return required_; }

 private:
  const uint32_t required_;
  std::atomic<uint32_t> bits_{0};
};

}  // namespace recon::core
