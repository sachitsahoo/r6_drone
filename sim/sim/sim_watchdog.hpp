#pragma once

#include <cstdint>

#include "hal/clock.hpp"
#include "hal/watchdog.hpp"

namespace recon::sim {

/// Simulated IWDG. Implements `hal::Watchdog` against the sim clock.
///
/// It cannot reset anything (there is no chip to reset), so it records instead: how many times
/// it was fed, when last, and whether it would have expired. A test that wants the
/// "after a watchdog reset" boot constructs one with `boot_after_reset = true` and hands
/// `reset_was_watchdog()` to the supervisor, exactly as firmware does at boot (ADR 0014 Q5).
class SimWatchdog final : public hal::Watchdog {
 public:
  /// \param timeout_us Expiry time without a feed, us. Firmware uses core::kIwdgTimeout_ms.
  /// \param boot_after_reset What `reset_was_watchdog()` reports.
  SimWatchdog(const hal::Clock& clock, uint32_t timeout_us, bool boot_after_reset = false)
      : clock_(clock),
        timeout_us_(timeout_us),
        boot_after_reset_(boot_after_reset),
        last_feed_us_(clock.now_us()) {}

  // --- hal::Watchdog ---
  void feed() override {
    update_expiry();
    last_feed_us_ = clock_.now_us();
    ++feed_count_;
  }
  bool reset_was_watchdog() const override { return boot_after_reset_; }

  // --- simulation ---
  /// True once the watchdog would have reset the chip: `timeout_us` passed with no feed.
  /// Latches, so a late feed cannot hide it. Call at least once per timeout (the sim steps
  /// every millisecond) so the unsigned elapsed time never wraps.
  bool expired() {
    update_expiry();
    return expired_;
  }
  uint32_t feed_count() const { return feed_count_; }
  uint32_t last_feed_us() const { return last_feed_us_; }

 private:
  void update_expiry() {
    if (clock_.now_us() - last_feed_us_ >= timeout_us_) {  // unsigned: correct across a wrap
      expired_ = true;
    }
  }

  const hal::Clock& clock_;
  uint32_t timeout_us_;
  bool boot_after_reset_;
  uint32_t last_feed_us_;
  uint32_t feed_count_ = 0;
  bool expired_ = false;
};

}  // namespace recon::sim
