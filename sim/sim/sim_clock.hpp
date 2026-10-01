#pragma once

#include <cstdint>

#include "hal/clock.hpp"

namespace recon::sim {

/// Virtual time. Advanced only by the simulation, never by the wall clock.
///
/// This is what makes a SIL test deterministic: a test that steps 1 s of virtual time runs
/// in a few milliseconds and produces the same result on every machine and every run.
class SimClock final : public hal::Clock {
 public:
  /// \param start_us Initial reading. Start near 2^32 to put a clock wrap inside a test.
  explicit SimClock(uint32_t start_us = 0) : now_us_(start_us) {}

  uint32_t now_us() const override { return now_us_; }

  /// Advances virtual time. Unsigned addition, so crossing 2^32 wraps exactly as the MCU
  /// timer does.
  void advance_us(uint32_t dt_us) { now_us_ += dt_us; }

 private:
  uint32_t now_us_;
};

}  // namespace recon::sim
