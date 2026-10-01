#include "safety/wheel_stall_detector.hpp"

#include <cmath>

#include "time/timestamp.hpp"

namespace recon::core {

bool WheelStallDetector::update(const Sample& s, uint32_t now_us, uint32_t stall_us) {
  const bool saturated = std::fabs(s.duty) >= s.duty_limit - kSaturationTolerance &&
                         s.duty_limit > 0.0F;  // a zero limit means "no output", not saturation
  // "Turning against the duty" needs a speed of the wrong sign; a tiny one already counts as
  // "not turning", so the two cases overlap harmlessly near zero.
  const bool still = std::fabs(s.speed_rad_s) < kStallSpeed_rad_s;
  const bool opposing = (s.duty > 0.0F && s.speed_rad_s < 0.0F) ||
                        (s.duty < 0.0F && s.speed_rad_s > 0.0F);
  if (!(s.speed_valid && saturated && (still || opposing))) {
    active_ = false;
    return false;
  }
  if (!active_) {
    active_ = true;
    since_us_ = now_us;
  }
  // Bounded: the condition trips at stall_us (<= 2 s), and the supervisor leaves ARMED and
  // resets the detector, so the interval never approaches the 71.6 min wrap.
  return elapsed_us(since_us_, now_us) >= stall_us;
}

}  // namespace recon::core
