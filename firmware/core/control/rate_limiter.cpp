#include "control/rate_limiter.hpp"

#include <cmath>

namespace recon::core {

float RateLimiter::step(float target, float max_step) {
  const float limit = std::fmax(0.0F, max_step);
  const float change = target - value_;
  value_ += std::fmax(-limit, std::fmin(limit, change));
  return value_;
}

}  // namespace recon::core
