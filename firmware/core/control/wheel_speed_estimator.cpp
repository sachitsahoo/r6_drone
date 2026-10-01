#include "control/wheel_speed_estimator.hpp"

#include <cmath>

#include "time/timestamp.hpp"

namespace recon::core {

namespace {

constexpr float kTwoPi = 6.2831853F;
constexpr float kSPerUs = 1e-6F;

/// Extra counts the plausibility bound allows on top of the physical maximum: one count of
/// quantisation, since a counter can tick once just inside either end of a sample.
constexpr float kPlausibilityMarginCounts = 1.0F;

uint8_t clamp_window(uint8_t n) {
  if (n < 1) {
    return 1;
  }
  return n > WheelSpeedEstimator::kMaxWindowSamples ? WheelSpeedEstimator::kMaxWindowSamples : n;
}

}  // namespace

WheelSpeedEstimator::WheelSpeedEstimator(float counts_per_rev, uint8_t window_samples)
    : counts_per_rev_(counts_per_rev), window_(clamp_window(window_samples)) {}

void WheelSpeedEstimator::set_window_samples(uint8_t window_samples) {
  window_ = clamp_window(window_samples);
  reset();
}

void WheelSpeedEstimator::reset() {
  head_ = 0;
  size_ = 0;
}

const WheelSpeedEstimator::Sample& WheelSpeedEstimator::newest() const {
  return history_[(head_ + size_ - 1) % history_.size()];
}

void WheelSpeedEstimator::push(const Sample& s) {
  // Keep N+1 samples: N intervals between the oldest and the newest.
  const size_t keep = static_cast<size_t>(window_) + 1;
  if (size_ == keep) {
    head_ = (head_ + 1) % history_.size();
    --size_;
  }
  history_[(head_ + size_) % history_.size()] = s;
  ++size_;
}

WheelSpeedEstimator::Result WheelSpeedEstimator::update(uint32_t count, uint32_t timestamp_us) {
  if (size_ > 0) {
    const Sample& prev = newest();
    // int32 reinterpretation of the unsigned difference: exact across the 2^32 wrap.
    const auto step = static_cast<int32_t>(count - prev.count);
    const float dt_s = static_cast<float>(elapsed_us(prev.timestamp_us, timestamp_us)) * kSPerUs;
    const float max_counts =
        kMaxPlausibleWheelSpeed_rad_s / kTwoPi * counts_per_rev_ * dt_s + kPlausibilityMarginCounts;
    if (std::fabs(static_cast<float>(step)) > max_counts) {
      reset();
      push({count, timestamp_us});
      return {0.0F, false, true};
    }
  }

  push({count, timestamp_us});
  if (size_ < 2) {
    return {0.0F, false, false};
  }

  const Sample& first = oldest();
  const uint32_t span_us = elapsed_us(first.timestamp_us, timestamp_us);
  if (span_us == 0) {
    return {0.0F, false, false};
  }
  const auto delta = static_cast<int32_t>(count - first.count);
  const float revs = static_cast<float>(delta) / counts_per_rev_;
  return {revs * kTwoPi / (static_cast<float>(span_us) * kSPerUs), true, false};
}

}  // namespace recon::core
