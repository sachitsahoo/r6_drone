#include "protocol/stale_command_filter.hpp"

#include "messages.hpp"
#include "time/timestamp.hpp"

namespace recon::protocol {

bool StaleCommandFilter::accept(uint8_t message_id, uint32_t frame_timestamp_us,
                                uint32_t now_us) {
  if (message_id >= kSlots) {
    return true;  // not an operator-to-robot command
  }
  update(now_us);
  if (message_id == static_cast<uint8_t>(MessageId::EstopRequest)) {
    // A stop is never refused. It still counts as an accepted frame for the reset timer.
    any_ = true;
    last_accept_rx_us_ = now_us;
    return true;
  }
  if (have_[message_id]) {
    // Signed difference: "newer" is correct across the operator clock's wrap.
    const auto ahead = static_cast<int32_t>(frame_timestamp_us - last_ts_[message_id]);
    if (ahead <= 0) {
      return false;
    }
  }
  have_[message_id] = true;
  last_ts_[message_id] = frame_timestamp_us;
  any_ = true;
  last_accept_rx_us_ = now_us;
  return true;
}

void StaleCommandFilter::update(uint32_t now_us) {
  if (any_ && core::elapsed_us(last_accept_rx_us_, now_us) >= reset_after_us_) {
    forget();  // latched: any_ = false, so the interval is never measured again
  }
}

void StaleCommandFilter::forget() {
  for (size_t i = 0; i < kSlots; ++i) {
    have_[i] = false;
  }
  any_ = false;
}

}  // namespace recon::protocol
