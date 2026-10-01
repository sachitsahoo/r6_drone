#pragma once

#include <cstddef>
#include <cstdint>

namespace recon::protocol {

/// Rejects operator frames that are not newer than the last accepted frame of the same
/// message ID, so a duplicated or delayed UDP datagram cannot be replayed as a fresh command.
/// Rule: `protocol/design-proposal.md`; details: ADR 0015 §3.
///
/// - **"Newer"** is `(int32_t)(t_new - t_last) > 0` on the frame's own `timestamp_us` (the
///   operator's clock). That is correct across the operator clock's wrap for gaps under
///   2^31 us (35.8 min). An equal timestamp is a duplicate and is rejected.
/// - **Baseline reset (ADR 0015 Q3).** A restarted operator app's clock starts again near 0,
///   so every frame would look old forever. Once no frame has been accepted for
///   `reset_after_us` of MCU receive time, every baseline is forgotten and the next frame of
///   each ID is accepted on its own. The reset is latched by `update()`, which the main loop
///   calls every pass, so the receive-time interval never approaches the MCU clock's wrap.
/// - **EstopRequest is always accepted.** Replaying a stop is harmless; refusing one is not.
/// - IDs outside the operator-to-robot range (0x01-0x1F) are not commands and always pass;
///   the caller deals with them.
///
/// \note Main loop only. Not ISR-safe. No allocation.
class StaleCommandFilter {
 public:
  /// \param reset_after_us Silence after which baselines are forgotten, us. The firmware
  ///        passes the comms timeout (`comms_timeout_ms` * 1000), so the filter forgets exactly
  ///        when the link counts as lost.
  explicit StaleCommandFilter(uint32_t reset_after_us) : reset_after_us_(reset_after_us) {}

  /// \return True if the frame is new enough to act on; its timestamp becomes the baseline.
  bool accept(uint8_t message_id, uint32_t frame_timestamp_us, uint32_t now_us);

  /// Latches the baseline reset once `reset_after_us` passes with no accepted frame.
  void update(uint32_t now_us);

  void set_reset_after_us(uint32_t reset_after_us) { reset_after_us_ = reset_after_us; }

 private:
  /// One slot per operator-to-robot ID, 0x00-0x1F (messages.yaml id_ranges).
  static constexpr size_t kSlots = 0x20;

  void forget();

  uint32_t reset_after_us_;
  uint32_t last_ts_[kSlots] = {};
  bool have_[kSlots] = {};
  bool any_ = false;
  uint32_t last_accept_rx_us_ = 0;
};

}  // namespace recon::protocol
