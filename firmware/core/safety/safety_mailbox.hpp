#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "safety/safety_supervisor.hpp"
#include "safety/spsc_ring.hpp"

namespace recon::core {

/// Requests the motor loop can have waiting. ADR 0015 Q2: four, so a "clear" and an "arm" sent
/// back to back both fit with room to spare.
inline constexpr size_t kSafetyRequestSlots = 4;

/// Frames from the main loop (producer) to the motor-loop ISR (consumer). Design: ADR 0015.
///
/// - **E-stop** is an atomic flag, never queued behind anything.
/// - **SafetyStateRequests** wait in a 4-deep SPSC ring. The ISR takes one per tick (ADR 0014).
/// - **The latest DriveCommand** is double-buffered: the producer writes the slot the consumer
///   is not reading, then publishes it with one atomic store. ADR 0015 calls this a sequence
///   lock; the double buffer is its wait-free form. An ISR reader must not spin waiting for a
///   writer it has preempted, because the writer cannot run until the ISR returns.
/// - **"Any other valid frame"** is an atomic flag.
///
/// \note Correct for one producer preempted by one consumer on a single core (the G474), and
///       for single-threaded tests. Not for two cores: there the producer could overwrite the
///       slot the consumer is copying.
class SafetyMailbox {
 public:
  // --- producer: main loop ---
  void post_estop() { estop_.store(true, std::memory_order_release); }
  /// \return False if the ring is full; the caller Nacks the request (ADR 0015).
  bool post_request(const SafetyRequest& request) { return requests_.push(request); }
  /// Stages, then publishes. Equivalent to `stage_drive_command` + `publish_drive_command`.
  void post_drive_command(float linear_m_s, float angular_rad_s) {
    stage_drive_command(linear_m_s, angular_rad_s);
    publish_drive_command();
  }
  /// Writes the slot the consumer is not reading. Invisible to `take()` until published.
  /// Split from publishing only so a test can run the consumer between the two steps, which is
  /// exactly where the motor-loop ISR can preempt the main loop.
  void stage_drive_command(float linear_m_s, float angular_rad_s);
  /// Makes the staged command the newest, in one atomic store.
  void publish_drive_command();
  void post_other_frame() { other_frame_.store(true, std::memory_order_release); }

  // --- consumer: motor-loop ISR ---
  /// Moves everything posted since the last call into `in`: the e-stop flag, at most one
  /// request, the newest DriveCommand if one arrived, and the other-frame flag. Fields with
  /// nothing new are left as they were.
  void take(SafetyInputs& in);

 private:
  struct DriveSlot {
    float linear_m_s;
    float angular_rad_s;
  };

  std::atomic<bool> estop_{false};
  std::atomic<bool> other_frame_{false};
  SpscRing<SafetyRequest, kSafetyRequestSlots> requests_;
  DriveSlot drive_[2] = {};
  /// Number of DriveCommands published. Slot `seq & 1` holds the newest.
  std::atomic<uint32_t> drive_seq_{0};
  uint32_t drive_seen_ = 0;  // consumer only
};

/// One Fault or Nack frame for the main loop to send, queued by the motor loop.
struct OutboundReport {
  enum class Kind : uint8_t { kFault, kNack };
  Kind kind = Kind::kFault;
  FaultReport fault{};
  NackReport nack{};
};

/// Reports the motor loop can have waiting. One tick can produce at most
/// kMaxFaultReportsPerTick + 1 (every flag at once plus a Nack), so 32 always holds a full
/// tick even with a backlog. A full queue drops the report and counts it.
inline constexpr size_t kOutboundReportSlots = 32;

/// Fault and Nack reports from the motor-loop ISR (producer) to the main loop (consumer).
class ReportQueue {
 public:
  /// Producer. \return False if dropped (counted in `dropped()`).
  bool push(const OutboundReport& report) {
    if (ring_.push(report)) {
      return true;
    }
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  /// Consumer.
  bool pop(OutboundReport& out) { return ring_.pop(out); }
  /// Reports lost to a full queue since boot. Wraps at 2^32.
  uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  SpscRing<OutboundReport, kOutboundReportSlots> ring_;
  std::atomic<uint32_t> dropped_{0};
};

}  // namespace recon::core
