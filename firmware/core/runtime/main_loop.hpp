#pragma once

#include <cstdint>

#include "hal/clock.hpp"
#include "hal/serial_port.hpp"
#include "hal/watchdog.hpp"
#include "protocol/frame.hpp"
#include "protocol/stale_command_filter.hpp"
#include "safety/check_in_monitor.hpp"
#include "safety/safety_mailbox.hpp"

namespace recon::core {

/// Marks a boot-report Fault(NONE) context, so it cannot be confused with an arm-interlock
/// context (2..4). The low byte carries the RCC_CSR reset flags (ADR 0015 Q4: information
/// only, never a fault).
inline constexpr uint32_t kBootReportMarker = 0x80000000U;

/// One main-loop pass: the body of the superloop in `main()` (ADR 0015 §1).
///
/// 1. latch the stale-filter baseline reset;
/// 2. drain the serial port through `FrameDecoder`; reject stale frames (Nack STALE_TIMESTAMP);
///    post the rest to the mailbox;
/// 3. send every Fault/Nack report the motor loop has queued;
/// 4. check in, and feed the IWDG only if every loop has checked in (ADR 0014).
///
/// Telemetry (StateTelemetry, LoopTiming, LinkStats) is not sent yet: its scheduler is a later
/// slice (ADR 0015, "Does not cover").
///
/// \note Call from exactly one context (the main loop). Never from an ISR. No allocation.
class MainLoop {
 public:
  /// Counters for LinkStats and tests. All monotonic since boot, wrapping at 2^32.
  struct Stats {
    uint32_t drive_commands = 0;    ///< DriveCommands posted to the mailbox.
    uint32_t stale_rejects = 0;     ///< Frames Nacked STALE_TIMESTAMP.
    uint32_t request_overflows = 0; ///< SafetyStateRequests Nacked because the ring was full.
    uint32_t frames_sent = 0;       ///< Fault and Nack frames written completely.
    uint32_t tx_short_writes = 0;   ///< Frames the serial port could not take in full.
  };

  /// \param comms_timeout_us When the stale filter forgets its baselines (ADR 0015 Q3).
  MainLoop(const hal::Clock& clock, hal::SerialPort& serial, hal::Watchdog& watchdog,
           SafetyMailbox& mailbox, ReportQueue& reports, CheckInMonitor& check_ins,
           uint32_t comms_timeout_us);

  void poll();

  /// Queues one Fault(NONE, kBootReportMarker | reset_flags) for the next poll. Call once at
  /// boot with the RCC_CSR reset-cause byte. A zero argument queues nothing.
  void queue_boot_report(uint8_t reset_flags);

  const Stats& stats() const { return stats_; }
  const protocol::FrameDecoder& decoder() const { return decoder_; }

 private:
  void handle(const protocol::DecodedFrame& frame, uint32_t now_us);
  void send_nack(uint8_t message_id, uint8_t seq, protocol::NackReason reason);
  void send_fault(protocol::FaultCode code, uint32_t context);
  template <typename Msg>
  void send(protocol::MessageId id, const Msg& msg);

  const hal::Clock& clock_;
  hal::SerialPort& serial_;
  hal::Watchdog& watchdog_;
  SafetyMailbox& mailbox_;
  ReportQueue& reports_;
  CheckInMonitor& check_ins_;
  protocol::FrameDecoder decoder_;
  protocol::StaleCommandFilter stale_;
  Stats stats_{};
  uint8_t tx_seq_ = 0;
  uint8_t boot_flags_ = 0;
};

}  // namespace recon::core
