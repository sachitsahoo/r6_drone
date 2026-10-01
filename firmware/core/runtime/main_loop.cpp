#include "runtime/main_loop.hpp"

#include "messages.hpp"

namespace recon::core {

using protocol::DecodedFrame;
using protocol::FaultCode;
using protocol::MessageId;
using protocol::NackReason;

namespace {

/// Last operator-to-robot message ID, from messages.yaml `id_ranges` (0x01-0x1F). Anything
/// above it is a robot-to-operator message that has no business arriving at the robot.
constexpr uint8_t kLastOperatorToRobotId = 0x1F;
// If the schema's ranges move, these fail and this constant must move with them.
static_assert(static_cast<uint8_t>(MessageId::ParamCommit) <= kLastOperatorToRobotId);
static_assert(static_cast<uint8_t>(MessageId::StateTelemetry) > kLastOperatorToRobotId);

/// Bytes read from the serial port per call. Small: the decoder takes one byte at a time, and
/// this only bounds the stack buffer.
constexpr size_t kRxChunkBytes = 64;

}  // namespace

MainLoop::MainLoop(const hal::Clock& clock, hal::SerialPort& serial, hal::Watchdog& watchdog,
                   SafetyMailbox& mailbox, ReportQueue& reports, CheckInMonitor& check_ins,
                   uint32_t comms_timeout_us)
    : clock_(clock),
      serial_(serial),
      watchdog_(watchdog),
      mailbox_(mailbox),
      reports_(reports),
      check_ins_(check_ins),
      stale_(comms_timeout_us) {}

void MainLoop::queue_boot_report(uint8_t reset_flags) { boot_flags_ = reset_flags; }

void MainLoop::poll() {
  const uint32_t now_us = clock_.now_us();
  stale_.update(now_us);

  if (boot_flags_ != 0) {
    send_fault(FaultCode::NONE, kBootReportMarker | boot_flags_);
    boot_flags_ = 0;
  }

  uint8_t buf[kRxChunkBytes];
  size_t n = 0;
  DecodedFrame frame{};
  while ((n = serial_.read(buf, sizeof buf)) > 0) {
    for (size_t i = 0; i < n; ++i) {
      if (decoder_.push_byte(buf[i], frame)) {
        handle(frame, now_us);
      }
    }
  }

  OutboundReport report;
  while (reports_.pop(report)) {
    if (report.kind == OutboundReport::Kind::kFault) {
      send_fault(report.fault.code, report.fault.context);
    } else {
      send_nack(report.nack.rejected_message_id, report.nack.rejected_seq, report.nack.reason);
    }
  }

  // Last: this pass is complete. Feed only if the motor loop has also run since the last feed.
  check_ins_.check_in(CheckInTask::kMainLoop);
  if (check_ins_.should_feed()) {
    watchdog_.feed();
  }
}

void MainLoop::handle(const DecodedFrame& f, uint32_t now_us) {
  if (f.message_id > kLastOperatorToRobotId) {
    // Known to the decoder, but travelling the wrong way. It proves nothing about the link.
    send_nack(f.message_id, f.seq, NackReason::UNKNOWN_MESSAGE_ID);
    return;
  }
  if (!stale_.accept(f.message_id, f.timestamp_us, now_us)) {
    ++stats_.stale_rejects;
    send_nack(f.message_id, f.seq, NackReason::STALE_TIMESTAMP);
    return;
  }

  // A payload that passed the decoder's length check but fails decode() has an out-of-range
  // field, an undeclared enum value or a bad magic. decode() does not say which, so messages
  // whose only field is a magic report BAD_MAGIC and the rest RANGE_REJECT.
  switch (static_cast<MessageId>(f.message_id)) {
    case MessageId::Heartbeat:
      mailbox_.post_other_frame();
      return;
    case MessageId::DriveCommand: {
      protocol::DriveCommand cmd{};
      if (!protocol::DriveCommand::decode(f.payload, f.payload_len, cmd)) {
        send_nack(f.message_id, f.seq, NackReason::RANGE_REJECT);
        return;
      }
      mailbox_.post_drive_command(cmd.cmd_linear_speed_m_s, cmd.cmd_angular_rate_rad_s);
      ++stats_.drive_commands;
      return;
    }
    case MessageId::SafetyStateRequest: {
      protocol::SafetyStateRequest req{};
      if (!protocol::SafetyStateRequest::decode(f.payload, f.payload_len, req)) {
        send_nack(f.message_id, f.seq, NackReason::RANGE_REJECT);
        return;
      }
      if (!mailbox_.post_request({true, req.requested_state, f.seq})) {
        ++stats_.request_overflows;
        send_nack(f.message_id, f.seq, NackReason::UNSPECIFIED);  // ADR 0015 §2
      }
      return;
    }
    case MessageId::EstopRequest: {
      protocol::EstopRequest req{};
      if (!protocol::EstopRequest::decode(f.payload, f.payload_len, req)) {
        send_nack(f.message_id, f.seq, NackReason::BAD_MAGIC);
        return;
      }
      mailbox_.post_estop();
      return;
    }
    case MessageId::ParamGet:
    case MessageId::ParamSet:
      // A valid frame (feeds the DISARMED watchdog, ADR 0014), but no parameter table exists
      // yet, so no ID is known. The param-table slice replaces this.
      mailbox_.post_other_frame();
      send_nack(f.message_id, f.seq, NackReason::UNKNOWN_PARAM_ID);
      return;
    case MessageId::ParamCommit:
      mailbox_.post_other_frame();
      send_nack(f.message_id, f.seq, NackReason::UNSPECIFIED);
      return;
    default:
      send_nack(f.message_id, f.seq, NackReason::UNKNOWN_MESSAGE_ID);
      return;
  }
}

template <typename Msg>
void MainLoop::send(MessageId id, const Msg& msg) {
  uint8_t payload[Msg::kPayloadBytes];
  const size_t payload_len = msg.encode(payload);
  uint8_t wire[protocol::kMaxWireFrameBytes];
  size_t len = 0;
  if (!protocol::encode_frame(static_cast<uint8_t>(id), tx_seq_++, clock_.now_us(), payload,
                              payload_len, wire, sizeof wire, len)) {
    return;  // cannot happen for schema messages: sizes are checked at generation time
  }
  // A short write leaves a partial frame on the wire; the operator's decoder rejects it and
  // loses at most one more frame (FrameDecoder resync guarantee). Counted, never silent.
  if (serial_.write(wire, len) == len) {
    ++stats_.frames_sent;
  } else {
    ++stats_.tx_short_writes;
  }
}

void MainLoop::send_nack(uint8_t message_id, uint8_t seq, NackReason reason) {
  protocol::Nack nack{};
  nack.rejected_message_id = message_id;
  nack.reason = reason;
  nack.rejected_seq = seq;
  send(MessageId::Nack, nack);
}

void MainLoop::send_fault(FaultCode code, uint32_t context) {
  protocol::Fault fault{};
  fault.fault_code = code;
  fault.context = context;
  send(MessageId::Fault, fault);
}

}  // namespace recon::core
