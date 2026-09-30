#pragma once

#include <cstddef>
#include <cstdint>

#include "messages.hpp"  // generated: version, size constants, payload_bytes_for()

namespace recon::protocol {

/// A frame that passed every framing-level validation stage.
///
/// The payload is copied out rather than pointed into the decoder's buffer, so the caller
/// may hold it while more bytes arrive. Fixed size, no allocation.
struct DecodedFrame {
  uint8_t message_id = 0;
  uint8_t seq = 0;
  uint32_t timestamp_us = 0;
  size_t payload_len = 0;
  uint8_t payload[kMaxPayloadBytes] = {};
};

/// Builds a complete wire frame: header, payload, CRC-32, COBS, trailing 0x00.
///
/// \param message_id   Message identifier from the generated MessageId enum.
/// \param seq          Per-sender sequence number, incremented once per frame sent.
/// \param timestamp_us Microseconds since MCU boot. Wraps every ~71.6 minutes.
/// \param payload      Encoded payload, or null when `payload_len` is 0.
/// \param payload_len  Payload length; must not exceed kMaxPayloadBytes.
/// \param out          Output buffer, at least kMaxWireFrameBytes long.
/// \param out_cap      Capacity of `out`.
/// \param out_len      Set to the number of wire bytes written, including the delimiter.
/// \return False if the payload is too long or `out_cap` is too small.
///
/// \note ISR-safe: no allocation, no blocking.
bool encode_frame(uint8_t message_id, uint8_t seq, uint32_t timestamp_us,
                  const uint8_t* payload, size_t payload_len,
                  uint8_t* out, size_t out_cap, size_t& out_len);

/// Byte-at-a-time frame decoder for a stream that may contain arbitrary garbage.
///
/// Drives the state machine in ADR 0002. Safe to call from a UART receive path: no
/// allocation, no blocking, no exceptions, and bounded work per byte.
///
/// Validation order is cheapest-and-most-discriminating first: COBS decode, minimum
/// length, protocol version, CRC, known message id, then exact payload length for that
/// id. Every rejection increments its own counter, because a decoder that silently drops
/// frames cannot be debugged from the operator's seat.
///
/// RESYNCHRONIZATION GUARANTEE: after an arbitrary run of garbage, at most one subsequent
/// frame is lost. This is inherent to delimiter-based framing rather than a defect -- when
/// garbage ends without a delimiter, the decoder is still accumulating, so the next
/// frame's bytes append to that partial buffer and the first delimiter closes a run that
/// is garbage plus frame. That run fails validation and the frame after it decodes
/// normally. Callers must not treat a single lost frame after a link disturbance as a
/// fault; the LinkStats counters are what distinguish a blip from a real problem.
class FrameDecoder {
 public:
  /// Rejection counters, monotonic since construction, wrapping at uint32_t.
  ///
  /// NOTE: `version_mismatch` has no corresponding field in the LinkStats message, whose
  /// seven counters were fixed when the message set was approved. It is observable here
  /// and in tests but is not currently transmitted. Adding it to LinkStats is a protocol
  /// schema change and therefore needs owner review.
  struct Stats {
    uint32_t frames_ok = 0;
    uint32_t crc_errors = 0;
    uint32_t cobs_errors = 0;
    uint32_t unknown_id = 0;
    uint32_t length_mismatch = 0;
    uint32_t version_mismatch = 0;
    uint32_t desyncs = 0;
  };

  enum class State : uint8_t {
    /// Collecting bytes, waiting for a delimiter.
    kAccumulating,
    /// Discarding bytes until the next delimiter, after an over-long run.
    kDesync,
  };

  /// Feeds one received byte.
  ///
  /// \param byte Received byte.
  /// \param out  Filled in only when the return value is true.
  /// \return True when this byte completed a frame that passed every validation stage.
  ///
  /// \post When this returns true, `out.payload_len` equals the length declared for
  ///       `out.message_id` by the schema.
  /// \note ISR-safe. Bounded work per byte: the only loop runs over one frame's bytes,
  ///       and only on a delimiter.
  bool push_byte(uint8_t byte, DecodedFrame& out);

  const Stats& stats() const { return stats_; }
  State state() const { return state_; }

  /// Clears the partial buffer and returns to kAccumulating. Leaves counters intact.
  void reset();

 private:
  /// Validates the buffered encoded frame. Increments exactly one counter either way.
  bool validate(DecodedFrame& out);

  uint8_t buffer_[kMaxWireFrameBytes] = {};
  size_t len_ = 0;
  State state_ = State::kAccumulating;
  Stats stats_{};
};

}  // namespace recon::protocol
