#include "protocol/frame.hpp"

#include "protocol/cobs.hpp"
#include "protocol/crc32.hpp"

namespace recon::protocol {

namespace {
/// Smallest frame that could possibly be valid: header plus CRC, with no payload.
constexpr size_t kMinLogicalFrameBytes = kHeaderBytes + kCrcBytes;
}  // namespace

bool encode_frame(uint8_t message_id, uint8_t seq, uint32_t timestamp_us,
                  const uint8_t* payload, size_t payload_len,
                  uint8_t* out, size_t out_cap, size_t& out_len) {
  if (payload_len > kMaxPayloadBytes) {
    return false;
  }
  if (out_cap == 0) {
    return false;  // no room even for the delimiter
  }

  uint8_t logical[kMaxLogicalFrameBytes] = {};
  logical[0] = kProtocolVersion;
  logical[1] = message_id;
  logical[2] = seq;
  detail::write_u32(logical + 3, timestamp_us);
  for (size_t i = 0; i < payload_len; ++i) {
    logical[kHeaderBytes + i] = payload[i];
  }

  // CRC covers the logical frame, not the COBS-encoded bytes (ADR 0002): the checksum
  // protects the content regardless of transport encoding, and the decoder never has to
  // trust a COBS decode before it has validated anything.
  const size_t checked_len = kHeaderBytes + payload_len;
  detail::write_u32(logical + checked_len, crc32(logical, checked_len));
  const size_t logical_len = checked_len + kCrcBytes;

  size_t encoded_len = 0;
  if (!cobs_encode(logical, logical_len, out, out_cap - 1, encoded_len)) {
    return false;  // leave room for the delimiter
  }
  out[encoded_len] = 0x00;
  out_len = encoded_len + 1;
  return true;
}

void FrameDecoder::reset() {
  len_ = 0;
  state_ = State::kAccumulating;
}

bool FrameDecoder::push_byte(uint8_t byte, DecodedFrame& out) {
  if (byte == 0x00) {
    if (state_ == State::kDesync) {
      // Re-aligned: the delimiter tells us where the next frame starts.
      state_ = State::kAccumulating;
      len_ = 0;
      return false;
    }
    // Two delimiters in a row is an idle line, not an error, so it is not counted.
    const bool complete = (len_ > 0) && validate(out);
    len_ = 0;
    return complete;
  }

  if (state_ == State::kDesync) {
    return false;  // discard until the next delimiter
  }

  if (len_ >= kMaxWireFrameBytes) {
    // More bytes than any legal frame can contain, with no delimiter in sight. Enter
    // DESYNC and discard *to the next delimiter* rather than merely clearing the buffer:
    // a plain reset re-aligns in the middle of the offending frame and then reads its
    // tail as a fresh header. This is the classic framing bug (ADR 0002).
    state_ = State::kDesync;
    ++stats_.desyncs;
    len_ = 0;
    return false;
  }

  buffer_[len_] = byte;
  ++len_;
  return false;
}

bool FrameDecoder::validate(DecodedFrame& out) {
  uint8_t logical[kMaxLogicalFrameBytes] = {};
  size_t logical_len = 0;

  if (!cobs_decode(buffer_, len_, logical, sizeof(logical), logical_len)) {
    ++stats_.cobs_errors;
    return false;
  }
  if (logical_len < kMinLogicalFrameBytes) {
    ++stats_.length_mismatch;
    return false;
  }

  // Version is checked before the CRC, per ADR 0002, because byte 0 must be readable by a
  // decoder that understands nothing else in the frame. Consequence: corruption that
  // lands on byte 0 is attributed to version_mismatch rather than crc_errors. Accepted --
  // the alternative is computing a CRC over frames from an incompatible peer.
  if (logical[0] != kProtocolVersion) {
    ++stats_.version_mismatch;
    return false;
  }

  const size_t payload_len = logical_len - kMinLogicalFrameBytes;
  const size_t checked_len = kHeaderBytes + payload_len;
  const uint32_t received_crc = detail::read_u32(logical + checked_len);
  if (crc32(logical, checked_len) != received_crc) {
    ++stats_.crc_errors;
    return false;
  }

  const uint8_t message_id = logical[1];
  size_t declared_payload = 0;
  if (!payload_bytes_for(message_id, declared_payload)) {
    ++stats_.unknown_id;
    return false;
  }
  // Replaces a self-reported length field, which could only ever disagree with reality.
  if (declared_payload != payload_len) {
    ++stats_.length_mismatch;
    return false;
  }

  out.message_id = message_id;
  out.seq = logical[2];
  out.timestamp_us = detail::read_u32(logical + 3);
  out.payload_len = payload_len;
  for (size_t i = 0; i < payload_len; ++i) {
    out.payload[i] = logical[kHeaderBytes + i];
  }
  ++stats_.frames_ok;
  return true;
}

}  // namespace recon::protocol
