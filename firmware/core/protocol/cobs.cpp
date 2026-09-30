#include "protocol/cobs.hpp"

namespace recon::protocol {

bool cobs_encode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap,
                 size_t& out_len) {
  if (out_cap < cobs_max_encoded_bytes(len)) {
    return false;
  }

  // `code_index` holds the slot where the current block's length byte will be written.
  // It is written only once the block ends, because its value is not known until then.
  size_t read_index = 0;
  size_t write_index = 1;
  size_t code_index = 0;
  uint8_t code = 1;

  while (read_index < len) {
    if (in[read_index] == 0) {
      // A zero terminates the current block and is itself encoded by the code byte.
      out[code_index] = code;
      code = 1;
      code_index = write_index;
      ++write_index;
      ++read_index;
    } else {
      out[write_index] = in[read_index];
      ++write_index;
      ++read_index;
      ++code;
      if (code == 0xFF) {
        // A full block: emit its length and start another. Without this a run longer
        // than 254 non-zero bytes could not be described by a single code byte.
        out[code_index] = code;
        code = 1;
        code_index = write_index;
        ++write_index;
      }
    }
  }

  out[code_index] = code;
  out_len = write_index;
  return true;

  // NOTE: an earlier version trimmed a trailing single-byte block here, on the theory
  // that it was a redundant reserved slot. It is not. When the input ends in 0x00 that
  // final code byte is what encodes the trailing zero, so trimming it silently dropped
  // the last byte of every such payload -- including a telemetry frame whose last field
  // happened to be zero. This is the standard encoding from the paper; leave it alone.
}

bool cobs_decode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap,
                 size_t& out_len) {
  if (len == 0) {
    return false;  // not even a code byte: cannot be a valid encoding
  }

  size_t read_index = 0;
  size_t write_index = 0;

  while (read_index < len) {
    const uint8_t code = in[read_index];
    if (code == 0) {
      return false;  // encoded data must never contain a zero
    }
    ++read_index;

    // Copy code-1 literal bytes.
    for (uint8_t i = 1; i < code; ++i) {
      if (read_index >= len) {
        return false;  // code byte promises more data than the frame contains
      }
      if (in[read_index] == 0) {
        // A zero in a literal position is impossible in a well-formed encoding. Rejecting
        // it here matters: the caller found this run between two delimiters, so a zero
        // inside it means the stream was already misframed.
        return false;
      }
      if (write_index >= out_cap) {
        return false;
      }
      out[write_index] = in[read_index];
      ++write_index;
      ++read_index;
    }

    // A block shorter than the maximum stood for a zero in the original data -- unless
    // the input ended, in which case the final block carries no implied zero.
    if (code != 0xFF && read_index < len) {
      if (write_index >= out_cap) {
        return false;
      }
      out[write_index] = 0;
      ++write_index;
    }
  }

  out_len = write_index;
  return true;
}

}  // namespace recon::protocol
