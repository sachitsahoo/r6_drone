#pragma once

#include <cstddef>
#include <cstdint>

namespace recon::protocol {

/// Consistent Overhead Byte Stuffing.
///
/// COBS removes every zero byte from a buffer, which makes 0x00 an unambiguous frame
/// delimiter on a byte stream that carries no framing of its own. Encoding n bytes
/// produces at most n + ceil(n/254) bytes; the frame layer appends the delimiter.
///
/// Reference: Cheshire & Baker, "Consistent Overhead Byte Stuffing", IEEE/ACM
/// Transactions on Networking, 1999.
///
/// These functions do not read or write the delimiter. Keeping the delimiter out of the
/// codec means the encoder cannot accidentally emit one and the decoder cannot depend on
/// finding one, which keeps the two concerns independently testable.

/// One COBS block: the maximum run of non-zero bytes a single code byte can describe.
constexpr size_t kCobsBlockBytes = 254;

/// Worst-case encoded size for `len` input bytes, excluding the delimiter.
///
/// Every 254-byte block costs one overhead byte, and a zero-length input still costs the
/// single code byte that encodes "empty".
constexpr size_t cobs_max_encoded_bytes(size_t len) {
  return len + (len / kCobsBlockBytes) + 1;
}

/// COBS-encodes `in` into `out`.
///
/// \param in       Input bytes. May be null only if `len` is 0.
/// \param len      Number of input bytes.
/// \param out      Output buffer, at least cobs_max_encoded_bytes(len) long.
/// \param out_cap  Capacity of `out`.
/// \param out_len  Set to the number of encoded bytes written on success.
/// \return False if `out_cap` is too small, in which case `out` is partially written and
///         `out_len` is unchanged.
///
/// \post On success the encoded bytes contain no 0x00, so the caller may append one as a
///       delimiter.
/// \note ISR-safe: no allocation, no blocking.
bool cobs_encode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap,
                 size_t& out_len);

/// COBS-decodes `in` into `out`.
///
/// \param in       Encoded bytes, with the delimiter already stripped.
/// \param len      Number of encoded bytes.
/// \param out      Output buffer.
/// \param out_cap  Capacity of `out`.
/// \param out_len  Set to the number of decoded bytes written on success.
/// \return False if the input is malformed or `out_cap` is too small. Malformed means: a
///         0x00 byte inside the encoded data, a code byte that points past the end of the
///         input, or a zero-length input.
///
/// \note Must tolerate arbitrary bytes: this is the first thing a corrupt or
///       desynchronized radio link reaches. Fuzz-tested.
/// \note ISR-safe: no allocation, no blocking.
bool cobs_decode(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap,
                 size_t& out_len);

}  // namespace recon::protocol
