#pragma once

#include <cstddef>
#include <cstdint>

namespace recon::protocol {

/// CRC-32/ISO-HDLC, the Ethernet / zlib / PNG CRC.
///
/// Parameters: polynomial 0x04C11DB7 (0xEDB88320 reflected), init 0xFFFFFFFF, input and
/// output reflected, final XOR 0xFFFFFFFF. Known-answer check: the CRC of the ASCII
/// string "123456789" is 0xCBF43926.
///
/// This variant is chosen so that Python's stdlib `zlib.crc32` computes exactly the same
/// function. The Python side of the protocol therefore needs no CRC implementation to
/// write, test, or keep in agreement with this one, which removes a class of
/// cross-language drift. See docs/decisions/0002-protocol-framing-and-codec.md.
namespace detail {

/// 256-entry lookup table, built at compile time so it lands in flash rather than being
/// computed at startup. 1024 bytes, or 0.20% of the G474RE's 512 KB.
struct Crc32Table {
  uint32_t entries[256];
};

/// Reflected CRC-32 polynomial: 0x04C11DB7 with its bits reversed.
constexpr uint32_t kCrc32ReflectedPoly = 0xEDB88320U;

constexpr Crc32Table make_crc32_table() {
  Crc32Table table{};
  for (uint32_t i = 0; i < 256U; ++i) {
    uint32_t remainder = i;
    for (int bit = 0; bit < 8; ++bit) {
      remainder = ((remainder & 1U) != 0U) ? (kCrc32ReflectedPoly ^ (remainder >> 1))
                                           : (remainder >> 1);
    }
    table.entries[i] = remainder;
  }
  return table;
}

inline constexpr Crc32Table kCrc32Table = make_crc32_table();

}  // namespace detail

/// Initial CRC state, before any data is fed in.
constexpr uint32_t kCrc32Init = 0xFFFFFFFFU;

/// Value XORed into the state to produce the final CRC.
constexpr uint32_t kCrc32XorOut = 0xFFFFFFFFU;

/// Folds `len` bytes into a running CRC state.
///
/// Streaming form, so a frame's CRC can be computed as it is assembled without a second
/// pass over the buffer.
///
/// \param state Running state; start from kCrc32Init.
/// \param data  Bytes to fold in. May be null only if `len` is 0.
/// \param len   Number of bytes.
/// \return Updated state. Pass through crc32_finalize() to get the CRC.
///
/// \note ISR-safe: pure, no state, no allocation, no blocking. Constant time per byte.
constexpr uint32_t crc32_update(uint32_t state, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    const uint8_t index = static_cast<uint8_t>((state ^ data[i]) & 0xFFU);
    state = detail::kCrc32Table.entries[index] ^ (state >> 8);
  }
  return state;
}

/// Converts a running state into the transmitted CRC value.
constexpr uint32_t crc32_finalize(uint32_t state) { return state ^ kCrc32XorOut; }

/// CRC-32/ISO-HDLC of a single buffer.
///
/// \param data Bytes to checksum. May be null only if `len` is 0.
/// \param len  Number of bytes.
/// \return The CRC, ready to place on the wire little-endian.
///
/// \note ISR-safe: pure, no allocation, no blocking.
constexpr uint32_t crc32(const uint8_t* data, size_t len) {
  return crc32_finalize(crc32_update(kCrc32Init, data, len));
}

}  // namespace recon::protocol
