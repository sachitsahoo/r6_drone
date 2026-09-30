#include "protocol/cobs.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <numeric>
#include <vector>

namespace {

using recon::protocol::cobs_decode;
using recon::protocol::cobs_encode;
using recon::protocol::cobs_max_encoded_bytes;
using recon::protocol::kCobsBlockBytes;

std::vector<uint8_t> encode(const std::vector<uint8_t>& in, bool* ok = nullptr) {
  std::vector<uint8_t> out(cobs_max_encoded_bytes(in.size()) + 8);
  size_t len = 0;
  const bool success = cobs_encode(in.data(), in.size(), out.data(), out.size(), len);
  if (ok != nullptr) {
    *ok = success;
  }
  out.resize(success ? len : 0);
  return out;
}

std::vector<uint8_t> decode(const std::vector<uint8_t>& in, bool* ok) {
  std::vector<uint8_t> out(in.size() + kCobsBlockBytes + 8);
  size_t len = 0;
  *ok = cobs_decode(in.data(), in.size(), out.data(), out.size(), len);
  out.resize(*ok ? len : 0);
  return out;
}

void expect_roundtrip(const std::vector<uint8_t>& original) {
  const auto encoded = encode(original);
  // The point of COBS: no zero survives encoding, so 0x00 can delimit frames.
  for (size_t i = 0; i < encoded.size(); ++i) {
    ASSERT_NE(encoded[i], 0u) << "zero byte at " << i << " in encoded output";
  }
  bool ok = false;
  const auto decoded = decode(encoded, &ok);
  ASSERT_TRUE(ok);
  EXPECT_EQ(decoded, original);
}

TEST(Cobs, EmptyInput) {
  const auto encoded = encode({});
  EXPECT_EQ(encoded, std::vector<uint8_t>{0x01});
  expect_roundtrip({});
}

TEST(Cobs, KnownVectorsFromThePaper) {
  // Cheshire & Baker, table 1. Pinning these means the encoding is the standard one and
  // not merely self-consistent -- the same argument as the CRC known-answer test.
  struct Case {
    std::vector<uint8_t> decoded;
    std::vector<uint8_t> encoded;
  };
  const std::vector<Case> cases = {
      {{0x00}, {0x01, 0x01}},
      {{0x00, 0x00}, {0x01, 0x01, 0x01}},
      {{0x11, 0x22, 0x00, 0x33}, {0x03, 0x11, 0x22, 0x02, 0x33}},
      {{0x11, 0x22, 0x33, 0x44}, {0x05, 0x11, 0x22, 0x33, 0x44}},
      {{0x11, 0x00, 0x00, 0x00}, {0x02, 0x11, 0x01, 0x01, 0x01}},
  };
  for (const auto& c : cases) {
    EXPECT_EQ(encode(c.decoded), c.encoded);
    bool ok = false;
    EXPECT_EQ(decode(c.encoded, &ok), c.decoded);
    EXPECT_TRUE(ok);
  }
}

// Trailing zeros are the case a naive "trim the last block" optimization silently breaks:
// the dropped code byte takes the final zero with it.
TEST(Cobs, InputEndingInZeroRoundTrips) {
  expect_roundtrip({0x00});
  expect_roundtrip({0x11, 0x00});
  expect_roundtrip({0x11, 0x22, 0x00});
  expect_roundtrip({0x00, 0x00, 0x00});
  expect_roundtrip({0x11, 0x00, 0x22, 0x00});
}

TEST(Cobs, RoundTripsEveryLengthOfNonZeroData) {
  for (size_t len = 0; len <= 2 * kCobsBlockBytes + 4; ++len) {
    std::vector<uint8_t> data(len);
    for (size_t i = 0; i < len; ++i) {
      data[i] = static_cast<uint8_t>((i % 255) + 1);  // never zero
    }
    expect_roundtrip(data);
  }
}

// 253/254/255 are where the single-code-byte block ends and a new one must be started.
TEST(Cobs, BlockBoundaryLengths) {
  for (size_t len : {kCobsBlockBytes - 2, kCobsBlockBytes - 1, kCobsBlockBytes,
                     kCobsBlockBytes + 1, kCobsBlockBytes + 2, 2 * kCobsBlockBytes,
                     2 * kCobsBlockBytes + 1}) {
    std::vector<uint8_t> data(len, 0xAB);
    expect_roundtrip(data);
    std::vector<uint8_t> with_zero(len, 0xAB);
    with_zero[len / 2] = 0x00;
    expect_roundtrip(with_zero);
    std::vector<uint8_t> trailing_zero(len, 0xAB);
    trailing_zero[len - 1] = 0x00;
    expect_roundtrip(trailing_zero);
  }
}

TEST(Cobs, RoundTripsAllZeros) {
  for (size_t len = 1; len <= 300; ++len) {
    expect_roundtrip(std::vector<uint8_t>(len, 0x00));
  }
}

TEST(Cobs, WorstCaseSizeBoundIsRespected) {
  for (size_t len = 0; len <= 600; ++len) {
    std::vector<uint8_t> data(len, 0xFF);
    EXPECT_LE(encode(data).size(), cobs_max_encoded_bytes(len)) << "len " << len;
  }
}

// --------------------------------------------------------------- malformed input

TEST(Cobs, RejectsEmptyEncodedInput) {
  bool ok = true;
  decode({}, &ok);
  EXPECT_FALSE(ok) << "not even a code byte cannot be a valid encoding";
}

TEST(Cobs, RejectsZeroByteInsideEncodedData) {
  bool ok = true;
  decode({0x03, 0x11, 0x00}, &ok);
  EXPECT_FALSE(ok);
}

TEST(Cobs, RejectsCodeByteThatOverrunsTheInput) {
  bool ok = true;
  decode({0x05, 0x11, 0x22}, &ok);  // promises 4 literals, only 2 present
  EXPECT_FALSE(ok);
}

TEST(Cobs, RejectsWhenOutputCapacityIsTooSmall) {
  const std::vector<uint8_t> encoded{0x05, 0x11, 0x22, 0x33, 0x44};
  uint8_t out[2] = {};
  size_t len = 0;
  EXPECT_FALSE(cobs_decode(encoded.data(), encoded.size(), out, sizeof(out), len));
}

TEST(Cobs, EncodeRejectsInsufficientCapacity) {
  const std::vector<uint8_t> data(10, 0xAA);
  uint8_t out[4] = {};
  size_t len = 0;
  EXPECT_FALSE(cobs_encode(data.data(), data.size(), out, sizeof(out), len));
}

// The decoder is the first thing a corrupt radio link reaches, so it must merely fail on
// arbitrary bytes rather than crash or read out of bounds.
TEST(Cobs, SurvivesArbitraryGarbage) {
  uint32_t state = 0x12345678U;
  for (int iteration = 0; iteration < 20000; ++iteration) {
    state = state * 1664525U + 1013904223U;  // deterministic LCG, reproducible failures
    const size_t len = (state >> 16) % 80;
    std::vector<uint8_t> garbage(len);
    for (size_t i = 0; i < len; ++i) {
      state = state * 1664525U + 1013904223U;
      garbage[i] = static_cast<uint8_t>(state >> 24);
    }
    bool ok = false;
    const auto decoded = decode(garbage, &ok);
    if (ok) {
      // Anything it claims to have decoded must re-encode to something decodable.
      bool reencode_ok = false;
      const auto reencoded = encode(decoded, &reencode_ok);
      ASSERT_TRUE(reencode_ok);
      bool again_ok = false;
      EXPECT_EQ(decode(reencoded, &again_ok), decoded);
      EXPECT_TRUE(again_ok);
    }
  }
}

}  // namespace
