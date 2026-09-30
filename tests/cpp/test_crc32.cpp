#include "protocol/crc32.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

using recon::protocol::crc32;
using recon::protocol::crc32_finalize;
using recon::protocol::crc32_update;
using recon::protocol::kCrc32Init;

std::vector<uint8_t> bytes_of(const std::string& s) {
  return std::vector<uint8_t>(s.begin(), s.end());
}

// The published check value for CRC-32/ISO-HDLC. This is the whole reason the variant was
// chosen over a hand-rolled one: a CRC implementation that is subtly wrong still
// round-trips against itself and looks correct, so self-consistency proves nothing.
TEST(Crc32, PublishedKnownAnswer) {
  const auto data = bytes_of("123456789");
  EXPECT_EQ(crc32(data.data(), data.size()), 0xCBF43926U);
}

// Python's zlib.crc32 computes exactly this function. These values were produced by
// zlib.crc32 and are what make the cross-language test vectors trustworthy.
TEST(Crc32, AgreesWithPythonZlibOnKnownInputs) {
  struct Case {
    const char* text;
    uint32_t expected;
  };
  const Case cases[] = {
      {"", 0x00000000U},
      {"a", 0xE8B7BE43U},
      {"abc", 0x352441C2U},
      {"The quick brown fox jumps over the lazy dog", 0x414FA339U},
  };
  for (const auto& c : cases) {
    const auto data = bytes_of(c.text);
    EXPECT_EQ(crc32(data.data(), data.size()), c.expected) << "input: " << c.text;
  }
}

TEST(Crc32, EmptyInputIsTheFinalizedInitialState) {
  EXPECT_EQ(crc32(nullptr, 0), crc32_finalize(kCrc32Init));
  EXPECT_EQ(crc32(nullptr, 0), 0x00000000U);
}

// The streaming form must equal the one-shot form, or a frame CRC computed as the frame is
// assembled would disagree with one computed over the finished buffer.
TEST(Crc32, IncrementalMatchesOneShot) {
  std::vector<uint8_t> data(300);
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>(i * 7 + 13);
  }
  for (size_t split = 0; split <= data.size(); split += 37) {
    uint32_t state = crc32_update(kCrc32Init, data.data(), split);
    state = crc32_update(state, data.data() + split, data.size() - split);
    EXPECT_EQ(crc32_finalize(state), crc32(data.data(), data.size()))
        << "split at " << split;
  }
}

TEST(Crc32, DetectsEverySingleBitFlip) {
  std::vector<uint8_t> data{0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x7F, 0x80};
  const uint32_t reference = crc32(data.data(), data.size());
  for (size_t byte = 0; byte < data.size(); ++byte) {
    for (int bit = 0; bit < 8; ++bit) {
      auto corrupted = data;
      corrupted[byte] ^= static_cast<uint8_t>(1U << bit);
      EXPECT_NE(crc32(corrupted.data(), corrupted.size()), reference)
          << "byte " << byte << " bit " << bit;
    }
  }
}

// A zero init would make leading zero bytes invisible. COBS keeps zeros off the wire, so
// this is defense in depth rather than the primary reason -- but it is free.
TEST(Crc32, LeadingZeroBytesChangeTheResult) {
  const std::vector<uint8_t> one_zero{0x00};
  const std::vector<uint8_t> two_zeros{0x00, 0x00};
  EXPECT_NE(crc32(one_zero.data(), one_zero.size()),
            crc32(two_zeros.data(), two_zeros.size()));
}

// The table is built by a constexpr function, so a mistake in it is a compile-time
// mistake. This pins the property rather than the 1024 bytes of contents.
TEST(Crc32, IsUsableInAConstantExpression) {
  static constexpr uint8_t kData[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  static_assert(crc32(kData, sizeof(kData)) == 0xCBF43926U,
                "CRC-32 must be computable at compile time");
  SUCCEED();
}

}  // namespace
