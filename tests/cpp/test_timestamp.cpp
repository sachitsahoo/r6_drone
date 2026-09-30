#include "time/timestamp.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using recon::core::elapsed_us;
using recon::core::kTimestampWrapPeriodUs;

constexpr uint32_t kUint32Max = 0xFFFFFFFFu;

TEST(ElapsedUs, ZeroWhenTimestampsAreEqual) {
  EXPECT_EQ(elapsed_us(0u, 0u), 0u);
  EXPECT_EQ(elapsed_us(123456u, 123456u), 0u);
}

TEST(ElapsedUs, SimpleForwardDifference) {
  EXPECT_EQ(elapsed_us(1000u, 2500u), 1500u);
  EXPECT_EQ(elapsed_us(0u, 1u), 1u);
}

// The 200 ms comms watchdog in CLAUDE.md is the motivating caller: it must fire
// correctly no matter where in the 71-minute timestamp cycle the link drops.
TEST(ElapsedUs, WatchdogIntervalIsCorrectAcrossTheWrap) {
  constexpr uint32_t kWatchdogTimeoutUs = 200000u;  // 200 ms, CLAUDE.md
  constexpr uint32_t kLastCommandUs = kUint32Max - 50000u;  // 50 ms before wrap
  const uint32_t now_us = kLastCommandUs + kWatchdogTimeoutUs;  // wraps past zero

  EXPECT_LT(now_us, kLastCommandUs) << "test setup must actually cross the wrap";
  EXPECT_EQ(elapsed_us(kLastCommandUs, now_us), kWatchdogTimeoutUs);
}

TEST(ElapsedUs, SingleTickAcrossTheWrapBoundary) {
  EXPECT_EQ(elapsed_us(kUint32Max, 0u), 1u);
  EXPECT_EQ(elapsed_us(kUint32Max, 1u), 2u);
}

TEST(ElapsedUs, LargestRepresentableInterval) {
  EXPECT_EQ(elapsed_us(0u, kUint32Max), kUint32Max);
  EXPECT_EQ(elapsed_us(1u, 0u), kUint32Max);
}

// Documents a precondition the function cannot enforce, so the limitation is
// pinned by a test rather than living only in a comment. A full wrap is
// indistinguishable from no elapsed time at all.
TEST(ElapsedUs, FullWrapIsIndistinguishableFromZero) {
  EXPECT_EQ(elapsed_us(777u, 777u), 0u);
  EXPECT_EQ(kTimestampWrapPeriodUs, 4294967296ULL);
  EXPECT_EQ(static_cast<uint64_t>(kUint32Max) + 1u, kTimestampWrapPeriodUs);
}

// Reversed arguments return a large value rather than zero. Also documented in
// the header; tested so nobody "fixes" it into a silent clamp without deciding to.
TEST(ElapsedUs, ReversedArgumentsReturnLargeValueNotZero) {
  EXPECT_EQ(elapsed_us(2000u, 1000u), kUint32Max - 999u);
  EXPECT_GT(elapsed_us(2000u, 1000u), 0u);
}

}  // namespace
