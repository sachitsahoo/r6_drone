// StaleCommandFilter: protocol/design-proposal.md (stale-command rejection), ADR 0015 §3.

#include "protocol/stale_command_filter.hpp"

#include <gtest/gtest.h>

#include <cstdint>

#include "messages.hpp"

namespace recon::protocol {
namespace {

constexpr uint32_t kResetUs = 200000;  // comms_timeout_ms default, in us
constexpr auto kDrive = static_cast<uint8_t>(MessageId::DriveCommand);
constexpr auto kHeartbeat = static_cast<uint8_t>(MessageId::Heartbeat);
constexpr auto kEstop = static_cast<uint8_t>(MessageId::EstopRequest);

TEST(StaleCommandFilter, TheFirstFrameOfEachIdIsAccepted) {
  StaleCommandFilter f(kResetUs);
  EXPECT_TRUE(f.accept(kDrive, 5000, 0));
  EXPECT_TRUE(f.accept(kHeartbeat, 1, 0)) << "baselines are per message ID";
}

TEST(StaleCommandFilter, NewerIsAcceptedOlderAndEqualAreRejected) {
  StaleCommandFilter f(kResetUs);
  ASSERT_TRUE(f.accept(kDrive, 1000, 0));
  EXPECT_FALSE(f.accept(kDrive, 1000, 1000)) << "a duplicate";
  EXPECT_FALSE(f.accept(kDrive, 999, 1000)) << "a delayed datagram";
  EXPECT_TRUE(f.accept(kDrive, 1001, 1000));
  EXPECT_FALSE(f.accept(kDrive, 1000, 2000)) << "the baseline moved to 1001";
}

TEST(StaleCommandFilter, NewerIsJudgedAcrossTheOperatorClockWrap) {
  StaleCommandFilter f(kResetUs);
  ASSERT_TRUE(f.accept(kDrive, 0xFFFFFFF0U, 0));
  EXPECT_TRUE(f.accept(kDrive, 0x00000010U, 1000)) << "32 us later, past the wrap";
  EXPECT_FALSE(f.accept(kDrive, 0xFFFFFFF8U, 2000)) << "before the wrap is now older";
}

TEST(StaleCommandFilter, EstopIsNeverRefused) {
  StaleCommandFilter f(kResetUs);
  ASSERT_TRUE(f.accept(kEstop, 1000, 0));
  EXPECT_TRUE(f.accept(kEstop, 1000, 1));
  EXPECT_TRUE(f.accept(kEstop, 1, 2));
}

TEST(StaleCommandFilter, IdsOutsideTheOperatorRangeAlwaysPass) {
  StaleCommandFilter f(kResetUs);
  const auto fault = static_cast<uint8_t>(MessageId::Fault);
  EXPECT_TRUE(f.accept(fault, 5, 0));
  EXPECT_TRUE(f.accept(fault, 5, 0));
}

TEST(StaleCommandFilter, ARestartedOperatorIsAcceptedAfterTheLinkGoesStale) {
  // ADR 0015 Q3: the app restarts, its clock is back near 0.
  StaleCommandFilter f(kResetUs);
  ASSERT_TRUE(f.accept(kDrive, 3000000, 0));
  for (uint32_t t = 1000; t < kResetUs; t += 1000) {
    f.update(t);
    ASSERT_FALSE(f.accept(kDrive, 50, t)) << "still stale-looking within the timeout";
  }
  f.update(kResetUs);
  EXPECT_TRUE(f.accept(kDrive, 50, kResetUs)) << "silence reached the timeout: baseline forgotten";
  EXPECT_FALSE(f.accept(kDrive, 49, kResetUs + 1000)) << "and the new baseline holds";
}

TEST(StaleCommandFilter, RejectedFramesDoNotKeepTheBaselineAlive) {
  // A restarted app's heartbeats are all rejected; that silence still counts.
  StaleCommandFilter f(kResetUs);
  ASSERT_TRUE(f.accept(kHeartbeat, 3000000, 0));
  for (uint32_t t = 20000; t < kResetUs; t += 20000) {
    f.update(t);
    EXPECT_FALSE(f.accept(kHeartbeat, t / 1000, t));
  }
  f.update(kResetUs);
  EXPECT_TRUE(f.accept(kHeartbeat, 200, kResetUs));
}

TEST(StaleCommandFilter, TheResetLatchSurvivesAnMcuClockWrap) {
  // Accept once, then go silent for one full 2^32 us wrap. A naive check would think the last
  // accept was recent; the latch, updated every main-loop pass, must already have fired.
  StaleCommandFilter f(kResetUs);
  ASSERT_TRUE(f.accept(kDrive, 3000000, 0));
  uint64_t t = 0;
  for (; t < (1ULL << 32) + 1000; t += 10000) {
    f.update(static_cast<uint32_t>(t));
  }
  EXPECT_TRUE(f.accept(kDrive, 5, static_cast<uint32_t>(t)));
}

TEST(StaleCommandFilter, AcceptAloneAppliesTheResetToo) {
  // Even if update() is not called between frames, accept() sees the silence.
  StaleCommandFilter f(kResetUs);
  ASSERT_TRUE(f.accept(kDrive, 3000000, 0));
  EXPECT_TRUE(f.accept(kDrive, 50, kResetUs));
}

}  // namespace
}  // namespace recon::protocol
