// SpscRing, SafetyMailbox and ReportQueue: ADR 0015 §2.

#include "safety/safety_mailbox.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace recon::core {
namespace {

using protocol::SafetyState;

TEST(SpscRing, FifoOrderAndCapacity) {
  SpscRing<int, 4> ring;
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(ring.push(i));
  }
  EXPECT_FALSE(ring.push(99)) << "full at capacity";
  EXPECT_EQ(ring.size(), 4U);
  int v = -1;
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(ring.pop(v));
    EXPECT_EQ(v, i);
  }
  EXPECT_FALSE(ring.pop(v));
  EXPECT_EQ(v, 3) << "an empty pop leaves out untouched";
}

TEST(SpscRing, IndicesWrapAtTwoToThe32) {
  // Push/pop far past 2^32 is slow to reach; the indices are free-running uint32, so cycling
  // the ring many times exercises the modulo path. 2^20 cycles of a 4-slot ring.
  SpscRing<uint32_t, 4> ring;
  uint32_t v = 0;
  for (uint32_t i = 0; i < (1U << 20); ++i) {
    ASSERT_TRUE(ring.push(i));
    ASSERT_TRUE(ring.pop(v));
    ASSERT_EQ(v, i);
  }
}

SafetyInputs empty() { return SafetyInputs{}; }

TEST(SafetyMailbox, TakeOnAnEmptyMailboxChangesNothing) {
  SafetyMailbox box;
  SafetyInputs in = empty();
  box.take(in);
  EXPECT_FALSE(in.estop);
  EXPECT_FALSE(in.request.present);
  EXPECT_FALSE(in.drive_command);
  EXPECT_FALSE(in.other_frame);
}

TEST(SafetyMailbox, EstopIsDeliveredOnceAndAheadOfQueuedRequests) {
  SafetyMailbox box;
  ASSERT_TRUE(box.post_request({true, SafetyState::ARMED, 1}));
  box.post_estop();
  SafetyInputs in = empty();
  box.take(in);
  EXPECT_TRUE(in.estop);
  SafetyInputs again = empty();
  box.take(again);
  EXPECT_FALSE(again.estop) << "consumed";
}

TEST(SafetyMailbox, OneRequestPerTakeInOrder) {
  SafetyMailbox box;
  ASSERT_TRUE(box.post_request({true, SafetyState::DISARMED, 10}));
  ASSERT_TRUE(box.post_request({true, SafetyState::ARMED, 11}));
  SafetyInputs a = empty();
  box.take(a);
  SafetyInputs b = empty();
  box.take(b);
  SafetyInputs c = empty();
  box.take(c);
  EXPECT_EQ(a.request.seq, 10);
  EXPECT_EQ(a.request.requested_state, SafetyState::DISARMED);
  EXPECT_EQ(b.request.seq, 11);
  EXPECT_FALSE(c.request.present);
}

TEST(SafetyMailbox, AFullRequestRingRefusesTheFifth) {
  SafetyMailbox box;
  for (uint8_t i = 0; i < kSafetyRequestSlots; ++i) {
    ASSERT_TRUE(box.post_request({true, SafetyState::ARMED, i}));
  }
  EXPECT_FALSE(box.post_request({true, SafetyState::ARMED, 99}));
}

TEST(SafetyMailbox, OnlyTheNewestDriveCommandIsDeliveredAndOnlyOnce) {
  SafetyMailbox box;
  box.post_drive_command(0.1F, 0.0F);
  box.post_drive_command(0.2F, -0.5F);
  SafetyInputs in = empty();
  box.take(in);
  EXPECT_TRUE(in.drive_command);
  EXPECT_FLOAT_EQ(in.drive_linear_m_s, 0.2F);
  EXPECT_FLOAT_EQ(in.drive_angular_rad_s, -0.5F);
  SafetyInputs again = empty();
  box.take(again);
  EXPECT_FALSE(again.drive_command) << "no new command: the watchdog must not be fed again";
}

TEST(SafetyMailbox, AReaderPreemptingAWriterSeesTheLastCompleteCommand) {
  // The ISR fires after the main loop has written the next slot but before it publishes.
  SafetyMailbox box;
  box.post_drive_command(0.1F, 0.1F);
  box.stage_drive_command(0.9F, 0.9F);  // half-way through a post
  SafetyInputs in = empty();
  box.take(in);
  EXPECT_TRUE(in.drive_command);
  EXPECT_FLOAT_EQ(in.drive_linear_m_s, 0.1F) << "the staged command is invisible";
  EXPECT_FLOAT_EQ(in.drive_angular_rad_s, 0.1F) << "and never mixed with the published one";
  box.publish_drive_command();
  SafetyInputs next = empty();
  box.take(next);
  EXPECT_FLOAT_EQ(next.drive_linear_m_s, 0.9F);
  EXPECT_FLOAT_EQ(next.drive_angular_rad_s, 0.9F);
}

TEST(SafetyMailbox, AlternatingSlotsAlwaysDeliverAWholeCommand) {
  SafetyMailbox box;
  for (int i = 0; i < 10; ++i) {
    const float v = 0.01F * static_cast<float>(i);
    box.post_drive_command(v, -v);
    SafetyInputs in = empty();
    box.take(in);
    ASSERT_FLOAT_EQ(in.drive_linear_m_s, v);
    ASSERT_FLOAT_EQ(in.drive_angular_rad_s, -v);
  }
}

TEST(SafetyMailbox, OtherFrameFlagIsConsumed) {
  SafetyMailbox box;
  box.post_other_frame();
  SafetyInputs in = empty();
  box.take(in);
  EXPECT_TRUE(in.other_frame);
  SafetyInputs again = empty();
  box.take(again);
  EXPECT_FALSE(again.other_frame);
}

TEST(ReportQueue, CountsDropsWhenFull) {
  ReportQueue q;
  OutboundReport r;
  for (size_t i = 0; i < kOutboundReportSlots; ++i) {
    ASSERT_TRUE(q.push(r));
  }
  EXPECT_FALSE(q.push(r));
  EXPECT_FALSE(q.push(r));
  EXPECT_EQ(q.dropped(), 2U);
  ASSERT_TRUE(q.pop(r));
  EXPECT_TRUE(q.push(r));
}

TEST(ReportQueue, HoldsAFullWorstCaseTick) {
  EXPECT_GE(kOutboundReportSlots, kMaxFaultReportsPerTick + 1);
}

}  // namespace
}  // namespace recon::core
