// CheckInMonitor: ADR 0014, "Hardware watchdog". The IWDG is fed only when every loop has
// checked in since the last feed.

#include "safety/check_in_monitor.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace recon::core {
namespace {

constexpr uint32_t kMotorAndMain = static_cast<uint32_t>(CheckInTask::kMotorLoop) |
                                   static_cast<uint32_t>(CheckInTask::kMainLoop);

TEST(CheckInMonitor, NoFeedBeforeAnyCheckIn) {
  CheckInMonitor m(kMotorAndMain);
  EXPECT_FALSE(m.should_feed());
}

TEST(CheckInMonitor, NoFeedIfAnyRequiredTaskIsMissing) {
  CheckInMonitor motor_only(kMotorAndMain);
  motor_only.check_in(CheckInTask::kMotorLoop);
  EXPECT_FALSE(motor_only.should_feed()) << "a hung main loop must starve the IWDG";

  CheckInMonitor main_only(kMotorAndMain);
  main_only.check_in(CheckInTask::kMainLoop);
  EXPECT_FALSE(main_only.should_feed()) << "a hung motor-loop ISR must starve the IWDG";
}

TEST(CheckInMonitor, FeedsWhenEveryTaskHasCheckedIn) {
  CheckInMonitor m(kMotorAndMain);
  m.check_in(CheckInTask::kMotorLoop);
  m.check_in(CheckInTask::kMainLoop);
  EXPECT_TRUE(m.should_feed());
}

TEST(CheckInMonitor, BitsClearAfterAFeed) {
  CheckInMonitor m(kMotorAndMain);
  m.check_in(CheckInTask::kMotorLoop);
  m.check_in(CheckInTask::kMainLoop);
  ASSERT_TRUE(m.should_feed());
  EXPECT_FALSE(m.should_feed()) << "each feed needs a fresh round of check-ins";
  m.check_in(CheckInTask::kMainLoop);
  EXPECT_FALSE(m.should_feed());
  m.check_in(CheckInTask::kMotorLoop);
  EXPECT_TRUE(m.should_feed());
}

TEST(CheckInMonitor, CheckInsAccumulateAcrossRefusedFeeds) {
  CheckInMonitor m(kMotorAndMain);
  m.check_in(CheckInTask::kMotorLoop);
  ASSERT_FALSE(m.should_feed());
  m.check_in(CheckInTask::kMainLoop);
  EXPECT_TRUE(m.should_feed()) << "a refused feed must not throw away the motor loop's bit";
}

TEST(CheckInMonitor, AnUnrequiredTaskIsIgnored) {
  // The pitch loop's bit is reserved: it may check in, but its absence never blocks a feed.
  CheckInMonitor m(kMotorAndMain);
  m.check_in(CheckInTask::kPitchLoop);
  EXPECT_FALSE(m.should_feed());
  m.check_in(CheckInTask::kMotorLoop);
  m.check_in(CheckInTask::kMainLoop);
  EXPECT_TRUE(m.should_feed());
}

TEST(CheckInMonitor, RequiringThePitchLoopBlocksUntilItChecksIn) {
  CheckInMonitor m(kMotorAndMain | static_cast<uint32_t>(CheckInTask::kPitchLoop));
  m.check_in(CheckInTask::kMotorLoop);
  m.check_in(CheckInTask::kMainLoop);
  EXPECT_FALSE(m.should_feed());
  m.check_in(CheckInTask::kPitchLoop);
  EXPECT_TRUE(m.should_feed());
}

TEST(CheckInMonitor, TimeoutIsTheAcceptedBuildConstant) {
  // ADR 0014 Q5. Changing this is an owner decision, not a tuning step.
  EXPECT_EQ(kIwdgTimeout_ms, 50U);
}

}  // namespace
}  // namespace recon::core
