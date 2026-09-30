#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "messages.hpp"

namespace {

using namespace recon::protocol;  // NOLINT: generated names are the subject of this file

TEST(GeneratedMessages, SizeConstantsMatchTheApprovedDesign) {
  // These numbers appear in protocol/design-proposal.md and in the bandwidth analysis in
  // docs/bringup/uart-link.md. A schema change that moves them makes those documents wrong.
  static_assert(kProtocolVersion == 1);
  static_assert(kHeaderBytes == 7);
  static_assert(kCrcBytes == 4);
  static_assert(kMaxPayloadBytes == 64);
  static_assert(kMaxLogicalFrameBytes == 75);
  static_assert(kMaxWireFrameBytes == 77);
  static_assert(DriveCommand::kPayloadBytes == 8);
  static_assert(StateTelemetry::kPayloadBytes == 31);
  static_assert(PowerTelemetry::kPayloadBytes == 12);
  static_assert(LinkStats::kPayloadBytes == 28);
  static_assert(Heartbeat::kPayloadBytes == 0);
  SUCCEED();
}

TEST(GeneratedMessages, LittleEndianByteOrderIsExplicit) {
  // ADR 0002 picked little-endian over network byte order. Pin the actual bytes, because
  // a round-trip test alone would pass for either convention.
  ParamGet msg{};
  msg.param_id = 0x1234;
  uint8_t buf[ParamGet::kPayloadBytes] = {};
  ASSERT_EQ(msg.encode(buf), 2u);
  EXPECT_EQ(buf[0], 0x34u) << "least significant byte first";
  EXPECT_EQ(buf[1], 0x12u);
}

TEST(GeneratedMessages, FloatsUseIeee754LittleEndian) {
  DriveCommand msg{};
  msg.cmd_linear_speed_m_s = 1.5F;   // 0x3FC00000
  msg.cmd_angular_rate_rad_s = 0.0F;
  uint8_t buf[DriveCommand::kPayloadBytes] = {};
  ASSERT_EQ(msg.encode(buf), 8u);
  EXPECT_EQ(buf[0], 0x00u);
  EXPECT_EQ(buf[1], 0x00u);
  EXPECT_EQ(buf[2], 0xC0u);
  EXPECT_EQ(buf[3], 0x3Fu);
}

TEST(GeneratedMessages, RoundTripsAStructWithEveryFieldKind) {
  StateTelemetry sent{};
  sent.wheel_speed_left_m_s = 0.25F;
  sent.wheel_speed_right_m_s = -0.5F;
  sent.body_pitch_rad = 0.125F;
  sent.camera_pitch_rad = -0.0625F;
  sent.safety_state = SafetyState::ARMED;
  sent.fault_flags = 0xBEEF;

  uint8_t buf[StateTelemetry::kPayloadBytes] = {};
  ASSERT_EQ(sent.encode(buf), StateTelemetry::kPayloadBytes);

  StateTelemetry got{};
  ASSERT_TRUE(StateTelemetry::decode(buf, sizeof(buf), got));
  EXPECT_FLOAT_EQ(got.wheel_speed_left_m_s, 0.25F);
  EXPECT_FLOAT_EQ(got.wheel_speed_right_m_s, -0.5F);
  EXPECT_FLOAT_EQ(got.body_pitch_rad, 0.125F);
  EXPECT_FLOAT_EQ(got.camera_pitch_rad, -0.0625F);
  EXPECT_EQ(got.safety_state, SafetyState::ARMED);
  EXPECT_EQ(got.fault_flags, 0xBEEFu);
}

TEST(GeneratedMessages, RoundTripsAFixedArrayField) {
  ParamSet sent{};
  sent.param_id = 0x0001;
  sent.type_tag = 2;
  sent.value[0] = 0xDE;
  sent.value[1] = 0xAD;
  sent.value[2] = 0xBE;
  sent.value[3] = 0xEF;

  uint8_t buf[ParamSet::kPayloadBytes] = {};
  ASSERT_EQ(sent.encode(buf), ParamSet::kPayloadBytes);
  ParamSet got{};
  ASSERT_TRUE(ParamSet::decode(buf, sizeof(buf), got));
  EXPECT_EQ(got.param_id, 0x0001u);
  EXPECT_EQ(got.value[0], 0xDEu);
  EXPECT_EQ(got.value[3], 0xEFu);
}

TEST(GeneratedMessages, DecodeRejectsWrongLength) {
  uint8_t buf[16] = {};
  DriveCommand out{};
  EXPECT_FALSE(DriveCommand::decode(buf, 7, out));
  EXPECT_FALSE(DriveCommand::decode(buf, 9, out));
  EXPECT_TRUE(DriveCommand::decode(buf, 8, out));
}

TEST(GeneratedMessages, DecodeRejectsOutOfRangeField) {
  DriveCommand msg{};
  msg.cmd_linear_speed_m_s = 99.0F;   // range is [-2, 2]
  uint8_t buf[DriveCommand::kPayloadBytes] = {};
  msg.encode(buf);
  DriveCommand out{};
  EXPECT_FALSE(DriveCommand::decode(buf, sizeof(buf), out));
}

// NaN must not reach a control loop. It fails the range check because NaN compares false
// against both bounds -- deliberate, and easy to break by "fixing" the comparison.
TEST(GeneratedMessages, DecodeRejectsNan) {
  DriveCommand msg{};
  msg.cmd_linear_speed_m_s = std::nanf("");
  uint8_t buf[DriveCommand::kPayloadBytes] = {};
  msg.encode(buf);
  DriveCommand out{};
  EXPECT_FALSE(DriveCommand::decode(buf, sizeof(buf), out));
  EXPECT_FALSE(msg.in_range());
}

TEST(GeneratedMessages, DecodeRejectsInfinity) {
  DriveCommand msg{};
  msg.cmd_linear_speed_m_s = std::numeric_limits<float>::infinity();
  uint8_t buf[DriveCommand::kPayloadBytes] = {};
  msg.encode(buf);
  DriveCommand out{};
  EXPECT_FALSE(DriveCommand::decode(buf, sizeof(buf), out));
}

TEST(GeneratedMessages, DecodeRejectsBadMagic) {
  EstopRequest msg{};
  msg.magic = 0x00000000;
  uint8_t buf[EstopRequest::kPayloadBytes] = {};
  msg.encode(buf);
  EstopRequest out{};
  EXPECT_FALSE(EstopRequest::decode(buf, sizeof(buf), out));

  msg.magic = 0x45535450;  // "ESTP"
  msg.encode(buf);
  EXPECT_TRUE(EstopRequest::decode(buf, sizeof(buf), out));
}

TEST(GeneratedMessages, MagicConstantsAreDistinct) {
  SafetyStateRequest arm{};
  EstopRequest estop{};
  ParamCommit commit{};
  arm.magic = 0x41524D21;
  estop.magic = 0x45535450;
  commit.magic = 0x434F4D54;
  EXPECT_TRUE(arm.magic_ok());
  EXPECT_TRUE(estop.magic_ok());
  EXPECT_TRUE(commit.magic_ok());
  // Swapping them must fail: a corrupt arm request must not act as an e-stop.
  arm.magic = 0x45535450;
  EXPECT_FALSE(arm.magic_ok());
}

TEST(GeneratedMessages, DecodeRejectsUndeclaredEnumValue) {
  uint8_t buf[StateTelemetry::kPayloadBytes] = {};
  buf[28] = 9;  // safety_state offset; 9 is not a declared SafetyState
  StateTelemetry out{};
  EXPECT_FALSE(StateTelemetry::decode(buf, sizeof(buf), out));

  buf[28] = static_cast<uint8_t>(SafetyState::ESTOP);
  EXPECT_TRUE(StateTelemetry::decode(buf, sizeof(buf), out));
  EXPECT_EQ(out.safety_state, SafetyState::ESTOP);
}

TEST(GeneratedMessages, EnumValidatorsAcceptOnlyDeclaredValues) {
  EXPECT_TRUE(is_valid_safety_state(0));
  EXPECT_TRUE(is_valid_safety_state(3));
  EXPECT_FALSE(is_valid_safety_state(4));
  EXPECT_FALSE(is_valid_safety_state(255));
}

TEST(GeneratedMessages, PayloadTableCoversEveryMessageAndNothingElse) {
  size_t n = 0;
  EXPECT_TRUE(payload_bytes_for(static_cast<uint8_t>(MessageId::Heartbeat), n));
  EXPECT_EQ(n, 0u);
  EXPECT_TRUE(payload_bytes_for(static_cast<uint8_t>(MessageId::LinkStats), n));
  EXPECT_EQ(n, 28u);
  EXPECT_FALSE(payload_bytes_for(0x00, n));
  EXPECT_FALSE(payload_bytes_for(0x03, n)) << "reserved for camera pitch";
  EXPECT_FALSE(payload_bytes_for(0xFF, n));
}

TEST(GeneratedMessages, DirectionRangesAreRespected) {
  for (uint16_t id = 0; id <= 0xFF; ++id) {
    size_t n = 0;
    if (!payload_bytes_for(static_cast<uint8_t>(id), n)) {
      continue;
    }
    const bool operator_to_robot = id >= 0x01 && id <= 0x1F;
    const bool robot_to_operator = id >= 0x20 && id <= 0x3F;
    EXPECT_TRUE(operator_to_robot || robot_to_operator)
        << "id 0x" << std::hex << id << " is outside both direction ranges";
  }
}

}  // namespace
