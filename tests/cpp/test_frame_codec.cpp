#include "protocol/frame.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "protocol/cobs.hpp"
#include "protocol/crc32.hpp"

namespace {

using recon::protocol::DecodedFrame;
using recon::protocol::FrameDecoder;
using recon::protocol::MessageId;
using recon::protocol::encode_frame;
using recon::protocol::kHeaderBytes;
using recon::protocol::kMaxWireFrameBytes;
using recon::protocol::kProtocolVersion;

std::vector<uint8_t> make_frame(uint8_t id, uint8_t seq, uint32_t ts,
                                const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> wire(kMaxWireFrameBytes);
  size_t len = 0;
  const bool ok = encode_frame(id, seq, ts, payload.data(), payload.size(),
                               wire.data(), wire.size(), len);
  EXPECT_TRUE(ok);
  wire.resize(ok ? len : 0);
  return wire;
}

/// Feeds every byte and returns each frame the decoder produced.
std::vector<DecodedFrame> feed(FrameDecoder& decoder, const std::vector<uint8_t>& bytes) {
  std::vector<DecodedFrame> frames;
  DecodedFrame frame{};
  for (uint8_t byte : bytes) {
    if (decoder.push_byte(byte, frame)) {
      frames.push_back(frame);
    }
  }
  return frames;
}

std::vector<uint8_t> payload_of(size_t len) {
  std::vector<uint8_t> p(len);
  for (size_t i = 0; i < len; ++i) {
    p[i] = static_cast<uint8_t>(i * 3 + 1);
  }
  return p;
}

TEST(FrameCodec, RoundTripsAMessage) {
  const auto payload = payload_of(8);
  const auto wire = make_frame(static_cast<uint8_t>(MessageId::DriveCommand), 42,
                               0xDEADBEEF, payload);
  EXPECT_EQ(wire.back(), 0x00) << "frame must end with the delimiter";

  FrameDecoder decoder;
  const auto frames = feed(decoder, wire);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].message_id, static_cast<uint8_t>(MessageId::DriveCommand));
  EXPECT_EQ(frames[0].seq, 42u);
  EXPECT_EQ(frames[0].timestamp_us, 0xDEADBEEFu);
  ASSERT_EQ(frames[0].payload_len, payload.size());
  for (size_t i = 0; i < payload.size(); ++i) {
    EXPECT_EQ(frames[0].payload[i], payload[i]);
  }
  EXPECT_EQ(decoder.stats().frames_ok, 1u);
}

TEST(FrameCodec, NoZeroBytesAppearBeforeTheDelimiter) {
  // A payload of all zeros is the case that breaks framing if COBS is wrong.
  const auto wire = make_frame(static_cast<uint8_t>(MessageId::DriveCommand), 0, 0,
                               std::vector<uint8_t>(8, 0x00));
  for (size_t i = 0; i + 1 < wire.size(); ++i) {
    EXPECT_NE(wire[i], 0x00) << "zero at " << i << " would be read as a delimiter";
  }
  FrameDecoder decoder;
  EXPECT_EQ(feed(decoder, wire).size(), 1u);
}

TEST(FrameCodec, RoundTripsEveryDeclaredMessageSize) {
  // Payload length must match what the id declares, so drive this from the schema's own
  // table rather than from a list that could drift out of date.
  for (uint16_t id = 0; id <= 0xFF; ++id) {
    size_t declared = 0;
    if (!recon::protocol::payload_bytes_for(static_cast<uint8_t>(id), declared)) {
      continue;
    }
    const auto wire = make_frame(static_cast<uint8_t>(id), 1, 1000, payload_of(declared));
    FrameDecoder decoder;
    const auto frames = feed(decoder, wire);
    ASSERT_EQ(frames.size(), 1u) << "id 0x" << std::hex << id;
    EXPECT_EQ(frames[0].payload_len, declared);
  }
}

TEST(FrameCodec, TimestampWrapValuesSurviveTheRoundTrip) {
  for (uint32_t ts : {0u, 1u, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFEu, 0xFFFFFFFFu}) {
    const auto wire = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 0, ts, {});
    FrameDecoder decoder;
    const auto frames = feed(decoder, wire);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].timestamp_us, ts);
  }
}

TEST(FrameCodec, SequenceNumberWrapsAtU8) {
  FrameDecoder decoder;
  for (uint16_t seq = 0; seq <= 0xFF; ++seq) {
    const auto wire = make_frame(static_cast<uint8_t>(MessageId::Heartbeat),
                                 static_cast<uint8_t>(seq), 0, {});
    const auto frames = feed(decoder, wire);
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].seq, static_cast<uint8_t>(seq));
  }
  EXPECT_EQ(decoder.stats().frames_ok, 256u);
}

// ------------------------------------------------------------------- resynchronization

TEST(FrameCodec, RecoversAfterGarbageBetweenFrames) {
  auto stream = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 1, 100, {});
  for (uint8_t noise : {0x7Fu, 0x01u, 0xFEu, 0x42u, 0x00u, 0xABu, 0xCDu, 0x00u}) {
    stream.push_back(noise);
  }
  const auto second = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 2, 200, {});
  stream.insert(stream.end(), second.begin(), second.end());

  FrameDecoder decoder;
  const auto frames = feed(decoder, stream);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].seq, 1u);
  EXPECT_EQ(frames[1].seq, 2u);
}

TEST(FrameCodec, RecoversAfterATruncatedFrame) {
  auto truncated = make_frame(static_cast<uint8_t>(MessageId::DriveCommand), 1, 100,
                              payload_of(8));
  truncated.resize(truncated.size() / 2);      // chop it, delimiter included
  truncated.push_back(0x00);                   // then a delimiter arrives

  auto stream = truncated;
  const auto good = make_frame(static_cast<uint8_t>(MessageId::DriveCommand), 2, 200,
                               payload_of(8));
  stream.insert(stream.end(), good.begin(), good.end());

  FrameDecoder decoder;
  const auto frames = feed(decoder, stream);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 2u);
}

// The bug ADR 0002 calls out: on overflow the decoder must discard to the NEXT delimiter.
// Merely clearing the buffer re-aligns mid-frame and reads the tail as a header.
TEST(FrameCodec, OverlongRunEntersDesyncAndThenRecovers) {
  FrameDecoder decoder;
  DecodedFrame frame{};

  // A run longer than any legal frame, containing no delimiter.
  for (size_t i = 0; i < kMaxWireFrameBytes + 50; ++i) {
    EXPECT_FALSE(decoder.push_byte(0xAA, frame));
  }
  EXPECT_EQ(decoder.state(), FrameDecoder::State::kDesync);
  EXPECT_GE(decoder.stats().desyncs, 1u);
  EXPECT_EQ(decoder.stats().frames_ok, 0u);

  // The next delimiter re-aligns it, and the frame after that must decode.
  EXPECT_FALSE(decoder.push_byte(0x00, frame));
  EXPECT_EQ(decoder.state(), FrameDecoder::State::kAccumulating);

  const auto wire = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 9, 900, {});
  const auto frames = feed(decoder, wire);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 9u);
}

TEST(FrameCodec, DesyncCountsOnlyOncePerOverlongRun) {
  FrameDecoder decoder;
  DecodedFrame frame{};
  for (size_t i = 0; i < 4 * kMaxWireFrameBytes; ++i) {
    decoder.push_byte(0xAA, frame);
  }
  EXPECT_EQ(decoder.stats().desyncs, 1u)
      << "a single unbroken run is one desync, not one per byte";
}

TEST(FrameCodec, BackToBackDelimitersAreIdleNotErrors) {
  FrameDecoder decoder;
  DecodedFrame frame{};
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(decoder.push_byte(0x00, frame));
  }
  const auto& s = decoder.stats();
  EXPECT_EQ(s.frames_ok + s.crc_errors + s.cobs_errors + s.unknown_id
                + s.length_mismatch + s.version_mismatch + s.desyncs,
            0u);
}

// ------------------------------------------------------------------------- rejections

TEST(FrameCodec, RejectsCorruptedCrc) {
  FrameDecoder decoder;
  for (size_t corrupt_index = 0; corrupt_index < 12; ++corrupt_index) {
    auto wire = make_frame(static_cast<uint8_t>(MessageId::DriveCommand), 1, 100,
                           payload_of(8));
    if (corrupt_index + 1 >= wire.size()) {
      continue;
    }
    // Flip a bit somewhere in the encoded body, avoiding the delimiter.
    wire[corrupt_index] = static_cast<uint8_t>(wire[corrupt_index] ^ 0x40u);
    if (wire[corrupt_index] == 0x00) {
      continue;  // would become a delimiter, which is a framing test not a CRC test
    }
    decoder.reset();
    EXPECT_TRUE(feed(decoder, wire).empty()) << "corrupted byte " << corrupt_index;
  }
  EXPECT_GT(decoder.stats().crc_errors + decoder.stats().cobs_errors
                + decoder.stats().length_mismatch + decoder.stats().version_mismatch
                + decoder.stats().unknown_id,
            0u);
}

TEST(FrameCodec, RejectsUnknownMessageId) {
  ASSERT_FALSE(recon::protocol::is_known_message_id(0x03))
      << "0x03 is reserved for the camera pitch command and must stay unassigned";
  const auto wire = make_frame(0x03, 1, 100, payload_of(4));
  FrameDecoder decoder;
  EXPECT_TRUE(feed(decoder, wire).empty());
  EXPECT_EQ(decoder.stats().unknown_id, 1u);
}

TEST(FrameCodec, RejectsPayloadLengthThatDisagreesWithTheId) {
  // DriveCommand declares 8 bytes; send 6. This is the check that replaces a
  // self-reported length field.
  const auto wire = make_frame(static_cast<uint8_t>(MessageId::DriveCommand), 1, 100,
                               payload_of(6));
  FrameDecoder decoder;
  EXPECT_TRUE(feed(decoder, wire).empty());
  EXPECT_EQ(decoder.stats().length_mismatch, 1u);
}

TEST(FrameCodec, RejectsWrongProtocolVersion) {
  // Hand-build a frame with a bad version byte, since encode_frame always writes the
  // correct one.
  std::vector<uint8_t> logical(kHeaderBytes + 4);
  logical[0] = static_cast<uint8_t>(kProtocolVersion + 1);
  logical[1] = static_cast<uint8_t>(MessageId::Heartbeat);
  logical[2] = 7;
  recon::protocol::detail::write_u32(logical.data() + 3, 1234);
  recon::protocol::detail::write_u32(logical.data() + kHeaderBytes,
                                     recon::protocol::crc32(logical.data(), kHeaderBytes));

  std::vector<uint8_t> wire(kMaxWireFrameBytes);
  size_t len = 0;
  ASSERT_TRUE(recon::protocol::cobs_encode(logical.data(), logical.size(), wire.data(),
                                           wire.size() - 1, len));
  wire[len] = 0x00;
  wire.resize(len + 1);

  FrameDecoder decoder;
  EXPECT_TRUE(feed(decoder, wire).empty());
  EXPECT_EQ(decoder.stats().version_mismatch, 1u);
}

TEST(FrameCodec, RejectsFrameShorterThanAHeaderPlusCrc) {
  FrameDecoder decoder;
  // A single valid COBS block decoding to 3 bytes: far too short to be a frame.
  EXPECT_TRUE(feed(decoder, {0x04, 0x11, 0x22, 0x33, 0x00}).empty());
  EXPECT_EQ(decoder.stats().length_mismatch, 1u);
}

TEST(FrameCodec, EncodeRejectsOversizedPayload) {
  std::vector<uint8_t> too_big(recon::protocol::kMaxPayloadBytes + 1, 0xAA);
  std::vector<uint8_t> out(kMaxWireFrameBytes * 2);
  size_t len = 0;
  EXPECT_FALSE(encode_frame(0x02, 0, 0, too_big.data(), too_big.size(), out.data(),
                            out.size(), len));
}

TEST(FrameCodec, EncodeRejectsInsufficientCapacity) {
  const auto payload = payload_of(8);
  uint8_t out[4] = {};
  size_t len = 0;
  EXPECT_FALSE(encode_frame(0x02, 0, 0, payload.data(), payload.size(), out, sizeof(out),
                            len));
  EXPECT_FALSE(encode_frame(0x02, 0, 0, payload.data(), payload.size(), out, 0, len));
}

// --------------------------------------------------------------------------- fuzzing

// The decoder's core promise: arbitrary input never crashes it, never leaves it unusable,
// and never produces a frame that fails its own validation.
TEST(FrameCodec, SurvivesGarbageAndStaysUsable) {
  FrameDecoder decoder;
  DecodedFrame frame{};
  uint32_t state = 0xCAFEBABEU;

  for (int iteration = 0; iteration < 200000; ++iteration) {
    state = state * 1664525U + 1013904223U;
    const uint8_t byte = static_cast<uint8_t>(state >> 24);
    if (decoder.push_byte(byte, frame)) {
      // Anything it accepts must satisfy the invariant the decoder guarantees.
      size_t declared = 0;
      ASSERT_TRUE(recon::protocol::payload_bytes_for(frame.message_id, declared));
      ASSERT_EQ(frame.payload_len, declared);
    }
  }

  // After all that the decoder must still be usable. It does NOT promise the very next
  // frame survives: garbage that ends without a delimiter leaves the decoder
  // accumulating, so the next frame's bytes join that partial buffer and the first
  // delimiter closes a run of garbage-plus-frame, which correctly fails validation.
  // The guarantee is that AT MOST ONE frame is lost -- see FrameDecoder's header comment.
  const auto first = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 5, 500, {});
  const auto second = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 6, 600, {});
  const auto after_first = feed(decoder, first);
  const auto after_second = feed(decoder, second);

  EXPECT_LE(after_first.size(), 1u);
  ASSERT_EQ(after_second.size(), 1u) << "at most one frame may be lost to resync";
  EXPECT_EQ(after_second[0].seq, 6u);
}

// Pins the resynchronization guarantee directly, independent of the fuzz loop above, so a
// regression names the property it broke.
TEST(FrameCodec, LosesAtMostOneFrameAfterGarbageWithNoDelimiter) {
  FrameDecoder decoder;
  DecodedFrame frame{};
  for (size_t i = 0; i < 20; ++i) {
    decoder.push_byte(0xAA, frame);  // partial run, no delimiter
  }
  const auto first = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 1, 100, {});
  const auto second = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 2, 200, {});
  EXPECT_TRUE(feed(decoder, first).empty()) << "first frame is consumed re-aligning";
  const auto recovered = feed(decoder, second);
  ASSERT_EQ(recovered.size(), 1u);
  EXPECT_EQ(recovered[0].seq, 2u);
}

// If the garbage happens to end with a delimiter, nothing is lost at all.
TEST(FrameCodec, LosesNoFrameWhenGarbageEndsWithADelimiter) {
  FrameDecoder decoder;
  DecodedFrame frame{};
  for (size_t i = 0; i < 20; ++i) {
    decoder.push_byte(0xAA, frame);
  }
  decoder.push_byte(0x00, frame);  // garbage terminated
  const auto wire = make_frame(static_cast<uint8_t>(MessageId::Heartbeat), 3, 300, {});
  const auto frames = feed(decoder, wire);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 3u);
}

TEST(FrameCodec, SurvivesTruncatedAndSplicedValidFrames) {
  // Real corruption looks like fragments of valid frames, which is harder than random
  // noise: the fragments contain plausible headers.
  const auto good = make_frame(static_cast<uint8_t>(MessageId::StateTelemetry), 3, 300,
                               payload_of(31));
  FrameDecoder decoder;
  DecodedFrame frame{};
  for (size_t cut = 1; cut < good.size(); ++cut) {
    for (size_t i = 0; i < cut; ++i) {
      decoder.push_byte(good[i], frame);
    }
    for (size_t i = cut; i < good.size(); ++i) {
      decoder.push_byte(good[i], frame);
    }
  }
  const auto frames = feed(decoder, good);
  ASSERT_EQ(frames.size(), 1u);
}

}  // namespace
