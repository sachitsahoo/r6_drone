// Command-line access to the real C++ codec, for cross-language verification.
//
// tests/python/test_cross_language.py drives this binary and compares its output against
// protocol/codec.py and against the committed vectors in protocol/test_vectors.json.
// Going through the actual firmware code matters: two implementations written from the
// same document can agree with the document and still disagree with each other.
//
// Host-only test tooling, so it may use iostream and std::string freely. The firmware
// language subset applies to firmware/core/, which this only links against.
//
// Usage:
//   protocol_vector_tool crc <hex>
//   protocol_vector_tool cobs-encode <hex>
//   protocol_vector_tool cobs-decode <hex>
//   protocol_vector_tool encode <id> <seq> <timestamp_us> <payload-hex>
//   protocol_vector_tool decode <wire-hex>
//
// Prints one line: a hex string, "OK <id> <seq> <ts> <payload-hex>", or "ERROR <what>".

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "protocol/cobs.hpp"
#include "protocol/crc32.hpp"
#include "protocol/frame.hpp"

namespace {

bool from_hex(const std::string& text, std::vector<uint8_t>& out) {
  if (text.size() % 2 != 0) {
    return false;
  }
  out.clear();
  for (size_t i = 0; i < text.size(); i += 2) {
    const std::string byte_text = text.substr(i, 2);
    char* end = nullptr;
    const long value = std::strtol(byte_text.c_str(), &end, 16);
    if (end == nullptr || *end != '\0') {
      return false;
    }
    out.push_back(static_cast<uint8_t>(value));
  }
  return true;
}

std::string to_hex(const uint8_t* data, size_t len) {
  static const char* kDigits = "0123456789abcdef";
  std::string out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out.push_back(kDigits[data[i] >> 4]);
    out.push_back(kDigits[data[i] & 0x0F]);
  }
  return out;
}

int fail(const std::string& what) {
  std::cout << "ERROR " << what << "\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace recon::protocol;

  if (argc < 3) {
    return fail("usage: <command> <args...>");
  }
  const std::string command = argv[1];
  std::vector<uint8_t> input;

  if (command == "crc") {
    if (!from_hex(argv[2], input)) {
      return fail("bad hex");
    }
    const uint32_t value = crc32(input.data(), input.size());
    // Fixed 8 hex digits, so the output is directly comparable to Python's
    // f"{crc:08x}" and to protocol/test_vectors.json.
    uint8_t big_endian[4] = {
        static_cast<uint8_t>((value >> 24) & 0xFFU),
        static_cast<uint8_t>((value >> 16) & 0xFFU),
        static_cast<uint8_t>((value >> 8) & 0xFFU),
        static_cast<uint8_t>(value & 0xFFU),
    };
    std::cout << to_hex(big_endian, sizeof(big_endian)) << "\n";
    return 0;
  }

  if (command == "cobs-encode") {
    if (!from_hex(argv[2], input)) {
      return fail("bad hex");
    }
    std::vector<uint8_t> out(cobs_max_encoded_bytes(input.size()) + 8);
    size_t len = 0;
    if (!cobs_encode(input.data(), input.size(), out.data(), out.size(), len)) {
      return fail("cobs_encode failed");
    }
    std::cout << to_hex(out.data(), len) << "\n";
    return 0;
  }

  if (command == "cobs-decode") {
    if (!from_hex(argv[2], input)) {
      return fail("bad hex");
    }
    std::vector<uint8_t> out(input.size() + 512);
    size_t len = 0;
    if (!cobs_decode(input.data(), input.size(), out.data(), out.size(), len)) {
      return fail("cobs_decode rejected");
    }
    std::cout << to_hex(out.data(), len) << "\n";
    return 0;
  }

  if (command == "encode") {
    if (argc < 6) {
      return fail("usage: encode <id> <seq> <ts> <payload-hex>");
    }
    const auto id = static_cast<uint8_t>(std::strtoul(argv[2], nullptr, 0));
    const auto seq = static_cast<uint8_t>(std::strtoul(argv[3], nullptr, 0));
    const auto ts = static_cast<uint32_t>(std::strtoul(argv[4], nullptr, 0));
    const std::string payload_hex = argv[5];
    if (!payload_hex.empty() && !from_hex(payload_hex, input)) {
      return fail("bad payload hex");
    }
    std::vector<uint8_t> wire(kMaxWireFrameBytes);
    size_t len = 0;
    if (!encode_frame(id, seq, ts, input.data(), input.size(), wire.data(), wire.size(),
                      len)) {
      return fail("encode_frame failed");
    }
    std::cout << to_hex(wire.data(), len) << "\n";
    return 0;
  }

  if (command == "decode") {
    if (!from_hex(argv[2], input)) {
      return fail("bad hex");
    }
    FrameDecoder decoder;
    DecodedFrame frame{};
    bool got = false;
    for (uint8_t byte : input) {
      if (decoder.push_byte(byte, frame)) {
        got = true;
        break;
      }
    }
    if (!got) {
      const auto& s = decoder.stats();
      std::string reason = "none";
      if (s.cobs_errors > 0) {
        reason = "cobs_errors";
      } else if (s.version_mismatch > 0) {
        reason = "version_mismatch";
      } else if (s.crc_errors > 0) {
        reason = "crc_errors";
      } else if (s.unknown_id > 0) {
        reason = "unknown_id";
      } else if (s.length_mismatch > 0) {
        reason = "length_mismatch";
      } else if (s.desyncs > 0) {
        reason = "desyncs";
      } else {
        reason = "incomplete";
      }
      std::cout << "REJECT " << reason << "\n";
      return 0;
    }
    std::cout << "OK " << static_cast<unsigned>(frame.message_id) << " "
              << static_cast<unsigned>(frame.seq) << " " << frame.timestamp_us << " "
              << to_hex(frame.payload, frame.payload_len) << "\n";
    return 0;
  }

  return fail("unknown command " + command);
}
