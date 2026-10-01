#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "hal/serial_port.hpp"

namespace recon::sim {

/// Fixed-capacity FIFO of bytes. No allocation, like the firmware's own buffers.
template <size_t N>
class ByteRing {
 public:
  static constexpr size_t kCapacity = N;

  /// \return False (and the byte is dropped) when full.
  bool push(uint8_t byte) {
    if (size_ == N) {
      return false;
    }
    buf_[(head_ + size_) % N] = byte;
    ++size_;
    return true;
  }

  /// \pre `!empty()`.
  uint8_t pop() {
    const uint8_t byte = buf_[head_];
    head_ = (head_ + 1) % N;
    --size_;
    return byte;
  }

  bool empty() const { return size_ == 0; }
  size_t size() const { return size_; }
  size_t space() const { return N - size_; }

 private:
  std::array<uint8_t, N> buf_{};
  size_t head_ = 0;
  size_t size_ = 0;
};

/// Transmit buffer size per endpoint. Placeholder -- the STM32 UART TX buffer is not sized
/// yet. Chosen to hold three maximum-length wire frames (3 x kMaxWireFrameBytes = 231 B),
/// so one control period's telemetry never has to be split across a full buffer.
inline constexpr size_t kSimTxBufferBytes = 256;

/// Receive buffer size per endpoint. Placeholder, same reasoning as kSimTxBufferBytes.
/// Small on purpose: an operator that stops reading must see overflow, as on the real UART.
inline constexpr size_t kSimRxBufferBytes = 256;

/// Line rate selected for the MCU <-> Pi link (docs/bringup/uart-link.md, owner 2026-09-30).
inline constexpr uint32_t kDefaultBaud = 460800;

/// UART 8N1: start bit + 8 data bits + stop bit per byte on the wire.
inline constexpr uint32_t kBitsPerUartByte = 10;

/// One end of a SimSerialLink. Behaves as the `hal::SerialPort` contract describes.
class SimSerialEndpoint final : public hal::SerialPort {
 public:
  size_t write(const uint8_t* data, size_t len) override;
  size_t read(uint8_t* out, size_t len) override;
  uint32_t rx_overflow_count() const override { return rx_overflow_; }

  /// Bytes written but not yet on the wire.
  size_t tx_pending() const { return tx_.size(); }
  /// Bytes received but not yet read.
  size_t rx_pending() const { return rx_.size(); }

 private:
  friend class SimSerialLink;

  ByteRing<kSimTxBufferBytes> tx_;
  ByteRing<kSimRxBufferBytes> rx_;
  uint32_t rx_overflow_ = 0;  // wraps at 2^32, per the interface
};

/// A full-duplex UART between two endpoints: the MCU and the Pi (or a test standing in for
/// the operator).
///
/// Bytes leave a TX buffer only as fast as the line rate allows, so bandwidth limits and
/// buffer overflow show up in simulation the way they would on the wire. Bit errors can be
/// injected to exercise the decoder's validation path end to end.
///
/// Deterministic: errors come from a seeded PRNG, never from the wall clock.
class SimSerialLink {
 public:
  struct Config {
    /// Line rate, bits per second.
    uint32_t baud = kDefaultBaud;
    /// Probability that a byte on the wire has one bit flipped, in [0, 1]. Applied
    /// independently per byte and per direction. 0 is a perfect line.
    float byte_error_rate = 0.0F;
    /// PRNG seed. Must be non-zero (xorshift has an all-zero fixed point); 0 is replaced
    /// by a fixed non-zero value.
    uint64_t seed = 1;
  };

  SimSerialLink() : SimSerialLink(Config{}) {}
  explicit SimSerialLink(const Config& config);

  /// The MCU's side of the link. Pass this to `core` as its `hal::SerialPort`.
  SimSerialEndpoint& mcu() { return mcu_; }
  /// The far side: the Pi relay, or the test acting as the operator.
  SimSerialEndpoint& host() { return host_; }

  /// Moves bytes across the line for `dt_us` of virtual time, in both directions.
  ///
  /// Fractional bytes carry over between calls, so stepping in many small increments
  /// transfers the same number of bytes as one large step.
  void step(uint32_t dt_us);

  /// Bytes that have had a bit flipped, both directions, since construction.
  uint32_t corrupted_bytes() const { return corrupted_bytes_; }

  /// Cuts (true) or restores (false) the line, both directions. While cut, bytes still leave
  /// the TX buffers at the line rate but never arrive: a Wi-Fi drop or an unplugged cable as
  /// the MCU sees it. Used by the ADR 0014 link-loss SIL test.
  void set_cut(bool cut) { cut_ = cut; }
  bool cut() const { return cut_; }

 private:
  /// Per-direction line state.
  struct Direction {
    /// Line time owed but not yet spent on a whole byte, in bit-microseconds (bits * us).
    uint64_t credit_bit_us = 0;
  };

  void transfer(SimSerialEndpoint& from, SimSerialEndpoint& to, Direction& dir, uint32_t dt_us);
  /// xorshift64: small, fast, and identical on every platform, which std:: distributions
  /// are not guaranteed to be.
  uint64_t next_random();
  /// Uniform in [0, 1).
  float next_unit();

  Config config_;
  SimSerialEndpoint mcu_;
  SimSerialEndpoint host_;
  Direction mcu_to_host_;
  Direction host_to_mcu_;
  uint64_t rng_state_;
  uint32_t corrupted_bytes_ = 0;
  bool cut_ = false;
};

}  // namespace recon::sim
