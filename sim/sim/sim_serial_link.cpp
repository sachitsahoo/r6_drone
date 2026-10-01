#include "sim/sim_serial_link.hpp"

namespace recon::sim {

namespace {

/// Microseconds per second, for converting baud (bits/s) to bits per microsecond.
constexpr uint64_t kUsPerS = 1000000;

/// Replacement seed for 0, which is xorshift64's fixed point. Any non-zero value works;
/// this one is the constant from Marsaglia's paper.
constexpr uint64_t kFallbackSeed = 88172645463325252ULL;

}  // namespace

size_t SimSerialEndpoint::write(const uint8_t* data, size_t len) {
  size_t accepted = 0;
  while (accepted < len && tx_.push(data[accepted])) {
    ++accepted;
  }
  return accepted;
}

size_t SimSerialEndpoint::read(uint8_t* out, size_t len) {
  size_t copied = 0;
  while (copied < len && !rx_.empty()) {
    out[copied++] = rx_.pop();
  }
  return copied;
}

SimSerialLink::SimSerialLink(const Config& config)
    : config_(config), rng_state_(config.seed != 0 ? config.seed : kFallbackSeed) {}

void SimSerialLink::step(uint32_t dt_us) {
  transfer(mcu_, host_, mcu_to_host_, dt_us);
  transfer(host_, mcu_, host_to_mcu_, dt_us);
}

void SimSerialLink::transfer(SimSerialEndpoint& from, SimSerialEndpoint& to, Direction& dir,
                             uint32_t dt_us) {
  // Integer bookkeeping: bits sent in dt_us = baud * dt_us / 1e6. Keeping the product in
  // bit-microseconds avoids float drift over long simulations.
  const uint64_t bit_us_per_byte = static_cast<uint64_t>(kBitsPerUartByte) * kUsPerS;
  dir.credit_bit_us += static_cast<uint64_t>(config_.baud) * dt_us;

  while (dir.credit_bit_us >= bit_us_per_byte && !from.tx_.empty()) {
    dir.credit_bit_us -= bit_us_per_byte;
    uint8_t byte = from.tx_.pop();
    if (cut_) {
      continue;  // the transmitter still clocks the byte out; nothing is at the other end
    }
    if (config_.byte_error_rate > 0.0F && next_unit() < config_.byte_error_rate) {
      byte ^= static_cast<uint8_t>(1U << (next_random() % 8U));
      ++corrupted_bytes_;
    }
    if (!to.rx_.push(byte)) {
      ++to.rx_overflow_;  // the receiver was not read in time; the byte is gone, as on a UART
    }
  }
  // An idle line does not bank credit: a UART cannot send later bytes faster to make up for
  // time it had nothing to send. Only the partial byte in progress carries over.
  if (from.tx_.empty()) {
    dir.credit_bit_us %= bit_us_per_byte;
  }
}

uint64_t SimSerialLink::next_random() {
  // Marsaglia, "Xorshift RNGs", J. Stat. Software 8(14), 2003: the 13/7/17 triple.
  rng_state_ ^= rng_state_ << 13;
  rng_state_ ^= rng_state_ >> 7;
  rng_state_ ^= rng_state_ << 17;
  return rng_state_;
}

float SimSerialLink::next_unit() {
  // Top 24 bits -> exactly representable in a float's mantissa, so the result is uniform
  // on a grid of 2^-24 and strictly below 1.
  constexpr float kTwoPow24 = 16777216.0F;
  return static_cast<float>(next_random() >> 40) / kTwoPow24;
}

}  // namespace recon::sim
