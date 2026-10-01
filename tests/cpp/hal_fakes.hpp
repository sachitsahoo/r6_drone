#pragma once

// Test doubles for every firmware/hal interface. Host-only.
//
// Each fake does the least that lets a core unit test drive it: samples are settable, and
// commands are recorded so a test can assert on them. They deliberately do NOT model physics
// or clamp inputs -- clamping is an implementation contract (see each interface), and a fake
// that clamped would hide a core bug that sends out-of-range values. The simulator in sim/
// is where physics lives.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "hal/absolute_encoder.hpp"
#include "hal/clock.hpp"
#include "hal/imu.hpp"
#include "hal/pitch_power_stage.hpp"
#include "hal/power_monitor.hpp"
#include "hal/serial_port.hpp"
#include "hal/wheel_encoder.hpp"
#include "hal/wheel_motor.hpp"

namespace recon::test {

class FakeClock final : public hal::Clock {
 public:
  uint32_t now_us() const override { return now_us_; }
  void set_us(uint32_t t_us) { now_us_ = t_us; }
  /// Unsigned addition, so advancing past 2^32 wraps exactly as the real clock does.
  void advance_us(uint32_t dt_us) { now_us_ += dt_us; }

 private:
  uint32_t now_us_ = 0;
};

/// A serial port with fixed-size buffers, so tests can exercise "transmit buffer full".
class FakeSerialPort final : public hal::SerialPort {
 public:
  static constexpr size_t kCapacity = 256;

  size_t write(const uint8_t* data, size_t len) override {
    const size_t n = std::min(len, kCapacity - tx_len_);
    std::copy(data, data + n, tx_.begin() + static_cast<std::ptrdiff_t>(tx_len_));
    tx_len_ += n;
    return n;
  }

  size_t read(uint8_t* out, size_t len) override {
    const size_t n = std::min(len, rx_len_ - rx_pos_);
    std::copy(rx_.begin() + static_cast<std::ptrdiff_t>(rx_pos_),
              rx_.begin() + static_cast<std::ptrdiff_t>(rx_pos_ + n), out);
    rx_pos_ += n;
    return n;
  }

  uint32_t rx_overflow_count() const override { return rx_overflow_; }

  // --- test controls ---
  /// Makes bytes available to read(). Bytes past capacity count as overflow, like a UART.
  void inject_rx(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      if (rx_len_ < kCapacity) {
        rx_[rx_len_++] = data[i];
      } else {
        ++rx_overflow_;
      }
    }
  }
  const uint8_t* tx_data() const { return tx_.data(); }
  size_t tx_size() const { return tx_len_; }
  void clear_tx() { tx_len_ = 0; }

 private:
  std::array<uint8_t, kCapacity> tx_{};
  std::array<uint8_t, kCapacity> rx_{};
  size_t tx_len_ = 0;
  size_t rx_len_ = 0;
  size_t rx_pos_ = 0;
  uint32_t rx_overflow_ = 0;
};

class FakeImu final : public hal::Imu {
 public:
  hal::ImuSample latest() const override { return sample_; }
  void set(const hal::ImuSample& sample) { sample_ = sample; }

 private:
  hal::ImuSample sample_{};  // value-initialised: zeros, valid == false
};

class FakeAbsoluteEncoder final : public hal::AbsoluteEncoder {
 public:
  hal::AngleSample latest() const override { return sample_; }
  void set(const hal::AngleSample& sample) { sample_ = sample; }

 private:
  hal::AngleSample sample_{};
};

class FakeWheelEncoder final : public hal::WheelEncoder {
 public:
  explicit FakeWheelEncoder(float counts_per_rev) : counts_per_rev_(counts_per_rev) {}
  uint32_t count() const override { return count_; }
  float counts_per_rev() const override { return counts_per_rev_; }
  void set_count(uint32_t count) { count_ = count; }

 private:
  uint32_t count_ = 0;
  float counts_per_rev_;
};

class FakeWheelMotor final : public hal::WheelMotor {
 public:
  void set_duty(float duty) override {
    last_duty_ = duty;
    stopped_ = false;
    ++set_duty_calls_;
  }
  void stop(hal::StopMode mode) override {
    last_stop_mode_ = mode;
    stopped_ = true;
  }

  float last_duty() const { return last_duty_; }
  bool stopped() const { return stopped_; }
  hal::StopMode last_stop_mode() const { return last_stop_mode_; }
  int set_duty_calls() const { return set_duty_calls_; }

 private:
  float last_duty_ = 0.0F;
  bool stopped_ = true;  // a motor that has never been commanded is stopped
  hal::StopMode last_stop_mode_ = hal::StopMode::kCoast;
  int set_duty_calls_ = 0;
};

class FakePitchPowerStage final : public hal::PitchPowerStage {
 public:
  explicit FakePitchPowerStage(bool has_current_sense = false)
      : has_current_sense_(has_current_sense) {}

  void set_phase_duties(float a, float b, float c) override { duties_ = {a, b, c}; }
  void set_enabled(bool enabled) override { enabled_ = enabled; }
  bool fault() const override { return fault_; }
  bool has_current_sense() const override { return has_current_sense_; }
  hal::PhaseCurrents latest_currents() const override {
    hal::PhaseCurrents c = currents_;
    c.valid = c.valid && has_current_sense_;  // the interface contract, enforced here too
    return c;
  }

  const std::array<float, 3>& duties() const { return duties_; }
  bool enabled() const { return enabled_; }
  void set_fault(bool fault) { fault_ = fault; }
  void set_currents(const hal::PhaseCurrents& currents) { currents_ = currents; }

 private:
  std::array<float, 3> duties_{};
  bool enabled_ = false;
  bool fault_ = false;
  bool has_current_sense_;
  hal::PhaseCurrents currents_{};
};

class FakePowerMonitor final : public hal::PowerMonitor {
 public:
  hal::PowerSample latest() const override { return sample_; }
  void set(const hal::PowerSample& sample) { sample_ = sample; }

 private:
  hal::PowerSample sample_{};
};

}  // namespace recon::test
