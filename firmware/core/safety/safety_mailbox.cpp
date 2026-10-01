#include "safety/safety_mailbox.hpp"

namespace recon::core {

void SafetyMailbox::stage_drive_command(float linear_m_s, float angular_rad_s) {
  // The slot after the newest. The consumer only ever reads the newest, and cannot be
  // mid-read here: on one core the producer runs only while the ISR is not running.
  const uint32_t next = drive_seq_.load(std::memory_order_relaxed) + 1U;
  drive_[next & 1U] = DriveSlot{linear_m_s, angular_rad_s};
}

void SafetyMailbox::publish_drive_command() {
  drive_seq_.fetch_add(1U, std::memory_order_release);
}

void SafetyMailbox::take(SafetyInputs& in) {
  if (estop_.exchange(false, std::memory_order_acquire)) {
    in.estop = true;
  }
  SafetyRequest request{};
  if (requests_.pop(request)) {
    in.request = request;
  }
  const uint32_t seq = drive_seq_.load(std::memory_order_acquire);
  if (seq != drive_seen_) {
    drive_seen_ = seq;
    const DriveSlot& slot = drive_[seq & 1U];
    in.drive_command = true;
    in.drive_linear_m_s = slot.linear_m_s;
    in.drive_angular_rad_s = slot.angular_rad_s;
  }
  if (other_frame_.exchange(false, std::memory_order_acquire)) {
    in.other_frame = true;
  }
}

}  // namespace recon::core
