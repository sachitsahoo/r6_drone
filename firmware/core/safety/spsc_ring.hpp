#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace recon::core {

/// Fixed-capacity single-producer single-consumer queue. Design: ADR 0015, "The mailbox".
///
/// One context pushes and one other context pops. On the G474 one of them is an ISR and the
/// other the main loop; either may be the producer. Lock-free and wait-free: each side
/// writes only its own index, publishes it with release ordering, and reads the other's with
/// acquire ordering, so a slot is never read before it is fully written.
///
/// \tparam T Trivially copyable element.
/// \tparam N Capacity. A power of two, so the free-running indices wrap cleanly at 2^32.
template <typename T, size_t N>
class SpscRing {
  static_assert(N > 0 && (N & (N - 1)) == 0, "capacity must be a power of two");
  static_assert(std::atomic<uint32_t>::is_always_lock_free,
                "an ISR is one side of the queue, so indices must be lock-free");

 public:
  static constexpr size_t kCapacity = N;

  /// Producer side. \return False (and `item` is dropped) when full.
  bool push(const T& item) {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    if (tail - head_.load(std::memory_order_acquire) == N) {
      return false;
    }
    slots_[tail % N] = item;
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  /// Consumer side. \return False when empty; `out` is untouched.
  bool pop(T& out) {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    if (head == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    out = slots_[head % N];
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  /// Snapshot of the fill level. Exact only from the consumer side.
  size_t size() const {
    return tail_.load(std::memory_order_acquire) - head_.load(std::memory_order_acquire);
  }

 private:
  T slots_[N] = {};
  // Free-running: unsigned subtraction gives the fill level across the 2^32 wrap.
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
};

}  // namespace recon::core
