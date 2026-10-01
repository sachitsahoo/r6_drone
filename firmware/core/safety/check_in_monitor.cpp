#include "safety/check_in_monitor.hpp"

namespace recon::core {

bool CheckInMonitor::should_feed() {
  if ((bits_.load(std::memory_order_relaxed) & required_) != required_) {
    return false;
  }
  // Clear only the required bits. A task checking in between the load and here sets a bit
  // that is already set, so nothing is lost; at worst it must check in again before the next
  // feed, which it does within one period (1 ms), far inside the 50 ms timeout.
  bits_.fetch_and(~required_, std::memory_order_relaxed);
  return true;
}

}  // namespace recon::core
