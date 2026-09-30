#include "time/timestamp.hpp"

namespace recon::core {

uint32_t elapsed_us(uint32_t earlier_us, uint32_t later_us) {
  // Modulo-2^32 unsigned subtraction. Deliberately not a signed difference or a
  // branch on later < earlier: both are wrong at the wrap boundary, and this is
  // correct there by definition of unsigned arithmetic. See header for the
  // preconditions this cannot check.
  return later_us - earlier_us;
}

}  // namespace recon::core
