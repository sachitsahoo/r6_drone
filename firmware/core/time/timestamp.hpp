#pragma once

#include <cstdint>

namespace recon::core {

/// Period after which a 32-bit microsecond timestamp repeats: 2^32 us ~= 4295 s ~= 71.6 min.
///
/// Derivation: 2^32 microseconds = 4'294'967'296 us. Any duration measured with
/// elapsed_us() must be shorter than this, or it is indistinguishable from a
/// shorter duration (see elapsed_us preconditions).
constexpr uint64_t kTimestampWrapPeriodUs = 4294967296ULL;  // 2^32

/// Microseconds elapsed from `earlier_us` to `later_us`, correct across one wrap.
///
/// MCU timestamps are `uint32_t` microseconds since boot and wrap every ~71.6
/// minutes (kTimestampWrapPeriodUs). Comparing them with `<` or subtracting them
/// as signed values breaks at the wrap. Unsigned subtraction does not: C++
/// guarantees unsigned arithmetic is modulo 2^N, so `later - earlier` yields the
/// true elapsed count even when `later` has wrapped past zero and is numerically
/// smaller than `earlier`.
///
/// \param earlier_us Timestamp taken first, microseconds since MCU boot.
/// \param later_us   Timestamp taken second, microseconds since MCU boot.
/// \return Elapsed microseconds in [0, 2^32).
///
/// \pre The true elapsed time is strictly less than kTimestampWrapPeriodUs
///      (~71.6 minutes). This function cannot detect a longer gap: two wraps look
///      like none. Callers that can legitimately be idle that long -- a disarmed
///      robot left powered, for example -- must track wraps at a higher level
///      rather than relying on this function.
/// \pre `later_us` was genuinely sampled at or after `earlier_us`. Passing them in
///      the wrong order returns a very large value rather than zero or an error,
///      because a reversed pair is indistinguishable from a nearly-full wrap.
///
/// \note ISR-safe: pure function, no state, no allocation, no blocking.
uint32_t elapsed_us(uint32_t earlier_us, uint32_t later_us);

}  // namespace recon::core
