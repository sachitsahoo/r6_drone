#include "stm_watchdog.hpp"

#include "device.hpp"
#include "safety/check_in_monitor.hpp"  // core::kIwdgTimeout_ms

namespace recon::stm32 {

namespace {

// IWDG key register values. [UNCLEAR] RM0440 "Independent watchdog (IWDG)": verify.
constexpr uint32_t kKeyStart = 0xCCCC;   // starts the watchdog
constexpr uint32_t kKeyUnlock = 0x5555;  // allows writes to PR and RLR
constexpr uint32_t kKeyReload = 0xAAAA;  // reloads the counter (the "feed")

// LSI nominal 32 kHz (ADR 0014 assumes +/-10%). [UNCLEAR] confirm in DS12288.
constexpr uint32_t kLsi_Hz = 32000;
// PR = 0 selects divide-by-4 -> 8 kHz, 125 us per count. [UNCLEAR] confirm PR encoding.
constexpr uint32_t kPrescalerCode = 0;
constexpr uint32_t kPrescalerDivide = 4;
constexpr uint32_t kCount_Hz = kLsi_Hz / kPrescalerDivide;
// 50 ms * 8 kHz = 400 counts. The counter reloads to RLR and resets on reaching 0, so 400
// counts is RLR = 399 or 400 depending on the edge. [UNCLEAR] 125 us either way.
constexpr uint32_t kReload = core::kIwdgTimeout_ms * kCount_Hz / 1000U - 1U;
static_assert(kReload <= 0xFFF, "RLR is 12 bits");

// Reset-cause flags occupy RCC_CSR bits 31..24 (RCC_CSR_*RSTF in the device header).
constexpr uint32_t kResetFlagsShift = 24;

}  // namespace

void StmWatchdog::capture_reset_cause() {
  const uint32_t csr = RCC->CSR;
  was_watchdog_ = (csr & RCC_CSR_IWDGRSTF) != 0;
  reset_flags_ = static_cast<uint8_t>(csr >> kResetFlagsShift);
  RCC->CSR |= RCC_CSR_RMVF;  // clear all reset flags
}

void StmWatchdog::start() {
  // A breakpoint must not reset the board (ADR 0014). Release builds run without a debugger,
  // where this bit has no effect.
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;

  IWDG->KR = kKeyStart;   // starting first also enables the LSI
  IWDG->KR = kKeyUnlock;
  IWDG->PR = kPrescalerCode;
  IWDG->RLR = kReload;
  // Bounded wait for the registers to cross into the LSI domain (a few LSI cycles). If it never
  // completes, the defaults (the longest timeout) stay in force, which fails toward "slow
  // reset", not "no reset".
  for (uint32_t spin = 0; spin < 100000U && (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0;
       ++spin) {
  }
  IWDG->KR = kKeyReload;
}

void StmWatchdog::feed() { IWDG->KR = kKeyReload; }

}  // namespace recon::stm32
