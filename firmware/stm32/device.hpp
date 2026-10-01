#pragma once

// The one place firmware/stm32 includes the ST device header (ADR 0016). STM32G474xx is
// defined on the compiler command line (firmware/stm32/CMakeLists.txt).
#include "stm32g4xx.h"

#include <cstdint>

namespace recon::stm32 {

/// Core and bus clock, Hz. The first image runs on HSI16, the reset default (RM0440, RCC):
/// SystemInit() in ST's template only enables the FPU, so SYSCLK = HCLK = PCLK1 = PCLK2 =
/// 16 MHz. The PLL to 170 MHz is a later, separate change: it needs flash wait states and
/// the voltage-range boost, more registers to get wrong before there is a board to check on.
inline constexpr uint32_t kSysClock_Hz = 16000000;

/// Timer kernel clock for TIM2/TIM6 (APB1). With the APB1 prescaler at 1, TIMxCLK = PCLK1.
inline constexpr uint32_t kApb1TimerClock_Hz = kSysClock_Hz;

}  // namespace recon::stm32
