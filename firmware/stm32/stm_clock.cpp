#include "stm_clock.hpp"

#include "device.hpp"

namespace recon::stm32 {

namespace {

constexpr uint32_t kTickRate_Hz = 1000000;  // 1 us per count: the HAL timestamp unit
constexpr uint32_t kPrescaler = kApb1TimerClock_Hz / kTickRate_Hz - 1U;  // PSC divides by PSC+1
static_assert(kApb1TimerClock_Hz % kTickRate_Hz == 0, "TIM2 must count whole microseconds");

}  // namespace

void StmClock::start() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  (void)RCC->APB1ENR1;  // read back: the clock enable needs a cycle before TIM2 is writable
  TIM2->CR1 = 0;
  TIM2->PSC = kPrescaler;
  TIM2->ARR = 0xFFFFFFFFU;  // free-running over the full 32 bits
  TIM2->CNT = 0;
  TIM2->EGR = TIM_EGR_UG;   // load PSC now rather than at the first overflow (71 min away)
  TIM2->CR1 = TIM_CR1_CEN;
}

uint32_t StmClock::now_us() const { return TIM2->CNT; }

}  // namespace recon::stm32
