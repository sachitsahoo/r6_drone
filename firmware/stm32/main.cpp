// Recon UGV firmware entry point. Design: ADR 0015 (two contexts, no RTOS).
//
//   TIM6 interrupt, 1 kHz:  core::MotorLoop::tick()   (supervisor -> DriveLoop -> outputs)
//   main() superloop:       core::MainLoop::poll()    (frames -> mailbox; reports -> frames;
//                           StmSerial::service()       IWDG feed on check-in)
//
// THIS IMAGE DRIVES NO MOTOR. Wheels and the pitch stage are null stubs (null_hardware.hpp)
// until the TB6612 / DRV8313 drivers are designed. It exists to bring up the clock, the UART
// link, the safety machine and the IWDG on a bare Nucleo, with nothing that can move.

#include "control/drive_loop.hpp"
#include "device.hpp"
#include "null_hardware.hpp"
#include "runtime/main_loop.hpp"
#include "runtime/motor_loop.hpp"
#include "safety/check_in_monitor.hpp"
#include "safety/safety_mailbox.hpp"
#include "safety/safety_supervisor.hpp"
#include "stm_clock.hpp"
#include "stm_serial.hpp"
#include "stm_watchdog.hpp"

namespace {

using namespace recon;

// NVIC priorities, ADR 0015 §5 (lower number = more urgent). 1 is reserved for pitch FOC.
constexpr uint32_t kMotorLoopIrqPriority = 2;

// TIM6 at 1 kHz: 16 MHz / (PSC + 1) = 1 MHz, / (ARR + 1) = 1 kHz.
constexpr uint32_t kTim6TickRate_Hz = 1000000;
constexpr uint32_t kTim6Prescaler = stm32::kApb1TimerClock_Hz / kTim6TickRate_Hz - 1U;
constexpr uint32_t kTim6Reload = core::kMotorLoopPeriod_us - 1U;

constexpr uint32_t kRequiredCheckIns = static_cast<uint32_t>(core::CheckInTask::kMotorLoop) |
                                       static_cast<uint32_t>(core::CheckInTask::kMainLoop);

// Every object is static: constructed before main() by __libc_init_array, never freed, no heap.
// Constructors here only store references; hardware is touched in main(), in a fixed order.
stm32::StmClock g_clock;
stm32::StmWatchdog g_watchdog;
stm32::StmSerial g_serial;
stm32::NullWheel g_left_wheel;
stm32::NullWheel g_right_wheel;
stm32::NullPitchStage g_pitch_stage;

core::DriveLoop g_drive(g_clock, g_left_wheel, g_left_wheel, g_right_wheel, g_right_wheel);
core::SafetyMailbox g_mailbox;
core::ReportQueue g_reports;
core::CheckInMonitor g_check_ins(kRequiredCheckIns);

// Built in main() as function-local statics, because the supervisor needs the boot reason,
// which is only known once main() has read RCC_CSR. The ISR reaches the motor loop through
// this pointer, set before the timer interrupt is enabled.
core::MotorLoop* g_motor_loop = nullptr;

core::MainLoop g_main_loop(g_clock, g_serial, g_watchdog, g_mailbox, g_reports, g_check_ins,
                           core::SafetyConfig{}.comms_timeout_ms * 1000U);

void start_motor_loop_timer() {
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
  (void)RCC->APB1ENR1;
  TIM6->CR1 = 0;
  TIM6->PSC = kTim6Prescaler;
  TIM6->ARR = kTim6Reload;
  TIM6->EGR = TIM_EGR_UG;  // load PSC/ARR now
  TIM6->SR = 0;            // UG sets UIF; clear it so the first interrupt is a real period
  TIM6->DIER = TIM_DIER_UIE;
  NVIC_SetPriority(TIM6_DAC_IRQn, kMotorLoopIrqPriority);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  TIM6->CR1 = TIM_CR1_CEN;
}

}  // namespace

extern "C" void TIM6_DAC_IRQHandler() {
  if ((TIM6->SR & TIM_SR_UIF) != 0) {
    TIM6->SR = ~TIM_SR_UIF;  // rc_w0: writing 0 clears, writing 1 leaves other flags alone
    g_motor_loop->tick();
  }
}

extern "C" void DMA1_Channel1_IRQHandler() { g_serial.on_dma_rx_interrupt(); }

// Called if a pure virtual is ever invoked (a bug). Loop until the IWDG resets the chip, which
// then boots into FAULT(WATCHDOG_RESET). Defined here so libsupc++'s version, which pulls in
// abort() and stdio, is never linked.
extern "C" void __cxa_pure_virtual() {
  while (true) {
  }
}

// __libc_init_array (called by ST's Reset_Handler to run static constructors) calls _init,
// normally supplied by crti.o. -nostartfiles drops crti.o; there is nothing to run here.
extern "C" void _init() {}

// libstdc++'s run-time checks (std::array bounds, atomic memory orders) call this when they
// fail. GCC 15 turns them on in unoptimised builds. Its default calls abort(), which drags in
// signal handling, stdio and malloc. Here a failed check spins until the IWDG resets the chip,
// so it is reported at the next boot as FAULT(WATCHDOG_RESET), not silently ignored.
namespace std {
[[noreturn]] void __glibcxx_assert_fail(const char* /*file*/, int /*line*/,
                                        const char* /*function*/,
                                        const char* /*condition*/) noexcept {
  while (true) {
  }
}
}  // namespace std

int main() {
  // 1. Why did we reset? Read before anything can cause another reset.
  g_watchdog.capture_reset_cause();

  // 2. Timebase, then the link.
  g_clock.start();
  g_serial.start();

  // 3. Safety machine, booting into FAULT(WATCHDOG_RESET) after an IWDG reset (ADR 0014 Q5).
  // Static storage, constructed here exactly once (main never returns). Built with
  // -fno-threadsafe-statics: no other context can race this first-time construction.
  static core::SafetySupervisor supervisor(core::SafetyConfig{}, g_watchdog.reset_was_watchdog());
  static core::MotorLoop motor_loop(g_clock, g_drive, supervisor, g_mailbox, g_reports,
                                    g_check_ins, g_left_wheel, g_right_wheel, g_pitch_stage);
  g_motor_loop = &motor_loop;
  g_main_loop.queue_boot_report(g_watchdog.reset_flags());  // ADR 0015 Q4: information only

  // 4. Watchdog last before the loops: from here on, a hang resets the chip within 50 ms.
  g_watchdog.start();
  start_motor_loop_timer();

  while (true) {
    g_main_loop.poll();
    g_serial.service();
  }
}
