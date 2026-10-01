#include "stm_serial.hpp"

#include "device.hpp"

namespace recon::stm32 {

namespace {

static_assert((kRxDmaBytes & (kRxDmaBytes - 1)) == 0, "RX ring must be a power of two");
static_assert((kTxRingBytes & (kTxRingBytes - 1)) == 0, "TX ring must be a power of two");

// Nucleo-G474RE: the ST-LINK VCP is wired to LPUART1 on PA2 (TX) and PA3 (RX), alternate
// function 12. [UNCLEAR] confirm against the Nucleo user manual (UM2505) and the DS12288
// alternate-function table.
constexpr uint32_t kTxPin = 2;
constexpr uint32_t kRxPin = 3;
constexpr uint32_t kAfLpuart1 = 12;
constexpr uint32_t kModerAlternate = 0b10;
constexpr uint32_t kPupdPullUp = 0b01;  // RX idles high; a pull-up keeps an unplugged line quiet

// LPUART baud register: BRR = 256 * f_ck / baud (RM0440, LPUART baud rate generation).
// 256 * 16 MHz / 460 800 = 8888.9 -> 8889, a 0.001% rate error.
// [UNCLEAR] confirm the formula and the BRR >= 0x300 lower bound in RM0440.
constexpr uint32_t kBrr = static_cast<uint32_t>(
    (256ULL * kSysClock_Hz + kBaud / 2) / kBaud);
static_assert(kBrr >= 0x300 && kBrr < (1U << 20), "LPUART BRR out of its legal range");

// DMAMUX request line for LPUART1_RX. [UNCLEAR] RM0440 DMAMUX request table gives 34; the
// device header has no symbol for it. Verify before relying on RX.
constexpr uint32_t kDmamuxReqLpuart1Rx = 34;

constexpr uint32_t kDmaRxPriority = 3;  // ADR 0015 §5: below the motor loop

}  // namespace

void StmSerial::start() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
  (void)RCC->APB1ENR2;  // read back before touching the peripherals

  // Pins: alternate function, pull-up on RX.
  constexpr uint32_t kPins[] = {kTxPin, kRxPin};
  for (const uint32_t pin : kPins) {
    GPIOA->MODER = (GPIOA->MODER & ~(0b11U << (2 * pin))) | (kModerAlternate << (2 * pin));
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << (4 * pin))) | (kAfLpuart1 << (4 * pin));
  }
  GPIOA->PUPDR = (GPIOA->PUPDR & ~(0b11U << (2 * kRxPin))) | (kPupdPullUp << (2 * kRxPin));

  // DMA1 channel 1 <- DMAMUX channel 0 <- LPUART1_RX. Peripheral to memory (DIR = 0), byte
  // sizes (PSIZE = MSIZE = 0), memory increment, circular, interrupt at transfer complete.
  DMA1_Channel1->CCR = 0;
  DMAMUX1_Channel0->CCR = kDmamuxReqLpuart1Rx << DMAMUX_CxCR_DMAREQ_ID_Pos;
  DMA1_Channel1->CPAR = reinterpret_cast<uint32_t>(&LPUART1->RDR);
  DMA1_Channel1->CMAR = reinterpret_cast<uint32_t>(rx_buf_);
  DMA1_Channel1->CNDTR = kRxDmaBytes;
  DMA1->IFCR = DMA_IFCR_CGIF1;
  DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_EN;
  NVIC_SetPriority(DMA1_Channel1_IRQn, kDmaRxPriority);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  // LPUART1: 8N1 (CR1 M bits 0, no parity), FIFO on, RX via DMA.
  LPUART1->CR1 = 0;
  LPUART1->BRR = kBrr;
  LPUART1->CR3 = USART_CR3_DMAR;
  LPUART1->CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void StmSerial::on_dma_rx_interrupt() {
  if ((DMA1->ISR & DMA_ISR_TCIF1) != 0) {
    DMA1->IFCR = DMA_IFCR_CTCIF1;
    rx_laps_.fetch_add(1, std::memory_order_release);
  }
}

uint32_t StmSerial::rx_written() const {
  // Total = laps * size + position. Three things can move under us: the DMA position, the
  // transfer-complete flag (set at the reload, before the interrupt runs) and the lap count
  // (when it does). Read all three, then re-read the flag and the laps; any change means the
  // reload landed inside the window, so read again. This is the main loop retrying against an
  // ISR, which always makes progress (the ISR is never the one waiting).
  while (true) {
    const uint32_t laps = rx_laps_.load(std::memory_order_acquire);
    const bool tc_before = (DMA1->ISR & DMA_ISR_TCIF1) != 0;
    const uint32_t remaining = DMA1_Channel1->CNDTR;
    const bool tc_after = (DMA1->ISR & DMA_ISR_TCIF1) != 0;
    if (laps != rx_laps_.load(std::memory_order_acquire) || tc_before != tc_after) {
      continue;
    }
    // A flag set but not yet serviced is a lap the counter doesn't know about yet.
    const uint32_t laps_done = laps + (tc_before ? 1U : 0U);
    return laps_done * kRxDmaBytes + (kRxDmaBytes - remaining);
  }
}

size_t StmSerial::read(uint8_t* out, size_t len) {
  const uint32_t written = rx_written();
  uint32_t unread = written - rx_consumed_;
  if (unread > kRxDmaBytes) {
    // The DMA lapped us: the oldest bytes were overwritten. Count them and skip to the oldest
    // byte still in the ring. The decoder resynchronises on the next delimiter.
    rx_overflow_.fetch_add(unread - kRxDmaBytes, std::memory_order_relaxed);
    rx_consumed_ = written - kRxDmaBytes;
    unread = kRxDmaBytes;
  }
  size_t n = 0;
  while (n < len && n < unread) {
    out[n++] = rx_buf_[rx_consumed_++ % kRxDmaBytes];
  }
  return n;
}

size_t StmSerial::write(const uint8_t* data, size_t len) {
  size_t n = 0;
  while (n < len && (tx_head_ - tx_tail_) < kTxRingBytes) {
    tx_buf_[tx_head_++ % kTxRingBytes] = data[n++];
  }
  service();
  return n;
}

void StmSerial::service() {
  while (tx_tail_ != tx_head_ && (LPUART1->ISR & USART_ISR_TXE_TXFNF) != 0) {
    LPUART1->TDR = tx_buf_[tx_tail_++ % kTxRingBytes];
  }
}

}  // namespace recon::stm32
