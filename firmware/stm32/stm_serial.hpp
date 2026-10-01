#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "hal/serial_port.hpp"

namespace recon::stm32 {

/// Receive ring filled by circular DMA, bytes. A power of two so the free-running byte counts
/// stay consistent across their 2^32 wrap. 512 B is 11 ms of a saturated 460 800 baud line, so
/// the main loop may stall up to 11 ms before bytes are lost (and counted).
inline constexpr size_t kRxDmaBytes = 512;
/// Transmit ring, bytes. Initial guess: six maximum-size frames (6 x 77 B). Telemetry, when it
/// arrives, may need TX DMA and a re-sized ring.
inline constexpr size_t kTxRingBytes = 512;
/// Line rate. docs/bringup/uart-link.md (owner, 2026-09-30).
inline constexpr uint32_t kBaud = 460800;

/// `hal::SerialPort` on LPUART1, the Nucleo-G474RE's ST-LINK virtual COM port, so on the bench
/// the Mac is the operator. ADR 0015 §1 and docs/bringup/uart-link.md:
///
/// - **RX: circular DMA**, mandatory at this rate (uart-link.md, consequence 4). The main loop
///   reads straight out of the DMA ring. A transfer-complete interrupt counts laps, so the
///   driver always knows how many bytes have arrived in total and can detect, and count, an
///   overrun of the ring. ADR 0015 also lists the IDLE interrupt; it is not needed, because
///   the main loop polls the DMA position on every pass instead of waiting to be told.
/// - **TX: a ring the main loop drains into the 8-byte hardware FIFO** without blocking
///   (`service()`), no interrupt. Fault and Nack frames are rare; throughput is 8 bytes per
///   main-loop pass, ample until telemetry arrives.
class StmSerial final : public hal::SerialPort {
 public:
  /// Clocks, pins (PA2 TX / PA3 RX, AF12), LPUART1, DMAMUX and DMA1 channel 1. Call once.
  void start();

  /// Moves queued TX bytes into the hardware FIFO. Call every main-loop pass. Never blocks.
  void service();

  /// The DMA1 channel 1 interrupt body: counts a completed lap of the RX ring.
  void on_dma_rx_interrupt();

  // --- hal::SerialPort ---
  size_t write(const uint8_t* data, size_t len) override;
  size_t read(uint8_t* out, size_t len) override;
  uint32_t rx_overflow_count() const override {
    return rx_overflow_.load(std::memory_order_relaxed);
  }

 private:
  /// Bytes the DMA has written since start, mod 2^32. Lock-free against the lap interrupt.
  uint32_t rx_written() const;

  alignas(4) uint8_t rx_buf_[kRxDmaBytes] = {};
  std::atomic<uint32_t> rx_laps_{0};  // written by the DMA interrupt
  uint32_t rx_consumed_ = 0;          // main loop only
  std::atomic<uint32_t> rx_overflow_{0};

  uint8_t tx_buf_[kTxRingBytes] = {};
  uint32_t tx_head_ = 0;  // main loop only: write() and service() run in the same context
  uint32_t tx_tail_ = 0;
};

}  // namespace recon::stm32
