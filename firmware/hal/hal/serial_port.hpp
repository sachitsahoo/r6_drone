#pragma once

#include <cstddef>
#include <cstdint>

namespace recon::hal {

/// Byte pipe to the Pi Zero over UART. Approved design: firmware/hal/design-proposal.md.
///
/// Bytes only. Framing (COBS, CRC-32, message IDs) belongs to `core/protocol`, so the same
/// codec runs unchanged over the real UART, the simulator, and replayed logs.
class SerialPort {
 public:
  /// Queues up to `len` bytes for transmission.
  ///
  /// \param data Bytes to send. May be null only if `len` is 0.
  /// \param len  Number of bytes offered.
  /// \return How many bytes were accepted, in [0, len]. Fewer than `len` means the transmit
  ///         buffer is full; the caller decides whether to retry or drop the frame.
  /// \note Never blocks. Not ISR-safe: call from the main loop only.
  virtual size_t write(const uint8_t* data, size_t len) = 0;

  /// Copies up to `len` received bytes into `out`.
  ///
  /// \param out Destination buffer of at least `len` bytes.
  /// \param len Capacity of `out`.
  /// \return Number of bytes copied, 0 if none are waiting.
  /// \note Never blocks. Not ISR-safe: call from the main loop only.
  virtual size_t read(uint8_t* out, size_t len) = 0;

  /// Bytes lost to receive-buffer overflow since boot. Wraps at 2^32; take differences with
  /// unsigned subtraction. Feeds `LinkStats` so the operator can see a saturated link.
  /// \note ISR-safe.
  virtual uint32_t rx_overflow_count() const = 0;

 protected:
  ~SerialPort() = default;  // see Clock for why this is protected and non-virtual
};

}  // namespace recon::hal
