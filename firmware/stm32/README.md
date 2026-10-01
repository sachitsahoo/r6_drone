# firmware/stm32 — target platform layer

## What it does

The Nucleo-G474RE firmware image: implementations of the [`../hal/`](../hal/) interfaces on
real peripherals, the interrupt handlers, and `main()`, which wires `core`'s loops to the
hardware. Design: [ADR 0015](../../docs/decisions/0015-mcu-loop-timing-and-safety-glue.md)
(what runs where) and [ADR 0016](../../docs/decisions/0016-stm32-register-access.md) (CMSIS
headers only).

## How it fits the architecture

The only directory that includes vendor headers, and only through `device.hpp`. Everything
here is a thin adapter. The decisions live in `core`: `core::MotorLoop` is the body of the
1 kHz interrupt and `core::MainLoop` the body of the superloop, and both run unchanged on the
simulator (`tests/cpp/test_sil_safety.cpp`).

```
TIM6 update IRQ (1 kHz, priority 2) -> core::MotorLoop::tick()
DMA1 ch1 IRQ (priority 3)           -> StmSerial::on_dma_rx_interrupt()   (counts RX laps)
main() superloop                    -> core::MainLoop::poll(); StmSerial::service()
```

| File | Implements | Notes |
|---|---|---|
| `device.hpp` | — | The one vendor include; `kSysClock_Hz` = 16 MHz (HSI16) |
| `stm_clock.*` | `hal::Clock` | TIM2, 32-bit at 1 MHz: the counter is the timestamp and wraps at 2^32 us |
| `stm_watchdog.*` | `hal::Watchdog` | IWDG at `core::kIwdgTimeout_ms`; RCC_CSR boot reason; frozen under the debugger |
| `stm_serial.*` | `hal::SerialPort` | LPUART1 (ST-LINK VCP), 460 800 baud, circular-DMA RX with overrun counting, non-blocking FIFO TX |
| `null_hardware.hpp` | `WheelMotor`, `WheelEncoder`, `PitchPowerStage` | **Do nothing.** The first image drives no motor |
| `main.cpp` | — | Static objects, boot order, IRQ handlers, superloop |
| `linker/stm32g474re.ld` | — | Ours: 512 KB flash, 96 KB SRAM, 4 KB stack reserve, **no heap** |

## Key design decisions

- **Deliberately thin.** Logic here is invisible to host tests and the simulator, so any
  decision-making in this directory is a smell.
- **Boot order** (`main()`): read the reset cause first; start the clock, then the UART; build
  the supervisor with the boot reason; start the IWDG; start the motor-loop timer last.
- **No heap, enforced by the link.** The linker script defines no heap, so anything that calls
  `malloc` fails to link. That is how the first build found libstdc++'s assertion handler
  pulling in `abort` → stdio → `malloc`. A failed library assertion or a pure-virtual call now
  spins until the IWDG resets the chip, so it is reported at the next boot.
- **HSI16, no PLL.** The PLL to 170 MHz needs flash wait states and the boost voltage range.
  It is a separate change, made once there is a board to check it on.
- **Built `RelWithDebInfo` by default**, so size and timing match what runs on the robot.

## Known limitations

- **Nothing here has run on hardware.** Every register value the device header cannot check is
  marked [UNCLEAR] in the source. [`docs/bringup/stm32-first-image.md`](../../docs/bringup/stm32-first-image.md)
  lists the check that settles each one.
- No motor drivers (TB6612, DRV8313), encoders, IMU or power monitor yet. Each needs a pin and
  timer design first.
- No telemetry: StateTelemetry, LoopTiming and LinkStats wait on the telemetry scheduler.
- TX is polled into the 8-byte FIFO, so throughput is 8 bytes per main-loop pass. That is fine
  for Fault and Nack frames, and may need DMA once telemetry runs.

## How to test

```
cmake -B build-stm32 -DTARGET=stm32 && cmake --build build-stm32
```

CI builds and links the image on every push, with the same Arm toolchain version as the
developer machine. On-target verification follows the bring-up checklist. The
hardware-in-the-loop safety rules in [`../../CLAUDE.md`](../../CLAUDE.md) apply to everything
here once motors exist: wheels off the ground, kill switch in reach, low output limits.
