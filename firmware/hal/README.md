# firmware/hal — hardware abstraction interfaces

## What it does

Declares the abstract interfaces through which portable code touches hardware. Nine, one
header each in [`hal/`](hal/). The first eight were designed in
[`design-proposal.md`](design-proposal.md) (approved 2026-10-01); `Watchdog` was added by
ADR 0014 (accepted 2026-10-01):

| Interface | Hardware |
|---|---|
| `Clock` | microsecond timebase |
| `SerialPort` | UART to the Pi |
| `Imu` | ICM-42688-P on the casing |
| `AbsoluteEncoder` | the single pitch encoder (ADR 0011) |
| `WheelEncoder` | N20 quadrature encoders |
| `WheelMotor` | TB6612FNG channels |
| `PitchPowerStage` | SimpleFOCMini / DRV8313 (ADR 0007) |
| `PowerMonitor` | INA226 |
| `Watchdog` | the G474 IWDG and its RCC_CSR reset flag (ADR 0014); timeout is a build constant |

## How it fits the architecture

The seam between [`../core/`](../core/) and everything platform-specific. Interfaces are
declared here; implementations live in [`../stm32/`](../stm32/) for the real robot and in
[`../../sim/`](../../sim/) for the simulator. Neither implementation is visible to `core`.

## Key design decisions

- **Protected, non-virtual destructors.** Nothing is deleted through a base pointer, and a
  virtual destructor would reference `operator delete`, which the no-heap build must not
  need. `compile_check.cpp` enforces this with `static_assert`s on both targets.
- **Non-blocking; every sample carries `timestamp_us` and `valid`.** The HAL never invents a
  value when a sensor fails; `core` decides what to do.
- **Interfaces are abstract base classes**, not templates. Compile-time polymorphism would
  avoid a vtable dispatch, but it pushes platform types into `core` signatures and makes the
  firewall harder to police. The dispatch cost is one indirect call per peripheral access at
  1 kHz, which is negligible on a 170 MHz M4. If profiling ever contradicts that, it becomes
  an ADR, not a quiet refactor.
- **Interfaces are narrow and physical.** `Motor` takes a normalized command; it does not
  know about PID. `Imu` returns rates and accelerations in SI units; it does not filter.
  Policy lives in `core`, mechanism lives in the implementation.
- **Units and sign conventions are documented on the interface**, not on each implementation,
  so every implementation is held to the same contract. Wheel speed positive = robot forward.
  Pitch positive = nose-down.

## Known limitations

- No implementations yet. `sim/` provides the first (slice 5); `firmware/stm32/` comes with
  the timer, DMA and interrupt work, which is owner-reviewed separately.
- No hardware watchdog interface: owner decision, it is designed with the safety state
  machine.
- `Clock` has microsecond resolution only. If that proves too coarse for timing the FOC loop
  (hard rule 4), a cycle-counter method gets added then.

## How to test

Interfaces have no behaviour, so they are not tested directly:

- `compile_check.cpp` builds every header under the firmware flags on host **and** stm32,
  and `static_assert`s the design rules (abstract, no virtual destructor, trivially copyable
  samples, `float` only).
- [`../../tests/cpp/hal_fakes.hpp`](../../tests/cpp/hal_fakes.hpp) has a fake per interface;
  `test_hal_fakes.cpp` pins their behaviour, since every core test will stand on them.
- `tools/check_core_purity.py` keeps vendor headers and allocation out of this directory.
