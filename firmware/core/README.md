# firmware/core — portable control code

## What it does

All the logic that decides what the robot should do: velocity control, differential-drive
kinematics, pitch stabilization, state estimation, protocol encode/decode, and the safety
state machine.

## How it fits the architecture

This is the architectural firewall. `firmware/core/` contains **no vendor or STM32 headers**.
It reaches hardware only through the abstract interfaces in [`../hal/`](../hal/), so the
identical code compiles three ways:

- into STM32 firmware, against `../stm32/` implementations
- into host unit tests, against test doubles
- into the simulator, against [`../../sim/`](../../sim/) implementations

## Key design decisions

- **Portability over convenience.** Anything that needs a peripheral register goes in
  `../stm32/`, behind a HAL interface. Retrofitting this separation after vendor headers
  spread through the tree is the most expensive mistake available on this project, so it is
  enforced in CI rather than left as a convention. See `.github/workflows/ci.yml`.
- **No dynamic allocation after init.** No `new`, `malloc`, growing `std::vector`, or
  `std::string`. Buffers are fixed-size members sized at compile time. A control loop that
  can fail on allocation is a control loop that can fail unpredictably.
- **`-fno-exceptions -fno-rtti`.** Errors are return values and explicit state, not thrown.
- **SI units with suffixes in every name.** `dt_s`, `speed_m_s`, `rate_rad_s`. Unit mistakes
  in control code are silent and expensive; the names make them visible at the call site.

## Known limitations

- Empty as of Phase 1. No control law exists yet.
- Every controller here will require an approved design before implementation
  (see "Owner-reviewed areas" in [`../../CLAUDE.md`](../../CLAUDE.md)).

## How to test

```
cmake -B build -DTARGET=host && cmake --build build
ctest --test-dir build --output-on-failure
```

Tests live in [`../../tests/cpp/`](../../tests/cpp/). If a change to this directory cannot be
tested on a laptop, that is a signal the change belongs in `../stm32/` instead.
