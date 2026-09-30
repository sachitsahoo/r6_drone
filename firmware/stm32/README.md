# firmware/stm32 — target platform layer

## What it does

STM32 HAL implementations of the [`../hal/`](../hal/) interfaces, plus interrupt service
routines, timer and DMA configuration, startup code, and the main loop that schedules
the control loops defined in [`../core/`](../core/).

## How it fits the architecture

The only directory permitted to include vendor headers. Everything here is a thin adapter:
translate a peripheral into a HAL interface, or hand a timer tick to a `core` loop. Any
logic that could be tested on a laptop belongs in `core` instead.

## Key design decisions

- **Deliberately thin.** Decision-making here is invisible to host tests and the simulator,
  so it is treated as a smell.
- **Nothing blocking in ISRs.** No printf, no heap, no waiting. ISRs capture data and set
  flags; work happens in the scheduled loop.
- **Measured loop execution time is logged**, not assumed. A 1 kHz loop that occasionally
  takes 1.2 ms is a bug that only telemetry will reveal.

## Known limitations

- Empty as of Phase 1. Phase 1's target build compiles `core` only and links no firmware image.
- Timer, interrupt, and DMA configuration is an owner-reviewed area — it requires an approved
  design before implementation.

## How to test

Not unit-testable off-target by design. Verification is:

1. The CI cross-compile proves it builds for `arm-none-eabi`.
2. Hardware bring-up procedures and measurements in [`../../docs/bringup/`](../../docs/bringup/).

Hardware-in-the-loop safety rules in [`../../CLAUDE.md`](../../CLAUDE.md) apply to everything
here: wheels off the ground, kill switch in reach, low default output limits.
