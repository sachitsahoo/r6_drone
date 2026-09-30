# sim — physics model and simulated HAL

## What it does

Implements the [`../firmware/hal/`](../firmware/hal/) interfaces against a physics model
instead of hardware, so the real [`../firmware/core/`](../firmware/core/) can be run,
tested, and tuned on a laptop.

## How it fits the architecture

The simulator is not a separate model of the robot's software — it runs the same control
code that ships to the MCU. That is the payoff for keeping `core` vendor-free: a
software-in-the-loop (SIL) test can exercise a real control law with no hardware attached.

## Key design decisions

- **Runs `core` unmodified.** If the simulator ever needs a special case inside `core`,
  that is a design failure, not a shortcut.
- **Simulated `Clock` is virtual time**, advanced explicitly by the test. Loops can be
  stepped deterministically, and a SIL test is reproducible rather than timing-dependent.
- **The plant model starts deliberately crude.** Real parameters (inertia, motor constants,
  friction) come from system identification on hardware, which has not happened. A
  placeholder model that closes a loop is useful; a detailed model built from guessed
  constants is false confidence.

## Known limitations

- Empty as of Phase 1.
- No camera or video simulation. Visual-stability evaluation needs real optics.
- Contact, impact, and tipping dynamics are out of scope for now.

## How to test

SIL tests live in [`../tests/`](../tests/) and run in CI with no hardware:

```
cmake -B build -DTARGET=host && cmake --build build
ctest --test-dir build --output-on-failure
```
