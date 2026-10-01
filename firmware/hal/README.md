# firmware/hal — hardware abstraction interfaces

## What it does

Declares the abstract interfaces through which portable code touches hardware:
`Motor`, `Encoder`, `Imu`, `PowerMonitor`, `Clock`, `SerialPort`.

## How it fits the architecture

The seam between [`../core/`](../core/) and everything platform-specific. Interfaces are
declared here; implementations live in [`../stm32/`](../stm32/) for the real robot and in
[`../../sim/`](../../sim/) for the simulator. Neither implementation is visible to `core`.

## Key design decisions

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

- Empty as of Phase 1.
- The pitch actuator interface is not designed yet. The actuator is now chosen — direct-drive
  DM3505 with our own FOC (ADRs 0007, 0008, 0011, the last two of which are proposed) — so
  the interface will be "set three phase duty cycles" plus **one** absolute encoder shared by
  FOC and estimation. Designed once ADR 0007 is approved.

## How to test

Interfaces have no behavior, so they are not tested directly. They are exercised through
test doubles in [`../../tests/cpp/`](../../tests/cpp/) and through the simulator.
