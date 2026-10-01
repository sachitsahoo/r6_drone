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

## What is here

| Directory | Contents | Design |
|---|---|---|
| `time/` | Wraparound-safe `elapsed_us` | hard rule 7 |
| `protocol/` | COBS, CRC-32, frame encoder and garbage-tolerant decoder | ADR 0002 |
| `safety/` | `SafetySupervisor` (DISARMED / ARMED / FAULT / ESTOP, arm interlocks, comms watchdog, fault latching, coast-then-brake), `WheelStallDetector`, `CheckInMonitor` (IWDG feed gate, `kIwdgTimeout_ms`) | ADR 0014; [theory](../../docs/theory/safety-state-machine.md); [learning note](../../docs/learning/safety-state-machine.md) |
| `control/` | Wheel velocity loop: `diff_drive`, `RateLimiter`, `WheelSpeedEstimator`, `WheelVelocityController` (PI, `kff = 0` by default), `DriveLoop` | ADR 0013 (+ amendment); [theory](../../docs/theory/wheel-velocity-loop.md); [learning note](../../docs/learning/wheel-velocity-loop.md) |

`safety/` follows the same pattern as `control/`: pure classes, inputs and outputs as plain
structs, no HAL calls, so every transition is tested from a table on the laptop. It includes
`hal/wheel_motor.hpp` only for the `StopMode` enum.

Config defaults come from the generated `protocol::param_defaults`, so a default is written
once, in `protocol/schema/params.yaml`. `control/geometry.hpp` copies two CAD numbers and is
guarded by `tests/python/test_geometry_drift.py`.

## Known limitations

- Every controller here requires an approved design before implementation
  (see "Owner-reviewed areas" in [`../../CLAUDE.md`](../../CLAUDE.md)).
- `DriveLoop` and `SafetySupervisor` are not yet wired to anything. The glue is proposed in
  ADR 0015 and waits on owner review: who calls `step()` at 1 kHz, who decodes frames into
  `SafetyInputs` (including stale-timestamp rejection, which is not implemented anywhere yet),
  how `ParamSet` reaches `set_config` (no param table yet), and how `LoopTiming` is sent.
- `SafetySupervisor` has no detectors for IMU_FAULT, OVERCURRENT or UNDERVOLTAGE (thresholds
  deferred by ADR 0014), nor for WIRE_LOOP_LIMIT (reserved for ADR 0009).
- The wheel gains are derived from the placeholder sim plant. They must be re-derived after
  system ID (theory doc, §3.1).

## How to test

```
cmake -B build -DTARGET=host && cmake --build build
ctest --test-dir build --output-on-failure
```

Tests live in [`../../tests/cpp/`](../../tests/cpp/). If a change to this directory cannot be
tested on a laptop, that is a signal the change belongs in `../stm32/` instead.
