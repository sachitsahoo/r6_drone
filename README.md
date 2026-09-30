# Recon UGV

Compact, rugged, two-wheeled reconnaissance robot with an actively pitch-stabilized camera body.

**Research question:** how well can active pitch stabilization improve visual stability on a
compact, impact-tolerant UGV under mass, power, and durability limits?

Personal computer-engineering portfolio project and possible Northeastern PEAK research project.

## Status

**Phase 1** — repo skeleton, build system, CI, protocol schema and codec, minimal simulator.
No hardware has been powered. No control law has been written. Every constant that will
eventually be tuned is currently absent, not guessed.

## Architecture

Three tiers. The faster and more safety-critical a loop, the closer to hardware it runs.

| Tier | Hardware | Responsibilities |
|---|---|---|
| MCU | STM32 Nucleo-G474RE | Motor PWM, encoders, velocity PID, diff-drive kinematics, IMU, pitch stabilization, power telemetry, safety state machine, watchdog |
| Robot SBC | Raspberry Pi Zero 2 W + Camera Module 3 Wide | H.264 video capture/stream, UDP <-> UART relay, link monitoring |
| Operator | Laptop | Xbox input (SDL2), video display + telemetry overlay, logging, replay, parameter tuning, simulator |

The central architectural bet is that `firmware/core/` contains no vendor headers and runs
on a laptop, so control code is testable and simulatable without hardware in the loop.
See [`firmware/core/README.md`](firmware/core/README.md).

## Build and test

```
cmake -B build -DTARGET=host && cmake --build build   # host build
ctest --test-dir build --output-on-failure             # C++ tests
pytest                                                 # Python tests
cmake -B build-stm32 -DTARGET=stm32 && cmake --build build-stm32
```

Requires CMake >= 3.20, a C++17 host compiler, Python >= 3.11, and
`arm-none-eabi-gcc` for the target build.

## Documentation

- [`docs/decisions/`](docs/decisions/) — Architecture Decision Records
- [`docs/theory/`](docs/theory/) — control and estimation derivations
- [`docs/learning/`](docs/learning/) — plain-language walkthroughs with self-check questions
- [`docs/bringup/`](docs/bringup/) — hardware bring-up steps and measurements
- [`CLAUDE.md`](CLAUDE.md) — project rules and conventions (authoritative)

## Conventions that bite

- Frames: x forward, y left, z up (right-handed, ROS REP-103).
- **Pitch is positive nose-down.** Camera pitch is world-relative.
- SI units internally, with unit suffixes in names: `speed_m_s`, `angle_rad`, `voltage_V`.
  Degrees appear only in UI.
