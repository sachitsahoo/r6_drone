# CLAUDE.md — Recon UGV

Compact, rugged, two-wheeled reconnaissance robot with an actively pitch-stabilized
camera body. Personal CE portfolio project and possible Northeastern PEAK research
project. Research question: how well can active pitch stabilization improve visual
stability on a compact, impact-tolerant UGV under mass, power, and durability limits?

**The owner must be able to understand and defend every design decision.**
Documentation is part of every task, not an afterthought. See "Documentation rules".

## System architecture

Three tiers. The faster and more safety-critical a loop, the closer to hardware it runs.

| Tier | Hardware | Responsibilities |
|---|---|---|
| MCU | STM32 Nucleo-G474RE | Motor PWM, encoders, velocity PID, diff-drive kinematics, IMU, pitch stabilization, power telemetry, safety state machine, watchdog |
| Robot SBC | Raspberry Pi Zero 2 W + Camera Module 3 Wide | H.264 video capture/stream, UDP <-> UART relay, link monitoring |
| Operator | Laptop | Xbox input (SDL2), video display + telemetry overlay, logging, replay, parameter tuning, simulator, future autonomy |

Bench hardware: TB6612FNG motor driver, 2x N20 gearmotors with magnetic encoders,
ICM-42688-P IMU (SPI), INA226 power monitor (I2C).
Pitch actuator: **undecided** (geared servo vs. gimbal BLDC with FOC). Do not assume one.

## Repository layout

```
firmware/core/     Portable C++ — control, kinematics, estimation, protocol, safety.
                   NO vendor headers. Compiles for host and target.
firmware/hal/      Abstract interfaces: Motor, Encoder, Imu, PowerMonitor, Clock, SerialPort
firmware/stm32/    STM32 HAL implementations, ISRs, timers, DMA, startup, main loop
sim/               Physics model + simulated HAL implementations running firmware/core
protocol/          Message schema (single source of truth) + code generators
robot_bridge/      Pi Zero 2 W daemon: UDP <-> UART relay, video pipeline
operator/          Laptop app: controller input, video, telemetry UI, logging, replay
tools/             Scripts: log analysis, plotting, system identification
tests/             Unit + software-in-the-loop (SIL) tests
docs/              Architecture, decisions/ (ADRs), theory/, bringup/, learning/
```

## Languages and toolchain (defaults — change only via an ADR)

- Firmware and simulator: C++17. Target: `arm-none-eabi-gcc`, CMake.
- Operator app, bridge, tools: Python 3.11+.
- Unit tests: GoogleTest for C++, pytest for Python.
- CI: GitHub Actions runs host build, all tests, and a target cross-compile on every push.

Commands (update when the build system exists):
```
cmake -B build -DTARGET=host && cmake --build build   # host build
ctest --test-dir build --output-on-failure             # C++ tests
pytest                                                 # Python tests
cmake -B build-stm32 -DTARGET=stm32 && cmake --build build-stm32
```

## Hard rules for firmware

1. `firmware/core/` never includes vendor or STM32 headers. It must build and pass tests on a PC.
2. No dynamic allocation after init: no `new`, `malloc`, `std::vector` growth, or `std::string` in firmware.
3. Compile with `-fno-exceptions -fno-rtti`. No blocking calls, printf, or heap use inside ISRs.
4. Every control loop has a fixed, documented rate, and measured execution time is logged.
5. All physical quantities use SI units internally, with unit suffixes in names:
   `speed_m_s`, `angle_rad`, `rate_rad_s`, `voltage_V`, `current_A`, `dt_s`. Degrees only in UI.
6. No magic numbers. Every constant is named, with a comment saying where the value comes from
   (datasheet section, measurement, derivation, or "initial guess — to be tuned").
7. Integer overflow and wraparound (encoder counters, timestamps) must be handled explicitly and tested.

## Conventions

- Frames: x forward, y left, z up (right-handed, ROS REP-103 style).
- **Pitch is positive nose-down** (positive rotation about +y). Camera pitch is world-relative.
  Every controller, estimator, and plot must follow this. If a sign seems wrong, stop and ask.
- Wheel speed positive = robot moving forward.
- Timestamps: `uint32_t` microseconds since MCU boot on the MCU; wraparound handled.

## Loop rates and safety (initial targets)

- Motor velocity loop 1 kHz. Pitch stabilization 500 Hz–1 kHz. IMU sampling ≥ 1 kHz.
- Commands from operator ~50 Hz. Telemetry to operator 50–100 Hz.
- Comms watchdog: no valid command for 200 ms → motors stop (stabilizer may keep running).
- Safety states: `DISARMED -> ARMED -> FAULT / ESTOP`. Motors can only move when `ARMED`.
  Leaving `FAULT` or `ESTOP` requires an explicit operator action. Test every transition.

## Protocol

- Schema in `protocol/` is the single source of truth; C++ and Python code is generated, never hand-edited.
- Framing: COBS, CRC-16, message ID, sequence number, timestamp, protocol version.
- The decoder must survive arbitrary garbage input (fuzz tested).
- Parameters (gains, limits) are get/set by ID over the protocol and stored in MCU flash.
- Logs are raw timestamped protocol frames, so replay reuses the same decoder.

## Workflow for every task

1. **Plan first.** For anything beyond a small fix, write a short plan: goal, approach,
   alternatives considered, files affected, how it will be tested. Wait for approval on
   anything in the owner-reviewed areas below.
2. **Tests define done.** Write or update tests first, from the acceptance criteria.
3. **Implement**, then run the build and all tests. Iterate until they pass. Never weaken,
   skip, or delete a test to make it pass — if a test seems wrong, explain why and ask.
4. **Document** (see below).
5. **Finish with a summary**: what changed, decisions made and why, anything uncertain,
   what the owner should review closely, and open questions.

## Owner-reviewed areas

Propose a design and wait for approval before implementing:
motor control, pitch stabilization, state estimation, safety state machine, watchdog,
STM32 timer/interrupt/DMA configuration, and any protocol schema change.
Build system, CI, tooling, UI, log tools, and test scaffolding may proceed directly.

## Documentation rules

The owner needs to understand, explain, and defend this code in interviews and research writeups.

- **Comments explain why, not what.** Document intent, assumptions, units, and failure behavior.
- **Every public function and class** gets a doc comment: purpose, parameters with units,
  return value, preconditions, and thread/ISR safety.
- **Every module directory** has a `README.md`: what it does, how it fits the architecture,
  key design decisions, known limitations, and how to test it.
- **Architecture Decision Records** in `docs/decisions/NNNN-short-title.md` for any choice
  between reasonable alternatives (context, options considered, decision, consequences).
  Examples: framing scheme, PID form, filter choice, IMU placement, language choices.
- **Control and estimation theory** goes in `docs/theory/`: derivations, block diagrams,
  equations with variable definitions, and references. Code comments link to the relevant section.
- **Learning notes**: for each owner-reviewed module, add `docs/learning/<module>.md` with
  a plain-language walkthrough and 5–10 "check your understanding" questions with answers,
  e.g. "Why does the inner loop use gyro rate instead of the estimated angle?"
- **Hardware bring-up** steps and measurements go in `docs/bringup/`.
- Prefer clear code over clever code. If something is subtle, say so explicitly.

## Hardware-in-the-loop safety

When flashing or running code on real hardware:
- Never command motors unless the owner has confirmed wheels are off the ground and a kill switch is in reach.
- Default motor output limits must be low; raising them requires owner approval.
- Never leave a motor-driving process running unattended.
- Do not change power, battery, or charging parameters without owner approval.

## Out of scope for now

ROS 2, autonomy, SLAM, LiDAR, custom PCB, self-righting. Do not add these or design for
them speculatively. Keep the core platform simple and reliable first.

## Current phase

Phase 1: repo skeleton, build system, CI, protocol schema and codec, minimal simulator.
Update this section as the project progresses.
