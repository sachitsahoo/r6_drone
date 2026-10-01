# CLAUDE.md — Recon UGV

Compact, rugged, two-wheeled reconnaissance robot with an actively pitch-stabilized
camera body: an inner chassis carries the wheels and drive motors, and an outer casing
carrying the camera and electronics rotates about the wheel axis — at least ±180°, so the
camera can level itself from any landing. See
`docs/decisions/0004-pitch-axis-architecture.md` — the original phrasing was ambiguous
and was read as a camera gimbal, which is the wrong machine. Personal CE portfolio project and possible Northeastern PEAK research
project. Research question: how well can active pitch stabilization improve visual
stability on a compact, impact-tolerant UGV under mass, power, and durability limits?

**The owner must be able to understand and defend every design decision.**
Documentation is part of every task, not an afterthought. See "Documentation rules".

## System architecture

Three tiers. The faster and more safety-critical a loop, the closer to hardware it runs.

| Tier | Hardware | Responsibilities |
|---|---|---|
| MCU | STM32G474 (Nucleo-G474RE on the bench; a small G474 board in the robot) | Motor PWM, encoders, velocity PID, diff-drive kinematics, IMU, pitch stabilization, power telemetry, safety state machine, watchdog |
| Robot SBC | Raspberry Pi Zero 2 W + Camera Module 3 Wide | H.264 video capture/stream, UDP <-> UART relay, link monitoring |
| Operator | Laptop | Xbox input (SDL2), video display + telemetry overlay, logging, replay, parameter tuning, simulator, future autonomy |

Bench hardware: TB6612FNG motor driver, 2x 12 V N20 gearmotors with magnetic encoders,
ICM-42688-P IMU (SPI), INA226 power monitor (I2C).

Machine (decided — ADR 0008): 70 x 182 mm casing, 105 mm wheels, 214 mm overall, **direct
drive** pitch actuator, no gearbox or belt.
Decided (ADR 0007): a bought DRV8313 power stage, with our own voltage-mode FOC on the G474.
Proposed, awaiting owner approval (ADRs 0009–0012) — treat as the working design, but do not
build on them as settled:
- Chassis is a single **spine** on the axis; nothing on the casing can reach the axis (0010).
- Pitch motor: **DM3505** (SparkFun ROB-27477) hollow-shaft gimbal motor, stator on the casing, wheel A's
  shaft through its bore. **One** absolute encoder, read off-axis, serves both FOC and
  estimation (0011). Trimming the sideways CoM offset to ≤ 2 mm is recommended (0011).
- Wiring crosses the joint in a wire loop, about ±3 turns, with a software unwind (0009).
- Power: 3S LiPo; battery and every board ride in the casing (0012).
IMU placement: **on the rotating casing**, 3.8 mm beside the spine's waist (ADRs 0005, 0010).
Chassis pitch is derived as casing pitch minus the encoder angle, so the encoder must be
absolute.

## Repository layout

```
firmware/core/     Portable C++ — control, kinematics, estimation, protocol, safety.
                   NO vendor headers. Compiles for host and target.
firmware/hal/      Abstract interfaces: Clock, SerialPort, Imu, AbsoluteEncoder, WheelEncoder,
                   WheelMotor, PitchPowerStage, PowerMonitor
firmware/stm32/    STM32 HAL implementations, ISRs, timers, DMA, startup, main loop
sim/               Physics model + simulated HAL implementations running firmware/core
protocol/          Message schema (single source of truth) + code generators
robot_bridge/      Pi Zero 2 W daemon: UDP <-> UART relay, video pipeline
operator/          Laptop app: controller input, video, telemetry UI, logging, replay
tools/             Scripts: log analysis, plotting, system identification
cad/               Parametric mechanical model (CadQuery). Exports STEP/STL and reports
                   mass and inertia from the solids. Deps are NOT in requirements-dev
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
- Framing: COBS, CRC-32/ISO-HDLC, message ID, sequence number, timestamp, protocol version.
  (CRC widened from CRC-16 to CRC-32 on 2026-09-30 — see `docs/decisions/0002`.)
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

Phase 1 (repo skeleton, build system, CI, protocol schema and codec, minimal simulator):
**complete 2026-10-01**, CI green on `31b998f`.
Phase 2: first owner-reviewed designs (wheel velocity loop, safety state machine), closed
against the sim plant before any hardware. Wheel loop (ADR 0013) and safety machine (ADR 0014)
are implemented as pure `core` logic with SIL tests (2026-10-01); the glue that runs them
(ADR 0015, accepted) is built: `core/runtime/` tested in SIL, and a first G474 image that links
but drives no motor and has not yet run on a board (ADR 0016: CMSIS headers only). Update this
section as the project progresses.
