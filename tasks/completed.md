# Completed tasks

---

## Phase 2.3 — ADR 0015 glue + first STM32 image (accepted 2026-10-01)

**Done 2026-10-01:** 279 C++ / 214 Python green; image links (11.2 KB flash, 7.4 KB RAM);
purity OK. Not run on hardware. Open: boot-report encoding (implementation-notes Phase 2.3).

Plan written 2026-10-01. Goal: everything ADR 0015 decides, host-tested where possible, plus
a linkable G474 image that **drives no motor** (wheel/pitch drivers are null stubs until the
TB6612 pin map is designed). Nothing can be flashed: the owner has no board yet.

Host (tests first):
- [x] `core/protocol/stale_command_filter`: per-ID newest timestamp, `(int32_t)` compare,
      baseline reset after a link-stale period (Q3); EstopRequest exempt (a stop is never refused)
- [x] `core/safety/safety_mailbox`: atomic e-stop, 4-deep SPSC request ring, double-buffered
      DriveCommand; outbound report ring (ISR -> main) for Fault/Nack frames
- [x] `core/runtime/`: `MotorLoop::tick()` (mailbox -> supervisor -> DriveLoop -> outputs ->
      check-in, exec time + overrun count) and `MainLoop::poll()` (serial -> decoder -> stale
      filter -> mailbox, reports -> frames, check-in, IWDG feed)
- [x] SIL: `test_sil_safety.cpp` drives the real glue instead of its test `Robot`
STM32 (compile + link only, verified on hardware later):
- [x] CMake: FetchContent ST cmsis-device-g4 v1.2.6 + Arm CMSIS_6 v6.3.0 (pinned SHA-256);
      C + ASM enabled for the stm32 target; own linker script; `recon_firmware.elf`
- [x] Drivers: TIM2 1 MHz `Clock`, IWDG `Watchdog` + RCC_CSR boot reason + DBGMCU freeze,
      TIM6 1 kHz ISR, LPUART1 (ST-LINK VCP) with circular DMA RX and polled-FIFO TX, NVIC
      priorities, null wheel/pitch/IMU/power stubs; HSI16 clock (no PLL yet)
- [x] Docs: ADR 0016, firmware/stm32 README, bring-up checklist, notes, theory/learning updates
- [x] Prove: ctest, pytest, STM32 image links (size report), purity guard. Commit at checkpoints.

---

## Bench serial tool (tooling, proceeds directly) — 2026-10-02

**Done 2026-10-02:** 27 tests (241 Python total) green; mutation-checked.

Goal: drive the Nucleo from the Mac for bring-up checklist steps 4-7
(docs/bringup/stm32-first-image.md) before any operator app exists.

- [x] Tests first (tests/python/test_bench_link.py, fake port, no pyserial): frames round-trip
      through protocol/codec.py; magics from the schema; seq and timestamp wrap; drive values
      outside the schema range refused locally; Fault/Nack/boot-report descriptions (reset-flag
      bits from RCC_CSR: OBL 25, PIN 26, BOR 27, SFT 28, IWDG 29, WWDG 30, LPWR 31)
- [x] tools/bench_link.py: `BenchLink` (pure, any byte port) + a line-based REPL with a 50 Hz
      Heartbeat + DriveCommand stream; pyserial imported only when a real port is opened
- [x] Safety: drive defaults to zero; on exit stop streaming and send a disarm request
- [x] pyserial in requirements-dev.txt; tools/README; bring-up doc points at the tool

---

## Phase 2.1 — wheel velocity loop (ADR 0013, accepted 2026-10-01)

**Done 2026-10-01.** 159 C++ / 207 Python green, STM32 build OK. Amended mid-way: the SIL step
test exposed 12% overshoot from feedforward; owner set `kff = 0` (ADR 0013 amendment).
Open: owner may want a reserved `kd` slot (recommended no).

- [x] Schema: 7 drive params at 0x0100+; `unit_suffix` maps `m/s^2` -> `_m_s2`; generator
      emits C++ `param_defaults::` so core initialises from the schema (no copied numbers)
- [x] `core/control/geometry.hpp` (wheel radius, track width) + Python drift test vs `cad/parameters.py`
- [x] Tests first, then `core/control/`: wheel_speed_estimator, wheel_velocity_controller,
      diff_drive, rate_limiter, drive_loop
- [x] `SimWheel`: constant load (duty-equivalent) + extra viscous drag, for the carpet case
- [x] SIL: step response (settle 5% < 100 ms, overshoot < 5%), turn in place, carpet case
      zero steady error, gains ±50% stable, not-ARMED never writes motors
- [x] Docs: theory/wheel-velocity-loop.md, learning/wheel-velocity-loop.md, core README,
      implementation-notes
- Not here: wiring ParamSet to the live config (no param table in core yet), who calls
  step() at 1 kHz (STM32 timer design), LoopTiming transmission (telemetry scheduler).

## Phase 2.2 — safety state machine (ADR 0014, accepted 2026-10-01)

Plan written 2026-10-01. CI green on `e3a5296` before starting. Owner: no reserved `kd` slot.

**Done 2026-10-01** (local, not pushed): 241 C++ / 214 Python green, STM32 build OK, purity
guard OK. Four interpretations await owner review (implementation-notes.html, Phase 2.2); the
glue is proposed as ADR 0015.

Goal: the pure safety logic from ADR 0014, closed against the sim in SIL, with no STM32 glue.

Approach (tests first, then code):
- [x] Schema: FaultCode WHEEL_STALL=8, WATCHDOG_RESET=9, WIRE_LOOP_LIMIT=10 (reserved);
      NackReason ARM_INTERLOCK=10, FAULT_ACTIVE=11; params 0x0200-0x0203. Python test: every
      FaultCode fits a `fault_flags` bit (1..16); param count 11 -> 15.
- [x] `hal/watchdog.hpp` (`feed()`, `reset_was_watchdog()`), added to compile_check;
      `sim/sim_watchdog.hpp` records feeds, reports expiry against the sim clock, can boot "after
      a watchdog reset".
- [x] `core/safety/`: `WheelStallDetector`, `SafetySupervisor` (inputs struct -> outputs struct,
      no HAL calls), `CheckInMonitor` (atomic mask, ISR-safe check-in).
- [x] `SimWheel`: `encoder_reversed` param, `freeze_encoder()`, `braking()` accessor;
      `SimSerialLink::set_cut()`.
- [x] Tests: transition table (4 states x 11 events, completeness enforced), interlocks,
      watchdog 199/200/201 ms + uint32 wrap + Heartbeat-in-ARMED, escalation, clearing,
      CheckInMonitor, SIL (link cut, frozen encoder, reversed encoder, carpet full stick,
      watchdog-reset boot, IWDG starves when a loop stops checking in).
- [x] Docs: theory/safety-state-machine.md, learning/safety-state-machine.md, core + sim + hal
      READMEs, implementation-notes.html, lessons.
- [x] Prove: ctest, pytest, STM32 build. Commit at checkpoints (owner pushes).

Alternatives considered: supervisor reads HAL directly (rejected: ADR wants every transition
table-testable); stall detection inside DriveLoop (rejected: DriveLoop never stops motors itself,
ADR 0013); LOOP_OVERRUN measured by the glue (rejected: the supervisor is stepped by the motor
loop, so the gap between its own steps *is* the loop gap, and that keeps it pure).

Not here (owner-reviewed, propose separately): STM32 IWDG/RCC_CSR driver, timer/ISR layout,
the glue that decodes frames into `SafetyInputs` (incl. stale-timestamp rejection) and applies
outputs to DriveLoop / WheelMotor / PitchPowerStage, param table wiring.

---

## CAD refactor: 70 x 182, direct drive (ADR 0008)  [2026-09-30, DONE]

Owner approved the layout in chat: spine chassis, hollow-shaft pitch motor at end A, IMU on a
ring around a thinned spine waist (r ~ 5 mm), full +/-180 deg leveling kept.

- [x] parameters.py: rescale; delete belt/band/pulley/bracket/standoff values; axial layout
      stations as DERIVED values; spine, cup, motor-end bearing, hollow bore (ASSUMPTION)
- [x] parts.py: delete drive band + motor bracket + chassis disc; add chassis_spine, end cap A
      (motor end); cap B keeps the 6704 and the pendulum datum
- [x] assembly.py: rotation-aware sweep (casing vs chassis sampled over a full turn, pruned by
      exact r_min / conservative r_max); motor, wheel motors, wheel shaft A as envelopes
- [x] build.py: rotating set, quantities, compare against ADR 0008's 0.133e-3
- [x] tests/python/test_cad.py: replace belt-era tests with direct-drive equivalents
- [x] ADR 0010 (axial layout, topology finding); amend ADR 0005; mechanical-requirements R1/R2/R5
- [x] implementation-notes.html entry; cad/README; component-measurements checklist
