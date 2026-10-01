# Recon UGV — tasks

Phase 1: repo skeleton, build system, CI, protocol schema and codec, minimal simulator.
Plan written 2026-09-30. Spec of record: ../CLAUDE.md

**Slices 1–5 complete.** 1–4 verified 2026-09-30; the `firmware/hal/` interfaces were
approved and implemented 2026-10-01; slice 5 (minimal simulator) landed 2026-10-01. Phase 1
is **complete**: CI green on `31b998f` (2026-10-01).

**Safety design inputs already decided by the owner** (for the state machine proposal):
wheels coast on comms timeout, escalating to brake if the link stays down long enough
(delay to be set there); the hardware watchdog (IWDG) is designed alongside it.

Mechanical work beyond Phase 1 has also happened: ADRs 0004–0012, a torque budget
(`tools/pitch_inertia_budget.py`), and a parametric CAD model in `cad/` at the 70 x 182
direct-drive design point. See `docs/mechanical-requirements.md`.

## Awaiting the owner

Each is written to be accepted as-is; open items listed in each are parameters, not blockers.

- [x] ADR 0007 — bought power stage, own voltage-mode FOC. **Accepted 2026-09-30.**
- [ ] ADR 0009 — wire loop with software unwind (the only option the spine leaves)
- [ ] ADR 0010 — spine chassis, IMU 3.8 mm beside the axis
- [ ] ADR 0011 — DM3505 (SparkFun ROB-27477), one off-axis encoder, stator on the casing
- [ ] ADR 0012 — 3S battery, every board in the casing
- [x] `firmware/hal/design-proposal.md` — approved 2026-10-01 and implemented.
- [x] ADR 0013 — wheel velocity loop. **Accepted 2026-10-01**, all five as recommended.
- [x] ADR 0014 — safety state machine, comms watchdog, IWDG. **Accepted 2026-10-01**, all six as recommended.
- [x] ADR 0015 — MCU loop timing, IWDG driver, safety glue. **Accepted 2026-10-01**, all five as recommended.
- [x] ADR 0016 — STM32 register access: CMSIS headers only. **Owner choice 2026-10-01.**
- [ ] Boot-report encoding, Fault(NONE, 0x80000000 | reset flags) vs a schema code (implementation-notes, Phase 2.3)
- [ ] ADR 0014 implementation interpretations (4, each with a recommendation): implementation-notes.html, Phase 2.2
- [ ] Protocol proposals 2–7 (`protocol/design-proposal.md`): 6 assigns `0x03` as a target-angle
      command; 7 adds the wire loop's turn count. 3–5 and 7 batch with the estimator design.

---

## Phase 1 plan

### Goal

A repo where `firmware/core/` compiles and passes tests on the laptop, cross-compiles for
the G474RE in CI, and exchanges protocol frames with a simulated robot — all before any
real hardware is powered. This front-loads the portability firewall so it can't be breached
later by accident, and gives every subsequent control task a test harness to land in.

### Approach

Four independent slices, in dependency order. 1–3 are not owner-reviewed and proceed
directly. 4 is owner-reviewed and stops for approval before any code.

### Alternatives considered

- **Bring up STM32 first, refactor to portable core later.** Rejected: the firewall is the
  whole architectural bet, and retrofitting it after vendor headers spread is the single
  most expensive mistake available here.
- **Hand-written protocol structs instead of a generator.** Rejected by CLAUDE.md — schema
  is single source of truth. Also guarantees C++/Python drift the first time a field moves.
- **Single CMake target with an `#ifdef` toolchain switch.** Rejected: a real toolchain file
  is how we keep `firmware/core/` honest. CI proving both targets is the point.

---

## 1. Repo skeleton + git  [proceed directly]

- [x] 1.1 `git init`, `main` branch, `.gitignore` (build/, build-stm32/, __pycache__, .venv, *.o, *.elf)
- [x] 1.2 Directory tree exactly per CLAUDE.md "Repository layout"
- [x] 1.3 `README.md` per module directory: purpose, fit in architecture, key decisions,
      known limitations, how to test. Stubs are fine; empty is not.
- [x] 1.4 `docs/decisions/0001-languages-and-toolchain.md` — records the C++17 / Python 3.11 /
      CMake / GoogleTest / pytest choices already made in CLAUDE.md, so they have a citable why

**Done when:** tree matches the spec, every module dir has a README, `git log` has one commit.

## 2. Dual-target CMake  [proceed directly]

- [x] 2.1 Top-level `CMakeLists.txt`, `-DTARGET=host|stm32`, C++17, `-Wall -Wextra -Werror`
- [x] 2.2 `firmware/core/` as a target-independent static lib. `-fno-exceptions -fno-rtti`
      on both targets, so host tests exercise the same language subset as the target.
- [x] 2.3 `cmake/arm-none-eabi.cmake` toolchain file; G474RE flags (cortex-m4, hard float)
- [x] 2.4 GoogleTest via FetchContent, host-only. `ctest` wired up.
- [x] 2.5 One trivial core unit test, so the harness is proven rather than assumed

**Done when:** all four commands in CLAUDE.md run clean:
`cmake -B build -DTARGET=host && cmake --build build` / `ctest --test-dir build --output-on-failure`
/ `pytest` / `cmake -B build-stm32 -DTARGET=stm32 && cmake --build build-stm32`

**Note:** 2.3 needs `arm-none-eabi-gcc` installed locally. If it's missing, CI still proves
the cross-compile and local target builds are skipped with a clear message — not silently passed.

## 3. CI  [proceed directly]

- [x] 3.1 `.github/workflows/ci.yml` on every push: host build, ctest, pytest, target cross-compile
- [x] 3.2 A guard that fails if anything under `firmware/core/` includes a vendor or STM32 header.
      The firewall is a rule today; this makes it enforced.

**Done when:** all jobs green, and the guard is proven by a deliberate temporary violation
that turns CI red before being reverted.

## 4. Protocol schema + codec  [OWNER-REVIEWED — propose, then stop]

Write a design proposal before any code. Must cover:

- [x] 4.1 Schema format and the generator's language (schema lives in `protocol/`)
- [x] 4.2 Frame layout: COBS + CRC-16 + message ID + sequence + timestamp + protocol version.
      Which CRC-16 polynomial, and byte order — stated, with a reason.
- [x] 4.3 Initial message set: command, telemetry, parameter get/set, fault/state
- [x] 4.4 `uint32_t` microsecond timestamp wraparound handling (~71 min) — explicit, and tested
- [x] 4.5 Decoder resynchronization strategy for arbitrary garbage input, plus the fuzz harness
- [x] 4.6 `docs/decisions/` ADR for framing choice; `docs/theory/` not needed for this one

**Done when:** owner has approved the proposal. Implementation is a separate task.

## 5. Minimal simulator  [proceed directly, after 4]

- [x] 5.1 Simulated HAL implementations behind `firmware/hal/` interfaces (interfaces done; fakes in `tests/cpp/hal_fakes.hpp`)
- [x] 5.2 Placeholder plant model — enough to close a loop, no real dynamics yet.
      Real parameters come from system identification on hardware, much later.
- [x] 5.3 First SIL test: core runs against simulated HAL, frames round-trip through the codec

**Done when:** a SIL test passes in CI with no hardware attached.

**Done locally 2026-10-01:** 102 C++ tests (28 new: 25 sim unit + 3 SIL), 203 Python, purity
guard and STM32 build all green. CI green on `31b998f`. Loop closing deferred to
the first approved controller (see implementation-notes.html, slice 5).

### Slice 5 plan (written 2026-10-01)

Goal: a host-only `recon_sim` library that implements every `firmware/hal/` interface
against virtual time and a placeholder plant, plus SIL tests that push real `core` code
(the frame codec) through it. **No control law**: velocity PID, FOC, estimation and the
safety machine are owner-reviewed and unapproved, so nothing here closes a feedback loop.
"Close a loop" in 5.2 is therefore deferred to the first approved controller; the plant
is tested open-loop against its own analytic response.

- `sim/sim/sim_clock.hpp` — virtual `Clock`, advanced only by the test (wraps at 2^32).
- `sim/sim/sim_serial_link.{hpp,cpp}` — two `SerialPort` endpoints joined by a byte pipe
  rate-limited to 460 800 baud (docs/bringup/uart-link.md), fixed TX/RX buffers, RX
  overflow counted like a UART, seeded deterministic bit-error injection.
- `sim/sim/wheel_plant.{hpp,cpp}` — first-order duty -> wheel speed, exact discretisation,
  coast/brake decay, encoder counts with 2^32 wrap. `SimWheelMotor` clamps and applies an
  output limit per the interface contract. All constants are placeholders (gear ratio not
  chosen; no system ID), passed in, never hard-coded in core.
- `sim/sim/static_sensors.hpp` — IMU (level, still), absolute encoder, power monitor,
  pitch stage: settable constant samples. No pitch dynamics until the FOC design exists.
- `sim/sim/sim_world.hpp` — steps clock, link and plants at a fixed dt.
- Tests: unit tests per component; `test_sil_link.cpp` — DriveCommands at 50 Hz across
  the link decoded through `hal::SerialPort&` by `FrameDecoder`; same under bit errors
  (no wrong payload ever accepted, every loss counted); StateTelemetry back to the operator
  built from the simulated encoder; timestamps across the clock wrap.

Alternatives considered: Python plant (rejected — the point is running C++ `core`
unmodified); reuse `hal_fakes.hpp` (rejected — fakes deliberately have no physics or
clamping); real-time threads (rejected — non-deterministic tests).

---

Phase 2.1 (ADR 0013) and Phase 2.2 (ADR 0014) are done: see `completed.md`.

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

## Not now — deliberately deferred

Motor control, pitch stabilization, state estimation, safety state machine, watchdog, STM32
timer/IRQ/DMA config. All owner-reviewed, all blocked on Phase 1's test harness existing.
The pitch actuator is chosen (direct drive, ADR 0008; DM3505, ADR 0011 proposed); its HAL
interface waits on ADR 0007.

Out of scope per CLAUDE.md: ROS 2, autonomy, SLAM, LiDAR, custom PCB, self-righting.

## Open questions

- ~~Repo naming~~ RESOLVED 2026-09-30: keep `r6_drone` as the repo/dir name. "Recon UGV" stays
  the project name in docs and READMEs. No rename, no ADR needed.
- ~~Schema generator language~~ RESOLVED: Python, ADR 0003 (Accepted).
- ~~arm-none-eabi-gcc availability~~ RESOLVED 2026-09-30: installed Arm GNU Toolchain
  15.3.Rel1 via `brew install --cask gcc-arm-embedded` (includes gdb and newlib specs).
  CMake 4.4.3 also installed — it was missing too. Target build verified locally.

