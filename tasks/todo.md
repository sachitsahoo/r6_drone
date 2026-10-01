# Recon UGV — tasks

Phase 1: repo skeleton, build system, CI, protocol schema and codec, minimal simulator.
Plan written 2026-09-30. Spec of record: ../CLAUDE.md

**Slices 1–4 complete and verified 2026-09-30.** Slice 5 (simulator) is blocked on the
`firmware/hal/` interfaces, which are owner-reviewed.

Mechanical work beyond Phase 1 has also happened: ADRs 0004–0012, a torque budget
(`tools/pitch_inertia_budget.py`), and a parametric CAD model in `cad/` at the 70 x 182
direct-drive design point. See `docs/mechanical-requirements.md`.

## Awaiting the owner

Each is written to be accepted as-is; open items listed in each are parameters, not blockers.

- [ ] ADR 0007 — bought power stage, own voltage-mode FOC. **Unblocks the pitch HAL interface.**
- [ ] ADR 0009 — wire loop with software unwind (the only option the spine leaves)
- [ ] ADR 0010 — spine chassis, IMU 3.8 mm beside the axis
- [ ] ADR 0011 — GM2804H, one off-axis encoder, sideways CoM trimmed to <= 2 mm
- [ ] ADR 0012 — 3S battery, every board in the casing
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

- [ ] 5.1 Simulated HAL implementations behind `firmware/hal/` interfaces
- [ ] 5.2 Placeholder plant model — enough to close a loop, no real dynamics yet.
      Real parameters come from system identification on hardware, much later.
- [ ] 5.3 First SIL test: core runs against simulated HAL, frames round-trip through the codec

**Done when:** a SIL test passes in CI with no hardware attached.

---

## Not now — deliberately deferred

Motor control, pitch stabilization, state estimation, safety state machine, watchdog, STM32
timer/IRQ/DMA config. All owner-reviewed, all blocked on Phase 1's test harness existing.
The pitch actuator is chosen (direct drive, ADR 0008; GM2804H, ADR 0011 proposed); its HAL
interface waits on ADR 0007.

Out of scope per CLAUDE.md: ROS 2, autonomy, SLAM, LiDAR, custom PCB, self-righting.

## Open questions

- ~~Repo naming~~ RESOLVED 2026-09-30: keep `r6_drone` as the repo/dir name. "Recon UGV" stays
  the project name in docs and READMEs. No rename, no ADR needed.
- ~~Schema generator language~~ RESOLVED: Python, ADR 0003 (Accepted).
- ~~arm-none-eabi-gcc availability~~ RESOLVED 2026-09-30: installed Arm GNU Toolchain
  15.3.Rel1 via `brew install --cask gcc-arm-embedded` (includes gdb and newlib specs).
  CMake 4.4.3 also installed — it was missing too. Target build verified locally.

