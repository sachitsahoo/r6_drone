# 0014 — Safety state machine, comms watchdog and hardware watchdog

- **Status:** **ACCEPTED** by the owner, 2026-10-01: all six questions answered with the
  recommendation (see "Owner decisions" at the end). Owner-reviewed area: safety state
  machine, watchdog, plus protocol schema changes (new fault codes, Nack reasons, params) and
  one new HAL interface. Not yet implemented.
- **Inputs already decided by the owner:** wheels **coast** on a comms timeout and escalate
  to **brake** if the link stays down long enough (`firmware/hal/hal/wheel_motor.hpp`);
  the hardware watchdog (IWDG) is designed together with this machine (`tasks/todo.md`).
- **Fixed by earlier decisions:** the four states `DISARMED -> ARMED -> FAULT / ESTOP` and
  "motors can only move when ARMED" (CLAUDE.md); `SafetyStateRequest` and `EstopRequest`
  with magic constants, stale-timestamp rejection, `ParamCommit` only when DISARMED
  (`protocol/design-proposal.md`); the wheel integrators reset on leaving ARMED (ADR 0013).
- **Questions earlier ADRs left for this one:** does a comms timeout disarm or only stop
  the wheels (ADR 0013); what ENCODER_FAULT and LOOP_OVERRUN do (ADR 0013); a hard stop for
  the wire loop (ADR 0009); a current bound for voltage-mode FOC (ADR 0007). The last two
  are pitch-side and are assigned here, not solved (see "Deferred to the pitch design").

## Context

ADR 0013 built a wheel loop that obeys one input, `armed`, and never stops the motors
itself. Something now has to decide when `armed` is true, stop the motors when it is not,
and make sure the machine can't get stuck in a state where the motors run with nobody in
control. Three things can lose control, and each needs its own guard:

| Failure | Example | Guard |
|---|---|---|
| The operator goes away | Wi-Fi drop, laptop app crash, Pi relay crash | comms watchdog (software, 200 ms) |
| The software is alive but wrong | encoder unplugged, loop running late, sensor dead | fault detectors -> FAULT |
| The software stops running | hard fault, infinite loop, ISR storm | IWDG (hardware) -> reset |

One hardware fact shapes the last row. **The STM32 timers keep generating PWM when the CPU
stops.** A hung core leaves both wheels at their last duty until the IWDG resets the chip.
So the IWDG timeout is a distance the robot can drive with no software at all.

## Decision

### States and what each one allows

| State | Wheels | Pitch stage | Entered by | Left by |
|---|---|---|---|---|
| DISARMED | coast, loop not run | disabled (casing floats) | boot; operator request | arm request that passes the interlocks |
| ARMED | velocity loop drives | stabilizer runs (when it exists) | operator arm request from DISARMED | disarm request, any fault, e-stop |
| FAULT | coast, then brake after `fault_brake_delay_ms` | disabled (Q6: except COMMS_TIMEOUT) | any detector, from ARMED or DISARMED | operator clear request, once no fault is still active |
| ESTOP | **brake immediately** | disabled | `EstopRequest`, from any state | operator request to DISARMED |

Rules that follow from the table:

- **Boot is DISARMED**, except after a watchdog reset, which boots into FAULT (Q5).
- **No state reaches ARMED except DISARMED.** FAULT and ESTOP both go through DISARMED
  first, so re-arming after any trouble takes two deliberate operator actions. The operator
  app may put "clear" and "arm" on one button; the MCU still sees two requests and checks
  both.
- **ESTOP brakes; FAULT coasts first.** An e-stop is the operator saying "stop now", so
  shorting the motor terminals is right. A fault is the robot noticing something on its
  own, often while moving. Coasting first matches the owner's comms-timeout decision and
  avoids a violent stop on a guess. One rule for every fault is simpler to defend than
  one rule per fault code.
- **DISARMED coasts** with no brake escalation, so the robot can be pushed and carried by
  hand.
- **Integrators reset on every exit from ARMED** (ADR 0013). That already happens:
  `DriveLoop::step(false)` resets the controllers and rate limiters.

### Arm interlocks

`SafetyStateRequest(ARMED)` is accepted only if **all** of these hold. Otherwise the MCU
replies `Nack(ARM_INTERLOCK)` and sends a `Fault` frame naming the first condition that
failed in `context`, so the operator can see which one:

1. the state is DISARMED (from anywhere else: `Nack(NOT_DISARMED)`, which already exists);
2. no fault flag is latched;
3. the link is fresh: a valid frame arrived within `comms_timeout_ms`;
4. **the last DriveCommand is zero** (|v| < 0.01 m/s and |w| < 0.01 rad/s), or none has
   arrived since boot. Arming with the stick deflected would make the robot move the
   moment it arms. Rate limiting softens that but doesn't remove it.

These are cheap, they each stop a familiar failure, and they don't change normal use: an
operator with the stick centred never sees them.

### Comms watchdog

- **What feeds it** depends on the state. In DISARMED, any valid operator-to-robot frame
  feeds it: Heartbeat, DriveCommand, a param or state request. **In ARMED, only DriveCommand
  feeds it.** If the operator app's input thread dies while its heartbeat thread keeps
  running, Heartbeat-fed would keep the robot driving the last command forever. A
  DriveCommand stream that stops is exactly the failure to catch.
- **"Valid"** means the frame passed the full decoder chain plus stale-timestamp rejection.
  A rejected frame never feeds the watchdog.
- **Time is MCU receive time** (`Clock::now_us()`), never the frame's own timestamp, which
  comes from the operator's clock.
- **Timeout:** `comms_timeout_ms`, default **200** (CLAUDE.md). At the 50 Hz command rate
  that is 10 missed commands. A timeout while DISARMED doesn't count as a fault, because an
  idle robot with no laptop attached is normal. A timeout while ARMED latches COMMS_TIMEOUT
  (Q1).
- **Between commands** the MCU keeps acting on the last accepted DriveCommand until the
  timeout fires. That is what makes the timeout a bound on stale control.

### Fault detectors

Each detector latches one `FaultCode` bit in `fault_flags`, sends a `Fault` frame once on
the rising edge, and moves ARMED or DISARMED to FAULT. A flag stays latched until the
operator clears it, so a fault that came and went is still visible.

| Code | Detector | Default | Active in |
|---|---|---|---|
| COMMS_TIMEOUT | no feeding frame for `comms_timeout_ms` | 200 ms | ARMED |
| ENCODER_FAULT | estimator plausibility trip (ADR 0013, > 100 rad/s jump), **first occurrence** | — | ARMED, DISARMED |
| WHEEL_STALL (new) | output saturated **and** (\|speed\| < 1 rad/s **or** speed opposes duty) for `wheel_stall_ms` | 500 ms | ARMED |
| LOOP_OVERRUN | gap between two motor-loop iterations > `loop_overrun_fault_us` | 5000 µs | ARMED, DISARMED |
| WATCHDOG_RESET (new) | the IWDG reset flag was set at boot | — | boot |
| MOTOR_DRIVER_FAULT | `PitchPowerStage::fault()` (DRV8313 nFAULT) | — | ARMED, DISARMED |
| IMU_FAULT, OVERCURRENT, UNDERVOLTAGE | code and action defined here; **thresholds deferred** | — | — |

Why each one, and why these numbers:

- **WHEEL_STALL is the important new one.** If an encoder is unplugged, its count stops
  changing, so the estimator reports a *valid* speed of 0. The integrator then winds to the
  duty limit and the wheel runs at 0.3 duty with nothing controlling it. **Worse, centring
  the stick doesn't stop it:** with ref = 0 and measured = 0 the error is zero, so the
  integrator holds its wound-up value. The operator's only way out would be the e-stop.
  A reversed encoder (motor wired backwards) fails the same way, as positive feedback, so
  the detector also trips when speed opposes duty.
  - The detector only looks at *saturated* output, so it can't trip in normal driving.
  - It doesn't fire just because the motor is saturated. On carpet at full stick the loop
    saturates (ADR 0013: 0.48 duty needed against a 0.3 limit), but the wheel still turns at
    about 5 rad/s in the commanded direction, so the detector stays quiet.
  - It does trip when pushing into a wall. That's wanted: holding the duty limit into a
    wall heats the motor and achieves nothing.
  - 500 ms is about 10 closed-loop time constants (τ_cl = 20 ms, ADR 0013), so no
    legitimate transient from rest lasts that long at the limit. 1 rad/s is about 2 encoder
    counts per 10 ms window, safely above the 0.449 rad/s resolution.
- **ENCODER_FAULT latches on the first glitch.** ADR 0013's plausibility check needs a jump
  of more than 22 counts in 1 ms. A hardware quadrature counter (timer encoder mode) can't
  produce that from a few noisy edges; it means a wiring or ground fault. The loop already
  rides through the glitch itself for one step. Tolerating repeats would only hide a
  hardware problem until it got worse.
- **LOOP_OVERRUN at a 5 ms gap, not on the first late iteration.** The wheel loop measures
  its own `dt`, so one late iteration is not dangerous. Every overrun is still counted in
  `LoopTiming.overrun_count`. What matters is extra loop delay `T_d`, which costs
  `ωc · T_d` of phase margin: 5 ms × 50 rad/s = 0.25 rad = **14°**, taking ADR 0013's 74°
  to 60°. That is still comfortable. Beyond it, the loop runs on stale data. A loop that
  stops entirely is the IWDG's job, not this detector's.
- **The deferred thresholds** belong to designs that haven't been written. The UNDERVOLTAGE
  threshold is a battery parameter, which CLAUDE.md reserves for the owner. IMU_FAULT and
  OVERCURRENT depend on the estimator and the pitch stage. This ADR only fixes their action:
  FAULT, like every other code.

**Clearing.** In FAULT, `SafetyStateRequest(DISARMED)` clears every latched flag whose
condition is no longer active, and moves to DISARMED if none remain. If one is still active
(an encoder still glitching, an nFAULT still asserted), the MCU replies
`Nack(FAULT_ACTIVE)` and stays in FAULT. COMMS_TIMEOUT can always be cleared, because the
clear request itself proves the link is back.

### Wheel stop timing

On entering FAULT the wheels coast. After a further `fault_brake_delay_ms` the wheels
brake. Default: **800 ms**, so a lost link brakes 1.0 s after the last command.

Numbers on the placeholder plant (coast τ = 0.5 s, brake τ = 0.05 s, top speed
9 rad/s × 52.5 mm = 0.47 m/s):

| Policy | Distance from 0.47 m/s |
|---|---|
| brake immediately | 24 mm |
| coast 800 ms, then brake | 190 mm (speed has already fallen to 20%) |
| coast forever | 235 mm |

On flat ground, the brake after 800 ms changes the stopping distance very little; coasting
has already done most of the work by then. The brake is there for the **slope**. On a 10°
slope a free-rolling robot accelerates at g·sin 10° = 1.7 m/s², so 800 ms of free rolling
from rest covers 0.54 m. [UNCLEAR] The real figure depends on how hard the N20 gearboxes are
to back-drive, which may be enough to stop them rolling at all. Measure it at bring-up, and
the delay is a param.

### Hardware watchdog (IWDG)

- **Timeout: 50 ms** (`kIwdgTimeout_ms`, a build constant, *not* a param: a watchdog that
  can be disabled over the radio is not a watchdog). At 0.47 m/s a hung core drives
  **24 mm** before the reset, the same as an immediate brake. The G474's IWDG runs from the
  32 kHz LSI with a 12-bit reload. [UNCLEAR] The LSI is only accurate to roughly ±10%
  (check the G474 datasheet), so the code configures for 50 ms and the analysis assumes
  anywhere from 45 to 55 ms.
- **Fed only when every loop has checked in.** Each periodic task sets its bit in a
  check-in mask: the 1 kHz motor loop, the main loop with the safety tick, and the pitch loop
  once it exists. The main loop feeds the IWDG only when all bits are set, then clears them.
  A stuck ISR with a healthy main loop, or the other way round, therefore still resets the
  chip. Feeding from a timer interrupt alone would keep a hung main loop alive forever.
- **The safety machine is checked by its own outputs, not by the watchdog.** If the
  supervisor runs but decides wrongly, the IWDG can't see it. That's what the transition
  tests are for.
- **Frozen under the debugger** (`DBGMCU` IWDG stop bit), so a breakpoint doesn't reset the
  board. Release builds are unaffected.
- **After the reset**, every pin returns to its reset state (a floating input). The motor
  drivers must see that as "off", which is a **hardware requirement**: pull-downs on the
  TB6612 STBY and PWM inputs, and on the DRV8313 enable lines. [UNCLEAR] Check whether the
  SimpleFOCMini already fits them. The firmware then boots into FAULT(WATCHDOG_RESET)
  (Q5).
- **ParamCommit must not starve it.** Flash erase on the G474 can take tens of ms. The
  commit routine has to run from the other flash bank, or feed between page operations.
  That's the param-table design's problem; it's recorded here so it isn't missed.
- **Rejected:** the window watchdog (WWDG). It also catches a loop running *too fast*, a
  failure this design has no mechanism for, and it costs a second clock domain to reason
  about.

### Where it lives

- `firmware/core/safety/`: pure logic, testable on a laptop.
  - `SafetySupervisor` is stepped at 1 kHz in the motor loop, immediately before
    `DriveLoop::step(armed)`. It takes its inputs as a struct (requests, frame-receipt
    events, detector flags, `now_us`) and returns outputs: state, `fault_flags`, wheel stop
    mode, pitch enable, and any Fault or Nack frames to send. It never touches the HAL, so
    every transition can be tested from a table.
  - `CheckInMonitor` keeps the IWDG check-in mask. It is also pure, and returns "feed / don't
    feed".
- The glue applies the outputs: `DriveLoop::step(state == ARMED)`, then
  `WheelMotor::stop(mode)` when not ARMED, then `PitchPowerStage::set_enabled(...)`. The
  glue is the slice that also wires DriveLoop and the param table.
- **New HAL interface `Watchdog`** with `feed()` and `bool reset_was_watchdog() const`.
  The STM32 version wraps the IWDG and the RCC_CSR reset flags. The sim version records feeds
  and lets a test request a "watchdog reset" boot.
- **E-stop latency:** frames are decoded in the main loop, so an `EstopRequest` acts within
  one main-loop iteration plus one motor-loop tick, which should be well under 2 ms.
  [UNCLEAR] The main-loop rate isn't fixed yet; the timer design sets it.

### Schema changes in this ADR (protocol version stays 1)

Nothing is deployed, so additions don't bump the version (same reasoning as the drive
params in ADR 0013).

- `FaultCode`: add `WHEEL_STALL = 8`, `WATCHDOG_RESET = 9`. Reserve `WIRE_LOOP_LIMIT = 10`
  for ADR 0009. `fault_flags` bit `n-1` is code `n`, so codes 1–16 fit the u16.
- `NackReason`: add `ARM_INTERLOCK = 10` and `FAULT_ACTIVE = 11`.
- Params, safety block `0x0200`:

  | ID | Name | Type | Default | Range | Source |
  |---|---|---|---|---|---|
  | 0x0200 | `comms_timeout_ms` | u16 | 200 | [50, 1000] | CLAUDE.md; floor = 2.5 command periods |
  | 0x0201 | `fault_brake_delay_ms` | u16 | 800 | [0, 5000] | coast to 20% speed on the placeholder plant (above) |
  | 0x0202 | `wheel_stall_ms` | u16 | 500 | [100, 2000] | ~10 × τ_cl (ADR 0013) |
  | 0x0203 | `loop_overrun_fault_us` | u16 | 5000 | [1500, 10000] | 14° of phase margin at ωc = 50 rad/s |

  The IWDG timeout is deliberately left out of this table.

### Deferred to the pitch design (recorded so they aren't lost)

- **Wire-loop hard stop (ADR 0009):** a software turn-count limit that latches
  WIRE_LOOP_LIMIT. Its limit and the unwind logic belong with the pitch loop and the turn
  count (protocol proposal 7).
- **FOC current bound (ADR 0007):** voltage-mode FOC can't limit current, so the commanded
  voltage has to be bounded by a param. The OVERCURRENT threshold comes with it.
- **IMU_FAULT detector:** belongs with the estimator.

## Test plan (written first, at implementation)

- **Transition table test:** every (state, event) pair, all 4 states × every request,
  e-stop, and detector event, with the expected next state, Nack or Fault frame, and outputs.
  An unlisted pair fails the test. That covers CLAUDE.md's "test every transition"
  mechanically rather than by memory.
- Interlocks: each condition refused on its own, with the right Nack and `context`.
- Comms watchdog: Heartbeat doesn't feed it in ARMED; rejected and stale frames don't feed
  it; timing is exact at 199/200/201 ms; the receipt clock is used across a `uint32_t` wrap.
- Escalation: coast at entry, brake at exactly `fault_brake_delay_ms`; ESTOP brakes at once.
- **SIL:** link cut in `SimSerialLink` → coast then brake on the right schedule; an encoder
  frozen in `SimWheel` (new test-only option) → WHEEL_STALL within 500 ms + one window, and
  the robot stops; a reversed encoder → WHEEL_STALL; full stick on the carpet plant → no
  fault.
- `CheckInMonitor`: no feed if any bit is missing; bits clear after a feed.
- Boot after a simulated watchdog reset → FAULT(WATCHDOG_RESET), and clearing works.

## Consequences

- A Wi-Fi blip longer than 200 ms means clear and re-arm (Q1). That's the price of a robot
  that never starts moving by itself when the link comes back.
- One more hardware requirement: pull-downs on the driver enable and PWM lines.
- The supervisor adds about 1 µs to the 1 kHz loop (an estimate; it will be measured and
  reported in LoopTiming).
- CLAUDE.md's "motors can only move when ARMED" gets one explicit, documented exception if
  Q6 is accepted.

## Questions for the owner

1. **Does a comms timeout disarm the robot?** Options: (a) latch FAULT(COMMS_TIMEOUT): the
   wheels coast then brake and the integrators reset, and re-arming needs a clear plus an
   arm; or (b) stay ARMED with the wheels stopped, and resume driving by itself when
   commands return. *Recommended: (a).* With (b), the robot starts moving on its own when
   Wi-Fi reconnects, at whatever the stick says at that moment, and the operator may not be
   looking. (b) would also coast "while ARMED", which breaks the meaning of ARMED.
2. **Brake escalation delay:** 800 ms after the fault (1.0 s after the last command on a
   link loss)? Shorter limits roll-away on a slope; longer stops more gently on flat ground.
   *Recommended: 800 ms, re-measured at bring-up* once gearbox back-drive is known.
3. **Wheel sensing faults:** latch ENCODER_FAULT on the first plausibility glitch, and add
   WHEEL_STALL (saturated and not moving, or moving the wrong way, for 500 ms)?
   *Recommended: yes to both.* WHEEL_STALL is the only thing that catches an unplugged
   encoder, which otherwise causes a runaway that centring the stick can't stop.
4. **LOOP_OVERRUN:** latch FAULT only when the loop gap exceeds 5 ms, and count every
   smaller overrun without acting on it? The alternative is to latch on the first missed
   deadline, which is stricter but trips on harmless jitter. *Recommended: 5 ms.*
5. **IWDG:** a 50 ms timeout, fed only when every loop has checked in, and boot into
   FAULT(WATCHDOG_RESET) after a watchdog reset rather than quietly into DISARMED?
   *Recommended: yes.* A silent reset hides a crash. Booting into FAULT makes the operator
   see that it happened.
6. **Does the pitch stabilizer keep running after a comms timeout?** CLAUDE.md allows it
   ("stabilizer may keep running"). The only cost is an explicit exception to "motors only
   move when ARMED", limited to COMMS_TIMEOUT. Every other fault, DISARMED and ESTOP disable
   the pitch stage. *Recommended: yes, keep it running.* A lost link is the most common
   fault. Dropping the casing would swing it loose on a coasting robot, and the camera is
   already level when the link comes back. Nothing is built on this yet; it binds the pitch
   design.

## Owner decisions (2026-10-01)

Accepted as written: "0014 all makes sense to me." All six questions take the recommended
answer: (1) a comms timeout latches FAULT; (2) brake 800 ms after the fault; (3) ENCODER_FAULT
on the first glitch plus WHEEL_STALL; (4) LOOP_OVERRUN at a 5 ms gap; (5) a 50 ms IWDG fed on
check-in, booting into FAULT(WATCHDOG_RESET); (6) the pitch stabilizer keeps running through
COMMS_TIMEOUT only.
