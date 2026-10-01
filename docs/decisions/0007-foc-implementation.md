# 0007 — FOC: bought power stage, own control code

- **Status:** **Accepted** (2026-09-30, by the owner). Implementation is still
  owner-reviewed: the FOC design and the timer configuration each get a proposal first.
  The operating point below was first written for ADR 0006's belt and was recomputed for the
  DM3505 before acceptance.
- **Related:** [0008](0008-reduced-scale-direct-drive.md) (direct drive),
  [0011](0011-pitch-motor-and-encoder.md) (the motor and its single encoder),
  [0012](0012-power-and-electronics-placement.md) (the 3S supply this stage needs)

## Context

The pitch actuator is a brushless motor, so something has to commutate it. "Do we need a
FOC driver board?" hides a fork, because the phrase covers two products that sit on
opposite sides of the architecture:

1. **A power stage.** Three half-bridges and a gate driver. Takes three PWM signals plus an
   enable, switches the phases, and has no idea what FOC is. SimpleFOCMini v1 is this: a
   TI DRV8313, 8–30 V, 2.5 A per phase, 3-PWM mode, no current sensing, no MCU.
2. **A motor controller.** Has its own microcontroller running the current and position
   loops. Takes a target angle or velocity over a link. ODrive and the STORM32 gimbal boards
   are this.

## Decision

**Buy a power stage. Write the FOC ourselves, on the G474.**

Specifically: a DRV8313-class 3-PWM module (SimpleFOCMini v1 or equivalent, roughly $1–8),
driven by TIM1 in complementary mode with dead-time, with the control code living in
`firmware/core/`.

## Why not a motor controller

- **It moves the loop off our MCU.** `CLAUDE.md` puts pitch stabilization on the MCU at
  500 Hz–1 kHz. A separate controller puts a serial link *inside* that loop, adding latency
  and jitter to the one loop this project exists to characterise.
- **It is undefendable.** The owner must be able to explain every design decision. "The
  board did it" is the opposite of that, and the research question is specifically about how
  well the stabilization performs — attributing performance to a black box is not a result.
- **The G474 is a motor-control part.** TIM1 is an advanced-control timer with three
  complementary outputs and hardware dead-time insertion; the part also carries five ADCs,
  comparators, and op-amps precisely for this. Buying a second MCU to do what this one was
  chosen for is strange.

## Why not build the power stage either

`CLAUDE.md` puts custom PCBs out of scope, and rightly: a DRV8313 module costs a few dollars
and a mistake in a hand-built gate drive stage destroys motors and MOSFETs. The power stage
is the one part of this system with no research content — it is a commodity.

## The operating point makes this much easier than it sounds

FOC has a reputation for complexity that mostly comes from current control. At our operating
point that complexity is unnecessary. Figures from ADR 0011's budget, for a trimmed casing:

| | Value |
|---|---|
| Current at the design point | **0.22–0.32 A** (17.8–25.8 mN m at Kt = 0.08 N m/A) |
| Driver rating | 2.5 A — **7.7–11.2x margin** |
| Electrical frequency at peak slew | **6.1 Hz** (11 pole pairs, casing at 3.5 rad/s, no reduction) |
| A 20 kHz FOC loop oversamples that by | **over 3000x** |
| TIM1 resolution at 20 kHz from 170 MHz | 8500 counts, **13.1 bits** |

Direct drive made the electrical frequency *five times lower* than under the belt (31 Hz),
which makes commutation timing even less demanding. It also made the current several times
higher, which is the half that needs watching: see the consequences.

Gimbal motors are deliberately high-resistance — the DM3505 is 6.3 Ω — so at a third of an amp a
current loop regulates little that the winding resistance does not already set. **Voltage-mode FOC**, where a voltage vector is
applied at the correct electrical angle without measuring current, is the standard approach
for gimbal motors and is what SimpleFOC itself defaults to for them.

That removes shunt resistors, current amplifiers, and ADC-to-PWM synchronisation — most of
what makes FOC hard to get right.

## What we would actually write

```
rotor angle -> electrical angle -> inverse Park -> SVPWM -> three duty cycles
```

**This is a pure function**, which matters more here than it might elsewhere: it takes an
angle and two voltage components and returns three numbers. No peripherals, no state, no
vendor headers. It belongs in `firmware/core/motor/` and is fully testable on a laptop —
exactly what the portability firewall exists for. `firmware/stm32/` does TIM1 configuration
and writes the duty cycles.

Expect a few hundred lines. Known-answer tests are straightforward: SVPWM has published
waveforms, and the inverse Park transform is two lines of trigonometry with an analytic
check at every 30 degrees.

## Options considered for the control code

1. **Write it (chosen).** Small, testable in `firmware/core/`, and defendable line by line.
2. **SimpleFOC library.** Excellent and well documented, but it is Arduino-framework C++.
   Using it means an Arduino core inside a CMake build that currently has none, and it would
   not sit in `firmware/core/` without dragging platform code across the firewall. Its value
   here is as a *reference implementation* to check our maths against, not as a dependency.
3. **ST MotorControl SDK (X-CUBE-MCSDK).** Generates a complete FOC stack for STM32 including
   G4. Rejected: it is a code generator that wants to own the project structure, it is built
   around current-sensed FOC we do not need, and generated code we cannot explain fails the
   project's central requirement.

## Consequences

- **Motor control is an owner-reviewed area**, so this ADR and the FOC design that follows
  both need approval before implementation.
- **No current sensing means no torque measurement.** For a research project that is a real
  loss, and direct drive made it a bigger one. Measured current would give measured torque,
  which would characterise bearing and wire-loop drag (ADR 0009) and show how much of the
  budget the gravity holding term actually takes (ADR 0011). SimpleFOCMini **v2.3** (DRV8316,
  8 A, three-phase low-side sensing at 150 mV/A) adds that for a few dollars and a larger
  board. **Worth considering on research grounds even though the control loop does not need it.**
- **The stage needs at least 8 V**, which is what forces a 3S battery (ADR 0012).
- The DRV8313 module's onboard 3.3 V LDO supplies only 10 mA. It must not power the MCU,
  the Pi, or the sensors.
- Voltage-mode FOC has no protection against a stalled motor drawing its full winding
  current. A current limit must come from somewhere — either the supply, or by bounding the
  commanded voltage — and that belongs in the safety state machine design.
- Commutation needs rotor angle, and with direct drive the rotor angle *is* the casing
  angle. **One encoder serves both** FOC and the estimator (ADR 0011). It has to be read
  off-axis, because wheel A's shaft runs through the motor.

## What remains open after acceptance

None of these needs hardware in hand, and none changes the decision.

1. **Pole pairs:** settled. The DM3505 datasheet gives 11 (ADR 0011).
2. **PWM frequency and dead-time:** STM32 timer configuration, owner-reviewed in its own right
   when that code is written.
3. **v1 or v2.3:** whether to buy the SimpleFOCMini with current sensing, on research grounds
   (see the consequences). The control loop doesn't need it. The hardware interface will
   leave room for an optional current reading, so the board can be chosen at purchase time.
