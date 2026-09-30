# 0007 — FOC: bought power stage, own control code

- **Status:** **PROPOSED — awaiting owner approval. Do not implement.**
- **Related:** [0006](0006-pitch-actuator.md) (the 2208 motor and 8:1 reduction),
  [0005](0005-imu-placement.md) (two sensors, not one)

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
point that complexity is unnecessary:

| | Value |
|---|---|
| Current required at the motor | **0.084 A** (8.9 mN m through `Kt = 9.549/KV`) |
| Driver rating | 2.5 A — **30x margin** |
| Electrical frequency at peak slew | **31 Hz** (7 pole pairs, casing at 3.5 rad/s through 8:1) |
| A 20 kHz FOC loop oversamples that by | **643x** |
| TIM1 resolution at 20 kHz from 170 MHz | 8500 counts, **13.1 bits** |

Gimbal motors are deliberately high-resistance — low KV means many turns — so at 84 mA a
current loop regulates almost nothing. **Voltage-mode FOC**, where a voltage vector is
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
  loss: measured current would give measured torque, which would let us characterise friction
  and quantify what the actuator is actually doing. SimpleFOCMini **v2.3** (DRV8316, 8 A,
  three-phase low-side sensing at 150 mV/A) adds that for a few dollars and a larger board.
  **Worth considering on research grounds even though the control loop does not need it.**
- The DRV8313 module's onboard 3.3 V LDO supplies only 10 mA. It must not power the MCU,
  the Pi, or the sensors.
- Voltage-mode FOC has no protection against a stalled motor drawing its full winding
  current. A current limit must come from somewhere — either the supply, or by bounding the
  commanded voltage — and that belongs in the safety state machine design.
- Commutation needs rotor angle. ADR 0005 already establishes that this is a *different*
  sensor from the casing-angle encoder the control loop uses.

## What must be settled before this is accepted

1. Confirm the motor's pole pair count on arrival (assumed 7, i.e. 12N14P).
2. Decide whether current sensing is worth the larger board, on research grounds rather than
   control grounds.
3. Agree the PWM frequency and dead-time, which is STM32 timer configuration and therefore
   owner-reviewed in its own right.
