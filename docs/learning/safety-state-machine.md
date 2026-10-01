# Learning note — the safety state machine

A plain-language walkthrough of [ADR 0014](../decisions/0014-safety-state-machine.md) and
[`firmware/core/safety/`](../../firmware/core/safety/). The derivations are in
[`docs/theory/safety-state-machine.md`](../theory/safety-state-machine.md).

## The job

The wheel loop (ADR 0013) does exactly what it's told, a thousand times a second. Something
else has to decide *whether* it should be running at all, and stop the motors when nobody is
in control. Control can be lost in three ways, and each has its own guard:

| What went wrong | Example | Guard |
|---|---|---|
| The operator went away | Wi-Fi dropped, laptop app crashed | comms watchdog, 200 ms |
| The software is running but wrong | encoder unplugged, loop running late | fault detectors → FAULT |
| The software stopped running | hard fault, infinite loop | hardware watchdog (IWDG), 50 ms → reset |

## Four states

- **DISARMED.** Where the robot boots. The wheels are free, so you can push it by hand. Nothing
  moves.
- **ARMED.** The only state in which the wheels are driven. You get here only from DISARMED,
  only by asking, and only if the arm checks pass.
- **FAULT.** The robot noticed a problem on its own. The wheels coast for 0.8 s, then brake.
  Every fault stays *latched*, still visible even after the problem goes away, until the
  operator clears it.
- **ESTOP.** The operator said "stop now". The wheels brake at once.

Getting back to ARMED after any trouble always takes two separate requests, "clear" and then
"arm". FAULT and ESTOP never go straight to ARMED.

## How one tick works (every 1 ms)

`SafetySupervisor::step()` takes a struct of everything that happened this millisecond (frames
received, detector readings, the time) and returns a struct of decisions: the state, which
flags are latched, whether the wheel loop may drive, how to stop the wheels, whether the
pitch motor may run, and any Fault or Nack frames to send. It never touches hardware itself.
That's what makes it testable: a test can feed it any situation and check the answer.

Inside the tick it goes: e-stop first, then the fault detectors, then the comms watchdog, then
the operator's request. That order means a "same-millisecond" race always resolves safely.
Ask to arm in the tick a fault appears, and the request finds the robot already in FAULT.

## The detectors, briefly

- **COMMS_TIMEOUT.** No DriveCommand for 200 ms while ARMED. Heartbeats don't count while
  ARMED. If the app's joystick thread crashed but its heartbeat thread kept going, a
  heartbeat-fed watchdog would keep the robot driving forever.
- **WHEEL_STALL.** The new one, and the most interesting. The motor is at its duty limit **and**
  the wheel is either not turning or turning the wrong way, for 500 ms. That catches an
  unplugged encoder, which otherwise causes a runaway that centring the stick can't stop
  (question 3 explains why).
- **ENCODER_FAULT.** The speed estimate jumped impossibly far in one step: a wiring fault.
  Latched on the first one.
- **LOOP_OVERRUN.** The motor loop went more than 5 ms between iterations.
- **MOTOR_DRIVER_FAULT.** The pitch driver raised its fault pin.
- **WATCHDOG_RESET.** The chip was reset by the IWDG. The robot boots into FAULT so you find
  out it crashed.

## The hardware watchdog

The STM32's timers keep making PWM even if the CPU stops. A crashed robot keeps driving at its
last duty until something resets the chip. The IWDG is a separate countdown, on its own
clock, that resets the chip unless it is "fed" every 50 ms. The trick is *who* feeds it.
Each loop sets a bit saying "I ran" (`CheckInMonitor`). The main loop feeds the IWDG only when
every bit is set. If either loop hangs, the bits never all get set, the IWDG runs out, and the
chip resets.

## Check your understanding

1. **Why does a comms timeout latch FAULT instead of just pausing the wheels until commands
   come back?** If driving resumed automatically, the robot would start moving the moment
   Wi-Fi reconnected, at whatever the stick said then, maybe with the operator looking away.
   A robot should never start moving by itself. (ADR 0014 Q1.)

2. **An unplugged encoder reads a perfectly valid 0. Why does that cause a runaway, and why
   doesn't centring the stick stop it?** With the stick pushed, the error (target minus
   measured 0) is positive, so the integrator winds up to the duty limit and the wheel runs
   flat out. Centre the stick and the target is 0, the "measurement" is 0, so the error is 0.
   The P term vanishes and the integrator *stops changing*, but it holds its wound-up value.
   The motor keeps running. Only WHEEL_STALL, or the e-stop, gets you out.

3. **Why doesn't WHEEL_STALL trip at full stick on carpet, where the motor really is
   saturated?** Because the wheel is still turning the right way, at about 5 rad/s. The
   detector needs saturation *and* a wheel that's (nearly) stopped or going backwards.
   Saturation on its own is just "asking for more than the motor can give".

4. **Why coast first on a fault, but brake at once on an e-stop?** An e-stop is the operator
   deciding to stop *now*, so the hardest stop is right. A fault is the robot guessing that
   something is wrong, often while moving. Coasting avoids a violent stop on a guess, and
   after 0.8 s it has already cut the speed to 20%. The brake that follows is mainly to stop it
   rolling down a slope.

5. **The watchdog compares times with `elapsed_us`, which breaks after 71.6 minutes. A robot
   can sit DISARMED for hours. How does arming stay safe?** Freshness is a latch. The first
   time the silence reaches 200 ms, `link_fresh_` goes false, and only a new frame sets it
   back. The supervisor runs every millisecond, so it always sees the 200 ms crossing; it never
   has to measure a 71-minute gap.

6. **Why is the IWDG timeout a build constant and not a parameter like the others?** A
   watchdog you can change or disable over the radio isn't a watchdog. It has to keep working
   when everything else, including the parameter code, has gone wrong.

7. **Why not feed the IWDG from the 1 kHz timer interrupt? It's the most regular thing in the
   system.** Because it keeps firing even when the main loop has hung. The robot would stay
   "alive" with no frame decoding and no e-stop handling. Feeding only when *every* loop has
   checked in means any one of them hanging resets the chip.

8. **The pitch stabilizer keeps running in FAULT after a comms timeout, and that alone. Isn't
   that a motor moving outside ARMED?** Yes, and it's the one documented exception (ADR 0014
   Q6). Losing the link is the most common fault, and the robot is coasting. If the casing
   were dropped, it would swing loose. Every other fault turns the pitch stage off, because a
   fault could mean the pitch hardware itself is wrong.

9. **What stops a frozen-encoder robot if the operator never presses anything?** WHEEL_STALL
   latches about 0.5 s after the wheel saturates, the wheels coast, and 0.8 s later they brake.
   The SIL test checks both wheels are at rest 2.5 s after the cable is pulled.

10. **Why does the supervisor measure LOOP_OVERRUN from the gap between its own steps, instead
    of reading a timer?** It's called once per motor-loop iteration, so the gap between two of
    its calls *is* the motor-loop gap. Measuring it that way needs no hardware, so the module
    stays pure and testable.
