# 0013 — Wheel velocity loop: PI with feedforward, windowed encoder speed

- **Status:** **ACCEPTED** by the owner, 2026-10-01: all five questions answered with
  the recommendation (see "Owner decisions" at the end). Owner-reviewed area: motor control,
  plus a protocol schema change for the gains.
- **Amended 2026-10-01 (owner):** feedforward defaults to `kff = 0`. The analysis below
  claimed a first-order closed loop with feedforward included; that was wrong, and the SIL
  test caught it. See "Amendment: feedforward off by default" at the end. Original text is
  kept with callouts beside each affected passage.
- **Related:** [0004](0004-pitch-axis-architecture.md) (drive and pitch are coupled),
  [0008](0008-reduced-scale-direct-drive.md) (105 mm wheels, 214 mm overall),
  `firmware/hal/hal/wheel_motor.hpp`, `firmware/hal/hal/wheel_encoder.hpp`,
  `sim/sim/wheel_plant.hpp` (the plant this will first be closed against)

## Context

Phase 1 left a simulator with a placeholder wheel plant and no loop closed on it. The first
controller to design is the wheel velocity loop. It is the simplest loop on the robot, it is
needed before anything moves, and the pitch stabilizer will later treat it as a
disturbance source (ADR 0004, direction 1), so its behaviour should be pinned down first.

What it has to do: take a `DriveCommand` (body linear speed `v`, yaw rate `w`) at ~50 Hz,
turn it into two wheel speed references, and hold each wheel at its reference by setting a
PWM duty at 1 kHz (CLAUDE.md loop rates), reading only a wrapping quadrature counter.

Two facts shape everything below.

**1. The encoder is coarse at 1 kHz.** With the placeholder 1400 counts/rev (gear ratio not
chosen yet), one count per 1 ms sample is `2*pi/1400/0.001 = 4.49 rad/s`, which is **0.236 m/s**
at the 52.5 mm wheel radius. Top speed at the default 0.3 duty limit is about 9.4 rad/s, which
is only ~2 counts per sample. A speed measured by differencing consecutive 1 kHz samples is
mostly quantisation noise.

**2. The plant is unidentified.** The sim's first-order model (31.4 rad/s at full duty,
τ = 50 ms) is a placeholder. Whatever is chosen has to be tunable from system-ID data
without changing its structure.

## Decision

### Block diagram

```
DriveCommand (v, w) ──► diff-drive ──► accel ──► w_ref ─┬──► kff ───────────────┐
      50 Hz           kinematics     limiter            │                      ▼
                     + saturation                       └─►(+)─► PI ────────►(+)─► clamp ─► WheelMotor
                                                           ▲-                         ±limit   set_duty
                                                           │                                │
                                          w_meas ◄── windowed count difference ◄── WheelEncoder
```

One instance per wheel. The kinematics and the limiter are shared.

### Speed measurement: windowed count difference

`w_meas = (count[k] - count[k-N]) / counts_per_rev * 2*pi / (N * dt)`, with `N = 10`
samples (10 ms) at 1 kHz. Counts are kept in an 11-entry ring buffer of `uint32_t`, and the
difference uses unsigned subtraction reinterpreted as `int32_t`, so the 2^32 wrap is handled
by construction (hard rule 7). The interval is measured from real timestamps
(`core::elapsed_us`), not assumed to be `N * 1 ms`, so a late loop iteration does not
become a speed error.

- Resolution: 0.449 rad/s (0.024 m/s), ten times finer than single-sample differencing.
- Cost: a pure delay of N/2 = 5 ms. That sets the bandwidth ceiling (below).
- Plausibility check: a count step larger than the most the wheel could physically turn in
  one sample (`max_wheel_speed * dt * margin`) raises `ENCODER_FAULT`, which already exists in
  the protocol's fault enum. What the safety state machine does with it is its own design.

Rejected alternatives:
- **Single-sample differencing plus a low-pass filter.** Roughly equivalent noise and lag,
  but the filter's corner is one more parameter to explain, and a moving window has an exact,
  finite memory.
- **Timer input capture (time between edges).** Best low-speed resolution. But it needs
  STM32 timer configuration (owner-reviewed), a change to `hal::WheelEncoder`, and special
  handling near zero speed where edges stop arriving. Worth revisiting if system ID shows
  the 10 ms window limits performance. The HAL change would be additive.
- **Higher gear ratio.** Improves resolution directly and is still open (docs/bom.md). This
  design works at any ratio; `N` is a parameter.

### Controller: PI plus feedforward, no derivative

Positional form, per wheel, at `dt` = 1 ms:

```
e      = w_ref - w_meas
u_ff   = kff * w_ref
u_pi   = kp * e + I
u      = clamp(u_ff + u_pi, -duty_limit, +duty_limit)
I     += ki * e * dt      only if that does not push u further into saturation
```

- **Feedforward** `kff ≈ 1 / (no-load speed per unit duty)` does most of the work: it is the
  steady-state duty for the requested speed. That means the PI only corrects model error and
  disturbances, so its gains can stay modest and quantisation noise stays out of the output.

  > **Amended 2026-10-01:** wrong as stated. With the PI zero cancelling the plant pole, the PI
  > alone already gives the ideal first-order response. Feedforward on top double-counts,
  > adding a closed-loop zero and ~12% step overshoot. It does not lower kp, and it does not
  > reduce quantisation noise. Default is now `kff = 0`; see the amendment at the end.
- **Integral** removes steady-state error from friction, load, battery sag, and the steady
  part of the pitch actuator's reaction torque (ADR 0004).
- **No derivative.** The derivative of a quantised speed is noise, and a first-order plant
  does not need phase lead. Adding one later is a gain set to non-zero, not a redesign. This
  ADR still declines to add a `kd` parameter, because an unused parameter is an unexplained
  one.
- **Anti-windup by conditional integration.** The integrator freezes when the output is
  saturated and the error would push it further in. That is simpler to explain and test than
  back-calculation, and it needs no extra gain.

**When the integrator resets.** Only on leaving `ARMED`. Outside `ARMED` it is held at 0
and the loop does not run, so a value wound up during a slip or before a fault never drives
the next arm. In every other case it is kept or frozen:

| Situation | Integrator |
|---|---|
| DISARMED / FAULT / ESTOP | reset to 0 |
| output saturated, error pushing further | frozen (anti-windup) |
| zero command while ARMED | kept: holds the robot on a slope |
| direction reversal, surface change | kept: the error drains or rebuilds it |
| comms timeout | decided by the safety state machine (disarm resets; stop-only keeps) |

Arming therefore starts from zero, and on a slope the robot sags for a few tens of ms while
the integrator rebuilds. Preloading a stale value is the riskier failure.

`I` is stored in duty units (`I += ki*e*dt`), not as `ki * integral(e)`. Gains can be set
over the protocol while ARMED (only `ParamCommit` requires DISARMED), and in this form a
change to `ki` alters only future accumulation. The output does not jump. With `ki = 0`
the integrator freezes at its current value.

Rejected alternatives:
- **P only.** Leaves a steady-state error under any load, including the pitch reaction.
- **Full PID.** The D term is noise here; see above.
- **State-space / LQR / observer.** Needs a plant model we do not have yet. It would also
  hide a one-state problem behind machinery the owner would need to defend.
- **Feedforward only (open-loop).** No disturbance rejection, and the pitch reaction is a
  known disturbance.
- **Velocity (incremental) PI form.** It handles windup implicitly, but it makes the
  integrator state invisible in telemetry and complicates resetting on arm.

### Initial gains: pole-zero cancellation on the placeholder plant

Plant: `w(s)/u(s) = K / (τ s + 1)`, with K = 31.4 rad/s per unit duty and τ = 0.05 s (sim placeholder).

Choose `ki / kp = 1 / τ` so the PI zero cancels the plant pole. The loop transfer function
becomes `kp K / (τ s)`, an integrator, and the closed loop is first order with bandwidth
`ωc = kp K / τ`.

> **Amended 2026-10-01:** true for the feedback path, and for the reference response only
> when `kff = 0`. With `kff = 1/K` the reference response is
> `((1 + ωcτ)s + ωc) / ((τs + 1)(s + ωc))`, which has a zero at -14.3 rad/s, slower than both
> poles (-20 and -50 rad/s). Hence the overshoot.

The 5 ms window delay costs `ωc * 5.5 ms` of phase at crossover (window plus half a sample).
For ωc = 50 rad/s (closed-loop τ = 20 ms, 2.5x faster than the open-loop plant) that is
16 degrees, leaving about 74 degrees of phase margin.

| Gain | Formula | Initial value | Unit |
|---|---|---|---|
| `kff` | 1 / K | 0.0318 | duty per rad/s |
| `kp` | ωc τ / K | 0.0796 | duty per rad/s |
| `ki` | kp / τ | 1.59 | duty per rad |

> **Amended 2026-10-01:** `kff` default is **0**. 1/K = 0.0318 remains the value that gives
> zero ramp lag, if hardware testing ever calls for it.

All three are "derived from the placeholder plant — to be replaced after system ID". The
derivation goes in `docs/theory/wheel-velocity-loop.md`, so re-deriving after system ID
means substituting the measured K and τ.

### One gain set for every surface (no gain scheduling)

The owner asked (2026-10-01) whether the gains should switch between floor types such as
carpet and hard floor. They should not, at least until data says otherwise:

- **Rolling resistance** (carpet pile deforming) is a near-constant load torque. Rejecting a
  constant load with zero steady-state error is exactly what the integral term does. The
  settled integrator value is the extra duty the surface needs, so logging it gives a free
  surface signature for the research writeup.

  > **Amended 2026-10-01:** with `kff = 0` the integrator holds the whole steady duty; the
  > surface signature is the integrator *difference* at equal speed (see amendment, theory §5).
- **Traction** limits acceleration before slip. Gains cannot add grip. The acceleration
  limiter is what keeps demands inside it.
- **The plant dynamics** (K, τ) are set mostly by the motor, gearbox and wheel inertia, and
  the surface shifts them only slightly. The ±50% gain-mismatch SIL test covers that.
- **Gain scheduling costs** a surface detector that can be wrong, bumpless transfer when
  switching, and twice the tuning and test surface.

Revisit if hardware system ID on carpet and hard floor gives K or τ more than ~2x apart.
The gains are runtime parameters, so a manual retune needs no reflash in the meantime.

### Kinematics, saturation and acceleration limit

- Track width `b` = **0.202 m**: wheel centres sit at ±(182/2 + 4 + 12/2) mm
  (`cad/parameters.py`: casing length, standoff, wheel width). The wheel radius is
  0.0525 m (ADR 0008). Both are compile-time geometry constants, not parameters.
- `v_L = v - w b/2`, `v_R = v + w b/2`; positive `w` turns left, per REP-103.
- **Saturation preserves curvature.** If either wheel reference exceeds `max_wheel_speed`,
  both are scaled by the same factor, so the robot follows the commanded arc more slowly
  instead of turning on a different radius.
- **Acceleration limit** on each wheel reference (a rate limiter), initial guess 1.0 m/s².
  Its main purpose is the casing: drive acceleration swings the casing like a pendulum
  (ADR 0004, direction 2). Until the stabilizer exists, a gentle default keeps that
  disturbance small. It also reduces wheel slip.

Note: `DriveCommand` accepts ±2.0 m/s, but the placeholder plant tops out at 1.65 m/s at
full duty and 0.49 m/s at the default 0.3 limit. Commands above `max_wheel_speed` saturate
as above. They are not rejected, because the range check is the decoder's job and its
bounds are a separate decision.

### Interfaces in core

```
firmware/core/control/
  wheel_speed_estimator.hpp/.cpp   update(count, timestamp_us) -> rad/s, fault flag
  wheel_velocity_controller.hpp/.cpp  update(w_ref, w_meas, dt_s) -> duty; reset()
  diff_drive.hpp/.cpp              (v, w) -> (wL_ref, wR_ref) with saturation
  rate_limiter.hpp/.cpp            per-wheel acceleration limit
```

These are pure functions of their inputs and their own state. They call no HAL methods, so
each is unit-testable alone. A thin `DriveLoop` reads the two encoders, steps everything,
and writes the two motors. Who calls `DriveLoop::step()` at 1 kHz (a timer ISR or the main
loop) is part of the STM32 timer design, which is owner-reviewed. This ADR does not decide it.

Behaviour outside `ARMED`: `DriveLoop` resets the integrators and the rate limiter, and does
not write the motors. How and when the motors stop (coast, then brake) belongs to the safety
state machine. Hard rule 4 is met by measuring `step()` execution time into the existing
`LoopTiming` message under `LoopId::MOTOR_VELOCITY`.

### Protocol schema change (owner-reviewed)

New entries in `protocol/schema/params.yaml`, each with a `source` note:

| Param | Default | Unit | Source |
|---|---|---|---|
| `wheel_kff` | 0.0318 | duty·s/rad | derived, placeholder plant |
| `wheel_kp` | 0.0796 | duty·s/rad | derived, ωc = 50 rad/s |
| `wheel_ki` | 1.59 | duty/rad | derived, ki = kp/τ |
| `wheel_duty_limit` | 0.3 | — | CLAUDE.md "low by default" |
| `wheel_max_speed_rad_s` | 9.0 | rad/s | just under 0.3 × 31.4 so the limit is rarely hit by feedforward alone |
| `wheel_accel_limit_m_s2` | 1.0 | m/s² | initial guess, to be tuned with the stabilizer |
| `wheel_speed_window_samples` | 10 | samples | resolution vs delay, see above |

Params 0x0100–0x01FF are proposed as the drive block, which leaves room for the pitch block.
No message changes.

## How it gets tested

- **Unit tests:**
  - Estimator: exact speed on synthetic counts, forward and backward through the 2^32 wrap,
    a late sample, and the plausibility fault.
  - Controller: steady state equals feedforward plus integral, anti-windup freezes in
    saturation and recovers without overshoot, and `reset()`.
  - Kinematics: signs, curvature-preserving saturation, and a zero command.
  - Rate limiter: the slope and an exact landing on the target.
- **SIL, on the sim plant:**
  - A 0.2 m/s step settles within 5% in under 100 ms with less than 5% overshoot.
  - A turn-in-place command yields equal and opposite wheel speeds.
  - Steady-state error under a constant load torque is zero. This needs a load input added
    to `SimWheel`; the sim is not owner-reviewed, so that can proceed with this ADR.
  - Gains perturbed ±50% (plant mismatch) stay stable.
  - A "carpet" case: the same step with added constant load and added viscous drag settles
    to zero error with the same gains.
- **On hardware, later:** system ID on carpet and on hard floor. That decides whether one
  gain set suffices.
- Each test asserts against a number derived in the theory doc, not one read off a run.

## Consequences

- The robot gets a wheel loop that is defensible from first principles, and its tuning
  reduces to measuring two numbers (K, τ) per wheel.
- 10 ms of window sets a bandwidth ceiling of roughly 50–100 rad/s. That is enough for
  driving. It is the first thing to revisit if the stabilizer needs a stiffer drive.
- Docs to write on approval: `docs/theory/wheel-velocity-loop.md` (derivation, block
  diagram) and `docs/learning/wheel-velocity-loop.md` (walkthrough and questions).

## Questions for the owner

1. **Controller form:** PI plus feedforward, no derivative, conditional-integration
   anti-windup. *Recommended.*
2. **Speed measurement:** a 10-sample windowed count difference, with the HAL unchanged.
   The alternative is timer input capture (HAL and timer change). *Recommended: windowed;
   revisit after system ID.*
3. **Output limit semantics** (open from slice 5): saturate at ±limit (duty 0.1 stays 0.1)
   rather than scale. The limit is also held in `core` (`wheel_duty_limit`) so anti-windup
   knows when it is saturated, and the HAL's limit stays as a backstop at or above it.
   *Recommended: saturate.*
4. **Zero command while ARMED:** keep the loop running and actively hold zero speed. The
   alternative is to coast at zero. *Recommended: hold*, so the robot doesn't roll on a
   slope; coasting is the safety machine's response to a lost link, not normal behaviour.
5. **Saturation:** scale both wheels to preserve curvature, rather than clipping each wheel
   independently. *Recommended.*

## Owner decisions (2026-10-01)

All five accepted as recommended:

1. PI plus feedforward, no derivative, conditional-integration anti-windup.
2. 10-sample windowed count difference; HAL unchanged. Revisit after system ID.
3. The output limit **saturates** (does not scale). `core` holds `wheel_duty_limit`; the
   HAL limit is a backstop at or above it. This also settles the slice 5 open question.
4. A zero command while ARMED actively holds zero speed.
5. Wheel-speed saturation scales both wheels to preserve curvature.

## Amendment: feedforward off by default (owner, 2026-10-01)

**What went wrong.** The SIL step test (`SilDrive.StepSettlesWithin5PercentIn100ms...`)
failed: 13% overshoot, 118 ms to settle. An independent continuous-time calculation gave
11.8% and 127 ms, so the cause was the design, not the code. The derivation above treated
the closed loop as first order, but it only followed the feedback path. The reference also
enters through `kff`. Full derivation: `docs/theory/wheel-velocity-loop.md` §3.3.

| Placeholder plant | `kff = 1/K` | `kff = 0` |
|---|---|---|
| Raw step overshoot / 5% settle | 11.8% / 127 ms | 0% / 60 ms |
| 1 m/s² ramp: lag / corner overshoot | 11 mm/s / 5.3% | 20 mm/s / 0% |
| Worst overshoot, plant ±50% | 18% | 4.4% |

**Decision.** `kff = 0` by default; the parameter stays. A speed overshoot is an extra
acceleration that swings the casing (ADR 0004, direction 2), the disturbance this project
studies. The feedforward's only gain, 9 mm/s less ramp lag, is below what an operator
would notice. Revisit with hardware data.

**Derivative stays absent (recommended; owner may override).** The owner asked whether a
`kd` slot set to 0 would also exist. Recommendation: no. a zero `kff` is a complete, working term switched off, but a `kd` without a
derivative filter would be noise the moment it was non-zero (see "No derivative"). If
hardware shows a need, D arrives as a designed, filtered term, not a reserved number.

**Consequence for the "surface signature".** With `kff = 0` the settled integrator holds
the whole steady duty (`d + w/K`), not only the surface's extra load. The signature is the
difference between two surfaces at the same speed (theory §5).

**Test bug fixed alongside.** `DriveLoop.ReferenceRampsAtTheAccelerationLimit` expected 50
ramp increments from 50 steps. The first step after construction has no previous
timestamp, so it integrates dt = 0; the code is right and the test now expects 49.
