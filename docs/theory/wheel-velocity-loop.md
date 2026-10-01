# Wheel velocity loop — theory

Derivations behind [ADR 0013](../decisions/0013-wheel-velocity-loop.md) and the code in
[`firmware/core/control/`](../../firmware/core/control/). Every bound asserted in
`tests/cpp/test_sil_drive.cpp` is derived here.

**All numbers use the placeholder plant** (`sim/sim/wheel_plant.hpp`): K = 31.4 rad/s per unit
duty, τ = 0.05 s, 1400 counts/rev. None are measured. After system ID, substitute the measured
K and τ into the formulas; the structure does not change.

## Variables

| Symbol | Meaning | Unit |
|---|---|---|
| v, ω | Body forward speed, yaw rate (+ = turning left) | m/s, rad/s |
| b | Track width, 0.202 m | m |
| r | Wheel radius, 0.0525 m | m |
| w, w_ref | Wheel angular speed, its reference | rad/s |
| u | PWM duty | — |
| e | Speed error, w_ref − w | rad/s |
| K, τ | Plant gain (rad/s per unit duty) and time constant | rad/s, s |
| ωc | Closed-loop bandwidth | rad/s |
| N, T | Estimator window in samples; sample period (1 ms) | —, s |
| C | Encoder counts per wheel revolution | counts |

## 1. Kinematics

A differential drive turns by running its wheels at different rim speeds. Each wheel's rim
speed is the body speed plus or minus the rotation of the body about its centre, at
half-track distance:

```
v_L = v − ω b/2        v_R = v + ω b/2        w_L = v_L / r,  w_R = v_R / r
```

The turning radius is `R = v / ω`, and it fixes the ratio `w_L / w_R`. Scaling both wheels by
one factor therefore preserves R. That is why saturation scales both wheels instead of
clipping each one: clipping only the faster wheel would change the ratio, and so the arc.

## 2. Speed estimate and its quantisation

The encoder gives an integer count. Over a window of N samples:

```
w_meas = Δcount · (2π / C) / (N T)        resolution  Δw = 2π / (C N T)
```

| N | Resolution (rad/s) | Rim speed (mm/s) | Delay N T / 2 |
|---|---|---|---|
| 1 | 4.49 | 236 | 0.5 ms |
| 10 | 0.449 | 24 | 5 ms |

A moving average over N samples delays the signal by N T / 2. At crossover frequency ωc, a
delay d costs phase `ωc d`. With N = 10, d = 5 ms plus half a sample for the zero-order
hold, so at ωc = 50 rad/s: 50 × 0.0055 = 0.275 rad = **16°**.

## 3. Controller

Plant, first order: `τ dw/dt = K u − w`, i.e. `G(s) = K / (τ s + 1)`.

### 3.1 PI with pole-zero cancellation

PI: `C(s) = kp + ki/s = kp (s + ki/kp) / s`. Choosing `ki / kp = 1/τ` puts the controller's
zero on the plant's pole:

```
L(s) = C(s) G(s) = kp (τ s + 1)/(τ s) · K/(τ s + 1) = (kp K / τ) / s = ωc / s
```

The open loop is a pure integrator, crossing over at `ωc = kp K / τ`. The loop gain's phase
is −90° everywhere, so the only phase loss is the estimator delay: margin = 90° − 16° = **74°**.
Closing the loop on the feedback path gives `ωc / (s + ωc)`: first order, with
`τ_cl = 1/ωc`.

For ωc = 50 rad/s (τ_cl = 20 ms, 2.5x faster than the open-loop plant):

```
kp = ωc τ / K = 50 × 0.05 / 31.4 = 0.0796       ki = kp / τ = 1.59
```

Why not faster? The window delay sets the ceiling. At ωc = 150 rad/s the delay would cost
47°, and quantisation noise passed through kp would triple.

### 3.2 Why no derivative

D acts on dw/dt. Quantisation makes w_meas jump by one step Δw = 0.449 rad/s between
consecutive 1 ms samples, which differentiates to **449 rad/s²**. The largest real
acceleration, at the 1 m/s² limit, is 1/r = **19 rad/s²**. The noise is about 24x the
signal. Filtering it enough to be usable adds back the lag D was meant to remove. A
first-order plant also gains nothing from D's phase lead, since §3.1 already has 74° of margin.

### 3.3 Feedforward adds a zero — why `kff = 0` (ADR 0013 amendment)

With feedforward, `u = kff·w_ref + C(s)·e`. The reference now enters by two paths. Solving
for W/W_ref with the cancellation of §3.1:

```
W/W_ref = (K kff s/(τs + 1) + ωc) / (s + ωc)
```

With `kff = 1/K` (exact feedforward):

```
W/W_ref = ((1 + ωcτ) s + ωc) / ((τ s + 1)(s + ωc))
        = 2.5·(s + 14.3) / (0.05·(s + 20)(s + 50))      [placeholder numbers]
```

The poles are at −20 and −50 rad/s, and the zero is at −14.3: slower than both. A slow
left-half-plane zero makes the step response rise past its final value. Working the error
directly, `e(t) = r(−0.667 e^{−20t} + 1.667 e^{−50t})`. It crosses zero at 30.5 ms and
bottoms out at −0.118 r at 61 ms: **11.8% overshoot**. The SIL test measured 13%, the extra
coming from the duty clamp.

With `kff = 0` the reference response is `ωc / (s + ωc)`. That has no overshoot, and a 5%
settle of 3 τ_cl = **60 ms**.

Intuitively, the cancelled PI is already the ideal controller for reference changes. The
feedforward pushes the wheel by the duty it predicts, and the PI pushes it again, so the
wheel overshoots.

### 3.4 Anti-windup

When `|u|` hits `duty_limit`, the plant receives less than the controller asks for, so the
error persists and a plain integrator grows without bound. Unwinding it later causes a
large overshoot. Conditional integration skips the update when the output is saturated
*and* the error has the same sign as the output. Integrating would then only deepen the
saturation. An error of the opposite sign is still integrated, because that pulls the
output back into range. The integrator is also bounded to ±duty_limit: anything beyond it
can never be applied.

## 4. Ramp tracking

The acceleration limit turns every step into a ramp of slope a (rad/s²). A first-order loop
tracking a ramp settles to a constant lag:

```
lag(t) = a τ_cl (1 − e^{−t/τ_cl})  →  a / ωc
```

At 1 m/s² (19.0 rad/s²): 19.0 / 50 = 0.38 rad/s, i.e. **20 mm/s** of rim speed. 100 ms into
a ramp the lag is 0.020 m/s × (1 − e⁻⁵) = 0.0199 m/s. That is the
`SilDrive.TracksTheDefaultAccelerationRamp` bound. With `kff = 1/K` the ramp lag would be
zero in steady state, at the cost of §3.3's overshoot. ADR 0013's amendment takes the
lag.

## 5. Disturbance rejection (carpet, slopes)

A constant load torque enters as a duty offset d: `τ dw/dt = K(u − d) − w`. In steady state
dw/dt = 0 and the integrator's update `ki e dt` must be zero, so **e = 0**. That is why
one gain set covers every surface (ADR 0013).

With e = 0 the P term is zero, so the integrator holds the whole steady duty:
`I = u − kff·w = d + (1 − K kff) w / K`. With `kff = 0` that is `d + w/K`: the surface's load
*plus* the duty any surface needs at that speed. The surface signature is therefore the
**difference** in settled integrator between two surfaces at the same speed, or `I − w/K`
once K is measured. With extra viscous drag c as well, the difference is `d + c·w/K`.
At zero speed (holding on a slope) `I = d` exactly.

Recovery is not instant. Because the PI zero cancels the plant pole, the disturbance
response keeps the plant's own pole (τ = 50 ms). This is the known cost of pole-zero
cancellation: fast reference tracking, but disturbances recover at the open-loop rate.
Tens of milliseconds is acceptable for driving. If the stabilizer later needs a stiffer
drive, the fix is a higher ki/kp ratio (no longer cancelling), traded against overshoot.

## References

- K. J. Åström and R. M. Murray, *Feedback Systems*, 2nd ed., Princeton, 2021. Ch. 11 (PID,
  anti-windup, setpoint weighting) and ch. 12 (zeros and transient response).
- ADR 0013 and its amendment; ADR 0004 (drive–pitch coupling).
