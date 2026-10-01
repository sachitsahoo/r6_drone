# Pitch axis: inertia, torque, and what they require

Supports [ADR 0011](../decisions/0011-pitch-motor-and-encoder.md) (the motor and the balance
requirement) and [ADR 0008](../decisions/0008-reduced-scale-direct-drive.md) (direct drive).
Reproduce with `python3 tools/pitch_inertia_budget.py`.

**Nothing here is measured yet.** The printed parts come from the CAD solids and the
electronics are point masses at estimated radii. The script exists so the conclusions can be
re-checked against weighed parts.

*History: the first revision of this note argued, for a 135 mm casing, that direct drive was
marginal and an 8:1 belt was needed (ADR 0006). The robot then shrank to 70 × 182 mm. Inertia
fell about 5×, the belt stopped fitting, and direct drive became the obvious choice. That
argument is in ADR 0008. This revision is the budget for the machine as it now stands.*

## Symbols

| Symbol | Meaning | Unit |
|---|---|---|
| `m` | total rotating mass (plastic plus contents) | kg |
| `I` | rotating inertia about the wheel axis | kg m^2 |
| `d_h` | horizontal offset of the casing's centre of mass from the axis, camera level | m |
| `d_v` | vertical offset (below the axis), camera level | m |
| `theta` | pitch error to be corrected | rad |
| `alpha` | angular acceleration | rad/s^2 |
| `a` | chassis linear acceleration | m/s^2 |
| `tau` | torque at the pitch axis | N m |
| `K_t` | motor torque per amp | N m/A |

## Inertia

The printed parts' mass and inertia come from the solids (`cad/build.py`). Everything else is
a point mass, `I = m r^2`, at the radius ADR 0010's layout puts it:

| | mass | I |
|---|---|---|
| Printed plastic (shell, caps, IMU bridge, camera mount) | 141 g | 0.137e-3 |
| Motor stator and windings, ~30 g at r = 10 mm (stator rides on the casing) | 30 g | 0.003e-3 |
| Battery, Pi, MCU, camera, drivers, encoder, IMU, wiring | 79 g | 0.031e-3 |
| **Total** | **250 g** | **0.171e-3 kg m^2** |

The full list with each part's radius and source is in the script's output.

### Finding 1: the shell is still 71% of the inertia

The 107 g shell sits at r ≈ 33 mm, the largest radius in the casing. A gram removed from the
wall saves about three times the inertia of a gram removed from a board at r ≈ 20 mm. Ribbing
the shell (R6) is still the best lever on inertia.

## Torque at the pitch axis

Three loads.

**Holding gravity**, whenever the centre of mass is sideways of the axis with the camera level:

```
tau_hold = m g d_h
```

With the camera held level the casing's world attitude is constant, so this is paid
**continuously**, for as long as the camera holds level. Only the sideways component counts:
a centre of mass directly below the axis costs nothing to hold.

**Correcting a pitch error.** Accelerate for half the interval and decelerate for the other
half: the peak is `alpha = 4 theta / t^2`, and `tau = I alpha`.

| Correction | `alpha` | `tau` |
|---|---|---|
| 10° in 200 ms | 17 rad/s^2 | 3.0 mN m |
| **10° in 100 ms** | **70 rad/s^2** | **11.9 mN m** |
| 10° in 50 ms | 279 rad/s^2 | 47.7 mN m |
| 30° in 100 ms | 209 rad/s^2 | 35.8 mN m |

**Rejecting chassis acceleration.** A casing whose centre of mass hangs `d_v` below the axis is
a pendulum. Accelerating the axle swings it with

```
tau_accel = m a d_v
```

This is the disturbance the research question is about (ADR 0004, coupling direction 2).

### Finding 2: holding gravity is the largest term, so balance trimming is a requirement

At the design point (10° in 100 ms, `a` = 3 m/s^2), against the GM2804H's rated
34.3 mN m at 0.8 A, so `K_t` = 43 mN m/A:

| Case | slew | hold | accel | total | % rated | current | heat while holding |
|---|---|---|---|---|---|---|---|
| Untrimmed, `d_h` = 10 mm | 11.9 | 24.5 | 0 | **36.4 mN m** | **106%** | 0.85 A | 2.2 W |
| Trimmed `d_h` = 2 mm, balanced | 11.9 | 4.9 | 0 | 16.8 mN m | 49% | 0.39 A | 0.09 W |
| Trimmed `d_h` = 2 mm, bottom-heavy `d_v` = 10 mm | 11.9 | 4.9 | 7.5 | 24.3 mN m | 71% | 0.57 A | 0.09 W |

Copper loss uses `P = 1.5 I_peak^2 R_phase`, with the listed 9 Ω taken as phase-to-phase.

An untrimmed casing runs the motor past its rating and heats a closed plastic shell
continuously. Trimming the sideways offset to 2 mm with the R4 masses removes nearly all of
the holding load. **That makes trimming a requirement, not an option**, and the R8 balance
measurement has to find `d_h`.

ADR 0008's earlier 22.5 mN m was this same calculation with the contents guessed at 35% of the
shell mass. That left out about 70 g of electronics and the motor, and it assumed the untrimmed 10 mm.

### Finding 3: `K_t` from the rated point, not from KV

The listing's no-load figure (about 1690 rpm at 10 V) implies 169 rpm/V and `K_t` ≈ 57 mN m/A.
The rated load point implies 43 mN m/A. The KV conversion depends on conventions a listing
doesn't state, so the budget uses the rated point, which is the more pessimistic of the two.

## Pendulum resonance

A casing with its centre of mass `d_v` below the axis swings at

```
f = sqrt(m g d_v / I) / (2 pi)
```

| `d_v` | 2 mm | 5 mm | 10 mm | 20 mm |
|---|---|---|---|---|
| `f` | 0.85 Hz | 1.35 Hz | 1.91 Hz | 2.70 Hz |

Shrinking the robot raised this: the same offset now resonates about twice as fast as at
135 mm, because inertia fell faster than mass. It is still well inside the band of chassis
motion over rough ground, so the choice remains a control design decision (owner-reviewed):

- **Bottom-heavy** gives passive recovery after a throw, at the cost of a resonance inside
  the loop and the acceleration torque above.
- **Balanced** removes both, but then the camera's attitude is entirely the actuator's job,
  including at power-up.

Either is within the motor's rating once `d_h` is trimmed.

## Why not a gearbox: backlash lands in the measurement

The research question is about *visual* stability, so transmission backlash is an error term
in the quantity being measured:

```
pixels = backlash_deg * horizontal_pixels / horizontal_fov_deg
```

At 1920 px across about 90° (to confirm for the Camera Module 3 Wide), 1° of backlash is
21 px. Small gearboxes commonly have 1–3°, the same size as the residual error stabilization
is trying to reach. Direct drive has none, which is a standing reason never to reintroduce a
gearbox to buy torque.

## What building the CAD revealed

The first build was **855 g of printed plastic** against a 700–900 g vehicle target. No
estimate had costed the end caps, discs or wheels. Rim-hub-and-spoke parts brought it to
398 g; at 70 × 182 with direct drive it is 212 g. Build geometry early: the inertia estimate
was fine, and the mass budget was not.

## What to measure to replace these estimates

1. Weigh the printed shell and every casing component. The shell carries 71% of `I`.
2. Find the centre of mass in both directions, `d_h` and `d_v`, using the R8 datum. Then trim
   `d_h` to 2 mm or less.
3. Measure `I` directly: hang the casing as a pendulum, time it, and invert the formula above.
4. Confirm the motor's rated point, and measure the wire loop's spring torque (ADR 0009).
5. Confirm the camera's horizontal FOV and output resolution.
