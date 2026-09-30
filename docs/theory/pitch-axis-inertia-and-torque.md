# Pitch axis: inertia, torque, and what they rule out

Supports [ADR 0006](../decisions/0006-pitch-actuator.md). Reproduce with
`python3 tools/pitch_inertia_budget.py`.

**Every number here is an estimate from the mechanical envelope, not a measurement.** The
script exists so the conclusions can be re-checked against weighed parts.

## Symbols

| Symbol | Meaning | Unit |
|---|---|---|
| `r_o`, `r_i` | casing shell outer, inner radius | m |
| `L` | casing axial length | m |
| `m_c` | total rotating mass (shell plus contents) | kg |
| `I` | rotating inertia about the wheel axis | kg m^2 |
| `d` | offset of the casing centre of mass from the rotation axis | m |
| `theta` | pitch error to be corrected | rad |
| `alpha` | angular acceleration | rad/s^2 |
| `tau` | torque at the pitch axis | N m |
| `n` | reduction ratio, motor to casing | — |

## Geometry

From the owner, 2026-09-30: outer diameter 135–145 mm, internal chassis diameter
115–125 mm, printed shell 2.5–3 mm, rotational clearance 2–3 mm, wheel-to-wheel ~165 mm,
total mass target 700–900 g.

The casing is therefore a **thin cylindrical shell** of roughly 65 mm inner and 68 mm outer
radius. That single fact drives everything below: essentially all of the shell's mass sits
at the largest radius in the machine.

## Inertia

Shell wall volume, excluding end caps:

```
V = pi (r_o^2 - r_i^2) L
```

Thick-walled cylinder about its own axis, and a component treated as a point mass at
radius `r`:

```
I_shell = m_shell (r_i^2 + r_o^2) / 2          I_part = m_part r^2
```

Two corners of the envelope — light/small/ABS at 85% wall fill with a bare MCU module, and
heavy/large/PETG at solid fill with the Nucleo-G474RE board as-is:

| | shell mass | I_shell | contents | I_contents | total | **I total** |
|---|---|---|---|---|---|---|
| Light | 126 g | 0.552e-3 | 48 g | 0.094e-3 | 174 g | **0.647e-3 kg m^2** |
| Heavy | 252 g | 1.273e-3 | 118 g | 0.261e-3 | 370 g | **1.534e-3 kg m^2** |

### Finding 1: the shell is 83–85% of the inertia

Not the electronics, not the camera — the printed wall. Two consequences:

- **Shell mass reduction has an outsized payoff.** A gram removed from the wall at
  r ≈ 66 mm costs about four times the inertia of a gram removed at r ≈ 33 mm. Ribbing or
  a lattice instead of solid 3 mm walls attacks 85% of the problem; relocating a circuit
  board attacks a few percent.
- **The rotating mass is only 174–370 g of the 700–900 g budget**, so most of the robot's
  mass is already off the pitch axis. There is headroom to move more (see battery placement
  in [ADR 0005](../decisions/0005-imu-placement.md)) but the shell itself cannot be moved.

## Torque at the pitch axis

Two loads, one continuous and one transient.

**Gravity**, whenever the centre of mass is off the rotation axis:

```
tau_gravity = m_c g d
```

| `d` | Light | Heavy |
|---|---|---|
| 0 mm | 0 | 0 |
| 5 mm | 8.5 mN m | 18.2 mN m |
| 10 mm | 17.0 mN m | 36.3 mN m |
| 20 mm | 34.1 mN m | 72.7 mN m |

This is a *holding* load. A direct-drive actuator fighting it burns current and heats with
the robot standing still.

**Correcting a pitch error.** Accelerating for half the interval and decelerating for the
other half gives a peak `alpha = 4 theta / t^2`, so `tau = I alpha`:

| Correction | `alpha` | Light | Heavy |
|---|---|---|---|
| 10° in 200 ms | 17 rad/s^2 | 11.3 mN m | 26.8 mN m |
| 10° in 100 ms | 70 rad/s^2 | 45.2 mN m | 107.1 mN m |
| 10° in 50 ms | 279 rad/s^2 | 180.6 mN m | **428.5 mN m** |
| 30° in 100 ms | 209 rad/s^2 | 135.5 mN m | 321.3 mN m |

### Finding 2: direct drive is marginal, and a small reduction collapses the problem

A moderate specification — 10° in 100 ms with a 10 mm CoM offset — needs **62 mN m (light)
to 144 mN m (heavy)** at the axis. That is at or beyond what small direct-drive gimbal
motors deliver, and it has to be delivered partly as continuous holding torque.

Reduction divides required motor torque by `n` and reflected inertia by `n^2`:

| `n` | motor torque (light) | motor torque (heavy) | reflected inertia (heavy) |
|---|---|---|---|
| 1:1 | 62.2 mN m | 143.5 mN m | 1534e-6 kg m^2 |
| 4:1 | 15.5 mN m | 35.9 mN m | 95.9e-6 |
| 13:1 | 4.8 mN m | 11.0 mN m | 9.1e-6 |
| 30:1 | 2.1 mN m | 4.8 mN m | 1.7e-6 |

At 13:1 the requirement is about 5–11 mN m, which is undemanding, and reflected inertia
becomes negligible next to any rotor.

**The geometry offers that reduction almost for free.** The casing is a ~135 mm cylinder, so
its own circumference is already a large pulley: a 10 mm drive pulley or capstan against a
130 mm casing is roughly 13:1 in one stage, with no gear teeth anywhere.

## Pendulum resonance

An unbalanced casing swings about the wheel axis with

```
f = sqrt(m_c g d / I) / (2 pi)
```

| `d` | Light | Heavy |
|---|---|---|
| 2 mm | 0.37 Hz | 0.35 Hz |
| 5 mm | 0.58 Hz | 0.55 Hz |
| 10 mm | 0.82 Hz | 0.77 Hz |
| 20 mm | 1.16 Hz | 1.10 Hz |

Sub-1 Hz for any realistic offset — squarely inside the band of chassis motion over rough
ground. This makes balance an explicit design choice rather than a packaging afterthought:

- **Bottom-heavy** gives a passive restoring torque that keeps the camera roughly upright
  without power, which is how throwable two-wheeled robots normally work. The same torque
  fights the actuator whenever it points off-level, adds a continuous holding load, and puts
  a lightly-damped sub-1 Hz resonance inside the control loop.
- **Balanced** (`d -> 0`) removes the holding load and the resonance, at the cost of no
  passive recovery: the camera's attitude is then entirely the actuator's responsibility,
  including at power-up and after an impact.

Which to choose is a control design question and therefore owner-reviewed. The analysis only
establishes that it *is* a choice with a sub-1 Hz consequence, not a detail.

## Backlash lands directly in the measurement

The project's research question is how much active pitch stabilization improves *visual*
stability. Transmission backlash is therefore not a side effect — it is an error term in the
quantity being measured. Converting angle to image displacement:

```
pixels = backlash_deg * horizontal_pixels / horizontal_fov_deg
```

Assuming 1920 px across a ~90° horizontal field of view (**confirm against the Camera
Module 3 Wide datasheet**):

| Backlash | Image jitter |
|---|---|
| 0.1° | 2 px |
| 0.5° | 11 px |
| 1.0° | 21 px |
| 2.0° | 43 px |

### Finding 3: a geared actuator with typical backlash would corrupt the result

Small gearboxes commonly show 1–3° of backlash. If stabilization reduces pitch error from
10° to a 1–2° residual, a 1.5° backlash is the *same size as the residual being reported* —
the experiment would be measuring the gearbox, not the controller. Backlash must be well
below the target residual error, which is a requirement on the transmission, not a
preference about motors.

## What to measure to replace these estimates

1. Weigh the printed shell and each casing component. Shell mass carries 85% of the answer.
2. Measure the CoM offset `d` — hang the casing and find where it balances.
3. Measure `I` directly: hang the casing as a pendulum, time the period, and invert
   `f = sqrt(m g d / I) / 2 pi`. This also validates the estimate above.
4. Confirm the camera's true horizontal FOV and output resolution.
5. Measure the chosen transmission's actual backlash, in degrees at the casing.

Until items 1–3 are done, treat every torque figure here as an order-of-magnitude bound.
