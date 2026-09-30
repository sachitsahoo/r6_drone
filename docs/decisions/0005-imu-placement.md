# 0005 — IMU on the rotating casing

- **Status:** Accepted (2026-09-30)
- **Related:** [0004](0004-pitch-axis-architecture.md) (the casing rotates about the wheel axis)

## Context

ADR 0004 established that an outer casing carrying the camera and electronics rotates
continuously about the wheel axis. That leaves the ICM-42688-P with two homes: the rotating
casing alongside the camera, or the inner chassis. `CLAUDE.md` already listed IMU placement as
a decision needing an ADR, and 0004 made it pivotal — it decides which pitch quantity is
measured and which is inferred.

## Options considered

1. **On the rotating casing (chosen).** Measures the stabilized platform's world-relative
   pitch and rate directly. The 500 Hz–1 kHz stabilization loop reads the gyro with no
   arithmetic and no dependence on the actuator encoder. This is standard practice for camera
   gimbals, for the same reason: you instrument the thing you are trying to hold still.
2. **On the inner chassis.** Measures chassis pitch directly; camera pitch is then derived by
   adding the actuator angle, injecting the encoder's resolution, quantization, and latency
   into the *inner* loop — the one with the tightest deadline. Rejected: it puts the cheapest
   available signal behind the most error-prone path.

Both placements can produce both quantities, given the actuator's encoder. The choice is about
which one is measured.

## Decision

The IMU rides on the **rotating casing**, as close to the wheel axis as the mechanical design
allows (see the mounting constraint below, which is not optional).

## Consequences

### Chassis pitch becomes a derived quantity

With the casing angle measured and the actuator encoder giving the relative angle:

```
theta_chassis = theta_casing - phi
```

where `theta_casing` is the casing's world-relative pitch (IMU), `phi` is the encoder angle of
the casing relative to the chassis, and all follow the project convention of positive
nose-down about +y. Chassis pitch therefore carries the sum of IMU and encoder error. That is
the right trade: nothing in the control loops needs chassis pitch at 1 kHz, whereas the
stabilizer needs casing rate at 1 kHz.

### The actuator must provide position feedback — this narrows the selection

Because `theta_chassis` is derived from `phi`, the pitch actuator without position feedback
makes chassis pitch unobservable. Combined with continuous rotation from ADR 0004:

- A standard hobby servo is out: its internal potentiometer is limited-travel, and
  continuous-rotation servo conversions typically discard position feedback entirely.
- Incremental feedback alone is awkward: a continuously rotating joint has no natural home
  position to index against at power-up.
- **An absolute position sensor is strongly preferred** — a magnetic absolute encoder
  (AS5048/AS5600 class) reads correctly at power-up regardless of where the casing came to
  rest, which matters for a robot that gets thrown.

This sharpens, and partly re-complicates, ADR 0004's conclusion. 0004 argued that the casing's
inertia pushes toward a geared actuator for torque. The absolute-feedback and
continuous-rotation requirements point toward a brushless motor with a magnetic encoder, which
is exactly the FOC gimbal arrangement 0004 said was chosen for the wrong reasons.

The genuine tension for the actuator ADR is therefore:

| | Direct-drive brushless | Geared |
|---|---|---|
| Torque for the casing's inertia | may be insufficient | sufficient |
| Backlash | none | present, and it sits inside the stabilization loop |
| Backdrivable after impact | yes | often not |
| Smoothness for video | excellent | gear cogging couples into the image |

Backlash and cogging inside a loop whose entire purpose is image stability are not
incidental — they are the thing being measured. **Resolving this needs a measured inertia
estimate and a torque budget**, which is the first task of the actuator ADR rather than a
preference to be argued.

### The MCU and the Pi belong in the casing

The IMU talks SPI at 1 kHz or more, and the camera uses a CSI ribbon. Neither is something to
run across a slip ring: both are short-range, high-rate, and noise-sensitive. So the IMU, the
camera, the Pi, and the MCU that samples the IMU all sit in the casing together.

What must then cross the rotating joint is the wheel motor drive and encoder wiring — roughly
a dozen conductors, all low-rate or DC, which is what a slip ring handles well. The pitch
actuator itself straddles the joint by construction and crosses nothing.

### Battery placement is now a real trade-off, not a packaging detail

- **Battery in the chassis** reduces the inertia the pitch actuator must accelerate, directly
  easing the torque problem above. Cost: high-current conductors cross the slip ring, where
  contact resistance and intermittency are worst.
- **Battery in the casing** keeps the slip ring to signal-level and low-current lines, at the
  price of adding the single heaviest component to the rotating mass.

Unresolved, and it should be decided together with the actuator's torque budget since the two
determine each other.

### Mounting constraint: put the IMU near the wheel axis

This is the consequence most likely to be discovered painfully rather than designed in. An
accelerometer offset by `r` from the rotation axis sees centripetal `w^2 r` and tangential
`alpha r` acceleration, neither of which it can distinguish from gravity. Apparent tilt error,
computed in this ADR's supporting analysis:

| Offset | at 5 rad/s | at 10 rad/s | at 20 rad/s |
|---|---|---|---|
| 5 mm | 0.7 deg | 2.9 deg | 11.5 deg |
| 15 mm | 2.2 deg | 8.7 deg | 31.5 deg |
| 30 mm | 4.4 deg | 17.0 deg | 50.7 deg |
| 50 mm | 7.3 deg | 27.0 deg | 63.9 deg |

A 30 mm offset at a modest 10 rad/s corrupts the gravity reference by 17 degrees. The
tangential term is comparable: an aggressive 200 rad/s^2 correction at 15 mm produces another
17 degrees of apparent tilt, and it appears exactly when the stabilizer is working hardest.

Two mitigations, and they are not exclusive:

1. **Mechanical**: mount the IMU as close to the wheel axis as the packaging allows. Every
   millimetre removed is a quadratic reduction in the centripetal term.
2. **Analytical**: subtract the known kinematic terms using the measured offset, rate, and
   angular acceleration. This requires the offset to be *measured*, which belongs in
   `docs/bringup/`, and it is part of the estimator design, which is owner-reviewed.

Relying on mitigation 2 alone would make the estimator's accuracy depend on a hand-measured
lever arm. Do the mechanical work first.

### Shock exposure

The casing is the outer surface of an impact-tolerant robot, so the IMU now sits on the
structure that takes the hits. The ICM-42688-P's shock survival rating should be checked
against the expected drop and throw energies during bring-up, and recorded in
`docs/bringup/`. Compliant mounting would reduce shock but adds mechanical lag inside the
stabilization loop — another tension to resolve with data rather than by assumption.

### Protocol telemetry is now partly mislabelled

`StateTelemetry.body_pitch_rate_rad_s` is described as "Measured chassis pitch rate" and cites
the ICM-42688-P gyro full scale as its source. With the IMU on the casing the gyro measures
*casing* rate; chassis rate is derived. The field's description and source are therefore
misleading, and the casing rate the inner loop actually runs on is not transmitted at all.
Corrections are proposed in `protocol/design-proposal.md` rather than applied, since schema
changes are owner-reviewed.

### Where the derivation will live

The full kinematics — the relation above, the centripetal and tangential correction terms, and
the resulting estimator structure — belong in `docs/theory/` and will be written with the
estimator design, which is owner-reviewed. This ADR records only the placement decision and
the constraints it imposes.
