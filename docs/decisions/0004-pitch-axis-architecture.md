# 0004 — Pitch axis architecture: the outer casing rotates about the wheel axis

- **Status:** Accepted (2026-09-30)
- **Related:** [0002](0002-protocol-framing-and-codec.md) (the telemetry fields this affects)

## Context

`CLAUDE.md` described the robot as having an "actively pitch-stabilized camera body." That
phrase reads two ways — a small camera assembly pitching inside a fixed shell, or the whole
shell, carrying the camera, rotating — and the protocol work proceeded on the first reading.
The owner clarified on 2026-09-30 that it is the second.

This is not a detail. The two readings imply different actuators, different telemetry ranges,
and a different answer to where the IMU goes.

## Decision

- An **inner chassis** carries the wheels, the drive motors, and the axle.
- An **outer casing** carries the camera and electronics and **rotates about the wheel axis**,
  driven by a third actuator coaxial with the drive axle.
- The casing rotates **continuously** — no mechanical travel limit.

So the pitch axis *is* the wheel axis, and the stabilized mass is the whole casing rather
than a camera assembly.

## Consequences

### The actuator trade-off inverts

The earlier framing weighed a geared servo against an FOC gimbal BLDC, leaning toward the
gimbal motor for smoothness. That reasoning was about the wrong load. Gimbal motors are
direct-drive and chosen for zero backlash and smooth, low-torque control of a light camera
assembly — tens of grams of optics.

The casing carries the camera, the Pi Zero 2 W, the battery, and the shell itself: hundreds
of grams, with inertia about the wheel axis dominated by how far that mass sits from the
axis. Torque and gearing now dominate the selection, which is the regime where a geared
actuator looks *better*, not worse.

**Actuator selection is therefore still open and must be re-argued against this load**, not
inherited from the earlier discussion. It needs its own ADR, with an inertia estimate and a
torque budget rather than a preference.

### Something must cross a continuously rotating joint

Continuous rotation forbids a simple wire loop. Either a slip ring (or contactless power and
data) carries signals across the joint, or the split is chosen so that the only crossing
conductors are tolerable. Which conductors cross depends on where the MCU and battery live:
if they ride in the casing, the wheel motor leads and encoder signals cross; if they sit on
the chassis, the camera and IMU signals cross.

This is a mechanical and electrical decision that has not been made, and it constrains IMU
placement below.

### The camera can right itself without a self-righting mechanism

`CLAUDE.md` puts self-righting out of scope. Continuous casing rotation delivers most of the
practical benefit anyway: after a throw or a flip, the camera returns to level even though
the robot has not righted itself. This is a genuine argument for continuous rotation over
limited travel and does not reopen the self-righting scope decision — the vehicle's
orientation is still not actively corrected.

### Drive and pitch are mechanically coupled, in both directions

This is the most important consequence for the research question.

1. **Actuator reaction disturbs drive.** Torque applied to the casing reacts equally and
   oppositely on the chassis. The chassis is held by the wheels against the ground, so that
   reaction appears as a wheel-torque disturbance — commanding a camera pitch correction
   nudges the robot.
2. **Drive acceleration disturbs pitch.** The casing hangs on the axle as a pendulum, so
   accelerating or braking the wheels applies an inertial torque about the pitch axis.

Direction 2 *is* the disturbance the stabilizer exists to reject, so it is the phenomenon
under study rather than a problem to design away. Direction 1 means the velocity loop and
the pitch loop are not independent, and a naive pair of separate PIDs will fight each other.
Whether that is handled by feedforward decoupling, by loop-rate separation, or by accepting
the interaction is an owner-reviewed control design decision and is deliberately not made
here.

### Two already-shipped consequences in the protocol

1. **`camera_pitch_rad`'s declared range is now wrong.** It is `[-1.5708, 1.5708]` — plus or
   minus 90 degrees — with the note "mechanical travel limit unknown until the actuator is
   chosen." With continuous rotation the true value covers the full circle. Because the
   generator emits range validation into the decoder, out-of-range telemetry is *rejected*,
   not merely mislabelled: a casing at 120 degrees would drop the frame. A fix is proposed
   in `protocol/design-proposal.md`; it is a schema change and therefore owner-reviewed.
2. **`body_pitch_rad` and `camera_pitch_rad` may be redundant**, depending on IMU placement.
   See below.

### IMU placement becomes the pivotal open decision

`CLAUDE.md` already lists IMU placement as ADR material. It now determines whether the two
pitch telemetry fields carry independent information.

- **IMU on the rotating casing.** Measures the camera's world-relative pitch and rate
  directly, which is exactly what the 500 Hz–1 kHz stabilization loop needs — no arithmetic,
  no dependence on the actuator encoder, lowest latency on gyro rate. Standard practice for
  camera gimbals. Chassis pitch is then derived as casing pitch minus the encoder angle, so
  `body_pitch_rad` becomes a computed value rather than a measured one.
- **IMU on the inner chassis.** Measures chassis pitch directly; camera pitch is derived by
  adding the encoder angle, which injects encoder resolution and latency into the inner loop.

Both give access to both quantities, given the actuator's encoder. The difference is which
one is measured and which is inferred, and the stabilization loop cares. Leaning toward the
casing, but this needs its own ADR with the wiring constraint above considered alongside it.
