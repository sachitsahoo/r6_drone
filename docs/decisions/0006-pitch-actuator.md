# 0006 — Pitch actuator and transmission

- **Status:** **PROPOSED — awaiting owner approval. Do not implement.**
- **Related:** [0004](0004-pitch-axis-architecture.md) (casing rotates continuously),
  [0005](0005-imu-placement.md) (actuator needs position feedback),
  [theory note](../theory/pitch-axis-inertia-and-torque.md) (the numbers below)

## Context

ADRs 0004 and 0005 narrowed this decision without settling it. 0004 established that the
whole casing rotates continuously about the wheel axis, so the load is hundreds of grams
rather than a camera assembly, and argued that torque favours gearing. 0005 established that
the actuator needs absolute position feedback, which pointed back toward a brushless motor
with a magnetic encoder. The two ADRs pulled in opposite directions and both said the same
thing: resolve it with an inertia estimate and a torque budget, not more argument.

That analysis now exists. Three findings decide it.

## The three findings

**1. The shell is 83–85% of the rotating inertia.** Total `I` is 0.647e-3 (light corner) to
1.534e-3 kg m^2, and the printed wall — not the electronics — accounts for almost all of it,
because a thin shell at 66 mm radius puts every gram at the largest radius in the machine.

**2. Direct drive is marginal; a modest reduction collapses the problem.** A moderate
specification of 10° in 100 ms with a 10 mm CoM offset needs 62–144 mN m at the axis, partly
as continuous holding torque. At 13:1 that becomes 5–11 mN m with negligible reflected
inertia. Crucially, **the geometry supplies that ratio almost for free**: a 10 mm pulley or
capstan against the 130 mm casing is about 13:1 in a single stage.

**3. Backlash lands inside the measurement.** At roughly 1920 px across a ~90° field of view,
1° of backlash is about 21 px of image jitter. Small gearboxes commonly show 1–3°. If
stabilization takes pitch error from 10° down to a 1–2° residual, a 1.5° backlash is the same
size as the residual being reported — the experiment would be characterising the gearbox
rather than the controller. For a project whose research question is visual stability, this is
disqualifying rather than merely undesirable.

## Options considered

### 1. Direct-drive brushless (FOC gimbal motor), no reduction

Zero backlash, backdrivable, smooth, and position feedback is already part of the standard
arrangement. But it must produce 62–144 mN m, much of it continuously against gravity, which
is at or beyond the small gimbal motors that would physically fit. Continuous holding current
in a sealed printed shell is also a thermal problem, and the casing is exactly where heat
cannot easily escape.

Viable **only if the casing is balanced to within a couple of millimetres**, which removes
the holding load and leaves only transient torque. That makes the actuator choice depend on
mechanical balancing being achieved and maintained — a coupling worth avoiding if something
else works.

### 2. Geared motor (N20-class with a spur or planetary gearbox)

Torque becomes trivial and the part is cheap, small, and familiar. Rejected on finding 3:
typical backlash is the same order as the residual error the project exists to measure.
Gear cogging also couples into the image, and the gearbox is not backdrivable, so impact
energy from a thrown robot loads the gear teeth instead of being absorbed by rotation.

Anti-backlash gearing exists but adds friction and cost and still does not reach the
sub-0.1° region comfortably.

### 3. Belt or capstan reduction onto the casing circumference (recommended)

A toothed belt, or a friction capstan, driving the casing directly — using the casing's own
~130 mm circumference as the output pulley.

- ~13:1 in one stage, from geometry already committed to.
- **Zero backlash** with a toothed belt; a capstan has none by construction.
- Motor-side requirement drops to 5–11 mN m, so a small brushless motor with a magnetic
  absolute encoder is comfortable, satisfying ADR 0005's feedback requirement.
- Reflected inertia becomes negligible (9e-6 kg m^2 at 13:1, heavy corner).
- Still backdrivable, so impact energy is absorbed by rotation rather than by gear teeth.
- Works with the casing balanced *or* deliberately bottom-heavy, so it does not force the
  balance decision.

Costs and risks, which are real:

- **Routing — RESOLVED before the CAD existed.** The concern was that the casing's outer
  surface is the impact surface, so the belt must engage internally, and 2–3 mm of rotational
  clearance looked too tight. Dedicating a 10–15 mm axial band at one end of the casing, where
  the chassis does not extend, removes the conflict entirely: the track projects inward freely
  and the binding constraint becomes axial length, which fits with room to spare. A 10 mm
  pulley against the 130 mm band gives 13:1, the ratio the torque budget assumed. See
  `docs/mechanical-requirements.md` R1.
- A belt needs tension and therefore an idler or an adjustable motor mount.
- A friction capstan can slip under shock — which is arguably a feature for impact
  survival and a defect for position tracking, since slip breaks the encoder's relationship
  to the casing angle unless the encoder reads the *casing* rather than the motor.
- Belt compliance adds a resonance, though at 13:1 with this inertia it should sit well
  above the control bandwidth. Worth measuring, not assuming.

## Proposed decision

**A small brushless motor with an absolute magnetic encoder, driving the casing through a
zero-backlash belt or capstan reduction of roughly 10–15:1.** Read the encoder on the
**casing side**, not the motor side, so that belt slip or compliance cannot corrupt the
measured casing angle — which ADR 0005 makes the basis for deriving chassis pitch.

Direct drive stays the fallback if the CAD cannot accommodate belt routing, conditional on
balancing the casing to within a couple of millimetres.

## Consequences

- Mechanical work is now on the critical path: the internal belt track has to exist before
  the actuator can be ordered sensibly. Requirements are written up in
  `docs/mechanical-requirements.md` so the CAD can be designed to them directly.
- **The belt drive frees the rotation axis.** The motor sits off-axis, leaving the centreline
  for the axle, the slip ring and the IMU. A direct-drive actuator would be coaxial and would
  compete with all three in a 165 mm wheel-to-wheel envelope. This is an argument for the belt
  that the torque budget alone did not surface.
- **The motor should sit in the casing**, not the chassis, so its phase currents stay off the
  slip ring. Costs 2–13% added inertia depending on mass and radius.
- Two encoders total on this axis if the motor also has one, or one casing-side encoder plus
  sensorless/open-loop commutation. FOC generally wants rotor position, so expect both.
- Shell mass reduction is the highest-leverage change available and should happen regardless
  of which option is chosen, since it attacks 85% of the inertia.
- The balance choice from the theory note remains open and is a control design decision.

## What must be measured before this is accepted

The recommendation rests on estimates. Before implementation:

1. Weigh the printed shell — it carries 85% of the inertia estimate.
2. Measure the CoM offset and the pendulum period, and invert for the true `I`.
3. Confirm the belt band survives detailed CAD (R1 shows it fits at the envelope level).
4. Confirm the camera's true horizontal FOV and resolution, which set the pixel-per-degree
   figure that finding 3 depends on.

If item 3 fails, this ADR should be superseded rather than amended.
