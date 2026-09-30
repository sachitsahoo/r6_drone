# 0009 — Crossing the rotating joint

- **Status:** **PROPOSED — awaiting owner approval.**
- **Related:** [0008](0008-reduced-scale-direct-drive.md), [0004](0004-pitch-axis-architecture.md)

## Context

The casing rotates relative to the chassis, and wiring has to cross that joint. ADR 0005
put the MCU, Pi, camera and IMU all in the casing, so what crosses is the two wheel motors'
power and their encoder signals — roughly 8 to 12 conductors, all DC or low-rate.

ADR 0008 made this harder by putting the motor on the centreline, where it now competes for
space with whatever carries those wires, and by cutting the torque budget from 67 mN m to
22.5 mN m, which makes friction matter.

## The question nobody had asked

ADR 0004 specified **continuous** rotation, justified by the camera righting itself after a
throw without a self-righting mechanism. That justification only needs **+/-180 degrees**.

Where continuous rotation actually earns its keep is accumulation:

- Righting from any landing orientation: +/-180 degrees is sufficient.
- Driving: the casing tracks chassis pitch, which oscillates about a mean. No accumulation.
- **Tumbling or rolling: the chassis turns through 360 degrees and the casing must follow to
  stay level. This accumulates, without bound.**

So the requirement is not "continuous" but "enough turns that accumulation is rare, plus a
way to recover when it is not."

## Options

### 1. Capsule slip ring at a separate axial station

Real parts: 8–30 circuits at 12.5–16 mm outer diameter; smaller counts down to 6.5 mm. Sits
on the centreline at a different axial station from the motor, which the 182 mm casing has
room for.

Truly unbounded rotation. But:

- **Friction lands directly on a 22.5 mN m budget.** A 3 mN m drag is 13% of it, and it is
  *continuous* — the actuator pays it whenever the camera holds level, not just while moving.
- A wear item, with a finite rotation life, in a robot that is meant to be thrown.
- Another part on a centreline that now holds the motor and the IMU.

### 2. Through-bore slip ring around the motor shaft

Concentric with the motor. Elegant, and constrains which motor can be bought — hollow-shaft
gimbal motors exist but the bore is typically 3–7 mm, while a 6-circuit through-bore unit is
around 33 mm outer diameter. Added complexity for no functional gain over option 1.

### 3. Flexible wire loop with bounded travel and a software unwind (proposed)

Let the wiring wrap. Allow roughly +/-3 turns of travel, track accumulated angle in firmware,
and **unwind 360 degrees when the accumulated angle approaches the limit**, preferably while
the robot is stationary.

- **Zero friction.** Nothing rubs, so none of the 22.5 mN m is spent on drag.
- **Zero wear**, no rotation life, nothing to fail mechanically.
- **Free**, and one fewer part on a crowded centreline.
- Cost: the camera spins through 360 degrees during an unwind, so the operator loses the
  picture briefly. It is deferrable — the firmware can wait for a stationary moment — and it
  only happens after substantial accumulation.

## Proposed decision

**Option 3: bounded travel with a software unwind**, with option 1 as the fallback if the
unwind behaviour proves unacceptable in use.

The deciding argument is friction, not cost. At 302 mm a slip ring's drag was noise against
a 67 mN m budget. At 214 mm it is 2–13% of the budget, paid continuously, in a machine whose
entire purpose is holding a camera steady.

## Consequences

- **This refines ADR 0004's "continuous rotation" rather than reversing it.** Rotation is
  mechanically bounded at roughly +/-3 turns and functionally unbounded via the unwind.
  ADR 0004's self-righting benefit is preserved in full, since that needs only +/-180 degrees.
- **The firmware gains a responsibility**: track accumulated casing angle across power
  cycles — or re-establish it at startup — and schedule unwinds. That is control and safety
  logic, so it is owner-reviewed and needs its own design.
- **The absolute encoder becomes more important, not less.** It reports angle within one
  turn; turn count is separate state the firmware must maintain. Losing it means losing the
  wrap count, which risks driving the wiring into its stop.
- **A hard stop is required**, mechanical or software, so a firmware fault cannot wind the
  loom until something tears. This belongs in the safety state machine design.
- Wiring must be routed and strain-relieved to survive repeated winding. Silicone-insulated
  stranded wire, a generous loop radius, and anchoring at both ends.

## What must be settled before this is accepted

1. How many turns the loom actually tolerates, measured on the real wiring rather than assumed.
2. Whether an unwind mid-mission is acceptable to the operator, which is a question about how
   the robot gets used rather than about mechanics.
3. Whether turn count survives a power cycle, or is re-established by driving to a known stop.
