# 0009 — Crossing the rotating joint

- **Status:** **PROPOSED — awaiting owner approval.** Revised 2026-09-30 after ADRs
  0010–0012: the spine removed the slip ring fallback, and the budget figures are now ADR 0011's.
- **Related:** [0008](0008-reduced-scale-direct-drive.md), [0004](0004-pitch-axis-architecture.md),
  [0010](0010-direct-drive-axial-layout.md), [0011](0011-pitch-motor-and-encoder.md),
  [0012](0012-power-and-electronics-placement.md)

## Context

The casing rotates relative to the chassis, and wiring has to cross that joint. ADR 0012
puts every electronic part, including the battery and the wheel driver, in the casing, so
what crosses is only the two wheel motors' leads and their encoder lines: **about 10
conductors**, all low-current or low-rate.

ADR 0008 made this harder by putting the motor on the centreline, and by shrinking the torque
budget to the point where friction matters. ADR 0011's design point is **16.8–24.3 mN m** at
the axis for a trimmed casing.

## First: the requirement is +/-180 degrees, not continuous rotation

ADR 0004 specified **continuous** rotation, justified by the camera righting itself after a
throw without a self-righting mechanism. That justification only needs **+/-180 degrees** —
enough to bring the camera level from any landing orientation, including upside down.
Everything beyond that is headroom for accumulation, not function. ADR 0004's wording was
carried forward without re-examination.

Where continuous rotation actually earns its keep is accumulation:

- Righting from any landing orientation: +/-180 degrees is sufficient.
- Driving: the casing tracks chassis pitch, which oscillates about a mean. No accumulation.
- **Tumbling or rolling: the chassis turns through 360 degrees and the casing must follow to
  stay level. This accumulates, without bound.**

So the requirement is not "continuous" but "enough turns that accumulation is rare, plus a
way to recover when it is not."

## Options

### 1. Capsule slip ring at a separate axial station

> **Ruled out by ADR 0010.** A capsule unit sits on the centreline, and the chassis spine
> occupies the centreline at every axial station. There is nowhere to put one. The friction
> analysis below is kept because it is why option 2 fails too.

Real parts: 8–30 circuits at 12.5–16 mm outer diameter; smaller counts down to 6.5 mm. Sits
on the centreline at a different axial station from the motor, which the 182 mm casing has
room for.

Truly unbounded rotation. But:

- **Friction exceeds the entire torque budget.** This is the finding that settles it, and it
  is far worse than first assumed. Vendor specifications for 12.5 mm capsules:

  | Source | Friction torque |
  |---|---|
  | Senring M125, OD 12.5 mm | "less than 0.06 N.m" = **60 mN m** |
  | ATO 12.5 mm | 0.05 N.m, +0.01 per 6 circuits = **50-70 mN m** at 12 circuits |
  | Another vendor datasheet | starting torque 2 N.cm = **20 mN m** |
  | **Axis torque budget (ADR 0011, trimmed)** | **16.8–24.3 mN m** |

  The most optimistic figure is 0.9x the whole budget; the typical one is 2-3x. And the drag
  is *continuous* — paid whenever the camera holds level, not only while it moves.

  **Caveat, stated because the gap is suspicious:** a first-principles estimate — roughly 24
  brushes at 0.1 N, mu around 0.3, at 5 mm radius — gives about 3.6 mN m, some 6x below the
  most optimistic datasheet. Those specifications are probably worst-case breakaway including
  seal and bearing drag. The true figure likely sits between 4 and 20 mN m and **should be
  measured** rather than taken from either source. But a design cannot be committed on the
  assumption that a datasheet overstates by 6x.

  This is also the scale change talking. At ADR 0006's 302 mm the budget was 67 mN m and a
  20 mN m slip ring was 30% — bad but survivable. Shrinking the robot made direct drive
  possible *and* made the slip ring impossible, from the same k^4 scaling: the robot shrank
  and the slip ring's friction did not.
- A wear item, with a finite rotation life, in a robot that is meant to be thrown.
- Another part on a centreline that now holds the spine, the motor and both wheel shafts.

### 2. Through-bore slip ring around the spine

After ADR 0010 this is the only slip ring geometry that could still be built: it would have to
ring the 16 mm spine, so its bore is at least 16 mm. Concentric with the axis. Elegant, and constrains which motor can be bought — hollow-shaft
gimbal motors exist but the bore is typically 3–7 mm, while a 6-circuit through-bore unit is
around 33 mm outer diameter. Added complexity for no functional gain — and
**worse on the point that matters**, since friction torque scales with contact radius and a
through-bore unit has a much larger one.

### 3. Flexible wire loop with bounded travel and an opportunistic unwind (proposed)

Let the wiring wrap. Allow several turns of travel, track accumulated angle in firmware, and
unwind 360 degrees **whenever the accumulated angle reaches one turn and the robot is
stationary** — not when it approaches the mechanical limit.

- **Zero sliding friction.** Nothing rubs, so none of the budget is spent on drag. (It is
  still a spring; see the consequences.)
- **Zero wear**, no rotation life, nothing to fail mechanically.
- **Free**, and one fewer part on a crowded centreline.
- Cost: the camera spins through 360 degrees during an unwind, so the operator loses the
  picture briefly. See the frequency analysis below — with an opportunistic policy this
  lands during pauses and is rare.

## How often would an unwind actually happen?

The cost of this option is entirely about how often the camera spins, so it is worth
answering rather than assuming.

**Almost nothing accumulates turns.**

- **Normal driving: zero.** The casing tracks chassis pitch, which oscillates about a mean.
  It does not go round.
- **A throw: approximately zero**, provided the pitch loop is disarmed in flight. It should
  be regardless — it can achieve nothing airborne and would fight the tumble. Disarm on
  launch detection, re-zero on landing.
- **A complete flip or roll-over: exactly one turn.** This is the only real source.

**And flips are a random walk.** Tipping forward and backward roughly cancel, so accumulation
grows as sqrt(N) rather than N. Median flips before hitting the limit, from a 20 000-trial
Monte Carlo of a +/-1 random walk:

| Loom limit | Symmetric | 60/40 bias | 70/30 bias |
|---|---|---|---|
| +/-3 turns | 7 flips | 7 flips | 5 flips |
| +/-5 turns | 19 flips | 15 flips | 9 flips |
| +/-8 turns | 48 flips | 30 flips | 18 flips |

The limit is quadratic in turns, so a loom that tolerates +/-5 rather than +/-3 nearly
triples the margin. A consistent bias — always tipping forward over obstacles — is the bad
case and erodes that advantage.

**The policy matters more than the number.** Unwinding *reactively* at the mechanical limit
is the wrong design: it fires mid-drive, at whatever moment the count happens to reach the
limit, which is precisely when losing the picture is most costly.

Unwinding *opportunistically* — at one accumulated turn, while stationary — means the limit
is essentially never reached, the spin lands during a pause when it costs least, and the
operator can be shown a "recentering" indication rather than an unexplained spin. Worst case,
a robot in constant motion that never pauses, degrades gracefully to the table above.

## Proposed decision

**Option 3: bounded travel with a software unwind.**

The deciding argument is friction, and it is not close. Even small capsule slip rings are
specified at 20–70 mN m against a 16.8–24.3 mN m budget, paid continuously in a machine whose
purpose is holding a camera steady. The only slip ring the spine still allows has a larger
contact radius and so more drag. **There is no practical fallback**, which is a reason to
measure the loop early (below) rather than a reason to hesitate.

The loop has no sliding friction. It is not torque-free, though: see the spring term below.

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
- **A wound loop is a torsion spring.** It adds a torque that grows with accumulated turns.
  It is position-dependent and repeatable, so the controller can cancel it from the turn count
  (feedforward) or let the integrator absorb it, but it comes out of the torque budget. Its
  size is unknown and gets measured alongside item 1 below.
- **Where the loop goes:** around the spine near end B, over wheel motor B's pocket
  (z ≈ 142–166 mm, spine radius 8 mm). Casing contents must stay outside about r = 20 mm
  there. Not yet reserved in `cad/assembly.py`.

## What remains open after acceptance

None of these change the decision, since the alternatives are ruled out. They set its
parameters.

1. How many turns the loom actually tolerates, measured on the real wiring rather than
   assumed. This is quadratic in value: +/-5 turns is nearly three times the margin of +/-3.
2. Whether an unwind mid-mission is acceptable to the operator. With the opportunistic
   policy this should be rare, but it is a question about how the robot gets used rather
   than about mechanics.
3. Whether turn count survives a power cycle, or is re-established by driving to a known stop.
4. The loop's spring torque per turn, measured on the real loom.
