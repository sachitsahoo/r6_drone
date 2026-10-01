# Completed tasks

---

## CAD refactor: 70 x 182, direct drive (ADR 0008)  [2026-09-30, DONE]

Owner approved the layout in chat: spine chassis, hollow-shaft pitch motor at end A, IMU on a
ring around a thinned spine waist (r ~ 5 mm), full +/-180 deg leveling kept.

- [x] parameters.py: rescale; delete belt/band/pulley/bracket/standoff values; axial layout
      stations as DERIVED values; spine, cup, motor-end bearing, hollow bore (ASSUMPTION)
- [x] parts.py: delete drive band + motor bracket + chassis disc; add chassis_spine, end cap A
      (motor end); cap B keeps the 6704 and the pendulum datum
- [x] assembly.py: rotation-aware sweep (casing vs chassis sampled over a full turn, pruned by
      exact r_min / conservative r_max); motor, wheel motors, wheel shaft A as envelopes
- [x] build.py: rotating set, quantities, compare against ADR 0008's 0.133e-3
- [x] tests/python/test_cad.py: replace belt-era tests with direct-drive equivalents
- [x] ADR 0010 (axial layout, topology finding); amend ADR 0005; mechanical-requirements R1/R2/R5
- [x] implementation-notes.html entry; cad/README; component-measurements checklist
