# tools — offline analysis scripts

## What it does

Log analysis, plotting, and system identification. Turns recorded sessions into the numbers
and figures that justify design decisions and answer the project's research question.

## How it fits the architecture

Entirely offline. Nothing here runs on the robot or in a control path. These scripts consume
the raw protocol frame logs written by [`../operator/`](../operator/), decoded with the
generated Python bindings from [`../protocol/`](../protocol/).

## Key design decisions

- **Reuse the generated decoder.** A second, hand-written log parser would drift from the
  schema and quietly produce wrong plots.
- **System identification output feeds the simulator.** Measured inertia, motor constants,
  and friction replace the placeholder plant model in [`../sim/`](../sim/) — which is the
  point at which simulation results start meaning something.
- **Every plot states its units**, and pitch plots state the sign convention, because
  positive nose-down is the opposite of most people's instinct.

## Contents

| Script | Purpose |
|---|---|
| `check_core_purity.py` | CI guard: enforces CLAUDE.md hard rules 1 and 2 over `firmware/core/`. |
| `pitch_inertia_budget.py` | Inertia and torque budget for the pitch axis at the current design point. Feeds ADR 0011 and `docs/theory/`. |

## Known limitations

- No log analysis or system identification yet, because nothing has run.
- `pitch_inertia_budget.py` runs on estimates from the mechanical envelope, not
  measurements. Its outputs are order-of-magnitude bounds until the shell is weighed and the
  casing'''s pendulum period is measured. See the theory note for what to measure.

## How to test

```
pytest
```

Analysis functions are tested against synthetic logs with known answers, so a plotting or
unit-conversion error is caught by a test rather than by a confusing figure.
