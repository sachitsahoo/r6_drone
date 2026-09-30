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

## Known limitations

- Empty as of Phase 1. Nothing has been logged because nothing has run.

## How to test

```
pytest
```

Analysis functions are tested against synthetic logs with known answers, so a plotting or
unit-conversion error is caught by a test rather than by a confusing figure.
