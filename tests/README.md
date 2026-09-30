# tests — unit and software-in-the-loop tests

## What it does

Holds the C++ unit and SIL tests (`cpp/`) and the Python tests (`python/`) that CI runs on
every push.

## How it fits the architecture

The reason [`../firmware/core/`](../firmware/core/) is vendor-free. SIL tests run the real
control code against [`../sim/`](../sim/)'s simulated HAL, so a control law can be exercised
before any hardware exists. Per [`../CLAUDE.md`](../CLAUDE.md), tests are written from the
acceptance criteria *before* the implementation, and a failing test is never weakened,
skipped, or deleted to get to green.

## Key design decisions

- **Virtual time in SIL tests.** The simulated `Clock` is advanced explicitly by the test
  rather than tracking wall time, so results are deterministic and CI cannot flake on timing.
- **Wraparound and overflow get dedicated tests.** A `uint32_t` microsecond timestamp wraps
  every ~71 minutes and encoder counters wrap too; both are cheap to test and expensive to
  discover in the field.
- **The protocol decoder is fuzzed**, not just round-trip tested. Round-trip tests only prove
  it handles frames it produced itself.

## Known limitations

- Only a build-system smoke test exists as of Phase 1.

## How to test

```
cmake -B build -DTARGET=host && cmake --build build
ctest --test-dir build --output-on-failure
pytest
```
