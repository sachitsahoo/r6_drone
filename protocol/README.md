# protocol — message schema and code generation

## What it does

Defines every message exchanged between MCU, robot bridge, and operator, and generates the
C++ and Python code that encodes and decodes them.

## How it fits the architecture

The schema in this directory is the **single source of truth**. C++ and Python bindings are
generated from it and are never hand-edited. The generator runs at build time on a
development machine or in CI; it is never deployed to the robot. Generated output is
gitignored and regenerated, so the schema cannot silently diverge from the code.

```
protocol/messages.<schema>
        |  generator (build-time only)
        +--> generated/messages.hpp  -> firmware/, sim/
        +--> generated/messages.py   -> operator/, robot_bridge/, tools/, tests/
```

## Key design decisions

Framing is COBS + CRC-32 + message ID + sequence number + timestamp + protocol version.
The rationale for each element is in ADR 0002:

- **A protocol version field exists because the two ends are built separately.** Firmware
  flashed from an older schema can meet an operator app built from a newer one; the version
  byte turns silent field misalignment into a clean rejection.
- **The decoder must survive arbitrary garbage** and resynchronize. A radio link delivers
  corrupt and truncated frames as a matter of course, so this is fuzz-tested, not hoped for.
- **Logs are raw timestamped protocol frames**, so replay reuses the same decoder as live
  operation rather than a second parser that can drift.

## Known limitations

- **Designed and approved (2026-09-30), not yet implemented.** See
  `design-proposal.md`, ADR 0002 (framing, CRC-32/ISO-HDLC, little-endian, resync) and
  ADR 0003 (YAML schema, Python generator).
- Protocol schema changes remain an owner-reviewed area: propose, then wait for approval.
- `../robot_bridge/` is constrained to frame-level awareness only and must not import the
  generated message definitions.

## How to test

Once implemented: round-trip encode/decode unit tests in both languages, cross-language
round-trip tests, a fuzz harness against the decoder, and explicit tests for `uint32_t`
microsecond timestamp wraparound (~71 minutes).
