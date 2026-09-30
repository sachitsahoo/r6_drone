# robot_bridge — Raspberry Pi Zero 2 W daemon

## What it does

Relays messages between the operator's UDP link and the MCU's UART, captures and streams
H.264 video from the Camera Module 3 Wide, and monitors link health.

## How it fits the architecture

The middle tier. It carries data; it does not make control decisions. Anything
safety-critical or timing-critical lives on the MCU, because the Pi runs a general-purpose
OS with no real-time guarantees and can be rebooted or lose its link at any time.

## Key design decisions

- **No control logic, ever.** If the bridge dies, the MCU's comms watchdog stops the motors
  after 200 ms. That only holds if the bridge is never in a control path.
- **Video and telemetry are separate paths.** Video is bulk and loss-tolerant; telemetry is
  small and ordered. Coupling them would let a video stall delay telemetry.

## Known limitations

- Empty as of Phase 1.
- **Constraint (decided 2026-09-30):** this daemon operates at frame level only. It counts
  `0x00` COBS delimiters and reads `seq` for loss detection, and must never decode payload
  fields or import the generated message definitions. That keeps a schema change from
  forcing a Pi redeploy. See `protocol/design-proposal.md`.
- Video pipeline latency is unmeasured and is a primary research metric. It belongs in
  [`../docs/bringup/`](../docs/bringup/) once measured.

## How to test

Python tests in [`../tests/python/`](../tests/python/) with a loopback serial port and a
local UDP socket, so relay logic is testable without a Pi:

```
pytest
```
