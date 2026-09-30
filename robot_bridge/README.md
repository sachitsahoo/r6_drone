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
- Open question: how much protocol awareness this daemon needs. A pure byte relay needs
  none; link monitoring may need frame-level awareness (COBS delimiters, sequence numbers)
  without field-level schema. Field-level coupling means a schema change forces a Pi
  redeploy, so the intent is to stay at frame level if monitoring allows it.
- Video pipeline latency is unmeasured and is a primary research metric. It belongs in
  [`../docs/bringup/`](../docs/bringup/) once measured.

## How to test

Python tests in [`../tests/python/`](../tests/python/) with a loopback serial port and a
local UDP socket, so relay logic is testable without a Pi:

```
pytest
```
