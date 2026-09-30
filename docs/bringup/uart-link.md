# MCU <-> Pi UART link: baud rate selection

**Status:** **460 800 baud selected (owner, 2026-09-30).** Unvalidated on hardware.
**Date:** 2026-09-30

## Why this is a bring-up document and not an ADR

Baud rate does not appear anywhere in the protocol. The frame layout, CRC, message set, and
decoder are identical at every rate, so this is a physical-layer parameter: one constant on
each end, changeable in minutes, with no effect on logs, schema, or generated code. It does
not block the protocol implementation.

That is worth stating plainly because it means this decision carries almost no commitment.
It can be set optimistically and walked back if the link misbehaves.

## Numbers

Load is the proposed telemetry set (ADR 0002 framing, CRC-32): 4801 B/s robot -> operator,
44-byte telemetry frames, 21-byte command frames. UART 8N1 carries 10 bits per byte.

| Baud | Capacity | Used | One telemetry frame | One command frame | Per byte |
|---|---|---|---|---|---|
| 115 200 | 11 520 B/s | **41.7%** | 3.82 ms | 1.82 ms | 86.8 us |
| 230 400 | 23 040 B/s | 20.8% | 1.91 ms | 0.91 ms | 43.4 us |
| 460 800 | 46 080 B/s | **10.4%** | 0.95 ms | 0.46 ms | 21.7 us |
| 921 600 | 92 160 B/s | 5.2% | 0.48 ms | 0.23 ms | 10.9 us |

## Consequence 1: latency and jitter, not throughput, is the real argument

41.7% average utilization at 115 200 sounds comfortable. The problem is what one frame does
to the line: a 44-byte telemetry frame takes **3.82 ms to serialize, which is 38% of the
10 ms period** at 100 Hz telemetry.

Anything that needs to go out — a `Fault`, a `Nack`, a `ParamValue` reply — queues behind up
to 3.82 ms of in-flight frame. Fault reporting latency is then dominated by serialization
rather than by detection, and the jitter is not small relative to the loop rates this project
cares about. At 460 800 the same frame occupies the line for 0.95 ms and the queueing delay
becomes negligible.

The same applies in the command direction: 1.82 ms of serialization per `DriveCommand` at
115 200 is added latency between operator input and wheel response, on top of USB polling,
UDP transit, and the relay hop.

## Consequence 2: the Pi Zero 2 W's UART is the actual constraint

This is the real cost of going faster, and it is a configuration problem rather than a
signal-integrity one.

The Pi has two UARTs with very different behavior:

- **mini-UART (`/dev/ttyS0`)** derives its baud clock from the VPU core clock. That clock
  changes with CPU frequency scaling, so the effective baud rate drifts under load unless
  `core_freq` is pinned in `config.txt`. This produces the worst kind of bug: a link that
  works at idle and corrupts frames when the video pipeline loads the CPU.
- **PL011 (`/dev/ttyAMA0`)** has a stable clock and proper flow control, and is the one to
  use — but on a Pi Zero 2 W it is wired to the Bluetooth module by default, with the
  mini-UART on the GPIO header.

So choosing a high baud rate forces a decision about Pi UART configuration:
`dtoverlay=disable-bt` (frees PL011 for the header, loses Bluetooth) or
`dtoverlay=miniuart-bt` (swaps them, Bluetooth on the unstable clock). Bluetooth is not used
by this project, so `disable-bt` is the obvious choice — but it needs to be a deliberate,
documented step, not something discovered while debugging frame corruption.

**To verify at bring-up:** which device the GPIO header UART is, and whether `core_freq` is
pinned. Confirm with a sustained loopback test *while the video pipeline is running*, not
on an idle Pi.

## Consequence 3: clock tolerance is not the limit here

Asynchronous UART requires both ends to agree on bit timing from independent clocks. The
practical budget for 8N1 is roughly 2% total accumulated error.

With an HSE crystal on the G474RE, the divisor error at these rates is a small fraction of a
percent, so none of the candidate rates is limited by the MCU's clock accuracy. With the
internal HSI RC oscillator (roughly +/-1% over temperature) the margin tightens considerably
and high baud becomes questionable.

**Therefore: use the HSE crystal for the USART clock path.** The exact divisor error must be
computed at bring-up from the actual USART kernel clock after the clock tree is configured,
not assumed from the datasheet's maximum frequency.

## Consequence 4: DMA is required at every candidate rate

At 115 200 a byte arrives every 86.8 us; at 921 600, every 10.9 us. Neither is serviceable by
polling from a 1 kHz control loop, so **circular DMA receive into a ring buffer is mandatory
regardless of the rate chosen** — higher baud shrinks the margin but does not change the
architecture.

This matters because DMA configuration is an owner-reviewed area. The relevant consequence
is that a UART overrun must be handled rather than ignored: lost bytes produce framing
errors, which is exactly the path the decoder's `DESYNC` handling exists for. The
`LinkStats` counters make those events visible instead of mysterious.

## Decision: 460 800 baud

Selected by the owner on 2026-09-30. It cuts per-frame serialization from 3.82 ms to 0.95 ms and utilization from
42% to 10%, leaving real headroom for `Fault` bursts and for telemetry fields that do not
exist yet, while staying far from any clock-tolerance concern.

921 600 is also fine and buys little beyond 460 800 — the remaining serialization time is
already below the noise of the rest of the pipeline. 115 200 is not recommended: it works,
but it spends a third of every telemetry period on the wire for no reason.

Whatever is chosen, the link should be validated with a sustained loopback under CPU load
before any motors are enabled, with `LinkStats` counters checked for `cobs_errors` and
`desyncs`.

## What this decision obliges, at bring-up

460 800 is above the rate at which the Pi's mini-UART clock drift stops being theoretical, so
the items below are prerequisites rather than nice-to-haves. None has been done — no hardware
has been powered.

- [ ] Pi: `dtoverlay=disable-bt` in `config.txt`, so PL011 (`/dev/ttyAMA0`) lands on the GPIO
      header instead of the mini-UART. Bluetooth is unused by this project.
- [ ] Pi: confirm which device the header UART actually is after the overlay, rather than
      assuming. `dmesg` and `/dev/serial*` symlinks.
- [ ] MCU: USART clocked from the HSE crystal, not HSI. Compute the actual divisor error from
      the configured USART kernel clock and record the number here.
- [ ] Sustained loopback test at 460 800 **while the video pipeline is running**, not on an
      idle Pi. This is the test that catches core-clock drift; an idle-Pi test will pass
      regardless and prove nothing.
- [ ] Check `LinkStats` for `cobs_errors` and `desyncs` over a multi-minute run. Non-zero is
      a physical-layer problem, not a decoder problem.
- [ ] Only then enable motors, wheels off the ground, kill switch in reach.

The baud rate will appear in firmware as a named constant with a `source:` comment pointing at
this document, per the no-magic-numbers rule. If the loopback test fails, dropping to 230 400
costs one constant on each end and nothing else.
