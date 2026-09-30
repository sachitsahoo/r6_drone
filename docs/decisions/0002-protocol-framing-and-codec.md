# 0002 — Protocol framing and codec

- **Status:** **PROPOSED — awaiting owner approval. Do not implement.**
- **Date:** 2026-09-30
- **Supersedes:** nothing
- **Related:** [0003](0003-schema-format-and-code-generation.md), [`protocol/design-proposal.md`](../../protocol/design-proposal.md)

## Context

CLAUDE.md fixes the framing ingredients — COBS, CRC-16, message ID, sequence number,
timestamp, protocol version — but not their layout, the CRC parameters, the byte order, or
where COBS sits relative to the CRC. It also requires that the decoder survive arbitrary
garbage and that logs be raw frames replayed through the same decoder.

The link is not one link. The same frames travel MCU->UART->Pi->UDP->laptop, and land in a
log file. UART is a byte stream with no framing and no reordering. UDP has framing but can
reorder, duplicate, and drop whole datagrams. A log file is a byte stream again.

## Decision 1: COBS wraps the CRC, not the reverse

Logical frame, before any transport encoding:

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | `protocol_version` | u8 |
| 1 | 1 | `message_id` | u8 |
| 2 | 1 | `seq` | u8, per-sender, increments every frame sent |
| 3 | 4 | `timestamp_us` | u32 LE, microseconds since MCU boot |
| 7 | N | `payload` | fixed size per `message_id`, N <= 64 |
| 7+N | 4 | `crc32` | u32 LE, computed over bytes `[0, 7+N)` |

On the wire: `COBS(header + payload + crc) + 0x00`.

CRC is computed over the *logical* frame and then the whole thing is COBS-encoded. The
alternative — CRC over the already-encoded bytes — was rejected because it forces the
decoder to trust a COBS decode before it has validated anything, and a corrupted COBS length
byte can produce a decode of the wrong length that then has to be unwound.

**`protocol_version` is byte 0 deliberately.** It must be readable by a decoder of any
vintage, including one that understands nothing else in the frame. Any other position makes
the version field dependent on the layout it exists to arbitrate.

**There is no length field.** COBS plus the `0x00` delimiter already makes frame boundaries
unambiguous, so a length byte would be redundant information that can disagree with reality
— a failure mode with no upside. Instead, every `message_id` has a schema-fixed payload
size, and the decoder rejects any frame whose decoded length does not match the size
declared for its ID. That is a stronger check than a self-reported length.

**Every message is fixed-size.** No variable-length payloads, no strings. This follows from
the no-dynamic-allocation rule and makes the length check exact rather than a range test.
Anything textual becomes a fixed `char[N]`, and the preference is to send an enum code plus a
numeric context value instead.

## Decision 2: CRC-32/ISO-HDLC

Parameters: polynomial `0x04C11DB7`, init `0xFFFFFFFF`, input reflected, output reflected,
final XOR `0xFFFFFFFF`. Known-answer check: CRC of the ASCII string `123456789` is
`0xCBF43926`. This is the Ethernet / zlib / PNG CRC.

**This deviates from CLAUDE.md, which specifies CRC-16.** The deviation is the substance of
this ADR and needs an explicit decision, not a silent change.

An earlier revision of this ADR chose CRC-16/CCITT-FALSE and justified it on frame size,
flash cost, and a layered defense. Those arguments did not survive being computed:

| | CRC-16 | CRC-32 |
|---|---|---|
| Robot -> operator throughput | 4569 B/s, 39.7% of a 115 200 baud UART | 4801 B/s, 41.7% |
| Byte-table size in flash | 512 B | 1024 B, or 0.20% of the G474RE's 512 KB |
| Cost per byte | one table lookup, XOR, shift | identical; `uint32` is native on a 32-bit core |
| False accept, 1000 candidate frames/s | one per 66 s | one per 49.7 days |
| Hamming distance at ~600-bit frames | 4 (all 3-bit errors detected) | 5 (all 4-bit errors) |

Two extra bytes per frame and 512 extra bytes of flash buy a 65 536-fold reduction in false
accepts. At CRC-16, a link generating garbage at 1000 delimiter-bounded candidates per second
admits a bad frame about once a minute; at CRC-32 it is once every seven weeks. The first
number is a hazard for a robot with motors; the second is not.

There is also a code-risk argument that points only at this specific polynomial: Python's
stdlib provides `zlib.crc32`, which computes exactly CRC-32/ISO-HDLC. The Python side
therefore needs no CRC implementation to write, test, or keep in agreement with the C++ one,
and the C++ table can be verified directly against it. Choosing CRC-16 would mean
hand-writing and maintaining a second implementation, which is a real source of
cross-language drift in exchange for nothing.

Options considered and rejected:

1. **CRC-16/CCITT-FALSE** (`0x1021`, init `0xFFFF`). The previous choice. Adequate for
   detecting bit corruption in real frames, and 65 536 times weaker against whole garbage
   frames, for savings that round to zero on this link.
2. **CRC-16/XMODEM.** Same polynomial with a zero init, so leading zero bytes do not change
   the CRC. Strictly worse than option 1 with no compensating benefit.
3. **CRC-32C (Castagnoli, `0x1EDC6F41`).** Better Hamming distance at long lengths and
   hardware-accelerated on some cores. Rejected: the G474RE's CRC peripheral does not
   accelerate it for free in a way that matters here, it is not in Python's stdlib, and at
   600-bit frames its advantage over ISO-HDLC is not the binding constraint.
4. **A cryptographic MAC.** Worth stating explicitly: no CRC of any width protects against
   *intentional* modification, since an attacker simply recomputes it. If hostile
   interference on the radio link ever becomes a requirement, that is a different mechanism
   (HMAC and a key), not a wider CRC. Out of scope now per CLAUDE.md; noted so the limit of
   this decision is on the record.

Implementation: a `static constexpr uint32_t[256]` table in flash, one lookup and one XOR
per byte, constant execution time inside a fixed-rate loop.

## Decision 3: Little-endian, with explicit byte-by-byte serialization

All multi-byte fields are little-endian. Both ends of this link are little-endian — Cortex-M4
and the laptop — so LE means zero byte swaps in the hot path. Big-endian "network order" is
the reflex for internet protocols and would cost a swap per field on both ends of a
point-to-point link for no benefit.

Generated code serializes field by field with explicit shifts and masks. It does **not**
`memcpy` a packed struct or type-pun. Reasons: no dependence on compiler struct padding, no
`#pragma pack`, and no unaligned multi-byte access on the target.

## Decision 4: Resynchronization is "discard to the next delimiter"

Because COBS guarantees no `0x00` inside an encoded frame, `0x00` is an unambiguous
terminator. The decoder accumulates bytes into a fixed buffer sized from the schema
(`kMaxWireFrameBytes`) and on each `0x00` attempts: COBS decode, minimum length, version
match, CRC, known `message_id`, exact length for that ID, then per-field range validation.

On **any** failure the buffer is discarded, a specific counter is incremented, and scanning
continues from the next byte.

On **buffer overflow** — more bytes arrive than any legal frame could contain without a
delimiter — the decoder must enter an explicit desynchronized state and discard bytes *until
it sees the next `0x00`*, rather than simply resetting the buffer. A plain reset
resynchronizes in the middle of the offending frame and then misinterprets its tail as a new
frame header. This is the classic framing bug and is called out here so it is tested
deliberately rather than discovered.

## Consequences

- **One decoder for UART, UDP, and logs.** COBS is retained end to end even though UDP
  already provides datagram boundaries. That is roughly 1% redundant overhead in exchange for
  a single codec, a single fuzz target, and logs that are byte-identical to the wire — which
  is what makes "replay reuses the same decoder" literally true, including for corrupt
  sessions.
- **UDP can reorder and duplicate; UART cannot.** The operator must therefore tolerate
  out-of-order frames. Proposed rule: telemetry consumers keep the newest frame by
  `timestamp_us`, and the MCU rejects any command whose `timestamp_us` is not newer than the
  last accepted command, so a delayed or duplicated command cannot be replayed as a fresh one.
- **Range validation and stale-command rejection stay, and are not a CRC substitute.** An
  earlier revision presented them as an alternative to widening the CRC, which was wrong:
  they defend against *bugs* — an operator sending NaN, a units error, a duplicated UDP
  datagram replayed as a fresh command — not only against corruption. They belong in the
  design at any CRC width. Details in `protocol/design-proposal.md`.
- **Magic constants on state-changing messages are retained, with a different rationale.**
  At CRC-32 they are no longer compensating for a weak checksum; they guard against a
  mis-routed or mis-generated message, which is a software fault rather than a link fault.
  Four bytes on messages sent once per session is not a cost worth optimizing.
- **`seq` is u8, so it wraps every 256 frames** (2.56 s at 100 Hz). Gap detection is exact up
  to 255 consecutive losses and blind to exactly-256 losses. `timestamp_us` is the
  authoritative timeline; `seq` exists only for cheap loss counting.
- **`timestamp_us` wraps every ~71.6 minutes.** The MCU never needs absolute time, but the
  operator does for logs and plots, so the generated Python includes a timestamp unwrapper.
  Bench sessions longer than 71 minutes are entirely plausible.
- Fixed-size messages mean adding a field is always a wire-incompatible change, which bumps
  `protocol_version` and forces both ends to be rebuilt. Accepted: strict version equality is
  simpler and safer than negotiating compatibility for a two-node system.
