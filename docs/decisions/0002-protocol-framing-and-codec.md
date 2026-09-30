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
| 7+N | 2 | `crc16` | u16 LE, computed over bytes `[0, 7+N)` |

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

## Decision 2: CRC-16/CCITT-FALSE

Parameters: polynomial `0x1021`, init `0xFFFF`, input not reflected, output not reflected,
final XOR `0x0000`. Known-answer check: CRC of the ASCII string `123456789` is `0x29B1`.

Options considered:

1. **CRC-16/CCITT-FALSE (chosen).** Hamming distance 4 for messages well beyond our
   ~600-bit frames, so every 1-, 2-, and 3-bit error is detected. The published check value
   gives a citable known-answer test, which matters because a silently wrong CRC
   implementation still round-trips against itself and looks fine.
2. **CRC-16/XMODEM.** Same polynomial but init `0x0000`. Rejected: with a zero init, leading
   zero bytes do not change the CRC. COBS removes zeros from the wire so this is close to
   harmless here, but there is no reason to choose the weaker of two otherwise identical options.
3. **CRC-16/MODBUS** (`0x8005`, reflected). Rejected: reflected algorithms are fine but
   less legible to verify by hand, and `0x1021` has better-published distance properties at
   our frame sizes.
4. **CRC-32.** Rejected for routine framing on size grounds, but see *Consequences* — there
   is a real safety argument for it that is being answered a different way.

Implementation: a `static constexpr uint16_t[256]` table, which lands in flash (512 bytes
of a 512 KB part) and costs one table lookup and one XOR per byte. At 100 Hz x ~75 bytes
this is negligible either way; the table is chosen for determinism of execution time, which
matters inside a fixed-rate loop.

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
- **A CRC-16 admits roughly 1 in 65 536 random frames.** On a noisy radio link producing
  garbage continuously, a bad frame passes validation every few minutes. That is tolerable for
  telemetry and intolerable for state changes, so the mitigation is layered rather than
  widening the CRC: schema-declared per-field ranges reject nonsense values, and any frame
  that changes safety state carries a 4-byte magic constant. Both are detailed in
  `protocol/design-proposal.md`. **This is the decision most worth challenging** — the
  alternative is CRC-32 on every frame for two extra bytes.
- **`seq` is u8, so it wraps every 256 frames** (2.56 s at 100 Hz). Gap detection is exact up
  to 255 consecutive losses and blind to exactly-256 losses. `timestamp_us` is the
  authoritative timeline; `seq` exists only for cheap loss counting.
- **`timestamp_us` wraps every ~71.6 minutes.** The MCU never needs absolute time, but the
  operator does for logs and plots, so the generated Python includes a timestamp unwrapper.
  Bench sessions longer than 71 minutes are entirely plausible.
- Fixed-size messages mean adding a field is always a wire-incompatible change, which bumps
  `protocol_version` and forces both ends to be rebuilt. Accepted: strict version equality is
  simpler and safer than negotiating compatibility for a two-node system.
