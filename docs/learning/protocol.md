# Learning notes: the protocol and codec

A plain-language walkthrough of `protocol/` and `firmware/core/protocol/`, followed by
check-your-understanding questions. Written because the owner has to explain and defend
this in interviews and a research writeup, not just make it work.

Reference documents: [ADR 0002](../decisions/0002-protocol-framing-and-codec.md) (framing
and codec), [ADR 0003](../decisions/0003-schema-format-and-code-generation.md) (schema and
generation), [`protocol/design-proposal.md`](../../protocol/design-proposal.md).

---

## The problem this solves

The robot and the laptop need to exchange structured data over a link that is nothing like
a function call. It is a stream of bytes with no message boundaries, it drops bytes, it
corrupts bytes, and over UDP it can deliver datagrams out of order or twice. Between the
two ends there are three different transports — UART, UDP, and a log file — and a single
decoder has to handle all three.

Four separate problems, each solved by one layer:

| Problem | Solution |
|---|---|
| Where does a message start and end? | COBS + a `0x00` delimiter |
| Did it arrive intact? | CRC-32 |
| What kind of message is it, and what's in it? | message id + a schema-generated struct |
| Do both ends agree on the format? | one schema, generated bindings, a version byte |

## Layer 1: COBS gives us frame boundaries

A UART hands you bytes with no notion of a message. The obvious fix is a delimiter byte —
but any byte you pick can also appear in the data, and then a payload containing that byte
splits a frame in half.

COBS (Consistent Overhead Byte Stuffing) removes every zero byte from the data. What comes
out is guaranteed zero-free, so `0x00` becomes an unambiguous "frame ends here".

It works by replacing each zero with a count. The encoder walks the data building a block
of non-zero bytes; when it hits a zero (or fills 254 bytes) it writes the block's length
in front of it. The decoder reads a length, copies that many bytes, and — if the block was
shorter than the 254-byte maximum and input remains — puts a zero back.

Cost: one byte per 254 bytes, plus one to start. About 0.4% for our frame sizes.

**The subtlety that bit us.** When the data *ends* with a zero, the final length byte is
what encodes that zero. An earlier version of `cobs_encode` "optimized away" what looked
like a redundant trailing length byte, which silently dropped the last byte of any payload
ending in `0x00` — including a telemetry frame whose last field happened to be zero. Six
tests caught it. See `firmware/core/protocol/cobs.cpp`, which now carries a comment saying
not to do that again.

## Layer 2: CRC-32 tells us whether to trust the bytes

A CRC is a fixed-size fingerprint of the data. The sender computes it and appends it; the
receiver recomputes it and compares. Different bytes almost always produce a different
fingerprint.

"Almost always" is the whole design question. A 16-bit CRC has 65,536 possible values, so
a completely random frame has about a 1-in-65,536 chance of passing. That sounds tiny until
you count: on a noisy link producing garbage continuously, a bad frame gets through about
once a minute. On a robot with motors, "once a minute a garbage frame is accepted as a
command" is not acceptable.

CRC-32 has about 4.3 billion possible values, so the same garbage stream gets one through
every seven weeks instead. It costs 2 bytes per frame and 512 extra bytes of lookup table —
0.2% of the MCU's flash.

We use **CRC-32/ISO-HDLC**, the same variant Ethernet and zlib use, and there is a second
reason beyond strength: Python's standard library already computes exactly it
(`zlib.crc32`). The Python side of the protocol therefore contains no CRC code at all, so
there is no second implementation to keep in agreement with the C++ one.

**Why the known-answer test matters.** A wrong CRC implementation still agrees with
itself, so a round-trip test passes happily. The only way to know you implemented the
*standard* CRC is to check a published value: `CRC("123456789") == 0xCBF43926`.

## Layer 3: the frame

```
version:u8 | message_id:u8 | seq:u8 | timestamp_us:u32 | payload:N | crc32:u32
```

then COBS-encode all of that and append `0x00`.

Three decisions worth understanding:

- **Version is byte 0.** It has to be readable by a decoder that understands nothing else
  in the frame. If it were anywhere else, its position would depend on the layout it exists
  to arbitrate.
- **There is no length field.** COBS plus the delimiter already tells you where the frame
  ends, so a length byte would be a second, redundant claim that can disagree with the
  first. Instead every message id has a schema-fixed payload size, and the decoder checks
  the actual decoded length against it. A frame that lies about its own length fails.
- **The CRC covers the logical frame, before COBS.** So the checksum protects the content
  regardless of transport, and the decoder never has to trust a COBS decode before it has
  validated anything.

## Layer 4: the decoder state machine, and the bug worth knowing about

The decoder has two states. In `kAccumulating` it collects bytes until a `0x00`. On the
delimiter it validates: COBS decode, minimum length, version, CRC, known id, exact length.
Any failure increments a specific counter and it starts over.

The interesting state is `kDesync`, entered when more bytes arrive than any legal frame
could hold without a delimiter. The tempting thing to do is clear the buffer and carry on.
**That is wrong**, and it is the classic framing bug: clearing the buffer re-aligns you in
the *middle* of the offending run, so the next bytes you read are that run's tail, which
you then interpret as a fresh header. The correct behavior is to discard *until the next
delimiter*, because only a delimiter tells you where a frame actually begins.

**The resynchronization guarantee.** After an arbitrary run of garbage, at most one
subsequent frame is lost. If garbage ends without a delimiter, the decoder is still
accumulating, so the next frame's bytes join the partial run and its delimiter closes
garbage-plus-frame, which correctly fails. The frame after that decodes normally. This is
inherent to delimiter framing, not a defect — and it is why a single lost frame after a
link disturbance must not be treated as a fault.

## Layer 5: the schema, and why there is a generator

The schema is one YAML file. Everything else — C++ structs, Python dataclasses, encode and
decode functions, size tables, range checks — is generated from it. Nobody hand-edits the
output; it is gitignored so that editing it is impossible to mistake for editing the
protocol.

The reason is drift. With two hand-written implementations, moving a field means editing
two files, and the day you forget one you get a bug that looks like corrupted hardware.

The generator earns more than that, though, because the schema carries **units and
ranges**:

- It refuses to build if a field declaring `unit: m/s` isn't named `..._m_s`. That turns a
  documentation convention into a compile error. It caught a real violation the first time
  it ran, on parameters named `_hz` instead of `_Hz`.
- It refuses to build if a range or default has no `source:` note saying where the number
  came from.
- It emits the range checks into the decoder, so a frame that passes the CRC still has to
  carry physically plausible values.

**NaN.** The range check is `value >= min && value <= max`, and NaN compares false against
both, so NaN is rejected. That is deliberate and easy to break by "fixing" the comparison —
hence a test named for it. A NaN speed command reaching a PID controller poisons its
integrator permanently.

## What is deliberately not here

- **Message `0x03` is reserved and unassigned.** It is the camera pitch command, and
  whether it carries a target *angle* or a target *torque* depends on whether the actuator
  is a geared servo or an FOC gimbal motor — which has no ADR yet. Reserving the id costs
  nothing; guessing its semantics would cost a protocol version bump.
- **No controller gains in `params.yaml`.** Declaring `kp`/`ki`/`kd` would presuppose a PID
  form, which is itself a decision requiring an ADR.
- **No authentication.** A CRC of any width stops accidents, not attackers — anyone can
  recompute one. Resisting deliberate interference needs a MAC and a key, which is out of
  scope.

---

## Check your understanding

**1. Why can't we just use `0xFF` as a frame delimiter and skip COBS entirely?**

Because `0xFF` appears in real data — any payload containing it would be split in two. The
problem isn't which byte you choose; it's that no byte is safe unless you first guarantee
it cannot appear in the data. COBS provides that guarantee for `0x00`.

**2. The frame has no length field. How does the decoder know a 44-byte frame wasn't
supposed to be 45 bytes?**

Two independent mechanisms. COBS plus the delimiter defines where the frame ends, so the
decoded length is a fact rather than a claim. Then the message id is looked up in a
schema-generated table and the decoded payload length must match the size that id declares.
A length *field* would add a third number that can contradict the other two.

**3. Why is the CRC computed before COBS encoding rather than after?**

So the checksum protects the logical content independently of how it's transported, and so
the decoder never has to trust a COBS decode before validating anything. If the CRC covered
the encoded bytes, a corrupted COBS length byte could produce a wrong-length decode that
then has to be unwound.

**4. We check the version byte before the CRC. What does that cost us?**

Attribution. Corruption landing on byte 0 gets counted as `version_mismatch` rather than
`crc_errors`, so the statistics slightly misreport what happened. We accept it because byte
0 must be readable by a decoder that agrees with us about nothing else — the alternative is
computing a CRC over frames from an incompatible peer.

**5. On buffer overflow, why isn't it enough to clear the buffer and keep going?**

Because clearing the buffer re-aligns the decoder in the middle of the over-long run. The
bytes that arrive next are that run's tail, and the decoder reads them as a fresh header —
so one disturbance produces a stream of plausible-looking garbage frames. Only a delimiter
identifies a real frame boundary, so the decoder must discard until it sees one.

**6. `seq` is one byte, so it wraps every 256 frames — about 2.5 seconds at 100 Hz. Why is
that acceptable?**

Because `seq` is only for cheap loss counting, and `timestamp_us` is the authoritative
timeline. Gap detection is exact for up to 255 consecutive losses. Missing *exactly* 256 is
undetectable by `seq` alone, but a 2.5-second total dropout is obvious from the timestamps
and from the comms watchdog firing long before.

**7. Both CRC-32 and a 4-byte magic constant guard the e-stop message. Isn't that
redundant?**

They guard different things. The CRC catches corruption in transit. The magic constant
catches a *software* fault — a mis-routed or mis-generated message arriving with a valid
CRC because it was built correctly and sent to the wrong place. Distinct magic values also
mean a corrupted arm request cannot act as an e-stop.

**8. `zlib.crc32` already exists, and the MCU could have used its hardware CRC peripheral.
Why is there a hand-written table-driven implementation in `firmware/core/`?**

Because `firmware/core/` must compile and pass tests on a laptop with no STM32 headers —
that's the portability firewall the whole architecture rests on. Using the CRC peripheral
would put a vendor dependency in portable code. If profiling later shows the software CRC
matters, the right move is a HAL interface with a peripheral-backed implementation in
`firmware/stm32/`, not an `#ifdef` in `core`.

**9. Why is the generated code gitignored instead of committed?**

Committing it invites someone to edit it, which recreates the two-sources-of-truth problem
the schema exists to prevent. The cost is real and accepted: the generator becomes a hard
build dependency, so a clone with no Python and no PyYAML cannot build the firmware.

**10. Generating both languages from one schema means they can't disagree. Why do
cross-language tests exist at all?**

Generation guarantees they were *built* together, not that they *behave* the same. The
frame codec — COBS, CRC, the state machine — is hand-written in both languages, because the
frame layout doesn't vary with the schema. Two implementations can each match the ADR and
still disagree on a block boundary or on which rejection fires first. So
`test_cross_language.py` drives the real C++ code through a helper binary and compares
against Python and against vectors sourced from stdlib zlib and the COBS paper.
