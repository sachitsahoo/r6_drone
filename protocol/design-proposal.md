# Protocol design proposal

**STATUS: APPROVED 2026-09-30. Implementation not yet started.**

Companion to [ADR 0002](../docs/decisions/0002-protocol-framing-and-codec.md) (framing and
codec) and [ADR 0003](../docs/decisions/0003-schema-format-and-code-generation.md) (schema
and code generation). Those two record the decisions between alternatives; this document is
the concrete design: message set, schema shape, decoder behavior, and test plan.

## Goal

A schema-driven protocol and codec that lets the operator, the bridge, and the MCU exchange
commands, telemetry, and parameters over a link that drops, corrupts, reorders, and
duplicates — with the decoder proven against garbage rather than assumed safe, and with C++
and Python unable to drift apart.

## Files this would affect

```
protocol/schema/messages.yaml      new    message and enum definitions
protocol/schema/params.yaml        new    tunable parameter table
protocol/generate.py               new    generator (gitignored output)
protocol/generated/                new    messages.hpp, messages.py (gitignored)
protocol/test_vectors.json         new    shared known-answer frames, hand-checked
firmware/core/protocol/            new    cobs, crc32, frame decoder/encoder
tests/cpp/test_cobs.cpp            new
tests/cpp/test_crc32.cpp           new
tests/cpp/test_frame_codec.cpp     new
tests/cpp/fuzz_frame_decoder.cpp   new    libFuzzer target, plus a seeded fallback
tests/python/test_schema.py        new    schema validation and unit-suffix lint
tests/python/test_generator.py     new    golden-output determinism
tests/python/test_codec.py         new    round-trip + hypothesis fuzz
tests/python/test_cross_language.py new    both languages vs test_vectors.json
CMakeLists.txt                     edit   run generator before compiling consumers
requirements-dev.txt               edit   add PyYAML, hypothesis
```

## Frame layout (from ADR 0002)

Logical: `version:u8 | message_id:u8 | seq:u8 | timestamp_us:u32 | payload:N | crc32:u32`
Wire: `COBS(logical) + 0x00`, so wire size is `13 + N` for `N < 242`.

`MAX_PAYLOAD_BYTES = 64`, enforced by the generator. Buffer sizes in firmware derive from
that constant rather than being written down anywhere, satisfying the no-magic-numbers rule.

## Message set

ID ranges are partitioned by direction so a corrupt ID is far more likely to be rejected than
to be misread as a legal message travelling the wrong way.

### Operator -> robot (`0x01`–`0x1F`)

| ID | Name | Payload | Rate | Notes |
|---|---|---|---|---|
| `0x01` | `Heartbeat` | 0 B | 50 Hz | Keeps the comms watchdog fed while `DISARMED` |
| `0x02` | `DriveCommand` | 8 B | 50 Hz | `cmd_linear_speed_m_s:f32`, `cmd_angular_rate_rad_s:f32` |
| `0x04` | `SafetyStateRequest` | 5 B | on demand | `requested_state:u8`, `magic:u32` = `0x41524D21` |
| `0x05` | `EstopRequest` | 4 B | on demand | `magic:u32` = `0x45535450` |
| `0x06` | `ParamGet` | 2 B | on demand | `param_id:u16` |
| `0x07` | `ParamSet` | 7 B | on demand | `param_id:u16`, `type_tag:u8`, `value:u8[4]` |
| `0x08` | `ParamCommit` | 4 B | on demand | `magic:u32` = `0x434F4D54`; writes flash |

`0x03` is intentionally unassigned: it is reserved for the camera pitch command, which cannot
be designed until the actuator is chosen (see *Deliberately deferred*).

### Robot -> operator (`0x20`–`0x3F`)

| ID | Name | Payload | Rate | Notes |
|---|---|---|---|---|
| `0x21` | `StateTelemetry` | 31 B | 100 Hz | see below |
| `0x22` | `PowerTelemetry` | 12 B | 10 Hz | `bus_voltage_V:f32`, `current_A:f32`, `power_W:f32` |
| `0x23` | `LoopTiming` | 9 B | 5 Hz | `loop_id:u8`, `last_exec_us:u16`, `max_exec_us:u16`, `overrun_count:u32` |
| `0x24` | `Fault` | 6 B | on event | `fault_code:u16`, `context:u32` |
| `0x25` | `ParamValue` | 7 B | on demand | mirrors `ParamSet` |
| `0x26` | `Nack` | 3 B | on event | `rejected_message_id:u8`, `reason:u8`, `rejected_seq:u8` |
| `0x27` | `LinkStats` | 28 B | 1 Hz | decoder reject counters, below |

`StateTelemetry` (31 B): `wheel_speed_left_m_s:f32`, `wheel_speed_right_m_s:f32`,
`cmd_linear_speed_m_s:f32`, `cmd_angular_rate_rad_s:f32`, `body_pitch_rad:f32`,
`body_pitch_rate_rad_s:f32`, `camera_pitch_rad:f32`, `safety_state:u8`, `fault_flags:u16`.

Both pitch fields follow the project convention: **positive is nose-down**, and the schema
says so in the field description, so the generated doc comment says so at every point of use.

`LinkStats` (28 B): `frames_ok:u32`, `crc_errors:u32`, `cobs_errors:u32`,
`unknown_id:u32`, `length_mismatch:u32`, `range_rejects:u32`, `desyncs:u32` — seven u32
counters. Link quality becomes an observable rather than a guess, which matters both for
radio debugging and for the research writeup.

`safety_state` enum: `DISARMED=0`, `ARMED=1`, `FAULT=2`, `ESTOP=3`.

### Bandwidth

Robot -> operator at the rates above is about **4.8 kB/s, ~38 kbps** (with CRC-32 per ADR
0002). A 115 200 baud UART carries 11 520 B/s, so this uses roughly 42% of it — workable but with little headroom for
bursts of `Fault` or `Nack`. **Recommendation: run the MCU<->Pi UART at 460 800 or higher.**
Operator -> robot is about 1 kB/s and is not a concern.

## Defending safety-critical messages

ADR 0002 now specifies CRC-32, which puts a false accept at roughly one per seven weeks on
a link generating 1000 garbage candidates per second. The layers below are **not** there to
compensate for the checksum — they defend against software faults, which a CRC of any width
cannot see, and they would be in the design even at CRC-64:

1. **Magic constants on state-changing messages.** A distinct 4-byte value per message, so a
   mis-routed or mis-generated `SafetyStateRequest` cannot act as an `EstopRequest`. This
   guards against a software fault, not a link fault. The values are ASCII-derived (`ARM!`,
   `ESTP`, `COMT`) so they are readable in a hex dump.
2. **Schema-declared per-field ranges, validated in the decoder.** A corrupt `DriveCommand`
   that passes CRC still has to carry a plausible speed. This is cheap, generated, and
   protects every message rather than only the dangerous ones.
3. **Stale-command rejection.** The MCU rejects any command whose `timestamp_us` is not newer
   than the last accepted command of that type, so a duplicated or delayed UDP datagram
   cannot be replayed as a fresh command.

`ParamCommit` writes MCU flash. Proposed rule: **rejected unless `DISARMED`.** A flash write
stalls the core for milliseconds, which would blow a 1 kHz loop deadline, and flash endurance
is finite.

## Decoder state machine

Three states, no allocation, no blocking, safe to call from the UART receive path:

- **`ACCUMULATING`** — append each byte to a fixed buffer. On `0x00`, validate (below). On
  buffer full without a delimiter, go to `DESYNC`.
- **`DESYNC`** — discard every byte until the next `0x00`, then return to `ACCUMULATING`
  with an empty buffer. *This is the step that a naive implementation gets wrong by simply
  resetting the buffer, which resynchronizes mid-frame and then reads the tail of a frame as
  a header.*
- Validation order, cheapest and most-discriminating first: COBS decode -> minimum length ->
  `protocol_version` equality -> CRC -> known `message_id` -> exact payload length for that
  ID -> per-field range check. Each failure increments its own counter and returns to
  `ACCUMULATING`.

Rejections are counted and reported via `LinkStats`, and optionally `Nack`'d. A decoder that
silently drops frames is a decoder that cannot be debugged from the operator's seat.

## Schema shape

Illustrative, not final:

```yaml
protocol_version: 1
max_payload_bytes: 64

enums:
  SafetyState:
    values: [{name: DISARMED, value: 0}, {name: ARMED, value: 1},
             {name: FAULT, value: 2}, {name: ESTOP, value: 3}]

messages:
  - name: DriveCommand
    id: 0x02
    direction: operator_to_robot
    rate_hz: 50
    description: Body-frame velocity command. Diff-drive kinematics run on the MCU.
    fields:
      - name: cmd_linear_speed_m_s
        type: f32
        unit: m/s
        range: [-2.0, 2.0]
        source: "initial guess — to be tuned after wheel/gearbox characterization"
        description: Positive is forward.
      - name: cmd_angular_rate_rad_s
        type: f32
        unit: rad/s
        range: [-6.28, 6.28]
        source: "initial guess — to be tuned"
        description: Positive is counter-clockwise about +z (turning left).
```

Types: `u8 u16 u32 i8 i16 i32 f32`, plus fixed-length arrays. **No `f64`** — the Cortex-M4
FPU is single precision, so a double would be emulated in software inside a control loop.

The generator rejects the schema if: IDs collide or fall outside their direction's range, a
payload exceeds `max_payload_bytes`, a field name does not carry its declared unit suffix
(hard rule 5), or a range or default lacks a `source:` note (hard rule 6).

## Parameters

`protocol/schema/params.yaml` declares each tunable with `id:u16`, type, unit, range,
default, and `source:`. The wire format is deliberately uniform — `param_id`, `type_tag`,
4-byte value — so one message pair serves every parameter and the MCU needs no per-parameter
code. The generator emits a `constexpr` table for the MCU and typed accessors for Python.

## Test plan (tests written before implementation)

**C++**
- `crc32`: known-answer `CRC("123456789") == 0xCBF43926`, cross-checked against Python's
  stdlib `zlib.crc32`; empty input; single byte; every
  `test_vectors.json` frame.
- `cobs`: round-trip for all lengths 0–254 and for 254/255-byte runs (the block-boundary
  cases); encoded output provably contains no `0x00`; malformed-input rejection.
- `frame_codec`: round-trip every message; reject wrong version, bad CRC, unknown ID, wrong
  length, out-of-range field; `seq` gap counting across the u8 wrap; timestamp wrap.
- `frame_decoder` resync: valid frame, then garbage, then valid frame -> both valid frames
  recovered. Oversized run with no delimiter -> `DESYNC`, then clean recovery. Truncated
  frame followed by a valid one -> the valid one is recovered.
- **Fuzz** (`libFuzzer`, with a seeded deterministic fallback so CI works without clang):
  no crash, no out-of-bounds read, no infinite loop, decoder still usable afterward, and
  never emits a frame that fails its own validation.

**Python**
- Schema validation, including deliberately bad schemas: colliding IDs, missing `source:`,
  a unit/suffix mismatch. Each must fail with a clear message.
- Generator determinism against golden output.
- Codec round-trip, plus `hypothesis` property tests over arbitrary byte strings.
- **Cross-language**: `test_vectors.json` holds hand-computed frames; both C++ and Python
  must encode to and decode from those exact bytes. This is the mechanism that catches
  drift, and it must be hand-checked at least once rather than generated by the code it tests.

**Acceptance criteria:** every test above passes on host and in CI; the cross-compile still
builds; `tools/check_core_purity.py` still passes over the new `firmware/core/protocol/`
(no allocation, no vendor headers); and `docs/theory/` is not needed, but
`docs/learning/protocol.md` is written, since the codec is an owner-reviewed module.

## Deliberately deferred

- **Camera pitch command (`0x03`).** Whether the message carries a target angle or a target
  torque depends on the actuator choice (geared servo vs. FOC gimbal BLDC), which needs its
  own ADR. Reserving the ID now costs nothing; guessing the semantics would cost a protocol
  version bump later.
- **`StateTelemetry`'s pitch and rate fields are provisional.** What the estimator actually
  produces is an owner-reviewed design that does not exist yet. The field list will likely
  change, which is a version bump — cheap now, while nothing is deployed.
- **MQTT / video.** Out of scope; video is a separate path by design (`robot_bridge/README.md`).

## Owner decisions (2026-09-30)

All six questions resolved. Recorded here so the reasoning behind the design is traceable
without digging through conversation history.

1. **CRC-32 — approved**, including the deviation from CLAUDE.md's CRC-16. See ADR 0002 for
   the numbers that settled it. CLAUDE.md's framing section now disagrees with the
   implementation and should be updated.
2. **`ParamCommit` restricted to `DISARMED` — approved**, on the grounds that it prevents
   mistakes rather than merely protecting loop timing. Consequence: gains can be applied in
   RAM and tested while `ARMED`, but persisting them requires disarming first. The MCU
   returns `Nack` with a distinct reason code if a commit arrives while armed.
3. **`robot_bridge/` stays at frame level — confirmed.** It counts `0x00` delimiters and
   reads `seq` for loss detection, and never decodes payload fields. A schema change
   therefore never requires a Pi redeploy. This is now a constraint on the bridge, not a
   preference: `robot_bridge/` must not import the generated message definitions.
4. **Three telemetry messages at three rates — confirmed.** `StateTelemetry` at 100 Hz,
   `PowerTelemetry` at 10 Hz, `LoopTiming` at 5 Hz. Each message is an internally consistent
   snapshot; there is no cross-message atomicity, so the operator must not assume a
   `PowerTelemetry` sample is simultaneous with a `StateTelemetry` sample. Timestamps make
   the actual relationship explicit.
5. **`seq` as u8 — confirmed.** Loss detection is exact up to 255 consecutive dropped frames
   and blind to exactly 256. `timestamp_us` is the authoritative timeline.
6. **UART baud — analyzed separately.** Baud does not appear in the protocol, so it is a
   bring-up parameter rather than a design decision. Full analysis, including the Pi Zero 2 W
   UART clock problem that is the real constraint, is in
   [`docs/bringup/uart-link.md`](../docs/bringup/uart-link.md). Recommendation: 460 800.

## Pending schema changes — PROPOSED, awaiting owner approval

Protocol schema changes are owner-reviewed, so these are written down rather than applied.

### 1. `camera_pitch_rad` range — a live defect, recommend fixing

ADR 0004 settled that the outer casing rotates **continuously** about the wheel axis. The
field still declares `[-1.5708, 1.5708]` (plus or minus 90 degrees) from when a limited-travel
camera gimbal was assumed. Because the generator emits range validation into the decoder, this
does not mislabel data — it **drops frames**:

```
casing at   89 deg -> accepted
casing at   91 deg -> REJECTED (frame dropped)
casing at  120 deg -> REJECTED (frame dropped)
casing at -150 deg -> REJECTED (frame dropped)
```

Proposed change:

```yaml
      - name: camera_pitch_rad
        type: f32
        unit: rad
        range: [-3.1416, 3.1416]          # was [-1.5708, 1.5708]
        source: >
          full circle — the casing rotates continuously about the wheel axis (ADR 0004),
          so any angle is physically reachable
        description: >
          World-relative camera pitch, wrapped to [-pi, pi]. POSITIVE IS NOSE-DOWN.
          Consumers that differentiate or plot this must unwrap it; a wrap is a 2*pi jump,
          not motion.
```

**Wrapped rather than unbounded**, matching `body_pitch_rad`. An accumulating absolute angle
would need a revolution counter to stay meaningful and would slowly lose angular resolution in
`f32` — and nothing in the control loop needs to know how many turns the casing has made. If
turn count ever matters, it belongs in its own field rather than smuggled into this one.

**Cost: none beyond the edit.** The field stays `f32`, the payload stays 31 bytes, and the wire
layout is unchanged, so this needs no `protocol_version` bump — only a regenerate. Tests that
pin payload sizes stay valid.

### 2. `version_mismatch` field for `LinkStats` — recommend, lower priority

Carried over from the codec work. `FrameDecoder` counts `version_mismatch`, but `LinkStats`
has no field for it, so it is observable locally and never transmitted. A version mismatch is
exactly the kind of link problem the operator needs to see. Cost is 4 bytes at 1 Hz, taking the
payload from 28 to 32 bytes — which *does* change a documented size, so it touches
`tests/python/test_schema.py`, `tests/cpp/test_generated_messages.cpp`, and the bandwidth figure
in `docs/bringup/uart-link.md` (4805 to 4809 B/s, immaterial).

### 3. `body_pitch_rate_rad_s` is now mislabelled — recommend fixing, costs nothing

ADR 0005 puts the IMU on the rotating casing. The field currently reads:

```yaml
      - name: body_pitch_rate_rad_s
        source: ICM-42688-P default gyro full scale +/-2000 deg/s, converted to rad/s
        description: Measured chassis pitch rate. POSITIVE IS NOSE-DOWN.
```

The gyro no longer measures chassis rate — it measures *casing* rate. Chassis rate is derived
by subtracting the encoder's angular rate. Calling this "measured" invites someone to trust it
as a direct reading, and citing the gyro full scale as its source is now the wrong provenance.

Proposed: keep the name and range, change the description to "Chassis pitch rate, derived from
the casing gyro minus the actuator encoder rate (ADR 0005)" and the source to note that the
range still comes from the gyro full scale because the derived value cannot exceed it. Text
only — no wire change, no regenerate needed beyond the doc comments.

### 4. The casing rate the inner loop runs on is not transmitted — recommend adding

With the IMU on the casing, the gyro's casing pitch rate is the single most important signal
in the 500 Hz–1 kHz stabilization loop. `StateTelemetry` carries `camera_pitch_rad` (the angle)
and `body_pitch_rate_rad_s` (a derived chassis rate) but not the measured casing rate, so the
operator cannot see the inner loop's actual input without reconstructing it.

Proposed: add `camera_pitch_rate_rad_s: f32`, range +/-34.9 rad/s, sourced to the gyro full
scale, described as the directly measured casing rate.

**This one has a real cost**, unlike the others: payload goes 31 to 35 bytes, wire 44 to 48.
That touches the size assertions in `tests/python/test_schema.py` and
`tests/cpp/test_generated_messages.cpp`, and the bandwidth figure in
`docs/bringup/uart-link.md` (4805 to 5205 B/s, or 45% of a 460800 baud link — still
comfortable). No `protocol_version` bump is needed since no existing field moves; the payload
simply grows, and the decoder's length check handles the rest.

Worth waiting on until the estimator is designed, since that will settle what else belongs in
this message and one combined change beats three.

### 5. Possible redundancy between the two pitch fields — resolved direction, still open

If the IMU rides on the rotating casing, `camera_pitch_rad` is measured and `body_pitch_rad` is
derived from it via the actuator encoder; if it rides on the chassis, the reverse. Either way
one field is computed from the other plus the encoder, so transmitting both is arguably
redundant — but transmitting both is also how the operator sees the estimator's two outputs
without recomputing anything. Not worth deciding until IMU placement is.

## Remaining dependencies before implementation

None. The camera pitch command (`0x03`) stays reserved and unassigned pending the pitch
actuator ADR, and `StateTelemetry`'s pitch fields stay provisional pending the estimator
design. Neither blocks the schema, generator, codec, or tests.
