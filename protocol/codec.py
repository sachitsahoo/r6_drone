"""Frame-level protocol codec for the operator app, the tools, and log replay.

Hand-written, not generated: the frame layout is fixed by ADR 0002 and does not vary with
the schema. Only the message structs and size tables are generated (see generate.py).

This is the Python counterpart of firmware/core/protocol/{cobs,crc32,frame}.{hpp,cpp} and
must agree with it byte for byte. Agreement is verified by
tests/python/test_cross_language.py, which drives the real C++ code through a helper
binary rather than trusting that two implementations written from the same document match.

Note that there is no CRC implementation here. CRC-32/ISO-HDLC is exactly what stdlib
`zlib.crc32` computes, which is why ADR 0002 chose that variant: one less implementation
to write, test, and keep in agreement.
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass, field
from enum import Enum

from messages import (
    CRC_BYTES,
    HEADER_BYTES,
    MAX_PAYLOAD_BYTES,
    MAX_WIRE_FRAME_BYTES,
    PAYLOAD_BYTES_BY_ID,
    PROTOCOL_VERSION,
)

COBS_BLOCK_BYTES = 254
DELIMITER = 0x00
MIN_LOGICAL_FRAME_BYTES = HEADER_BYTES + CRC_BYTES


class RejectReason(Enum):
    """Why a run of bytes between two delimiters was not a valid frame."""

    COBS_ERROR = "cobs_errors"
    LENGTH_MISMATCH = "length_mismatch"
    VERSION_MISMATCH = "version_mismatch"
    CRC_ERROR = "crc_errors"
    UNKNOWN_ID = "unknown_id"


class CobsError(ValueError):
    """Raised when COBS-encoded input is malformed."""


def cobs_encode(data: bytes) -> bytes:
    """COBS-encodes `data`, excluding the delimiter.

    Mirrors firmware/core/protocol/cobs.cpp. The result never contains 0x00, so the caller
    may append one as a frame delimiter.
    """
    out = bytearray()
    block = bytearray()
    for byte in data:
        if byte == 0:
            out.append(len(block) + 1)
            out.extend(block)
            block.clear()
        else:
            block.append(byte)
            if len(block) == COBS_BLOCK_BYTES:
                out.append(0xFF)
                out.extend(block)
                block.clear()
    out.append(len(block) + 1)
    out.extend(block)
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """COBS-decodes `data`, which must have the delimiter already stripped.

    Raises CobsError on malformed input: an empty run, a zero byte anywhere, or a code
    byte promising more literals than remain.
    """
    if not data:
        raise CobsError("empty input cannot be a valid COBS encoding")

    out = bytearray()
    index = 0
    length = len(data)
    while index < length:
        code = data[index]
        if code == 0:
            raise CobsError(f"zero code byte at offset {index}")
        index += 1
        literals = code - 1
        if index + literals > length:
            raise CobsError(f"code byte at {index - 1} promises {literals} bytes, "
                            f"only {length - index} remain")
        chunk = data[index:index + literals]
        if 0 in chunk:
            raise CobsError(f"zero byte inside literal run at offset {index}")
        out.extend(chunk)
        index += literals
        if code != 0xFF and index < length:
            out.append(0)
    return bytes(out)


def crc32(data: bytes) -> int:
    """CRC-32/ISO-HDLC. stdlib zlib computes exactly the variant ADR 0002 specifies."""
    return zlib.crc32(data) & 0xFFFFFFFF


def encode_frame(message_id: int, seq: int, timestamp_us: int, payload: bytes) -> bytes:
    """Builds a complete wire frame including the trailing delimiter."""
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ValueError(f"payload {len(payload)} B exceeds {MAX_PAYLOAD_BYTES} B")
    if not 0 <= message_id <= 0xFF:
        raise ValueError(f"message_id {message_id} is not a byte")
    if not 0 <= seq <= 0xFF:
        raise ValueError(f"seq {seq} is not a byte")
    if not 0 <= timestamp_us <= 0xFFFFFFFF:
        raise ValueError(f"timestamp_us {timestamp_us} does not fit in u32")

    header = bytes([PROTOCOL_VERSION, message_id, seq]) + timestamp_us.to_bytes(4, "little")
    checked = header + payload
    logical = checked + crc32(checked).to_bytes(4, "little")
    return cobs_encode(logical) + bytes([DELIMITER])


@dataclass(frozen=True)
class DecodedFrame:
    message_id: int
    seq: int
    timestamp_us: int
    payload: bytes


def decode_logical_frame(logical: bytes) -> DecodedFrame:
    """Validates an already-COBS-decoded frame.

    Validation order matches ADR 0002 and firmware/core/protocol/frame.cpp: minimum
    length, version, CRC, known id, then exact payload length for that id.

    Raises ValueError, whose `args[0]` is the matching RejectReason.
    """
    if len(logical) < MIN_LOGICAL_FRAME_BYTES:
        raise ValueError(RejectReason.LENGTH_MISMATCH,
                         f"{len(logical)} B is shorter than a header plus CRC")
    if logical[0] != PROTOCOL_VERSION:
        raise ValueError(RejectReason.VERSION_MISMATCH,
                         f"version {logical[0]} != {PROTOCOL_VERSION}")

    payload_len = len(logical) - MIN_LOGICAL_FRAME_BYTES
    checked_len = HEADER_BYTES + payload_len
    received = int.from_bytes(logical[checked_len:checked_len + CRC_BYTES], "little")
    if crc32(logical[:checked_len]) != received:
        raise ValueError(RejectReason.CRC_ERROR, "CRC mismatch")

    message_id = logical[1]
    declared = PAYLOAD_BYTES_BY_ID.get(message_id)
    if declared is None:
        raise ValueError(RejectReason.UNKNOWN_ID, f"id 0x{message_id:02X} is not in the schema")
    if declared != payload_len:
        raise ValueError(RejectReason.LENGTH_MISMATCH,
                         f"id 0x{message_id:02X} declares {declared} B, got {payload_len} B")

    return DecodedFrame(
        message_id=message_id,
        seq=logical[2],
        timestamp_us=int.from_bytes(logical[3:7], "little"),
        payload=logical[HEADER_BYTES:HEADER_BYTES + payload_len],
    )


@dataclass
class Stats:
    frames_ok: int = 0
    crc_errors: int = 0
    cobs_errors: int = 0
    unknown_id: int = 0
    length_mismatch: int = 0
    version_mismatch: int = 0
    desyncs: int = 0


@dataclass
class FrameDecoder:
    """Streaming decoder, mirroring firmware/core/protocol/frame.hpp.

    Same resynchronization guarantee: after an arbitrary run of garbage, at most one
    subsequent frame is lost. See that header for why.
    """

    stats: Stats = field(default_factory=Stats)
    _buffer: bytearray = field(default_factory=bytearray, repr=False)
    _desync: bool = False

    def reset(self) -> None:
        self._buffer.clear()
        self._desync = False

    def push(self, data: bytes) -> list[DecodedFrame]:
        """Feeds bytes and returns every frame that completed and validated."""
        frames: list[DecodedFrame] = []
        for byte in data:
            if byte == DELIMITER:
                if self._desync:
                    self._desync = False
                    self._buffer.clear()
                    continue
                if self._buffer:
                    frame = self._validate(bytes(self._buffer))
                    if frame is not None:
                        frames.append(frame)
                self._buffer.clear()
                continue

            if self._desync:
                continue

            if len(self._buffer) >= MAX_WIRE_FRAME_BYTES:
                # Discard to the NEXT delimiter, not merely clear the buffer: a plain
                # reset re-aligns mid-frame and reads that frame's tail as a header.
                self._desync = True
                self.stats.desyncs += 1
                self._buffer.clear()
                continue

            self._buffer.append(byte)
        return frames

    def _validate(self, encoded: bytes) -> DecodedFrame | None:
        try:
            logical = cobs_decode(encoded)
        except CobsError:
            self.stats.cobs_errors += 1
            return None
        try:
            frame = decode_logical_frame(logical)
        except ValueError as exc:
            reason = exc.args[0]
            counter = reason.value if isinstance(reason, RejectReason) else "cobs_errors"
            setattr(self.stats, counter, getattr(self.stats, counter) + 1)
            return None
        self.stats.frames_ok += 1
        return frame


class TimestampUnwrapper:
    """Turns wrapping u32 microsecond timestamps into a monotonic timeline.

    MCU timestamps wrap every ~71.6 minutes (2^32 us). The MCU never needs absolute time,
    but logs and plots do, and bench sessions longer than 71 minutes are entirely
    plausible. See docs/decisions/0002 consequences.

    Detects a wrap when a timestamp jumps backwards by more than `tolerance_us`. The
    tolerance exists because UDP can reorder frames: a small backwards step is a
    late-arriving frame, not a wrap.
    """

    PERIOD_US = 1 << 32

    def __init__(self, tolerance_us: int = 1_000_000) -> None:
        self._tolerance_us = tolerance_us
        self._wraps = 0
        self._previous: int | None = None

    def unwrap(self, timestamp_us: int) -> int:
        """Returns a monotonically increasing microsecond count."""
        if self._previous is not None:
            backwards = self._previous - timestamp_us
            if backwards > self._tolerance_us:
                self._wraps += 1
        self._previous = timestamp_us
        return timestamp_us + self._wraps * self.PERIOD_US

    @property
    def wraps(self) -> int:
        return self._wraps
