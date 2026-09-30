"""Tests for the Python frame codec (protocol/codec.py).

Cross-language agreement with the C++ implementation is verified separately in
test_cross_language.py; this file tests Python's behavior on its own, including
property-based fuzzing of the decoder against arbitrary bytes.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO_ROOT / "protocol"), str(REPO_ROOT / "protocol" / "generated")]

import codec  # noqa: E402
import messages as M  # noqa: E402


# ------------------------------------------------------------------------------- COBS

@given(st.binary(max_size=600))
@settings(max_examples=400)
def test_cobs_round_trips_arbitrary_data(data: bytes) -> None:
    encoded = codec.cobs_encode(data)
    assert 0 not in encoded, "encoded output must be delimiter-free"
    assert codec.cobs_decode(encoded) == data


@pytest.mark.parametrize("data", [b"", b"\x00", b"\x00\x00", b"\x11\x00", b"\x00\x11",
                                  b"\xaa" * 253, b"\xaa" * 254, b"\xaa" * 255,
                                  b"\xaa" * 254 + b"\x00", b"\xaa" * 508])
def test_cobs_block_boundaries_and_trailing_zeros(data: bytes) -> None:
    assert codec.cobs_decode(codec.cobs_encode(data)) == data


def test_cobs_rejects_malformed_input() -> None:
    with pytest.raises(codec.CobsError):
        codec.cobs_decode(b"")
    with pytest.raises(codec.CobsError):
        codec.cobs_decode(b"\x03\x11\x00")      # zero inside literals
    with pytest.raises(codec.CobsError):
        codec.cobs_decode(b"\x05\x11\x22")      # promises more than it carries
    with pytest.raises(codec.CobsError):
        codec.cobs_decode(b"\x00")              # zero code byte


# -------------------------------------------------------------------------- framing

def test_frame_round_trip() -> None:
    payload = M.DriveCommand(1.5, -0.25).encode()
    wire = codec.encode_frame(M.MessageId.DriveCommand, 42, 0xDEADBEEF, payload)
    assert wire[-1] == 0x00
    assert 0 not in wire[:-1]

    decoder = codec.FrameDecoder()
    frames = decoder.push(wire)
    assert len(frames) == 1
    assert frames[0].message_id == M.MessageId.DriveCommand
    assert frames[0].seq == 42
    assert frames[0].timestamp_us == 0xDEADBEEF
    assert frames[0].payload == payload
    assert decoder.stats.frames_ok == 1


def test_every_declared_message_round_trips() -> None:
    decoder = codec.FrameDecoder()
    for message_id, declared in M.PAYLOAD_BYTES_BY_ID.items():
        wire = codec.encode_frame(message_id, 1, 1000, bytes(declared))
        frames = decoder.push(wire)
        assert len(frames) == 1, f"id 0x{message_id:02X}"
        assert len(frames[0].payload) == declared


def test_encode_rejects_bad_arguments() -> None:
    with pytest.raises(ValueError):
        codec.encode_frame(0x02, 0, 0, bytes(M.MAX_PAYLOAD_BYTES + 1))
    with pytest.raises(ValueError):
        codec.encode_frame(0x02, 0, 1 << 32, b"")
    with pytest.raises(ValueError):
        codec.encode_frame(0x02, 256, 0, b"")


@pytest.mark.parametrize("timestamp", [0, 1, 2**31 - 1, 2**31, 2**32 - 2, 2**32 - 1])
def test_timestamp_extremes_survive(timestamp: int) -> None:
    wire = codec.encode_frame(0x01, 0, timestamp, b"")
    frames = codec.FrameDecoder().push(wire)
    assert frames[0].timestamp_us == timestamp


def test_rejects_unknown_id_wrong_length_and_bad_version() -> None:
    decoder = codec.FrameDecoder()

    decoder.push(codec.encode_frame(0x03, 0, 0, bytes(4)))   # 0x03 is reserved
    assert decoder.stats.unknown_id == 1

    decoder.push(codec.encode_frame(0x02, 0, 0, bytes(6)))   # DriveCommand declares 8
    assert decoder.stats.length_mismatch == 1

    # Hand-build a frame with a wrong version byte.
    header = bytes([M.PROTOCOL_VERSION + 1, 0x01, 0]) + (0).to_bytes(4, "little")
    logical = header + codec.crc32(header).to_bytes(4, "little")
    decoder.push(codec.cobs_encode(logical) + b"\x00")
    assert decoder.stats.version_mismatch == 1

    assert decoder.stats.frames_ok == 0


def test_detects_single_bit_corruption() -> None:
    payload = M.DriveCommand(1.0, 1.0).encode()
    wire = bytearray(codec.encode_frame(0x02, 1, 100, payload))
    caught = 0
    for index in range(len(wire) - 1):          # leave the delimiter alone
        for bit in range(8):
            corrupted = bytearray(wire)
            corrupted[index] ^= 1 << bit
            if 0 in corrupted[:-1]:
                continue                        # becomes a delimiter: a framing case
            decoder = codec.FrameDecoder()
            if not decoder.push(bytes(corrupted)):
                caught += 1
    assert caught > 0
    # Every non-delimiter single-bit flip must be rejected by something.
    for index in range(len(wire) - 1):
        for bit in range(8):
            corrupted = bytearray(wire)
            corrupted[index] ^= 1 << bit
            if 0 in corrupted[:-1]:
                continue
            assert not codec.FrameDecoder().push(bytes(corrupted)), \
                f"bit {bit} of byte {index} slipped through"


# ------------------------------------------------------------------ resynchronization

def test_recovers_after_delimiter_terminated_garbage() -> None:
    """Garbage that ends with a delimiter costs nothing: the next frame starts clean."""
    first = codec.encode_frame(0x01, 1, 100, b"")
    second = codec.encode_frame(0x01, 2, 200, b"")
    garbage = bytes([0x7F, 0x01, 0xFE, 0x42, 0x00, 0xAB, 0xCD, 0x00])
    frames = codec.FrameDecoder().push(first + garbage + second)
    assert [f.seq for f in frames] == [1, 2]


def test_garbage_not_ending_in_a_delimiter_costs_the_next_frame() -> None:
    """The other half of the guarantee, and the reason the test above terminates its noise.

    Trailing garbage with no delimiter leaves the decoder accumulating, so the next
    frame's bytes join that partial run and its delimiter closes garbage-plus-frame.
    Exactly one frame is lost, never more.
    """
    first = codec.encode_frame(0x01, 1, 100, b"")
    second = codec.encode_frame(0x01, 2, 200, b"")
    third = codec.encode_frame(0x01, 3, 300, b"")
    frames = codec.FrameDecoder().push(first + b"\xab" + second + third)
    assert [f.seq for f in frames] == [1, 3]


def test_loses_at_most_one_frame_after_undelimited_garbage() -> None:
    """Matches the guarantee documented in firmware/core/protocol/frame.hpp."""
    decoder = codec.FrameDecoder()
    decoder.push(b"\xaa" * 20)                 # partial run, no delimiter
    assert decoder.push(codec.encode_frame(0x01, 1, 100, b"")) == []
    recovered = decoder.push(codec.encode_frame(0x01, 2, 200, b""))
    assert [f.seq for f in recovered] == [2]


def test_overlong_run_desyncs_once_then_recovers() -> None:
    decoder = codec.FrameDecoder()
    decoder.push(b"\xaa" * (4 * M.MAX_WIRE_FRAME_BYTES))
    assert decoder.stats.desyncs == 1, "one unbroken run is one desync"
    decoder.push(b"\x00")
    frames = decoder.push(codec.encode_frame(0x01, 9, 900, b""))
    assert [f.seq for f in frames] == [9]


def test_back_to_back_delimiters_are_not_errors() -> None:
    decoder = codec.FrameDecoder()
    decoder.push(b"\x00" * 10)
    assert vars(decoder.stats) == vars(codec.Stats())


def test_byte_at_a_time_matches_bulk_push() -> None:
    wire = codec.encode_frame(0x21, 3, 300, bytes(31))
    bulk = codec.FrameDecoder().push(wire)
    streamed = []
    decoder = codec.FrameDecoder()
    for byte in wire:
        streamed.extend(decoder.push(bytes([byte])))
    assert bulk == streamed


# --------------------------------------------------------------------------- fuzzing

@given(st.binary(max_size=400))
@settings(max_examples=600)
def test_decoder_never_raises_on_arbitrary_input(data: bytes) -> None:
    decoder = codec.FrameDecoder()
    for frame in decoder.push(data):
        # Anything accepted must satisfy the decoder's own invariant.
        assert M.PAYLOAD_BYTES_BY_ID[frame.message_id] == len(frame.payload)


@given(st.binary(max_size=200), st.binary(max_size=200))
@settings(max_examples=300)
def test_a_valid_frame_survives_arbitrary_surrounding_noise(pre: bytes, post: bytes) -> None:
    wire = codec.encode_frame(0x01, 5, 500, b"")
    decoder = codec.FrameDecoder()
    frames = decoder.push(pre + b"\x00" + wire + post)
    assert any(f.seq == 5 for f in frames), "a delimiter before the frame guarantees it"


# ----------------------------------------------------------------- timestamp unwrap

def test_timestamp_unwrapper_produces_a_monotonic_timeline() -> None:
    unwrapper = codec.TimestampUnwrapper()
    period = codec.TimestampUnwrapper.PERIOD_US
    assert unwrapper.unwrap(0) == 0
    assert unwrapper.unwrap(1_000_000) == 1_000_000
    assert unwrapper.unwrap(period - 1) == period - 1
    assert unwrapper.unwrap(5) == period + 5, "a large backwards step is a wrap"
    assert unwrapper.wraps == 1
    assert unwrapper.unwrap(10) == period + 10


def test_timestamp_unwrapper_tolerates_udp_reordering() -> None:
    """A small backwards step is a late-arriving datagram, not a 71-minute wrap."""
    unwrapper = codec.TimestampUnwrapper(tolerance_us=1_000_000)
    unwrapper.unwrap(10_000_000)
    assert unwrapper.unwrap(9_900_000) == 9_900_000
    assert unwrapper.wraps == 0
