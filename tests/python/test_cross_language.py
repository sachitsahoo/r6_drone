"""Verifies the C++ and Python codecs agree byte for byte.

This is the mechanism that catches drift. Generating both languages from one schema
guarantees they were *built* together; it does not guarantee they *behave* identically.
Two implementations written from the same ADR can each match the document and still
disagree with each other -- on byte order, on a block boundary, on which rejection fires
first.

So these tests drive the real C++ code through tests/cpp/protocol_vector_tool and compare
against protocol/codec.py and against the committed vectors in protocol/test_vectors.json.
Those vectors come from stdlib zlib and from the COBS paper, not from the code under test.

Skipped when the C++ tool has not been built. CI always builds it, so a skip locally never
hides a real failure in CI.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO_ROOT / "protocol"), str(REPO_ROOT / "protocol" / "generated")]

import codec  # noqa: E402
import messages as M  # noqa: E402

VECTORS = json.loads((REPO_ROOT / "protocol" / "test_vectors.json").read_text())


def find_tool() -> Path | None:
    for candidate in sorted(REPO_ROOT.glob("build*/tests/cpp/protocol_vector_tool")):
        if candidate.is_file():
            return candidate
    return None


TOOL = find_tool()
requires_tool = pytest.mark.skipif(
    TOOL is None,
    reason="protocol_vector_tool not built; run cmake --build build")


def run_tool(*args: str) -> str:
    assert TOOL is not None
    result = subprocess.run([str(TOOL), *args], capture_output=True, text=True,
                            timeout=30, check=False)
    assert result.returncode == 0, f"{args} failed: {result.stdout}{result.stderr}"
    return result.stdout.strip()


# ------------------------------------------------- committed vectors vs Python

@pytest.mark.parametrize("case", VECTORS["crc32"], ids=lambda c: c["data"] or "empty")
def test_python_crc_matches_committed_vectors(case: dict) -> None:
    assert f"{codec.crc32(bytes.fromhex(case['data'])):08x}" == case["crc"]


@pytest.mark.parametrize("case", VECTORS["cobs"], ids=lambda c: c["decoded"])
def test_python_cobs_matches_committed_vectors(case: dict) -> None:
    decoded = bytes.fromhex(case["decoded"])
    encoded = bytes.fromhex(case["encoded"])
    assert codec.cobs_encode(decoded).hex() == case["encoded"]
    assert codec.cobs_decode(encoded) == decoded


@pytest.mark.parametrize("case", VECTORS["frames"], ids=lambda c: c["name"])
def test_python_frames_match_committed_vectors(case: dict) -> None:
    wire = codec.encode_frame(case["message_id"], case["seq"], case["timestamp_us"],
                              bytes.fromhex(case["payload"]))
    assert wire.hex() == case["wire"]


def test_the_hand_traced_vector_is_exactly_what_adr_0002_specifies() -> None:
    """Byte-by-byte trace of the first frame vector, so the layout is checked by hand once.

        version   01
        id        02                      DriveCommand
        seq       2a                      42
        timestamp efbeadde                little-endian 0xDEADBEEF
        payload   0000c03f 000080be       1.5f and -0.25f, IEEE-754 little-endian
        crc32     84d43b94                little-endian 0x943bd484 over the 15 bytes above
        wire      COBS(the 19 bytes above) + 00
    """
    case = VECTORS["frames"][0]
    assert case["name"] == "DriveCommand"

    header = bytes([1, 0x02, 42]) + (0xDEADBEEF).to_bytes(4, "little")
    assert header.hex() == "01022aefbeadde"

    payload = bytes.fromhex(case["payload"])
    assert payload.hex() == "0000c03f000080be"

    checked = header + payload
    assert len(checked) == 15
    assert codec.crc32(checked) == 0x943BD484
    logical = checked + (0x943BD484).to_bytes(4, "little")
    assert len(logical) == 19 == M.HEADER_BYTES + 8 + M.CRC_BYTES

    assert (codec.cobs_encode(logical) + b"\x00").hex() == case["wire"]


# ------------------------------------------------------- C++ vs committed vectors

@requires_tool
@pytest.mark.parametrize("case", VECTORS["crc32"], ids=lambda c: c["data"] or "empty")
def test_cpp_crc_matches_committed_vectors(case: dict) -> None:
    assert run_tool("crc", case["data"]) == case["crc"]


@requires_tool
@pytest.mark.parametrize("case", VECTORS["cobs"], ids=lambda c: c["decoded"])
def test_cpp_cobs_matches_committed_vectors(case: dict) -> None:
    assert run_tool("cobs-encode", case["decoded"]) == case["encoded"]
    assert run_tool("cobs-decode", case["encoded"]) == case["decoded"]


@requires_tool
@pytest.mark.parametrize("case", VECTORS["frames"], ids=lambda c: c["name"])
def test_cpp_frames_match_committed_vectors(case: dict) -> None:
    produced = run_tool("encode", str(case["message_id"]), str(case["seq"]),
                        str(case["timestamp_us"]), case["payload"])
    assert produced == case["wire"]


# --------------------------------------------------------------- C++ vs Python

@requires_tool
@pytest.mark.parametrize("case", VECTORS["frames"], ids=lambda c: c["name"])
def test_cpp_decodes_what_python_encoded(case: dict) -> None:
    wire = codec.encode_frame(case["message_id"], case["seq"], case["timestamp_us"],
                              bytes.fromhex(case["payload"]))
    output = run_tool("decode", wire.hex())
    # Parse fields rather than compare the raw line: a zero-length payload leaves a
    # trailing separator that strip() removes, which is formatting, not behavior.
    parts = output.split()
    assert parts[0] == "OK", output
    assert int(parts[1]) == case["message_id"]
    assert int(parts[2]) == case["seq"]
    assert int(parts[3]) == case["timestamp_us"]
    payload_hex = parts[4] if len(parts) > 4 else ""
    assert payload_hex == case["payload"]


@requires_tool
def test_python_decodes_what_cpp_encoded_for_every_declared_message() -> None:
    """Drive this from the generated size table so a new message is covered automatically."""
    for message_id, declared in sorted(M.PAYLOAD_BYTES_BY_ID.items()):
        payload = bytes((i * 7 + 1) & 0xFF for i in range(declared))
        wire_hex = run_tool("encode", str(message_id), "17", "123456", payload.hex())
        frames = codec.FrameDecoder().push(bytes.fromhex(wire_hex))
        assert len(frames) == 1, f"id 0x{message_id:02X}"
        assert frames[0].message_id == message_id
        assert frames[0].seq == 17
        assert frames[0].timestamp_us == 123456
        assert frames[0].payload == payload


@requires_tool
def test_both_languages_agree_on_cobs_for_awkward_inputs() -> None:
    awkward = [
        b"", b"\x00", b"\x00\x00\x00", b"\x11\x00", b"\x00\x11",
        b"\xaa" * 253, b"\xaa" * 254, b"\xaa" * 255, b"\xaa" * 254 + b"\x00",
        bytes(range(256)),
    ]
    for data in awkward:
        assert run_tool("cobs-encode", data.hex()) == codec.cobs_encode(data).hex(), \
            f"disagreement on {len(data)}-byte input"


@requires_tool
def test_both_languages_agree_on_rejection_reasons() -> None:
    """Not just *that* both reject, but that they attribute it to the same counter."""
    cases = [
        ("unknown id", codec.encode_frame(0x03, 0, 0, bytes(4)), "unknown_id"),
        ("wrong length", codec.encode_frame(0x02, 0, 0, bytes(6)), "length_mismatch"),
    ]
    for name, wire, expected_counter in cases:
        assert run_tool("decode", wire.hex()) == f"REJECT {expected_counter}", name

        decoder = codec.FrameDecoder()
        decoder.push(wire)
        assert getattr(decoder.stats, expected_counter) == 1, name
        assert decoder.stats.frames_ok == 0, name


@requires_tool
def test_both_languages_reject_a_corrupted_crc_identically() -> None:
    wire = bytearray(codec.encode_frame(0x02, 1, 100, M.DriveCommand(1.0, 1.0).encode()))
    wire[-2] ^= 0x40           # corrupt inside the encoded CRC region
    if 0 in wire[:-1]:
        pytest.skip("corruption produced a delimiter; framing case, not a CRC case")
    assert run_tool("decode", bytes(wire).hex()).startswith("REJECT")
    assert codec.FrameDecoder().push(bytes(wire)) == []


def test_the_tool_is_present_when_the_job_says_it_should_be() -> None:
    """Guards against the skips above silently hiding every cross-language test.

    The CI job that builds C++ sets RECON_REQUIRE_VECTOR_TOOL=1. The Python-only job
    does not, because it deliberately runs without a C++ toolchain. Keying this on a
    dedicated variable rather than on CI keeps the skip honest in both jobs.
    """
    import os
    if os.environ.get("RECON_REQUIRE_VECTOR_TOOL") == "1":
        assert TOOL is not None, "this job must build protocol_vector_tool first"
