"""Tests for the protocol schema and its validator.

Two jobs. First, pin facts about the *real* schema that other code and documents depend on
— payload sizes appear in protocol/design-proposal.md and the bandwidth analysis in
docs/bringup/uart-link.md, so a silent change there would quietly invalidate both.

Second, prove each validation rule actually fires. A validator that stopped catching
violations would read as green, which is worse than no validator (the same argument as
tests/python/test_check_core_purity.py).
"""

from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "protocol"))

from schema_loader import (  # noqa: E402
    CRC_BYTES,
    HEADER_BYTES,
    SchemaError,
    load_schema,
    load_schema_dicts,
    default_schema_paths,
    unit_suffix,
)


@pytest.fixture(scope="module")
def schema():
    return load_schema(*default_schema_paths())


# --------------------------------------------------------------------------- real schema

def test_real_schema_is_valid(schema) -> None:
    assert schema.protocol_version == 1
    assert len(schema.messages) == 14
    assert len(schema.enums) == 4
    assert len(schema.params) == 11  # 4 mechanism params + 7 drive params (ADR 0013)


# Values quoted in protocol/design-proposal.md and docs/bringup/uart-link.md. If a schema
# edit changes one of these, those documents are now wrong and must be updated with it.
EXPECTED_PAYLOAD_BYTES = {
    "Heartbeat": 0, "DriveCommand": 8, "SafetyStateRequest": 5, "EstopRequest": 4,
    "ParamGet": 2, "ParamSet": 7, "ParamCommit": 4, "StateTelemetry": 31,
    "PowerTelemetry": 12, "LoopTiming": 9, "Fault": 6, "ParamValue": 7,
    "Nack": 3, "LinkStats": 28,
}


@pytest.mark.parametrize(("name", "expected"), sorted(EXPECTED_PAYLOAD_BYTES.items()))
def test_payload_sizes_match_the_documented_design(schema, name: str, expected: int) -> None:
    assert schema.message_by_name(name).payload_bytes(schema.enums) == expected


def test_every_message_is_covered_by_the_size_assertions(schema) -> None:
    """Guards against a new message being added without pinning its documented size."""
    assert {m.name for m in schema.messages} == set(EXPECTED_PAYLOAD_BYTES)


def test_frame_size_constants(schema) -> None:
    assert HEADER_BYTES == 7 and CRC_BYTES == 4
    assert schema.max_logical_frame_bytes == 7 + 64 + 4 == 75
    # COBS of 75 bytes adds 1 overhead byte, plus 1 delimiter.
    assert schema.max_wire_frame_bytes == 77


def test_message_ids_lie_in_their_direction_range(schema) -> None:
    for m in schema.messages:
        low, high = schema.id_ranges[m.direction]
        assert low <= m.id <= high, f"{m.name} id 0x{m.id:02X} outside {m.direction}"


def test_camera_pitch_id_stays_reserved(schema) -> None:
    """0x03 is reserved pending the pitch actuator ADR. Assigning it needs that decision."""
    assert 0x03 not in {m.id for m in schema.messages}


def test_camera_pitch_covers_the_full_circle(schema) -> None:
    """The outer casing turns through at least a full turn about the wheel axis (ADRs 0004, 0009).

    Regression test. This field originally declared +/-90 deg, inherited from assuming a
    limited-travel camera gimbal. Because the generator emits range validation into the
    decoder, that did not mislabel data -- it dropped every frame with the casing past 90
    degrees. Narrowing this range again would silently reintroduce that.
    """
    field = next(f for f in schema.message_by_name("StateTelemetry").fields
                 if f.name == "camera_pitch_rad")
    assert field.range is not None
    low, high = field.range
    assert low <= -math.pi + 1e-4, f"low bound {low} excludes reachable angles"
    assert high >= math.pi - 1e-4, f"high bound {high} excludes reachable angles"


def test_pitch_fields_document_the_sign_convention(schema) -> None:
    """Positive nose-down is the opposite of most people's instinct, so it must be stated."""
    for f in schema.message_by_name("StateTelemetry").fields:
        if "pitch" in f.name and f.unit == "rad":
            assert "NOSE-DOWN" in f.description.upper(), f"{f.name} omits the sign convention"


def test_state_changing_messages_carry_a_magic_constant(schema) -> None:
    for name in ("SafetyStateRequest", "EstopRequest", "ParamCommit"):
        consts = [f.const for f in schema.message_by_name(name).fields if f.const is not None]
        assert len(consts) == 1, f"{name} must carry exactly one magic constant"


def test_magic_constants_are_all_distinct(schema) -> None:
    """A corrupt arm request must not be able to act as an e-stop, or vice versa."""
    consts = [f.const for m in schema.messages for f in m.fields if f.const is not None]
    assert len(consts) == len(set(consts))


def test_every_ranged_field_has_a_source(schema) -> None:
    """CLAUDE.md hard rule 6, asserted over the real schema rather than only synthetics."""
    for m in schema.messages:
        for f in m.fields:
            if f.range is not None:
                assert f.source, f"{m.name}.{f.name} has a range but no source"


def test_unit_suffix_mapping() -> None:
    assert unit_suffix("m/s") == "_m_s"
    assert unit_suffix("rad/s") == "_rad_s"
    assert unit_suffix("rad") == "_rad"
    assert unit_suffix("V") == "_V"
    assert unit_suffix("Hz") == "_Hz"
    assert unit_suffix("m/s^2") == "_m_s2", "^ cannot appear in an identifier"


# ---------------------------------------------------------------------- validator rules

MINIMAL = {
    "protocol_version": 1,
    "max_payload_bytes": 64,
    "id_ranges": {"operator_to_robot": [0x01, 0x1F], "robot_to_operator": [0x20, 0x3F]},
    "enums": {
        "Mode": {"underlying": "u8", "description": "d",
                 "values": [{"name": "A", "value": 0}, {"name": "B", "value": 1}]},
    },
    "messages": [
        {"name": "Cmd", "id": 0x02, "direction": "operator_to_robot", "description": "d",
         "fields": [{"name": "speed_m_s", "type": "f32", "unit": "m/s",
                     "range": [-1.0, 1.0], "source": "guess", "description": "d"}]},
    ],
}

MINIMAL_PARAMS = {
    "params": [
        {"id": 1, "name": "gain", "type": "u16", "range": [0, 10], "default": 5,
         "writable": True, "source": "guess", "description": "d"},
    ],
}


def mutate(**overrides):
    """A copy of the minimal valid schema with top-level keys replaced."""
    d = copy.deepcopy(MINIMAL)
    d.update(overrides)
    return d


def test_minimal_schema_is_valid() -> None:
    s = load_schema_dicts(copy.deepcopy(MINIMAL), copy.deepcopy(MINIMAL_PARAMS))
    assert len(s.messages) == 1


def expect_error(messages_raw, fragment: str, params_raw=None) -> None:
    with pytest.raises(SchemaError) as exc:
        load_schema_dicts(messages_raw, params_raw or copy.deepcopy(MINIMAL_PARAMS))
    joined = "\n".join(exc.value.errors)
    assert fragment in joined, f"expected {fragment!r} in:\n{joined}"


def test_rejects_unit_suffix_mismatch() -> None:
    d = mutate()
    d["messages"][0]["fields"][0]["name"] = "speed"   # declares m/s, lacks _m_s
    expect_error(d, "must end with '_m_s'")


def test_rejects_range_without_source() -> None:
    d = mutate()
    del d["messages"][0]["fields"][0]["source"]
    expect_error(d, "no `source` note")


def test_rejects_duplicate_message_id() -> None:
    d = mutate()
    d["messages"].append({**copy.deepcopy(d["messages"][0]), "name": "Other"})
    expect_error(d, "already used by Cmd")


def test_rejects_id_outside_direction_range() -> None:
    d = mutate()
    d["messages"][0]["id"] = 0x30          # robot_to_operator range
    expect_error(d, "outside the operator_to_robot range")


def test_rejects_unknown_field_type() -> None:
    d = mutate()
    d["messages"][0]["fields"][0]["type"] = "f64"   # no doubles: M4 FPU is single precision
    expect_error(d, "unknown type 'f64'")


def test_rejects_payload_over_max() -> None:
    d = mutate(max_payload_bytes=8)
    d["messages"][0]["fields"] = [
        {"name": f"pad{i}", "type": "u32", "description": "d"} for i in range(4)
    ]
    expect_error(d, "exceeds max_payload_bytes 8")


def test_rejects_duplicate_field_name() -> None:
    d = mutate()
    d["messages"][0]["fields"].append(copy.deepcopy(d["messages"][0]["fields"][0]))
    expect_error(d, "duplicate field name")


def test_rejects_overlapping_id_ranges() -> None:
    d = mutate(id_ranges={"operator_to_robot": [0x01, 0x30],
                          "robot_to_operator": [0x20, 0x3F]})
    expect_error(d, "overlaps")


def test_rejects_unknown_enum_reference() -> None:
    d = mutate()
    d["messages"][0]["fields"][0] = {"name": "mode", "type": "enum", "enum": "Nope",
                                     "description": "d"}
    expect_error(d, "unknown enum 'Nope'")


def test_rejects_duplicate_enum_value() -> None:
    d = mutate()
    d["enums"]["Mode"]["values"][1]["value"] = 0
    expect_error(d, "duplicate value 0")


def test_rejects_bad_direction() -> None:
    d = mutate()
    d["messages"][0]["direction"] = "sideways"
    expect_error(d, "direction must be one of")


def test_rejects_non_integer_id() -> None:
    """YAML will happily hand us a string; the validator must not coerce it."""
    d = mutate()
    d["messages"][0]["id"] = "0x02"
    expect_error(d, "expected an integer")


def test_rejects_const_on_float() -> None:
    d = mutate()
    d["messages"][0]["fields"][0]["const"] = 1
    expect_error(d, "const is only supported on integer scalars")


def test_rejects_param_default_outside_range() -> None:
    p = copy.deepcopy(MINIMAL_PARAMS)
    p["params"][0]["default"] = 99
    expect_error(mutate(), "outside range", params_raw=p)


def test_rejects_param_without_source() -> None:
    p = copy.deepcopy(MINIMAL_PARAMS)
    del p["params"][0]["source"]
    expect_error(mutate(), "missing `source` note", params_raw=p)


def test_rejects_param_with_non_boolean_writable() -> None:
    p = copy.deepcopy(MINIMAL_PARAMS)
    p["params"][0]["writable"] = "yes"
    expect_error(mutate(), "writable must be true or false", params_raw=p)


def test_reports_every_error_at_once() -> None:
    """One run should surface all problems, not make the author fix them serially."""
    d = mutate()
    d["messages"][0]["fields"][0]["name"] = "speed"    # suffix violation
    del d["messages"][0]["fields"][0]["source"]        # missing source
    d["messages"][0]["direction"] = "sideways"         # bad direction
    with pytest.raises(SchemaError) as exc:
        load_schema_dicts(d, copy.deepcopy(MINIMAL_PARAMS))
    assert len(exc.value.errors) >= 3
