"""Load and validate the protocol schema.

Separated from `generate.py` so the validation rules can be tested directly against
deliberately-malformed schemas, rather than only through generated output.

The validator is strict on purpose. YAML's type coercion is loose (`0x10` may parse as a
string, `yes` as a boolean), and a schema error that slips through becomes a wire-format
bug that looks like impossible hardware behavior. Every value's type is checked explicitly
and every violation is collected before raising, so one run reports every problem rather
than making the author fix them one at a time.

Two CLAUDE.md hard rules are enforced here that no compiler can check:
  - rule 5: a field with a declared `unit` must carry the matching suffix in its name
  - rule 6: a field or parameter with a `range` or `default` must carry a `source` note
"""

from __future__ import annotations

from dataclasses import dataclass, field as dc_field
from pathlib import Path
from typing import Any

import yaml

# Framing constants, fixed by ADR 0002 and not schema-driven. They live here because the
# generator needs them to compute buffer sizes, and a second copy would be a second truth.
HEADER_BYTES = 7   # version:1 + message_id:1 + seq:1 + timestamp_us:4
CRC_BYTES = 4      # CRC-32/ISO-HDLC
COBS_BLOCK = 254   # COBS adds one overhead byte per 254 bytes of input
DELIMITER_BYTES = 1

SCALAR_SIZES: dict[str, int] = {
    "u8": 1, "u16": 2, "u32": 4,
    "i8": 1, "i16": 2, "i32": 4,
    "f32": 4,
}

# f64 is deliberately absent: the Cortex-M4 FPU is single precision, so a double would be
# emulated in software inside a control loop. See docs/decisions/0002.
FLOAT_TYPES = {"f32"}
SIGNED_TYPES = {"i8", "i16", "i32"}

DIRECTIONS = ("operator_to_robot", "robot_to_operator")


class SchemaError(Exception):
    """Raised with every validation failure found, one per line."""

    def __init__(self, errors: list[str]) -> None:
        self.errors = errors
        super().__init__(
            f"{len(errors)} schema validation error(s):\n  " + "\n  ".join(errors)
        )


def unit_suffix(unit: str) -> str:
    """Expected name suffix for a declared unit, per CLAUDE.md hard rule 5.

    `m/s` -> `_m_s`, `rad/s` -> `_rad_s`, `V` -> `_V`. Case-sensitive, because unit
    symbols are case-sensitive: `Hz` is hertz and `hz` is nothing. CLAUDE.md's own
    examples (`voltage_V`, `current_A`) preserve symbol case.
    """
    return "_" + unit.replace("/", "_")


@dataclass(frozen=True)
class EnumValue:
    name: str
    value: int


@dataclass(frozen=True)
class Enum:
    name: str
    underlying: str
    description: str
    values: tuple[EnumValue, ...]

    @property
    def size_bytes(self) -> int:
        return SCALAR_SIZES[self.underlying]


@dataclass(frozen=True)
class Field:
    name: str
    type: str                      # scalar type, or "enum"
    count: int = 1                 # >1 means a fixed-length array
    unit: str | None = None
    range: tuple[float, float] | None = None
    source: str | None = None
    description: str = ""
    enum: str | None = None        # enum name when type == "enum"
    const: int | None = None       # fixed value the decoder must see

    def scalar_type(self, enums: dict[str, Enum]) -> str | None:
        """Underlying scalar type, or None if it cannot be resolved.

        Returns None rather than raising because this is called while errors are still
        being collected: a field referencing an unknown enum has already been recorded as
        an error, and crashing here would discard every other error found in the same run.
        """
        if self.type != "enum":
            return self.type if self.type in SCALAR_SIZES else None
        enum = enums.get(self.enum) if self.enum else None
        return enum.underlying if enum else None

    def size_bytes(self, enums: dict[str, Enum]) -> int:
        """Wire size in bytes, or 0 if the type is unresolvable (an error is already logged)."""
        scalar = self.scalar_type(enums)
        return SCALAR_SIZES[scalar] * self.count if scalar else 0


@dataclass(frozen=True)
class Message:
    name: str
    id: int
    direction: str
    description: str
    fields: tuple[Field, ...]
    rate_hz: int | None = None

    def payload_bytes(self, enums: dict[str, Enum]) -> int:
        return sum(f.size_bytes(enums) for f in self.fields)


@dataclass(frozen=True)
class Param:
    id: int
    name: str
    type: str
    range: tuple[float, float]
    default: float
    writable: bool
    source: str
    description: str = ""
    unit: str | None = None


@dataclass
class Schema:
    protocol_version: int
    max_payload_bytes: int
    id_ranges: dict[str, tuple[int, int]]
    enums: dict[str, Enum] = dc_field(default_factory=dict)
    messages: tuple[Message, ...] = ()
    params: tuple[Param, ...] = ()

    @property
    def max_logical_frame_bytes(self) -> int:
        return HEADER_BYTES + self.max_payload_bytes + CRC_BYTES

    @property
    def max_wire_frame_bytes(self) -> int:
        """Worst-case bytes on the wire for the largest legal frame.

        COBS encoding of n bytes produces at most n + ceil(n/254) bytes, and the frame is
        terminated by one delimiter. Buffer sizes in firmware derive from this rather than
        being written down, satisfying the no-magic-numbers rule.
        """
        n = self.max_logical_frame_bytes
        overhead = -(-n // COBS_BLOCK)  # ceil division
        return n + overhead + DELIMITER_BYTES

    def message_by_name(self, name: str) -> Message:
        for m in self.messages:
            if m.name == name:
                return m
        raise KeyError(name)


def _require_int(value: Any, what: str, errors: list[str]) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int):
        errors.append(f"{what}: expected an integer, got {value!r} ({type(value).__name__})")
        return None
    return value


def _parse_enums(raw: dict[str, Any], errors: list[str]) -> dict[str, Enum]:
    enums: dict[str, Enum] = {}
    for name, body in (raw or {}).items():
        underlying = body.get("underlying")
        if underlying not in SCALAR_SIZES or underlying in FLOAT_TYPES:
            errors.append(f"enum {name}: underlying must be an integer scalar, got {underlying!r}")
            underlying = "u8"
        seen_values: set[int] = set()
        seen_names: set[str] = set()
        values: list[EnumValue] = []
        for entry in body.get("values", []):
            v = _require_int(entry.get("value"), f"enum {name}.{entry.get('name')}", errors)
            if v is None:
                continue
            if v in seen_values:
                errors.append(f"enum {name}: duplicate value {v}")
            if entry["name"] in seen_names:
                errors.append(f"enum {name}: duplicate name {entry['name']}")
            seen_values.add(v)
            seen_names.add(entry["name"])
            values.append(EnumValue(entry["name"], v))
        if not values:
            errors.append(f"enum {name}: has no values")
        enums[name] = Enum(name, underlying, body.get("description", ""), tuple(values))
    return enums


def _parse_field(raw: dict[str, Any], where: str, enums: dict[str, Enum],
                 errors: list[str]) -> Field:
    name = raw.get("name", "<unnamed>")
    ftype = raw.get("type")
    enum_name = raw.get("enum")

    if ftype == "enum":
        if enum_name not in enums:
            errors.append(f"{where}.{name}: unknown enum {enum_name!r}")
    elif ftype not in SCALAR_SIZES:
        errors.append(f"{where}.{name}: unknown type {ftype!r} "
                      f"(known: {', '.join(sorted(SCALAR_SIZES))}, enum)")

    count = raw.get("count", 1)
    if not isinstance(count, int) or isinstance(count, bool) or count < 1:
        errors.append(f"{where}.{name}: count must be a positive integer, got {count!r}")
        count = 1

    unit = raw.get("unit")
    rng = raw.get("range")
    source = raw.get("source")
    const = raw.get("const")

    # Hard rule 5: units live in the name, not only in the schema.
    if unit is not None:
        expected = unit_suffix(unit)
        if not name.endswith(expected):
            errors.append(
                f"{where}.{name}: declares unit {unit!r} so its name must end with "
                f"{expected!r} (CLAUDE.md hard rule 5)")

    # Hard rule 6: no unexplained constants.
    if rng is not None and not source:
        errors.append(f"{where}.{name}: has a range but no `source` note "
                      f"(CLAUDE.md hard rule 6)")

    if rng is not None:
        if (not isinstance(rng, list)) or len(rng) != 2:
            errors.append(f"{where}.{name}: range must be [min, max], got {rng!r}")
            rng = None
        elif rng[0] > rng[1]:
            errors.append(f"{where}.{name}: range min {rng[0]} exceeds max {rng[1]}")

    if const is not None:
        if _require_int(const, f"{where}.{name}.const", errors) is None:
            const = None
        elif ftype == "enum" or ftype in FLOAT_TYPES:
            errors.append(f"{where}.{name}: const is only supported on integer scalars")

    return Field(
        name=name, type=ftype if ftype else "u8", count=count, unit=unit,
        range=(float(rng[0]), float(rng[1])) if rng else None,
        source=source, description=raw.get("description", ""),
        enum=enum_name, const=const,
    )


def _parse_messages(raw: list[dict[str, Any]], schema_max_payload: int,
                    id_ranges: dict[str, tuple[int, int]], enums: dict[str, Enum],
                    errors: list[str]) -> tuple[Message, ...]:
    messages: list[Message] = []
    seen_ids: dict[int, str] = {}
    seen_names: set[str] = set()

    for raw_msg in raw or []:
        name = raw_msg.get("name", "<unnamed>")
        if name in seen_names:
            errors.append(f"message {name}: duplicate name")
        seen_names.add(name)

        msg_id = _require_int(raw_msg.get("id"), f"message {name}.id", errors)
        direction = raw_msg.get("direction")

        if direction not in DIRECTIONS:
            errors.append(f"message {name}: direction must be one of {DIRECTIONS}, "
                          f"got {direction!r}")
        elif msg_id is not None:
            low, high = id_ranges[direction]
            if not low <= msg_id <= high:
                errors.append(
                    f"message {name}: id 0x{msg_id:02X} is outside the "
                    f"{direction} range [0x{low:02X}, 0x{high:02X}]")

        if msg_id is not None:
            if msg_id in seen_ids:
                errors.append(f"message {name}: id 0x{msg_id:02X} already used by "
                              f"{seen_ids[msg_id]}")
            seen_ids[msg_id] = name

        fields = tuple(
            _parse_field(f, f"message {name}", enums, errors)
            for f in (raw_msg.get("fields") or [])
        )

        field_names = [f.name for f in fields]
        for dup in {n for n in field_names if field_names.count(n) > 1}:
            errors.append(f"message {name}: duplicate field name {dup!r}")

        payload = sum(f.size_bytes(enums) for f in fields)
        if payload > schema_max_payload:
            errors.append(f"message {name}: payload {payload} B exceeds "
                          f"max_payload_bytes {schema_max_payload}")

        messages.append(Message(
            name=name, id=msg_id if msg_id is not None else 0,
            direction=direction if direction in DIRECTIONS else DIRECTIONS[0],
            description=raw_msg.get("description", "").strip(),
            fields=fields, rate_hz=raw_msg.get("rate_hz"),
        ))
    if not messages:
        errors.append("schema defines no messages")
    return tuple(messages)


def _parse_params(raw: list[dict[str, Any]], errors: list[str]) -> tuple[Param, ...]:
    params: list[Param] = []
    seen_ids: dict[int, str] = {}
    seen_names: set[str] = set()

    for raw_param in raw or []:
        name = raw_param.get("name", "<unnamed>")
        pid = _require_int(raw_param.get("id"), f"param {name}.id", errors)
        ptype = raw_param.get("type")

        if ptype not in SCALAR_SIZES:
            errors.append(f"param {name}: unknown type {ptype!r}")
            ptype = "u16"
        if SCALAR_SIZES[ptype] > 4:
            errors.append(f"param {name}: type {ptype} exceeds the 4-byte wire value field")

        if name in seen_names:
            errors.append(f"param {name}: duplicate name")
        seen_names.add(name)
        if pid is not None:
            if pid in seen_ids:
                errors.append(f"param {name}: id 0x{pid:04X} already used by {seen_ids[pid]}")
            seen_ids[pid] = name

        source = raw_param.get("source")
        if not source:
            errors.append(f"param {name}: missing `source` note (CLAUDE.md hard rule 6)")

        unit = raw_param.get("unit")
        if unit is not None and not name.endswith(unit_suffix(unit)):
            errors.append(f"param {name}: declares unit {unit!r} so its name must end with "
                          f"{unit_suffix(unit)!r} (CLAUDE.md hard rule 5)")

        rng = raw_param.get("range")
        default = raw_param.get("default")
        if (not isinstance(rng, list)) or len(rng) != 2:
            errors.append(f"param {name}: range must be [min, max], got {rng!r}")
            rng = [0, 0]
        if default is None:
            errors.append(f"param {name}: missing default")
            default = rng[0]
        elif not rng[0] <= default <= rng[1]:
            errors.append(f"param {name}: default {default} is outside range {rng}")

        writable = raw_param.get("writable")
        if not isinstance(writable, bool):
            errors.append(f"param {name}: writable must be true or false, got {writable!r}")
            writable = False

        params.append(Param(
            id=pid if pid is not None else 0, name=name, type=ptype,
            range=(float(rng[0]), float(rng[1])), default=default, writable=writable,
            source=str(source).strip() if source else "",
            description=raw_param.get("description", "").strip(), unit=unit,
        ))
    return tuple(params)


def load_schema_dicts(messages_raw: dict[str, Any],
                      params_raw: dict[str, Any] | None = None) -> Schema:
    """Validate already-parsed schema dictionaries. Raises SchemaError on any problem."""
    errors: list[str] = []

    version = _require_int(messages_raw.get("protocol_version"), "protocol_version", errors)
    max_payload = _require_int(messages_raw.get("max_payload_bytes"), "max_payload_bytes",
                               errors)
    if max_payload is None:
        max_payload = 64

    raw_ranges = messages_raw.get("id_ranges") or {}
    id_ranges: dict[str, tuple[int, int]] = {}
    for direction in DIRECTIONS:
        pair = raw_ranges.get(direction)
        if (not isinstance(pair, list)) or len(pair) != 2:
            errors.append(f"id_ranges.{direction}: expected [low, high], got {pair!r}")
            id_ranges[direction] = (0, 0xFF)
        else:
            id_ranges[direction] = (int(pair[0]), int(pair[1]))

    a, b = id_ranges[DIRECTIONS[0]], id_ranges[DIRECTIONS[1]]
    if a[0] <= b[1] and b[0] <= a[1]:
        errors.append(f"id_ranges: {DIRECTIONS[0]} {a} overlaps {DIRECTIONS[1]} {b}")

    enums = _parse_enums(messages_raw.get("enums"), errors)
    messages = _parse_messages(messages_raw.get("messages"), max_payload, id_ranges,
                               enums, errors)
    params = _parse_params((params_raw or {}).get("params"), errors)

    if errors:
        raise SchemaError(errors)

    return Schema(
        protocol_version=version if version is not None else 0,
        max_payload_bytes=max_payload, id_ranges=id_ranges,
        enums=enums, messages=messages, params=params,
    )


def load_schema(messages_path: Path, params_path: Path | None = None) -> Schema:
    """Load and validate the schema from disk."""
    messages_raw = yaml.safe_load(messages_path.read_text())
    params_raw = yaml.safe_load(params_path.read_text()) if params_path else None
    return load_schema_dicts(messages_raw, params_raw)


def default_schema_paths() -> tuple[Path, Path]:
    here = Path(__file__).resolve().parent / "schema"
    return here / "messages.yaml", here / "params.yaml"
