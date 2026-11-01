"""Structured JSON Lines serializer and streaming parser for decoded messages and track states.

Authoritative reference: PROJECT.md Section 5 and docs/02_PROTOCOL_SPECIFICATION.md Section 9.
"""

from __future__ import annotations

import dataclasses
import enum
import json
from typing import Any


class _ADSBJSONEncoder(json.JSONEncoder):
    """Custom JSON encoder supporting dataclasses, enums, bytes, and sets."""

    def default(self, o: Any) -> Any:
        if dataclasses.is_dataclass(o) and not isinstance(o, type):
            return dataclasses.asdict(o)
        if isinstance(o, enum.Enum):
            return o.name
        if isinstance(o, (bytes, bytearray)):
            return o.hex().upper()
        if isinstance(o, set):
            return sorted(o)
        return super().default(o)


def serialize_json_line(data: Any) -> str:
    """Serialize a single data structure or dataclass into a JSON line terminated by '\\n'."""
    return json.dumps(data, cls=_ADSBJSONEncoder, separators=(",", ":")) + "\n"


def parse_json_line(line: str) -> dict[str, Any]:
    """Parse a single JSON line string into a Python dictionary.

    Parameters:
        line: JSON string.

    Returns:
        dict[str, Any]: Parsed JSON dictionary.

    Raises:
        ValueError: If line is empty or contains malformed JSON.
    """
    s = line.strip()
    if not s:
        raise ValueError("Empty JSON line")
    try:
        res: Any = json.loads(s)
        if not isinstance(res, dict):
            raise ValueError(f"JSON line must decode to an object/dict, got {type(res).__name__}")
        return res
    except json.JSONDecodeError as err:
        raise ValueError(f"Malformed JSON line: {err}") from err


def serialize_json_stream(items: list[Any]) -> str:
    """Serialize a list of objects into newline-delimited JSON (JSON Lines format)."""
    return "".join(serialize_json_line(item) for item in items)


def parse_json_stream(stream: str) -> list[dict[str, Any]]:
    """Parse a multi-line JSON Lines stream into a list of dictionaries.

    Empty lines and malformed lines are skipped.
    """
    records: list[dict[str, Any]] = []
    for line in stream.splitlines():
        trimmed = line.strip()
        if not trimmed:
            continue
        try:
            records.append(parse_json_line(trimmed))
        except ValueError:
            continue
    return records


__all__ = [
    "parse_json_line",
    "parse_json_stream",
    "serialize_json_line",
    "serialize_json_stream",
]
