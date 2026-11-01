"""Raw AVR ASCII hex wire format serializer and parser.

Format specifications:
1. Standard AVR (*<hex>;\r\n):
   - Example 112-bit: *8D4840D6202CC371C32CE0576098;\r\n
   - Example 56-bit: *02E197B867A348;\r\n
2. Timestamped MLAT AVR (@<12_hex_timestamp><hex>;\r\n):
   - 48-bit 12 MHz MLAT timestamp formatted as 12 uppercase hex digits.
   - Example: @0000001A2B3C8D4840D6202CC371C32CE0576098;\r\n
"""

from __future__ import annotations

import re


def serialize_avr_frame(raw_bytes: bytes, mlat_timestamp_12mhz: int | None = None) -> str:
    """Serialize raw Mode S or Mode A/C payload bytes into AVR ASCII format.

    Parameters:
        raw_bytes: Payload bytes (typically 2, 7, or 14 bytes).
        mlat_timestamp_12mhz: Optional 48-bit 12 MHz MLAT clock counter.
            If provided, serialized with '@' prefix and 12-hex timestamp.
            Otherwise serialized with '*' prefix.

    Returns:
        str: AVR formatted line terminated with ';\\r\\n'.
    """
    hex_payload = raw_bytes.hex().upper()
    if mlat_timestamp_12mhz is not None:
        ts_48 = mlat_timestamp_12mhz & 0xFFFFFFFFFFFF
        return f"@{ts_48:012X}{hex_payload};\r\n"
    return f"*{hex_payload};\r\n"


def parse_avr_frame(line: str) -> tuple[bytes, int | None]:
    """Parse a single AVR ASCII frame into raw bytes and optional MLAT timestamp.

    Parameters:
        line: AVR line, e.g. '*8D...;' or '@0000001A2B3C8D...;'.
            Leading/trailing whitespace and newlines are stripped.

    Returns:
        tuple[bytes, int | None]: (payload_bytes, mlat_timestamp_12mhz).

    Raises:
        ValueError: If line does not start with '*' or '@', does not end with ';',
            or contains invalid hex characters or malformed timestamp.
    """
    s = line.strip()
    if not s:
        raise ValueError("Empty AVR frame string")

    if not s.endswith(";"):
        raise ValueError(f"Invalid AVR frame: missing terminating ';': '{s}'")

    body = s[:-1]
    if len(body) < 2:
        raise ValueError(f"AVR frame too short: '{s}'")

    prefix = body[0]
    content = body[1:]

    if prefix == "*":
        try:
            payload = bytes.fromhex(content)
        except ValueError as err:
            raise ValueError(f"Invalid hex in unstamped AVR frame '{s}': {err}") from err
        return payload, None

    if prefix == "@":
        if len(content) < 12:
            raise ValueError(
                f"Timestamped AVR frame requires at least 12 hex digits for timestamp: '{s}'"
            )
        ts_hex = content[:12]
        payload_hex = content[12:]
        try:
            ts = int(ts_hex, 16)
        except ValueError as err:
            raise ValueError(f"Invalid timestamp hex in AVR frame '{s}': {err}") from err

        try:
            payload = bytes.fromhex(payload_hex)
        except ValueError as err:
            raise ValueError(f"Invalid payload hex in AVR frame '{s}': {err}") from err

        return payload, ts

    raise ValueError(f"Invalid AVR frame prefix '{prefix}': expected '*' or '@'")


_AVR_FRAME_REGEX = re.compile(r"([*@][0-9A-Fa-f]+;)")


def parse_avr_stream(stream: str) -> list[tuple[bytes, int | None]]:
    """Parse a multi-frame AVR ASCII stream into a list of parsed frames.

    Handles arbitrary delimiter whitespace, newlines, or concatenated frames.
    Invalid frames or non-matching segments between delimiters are skipped.

    Parameters:
        stream: String containing one or more AVR frames.

    Returns:
        list[tuple[bytes, int | None]]: List of (payload_bytes, mlat_timestamp_12mhz).
    """
    matches = _AVR_FRAME_REGEX.findall(stream)
    results: list[tuple[bytes, int | None]] = []
    for match in matches:
        try:
            results.append(parse_avr_frame(match))
        except ValueError:
            continue
    return results


__all__ = [
    "parse_avr_frame",
    "parse_avr_stream",
    "serialize_avr_frame",
]
