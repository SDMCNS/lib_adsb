"""Mode-S Beast binary wire format serializer and parser.

Format:
  <0x1A><type><6-byte timestamp><1-byte signal><payload>
Escaping:
  Any 0x1A in timestamp, signal, or payload is escaped as 0x1A 0x1A.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class BeastFrame:
    """Decoded Beast binary frame."""

    frame_type: str
    mlat_timestamp: int
    rssi_byte: int
    payload: bytes


def serialize_beast_frame(
    frame_type: str,
    mlat_timestamp: int,
    rssi_byte: int,
    payload: bytes,
) -> bytes:
    """Serialize frame fields into Beast binary escaped wire format."""
    ts_bytes = mlat_timestamp.to_bytes(6, "big")
    sig_bytes = bytes([rssi_byte & 0xFF])
    body = ts_bytes + sig_bytes + payload

    # Escape any 0x1A as 0x1A 0x1A
    escaped_body = bytearray()
    for b in body:
        escaped_body.append(b)
        if b == 0x1A:
            escaped_body.append(0x1A)

    header = b"\x1a" + frame_type.encode("ascii")
    return bytes(header + escaped_body)


def parse_beast_frame(raw_data: bytes) -> BeastFrame:
    """Parse Beast binary wire format bytes into BeastFrame."""
    if len(raw_data) < 2 or raw_data[0] != 0x1A:
        raise ValueError("Invalid Beast frame: must start with 0x1A")

    frame_type = chr(raw_data[1])
    idx = 2
    unescaped = bytearray()

    while idx < len(raw_data):
        b = raw_data[idx]
        if b == 0x1A:
            idx += 1
            if idx >= len(raw_data) or raw_data[idx] != 0x1A:
                raise ValueError("Invalid 0x1A escape sequence in Beast frame")
        unescaped.append(raw_data[idx])
        idx += 1

    if len(unescaped) < 7:
        raise ValueError(f"Beast frame body too short: {len(unescaped)} bytes")

    mlat_ts = int.from_bytes(unescaped[:6], "big")
    rssi = unescaped[6]
    payload = bytes(unescaped[7:])

    return BeastFrame(
        frame_type=frame_type,
        mlat_timestamp=mlat_ts,
        rssi_byte=rssi,
        payload=payload,
    )


def parse_beast_stream(data: bytes) -> list[BeastFrame]:
    """Parse a stream of concatenated Beast binary frames."""
    frames: list[BeastFrame] = []
    i = 0
    n = len(data)
    while i < n:
        if data[i] != 0x1A:
            i += 1
            continue
        if i + 1 >= n:
            break
        ftype_byte = data[i + 1]
        if ftype_byte in (0x31, 0x32, 0x33):  # '1', '2', '3'
            ftype = chr(ftype_byte)
            expected_body_len = 9 if ftype == "1" else (14 if ftype == "2" else 21)
            j = i + 2
            body = bytearray()
            valid = True
            while len(body) < expected_body_len and j < n:
                b = data[j]
                if b == 0x1A:
                    j += 1
                    if j >= n or data[j] != 0x1A:
                        valid = False
                        break
                body.append(data[j])
                j += 1
            if valid and len(body) == expected_body_len:
                mlat_ts = int.from_bytes(body[:6], "big")
                rssi = body[6]
                payload = bytes(body[7:])
                frames.append(
                    BeastFrame(
                        frame_type=ftype,
                        mlat_timestamp=mlat_ts,
                        rssi_byte=rssi,
                        payload=payload,
                    )
                )
                i = j
                continue
        i += 1
    return frames


__all__ = [
    "BeastFrame",
    "parse_beast_frame",
    "parse_beast_stream",
    "serialize_beast_frame",
]
