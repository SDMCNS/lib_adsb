"""SBS-1 BaseStation 22-column CSV wire format serializer and parser.

Format:
MSG,transmission_type,session_id,aircraft_id,hex_ident,flight_id,date_gen,time_gen,date_log,time_log,callsign,altitude,ground_speed,track,lat,lon,vertical_rate,squawk,alert,emergency,spi,is_on_ground

Authoritative reference: docs/02_PROTOCOL_SPECIFICATION.md Section 9.3.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class SBSMessage:
    """SBS-1 BaseStation 22-column message model."""

    message_type: str = "MSG"
    transmission_type: int = 1
    session_id: str = "1"
    aircraft_id: str = "1"
    hex_ident: str = ""
    flight_id: str = "1"
    date_gen: str = ""
    time_gen: str = ""
    date_log: str = ""
    time_log: str = ""
    callsign: str = ""
    altitude: int | None = None
    ground_speed: float | None = None
    track: float | None = None
    lat: float | None = None
    lon: float | None = None
    vertical_rate: int | None = None
    squawk: str = ""
    alert: int | None = None
    emergency: int | None = None
    spi: int | None = None
    is_on_ground: int | None = None


def serialize_sbs_message(msg: SBSMessage) -> str:
    """Serialize SBSMessage into standard 22-column CSV line terminated with '\\r\\n'."""
    fields = [
        msg.message_type,
        str(msg.transmission_type),
        msg.session_id,
        msg.aircraft_id,
        msg.hex_ident.upper(),
        msg.flight_id,
        msg.date_gen,
        msg.time_gen,
        msg.date_log,
        msg.time_log,
        msg.callsign,
        "" if msg.altitude is None else str(msg.altitude),
        "" if msg.ground_speed is None else f"{msg.ground_speed:.1f}".rstrip("0").rstrip("."),
        "" if msg.track is None else f"{msg.track:.1f}".rstrip("0").rstrip("."),
        "" if msg.lat is None else f"{msg.lat:.5f}",
        "" if msg.lon is None else f"{msg.lon:.5f}",
        "" if msg.vertical_rate is None else str(msg.vertical_rate),
        msg.squawk,
        "" if msg.alert is None else str(msg.alert),
        "" if msg.emergency is None else str(msg.emergency),
        "" if msg.spi is None else str(msg.spi),
        "" if msg.is_on_ground is None else str(msg.is_on_ground),
    ]
    return ",".join(fields) + "\r\n"


def parse_sbs_message(line: str) -> SBSMessage:
    """Parse a single SBS-1 BaseStation 22-column CSV line into an SBSMessage.

    Parameters:
        line: Single CSV line.

    Returns:
        SBSMessage: Parsed dataclass instance.

    Raises:
        ValueError: If line contains fewer than 22 columns or invalid numeric types.
    """
    raw_line = line.strip()
    if not raw_line:
        raise ValueError("Empty SBS message line")

    parts = raw_line.split(",")
    if len(parts) < 22:
        raise ValueError(
            f"Invalid SBS-1 format: expected at least 22 columns, found {len(parts)} in '{raw_line}'"
        )

    # Transmission type
    try:
        ttype = int(parts[1])
    except ValueError as err:
        raise ValueError(f"Invalid transmission type '{parts[1]}' in SBS line") from err

    def _opt_int(s: str) -> int | None:
        s = s.strip()
        if not s:
            return None
        try:
            return int(s)
        except ValueError as e:
            raise ValueError(f"Invalid integer field '{s}' in SBS line") from e

    def _opt_float(s: str) -> float | None:
        s = s.strip()
        if not s:
            return None
        try:
            return float(s)
        except ValueError as e:
            raise ValueError(f"Invalid float field '{s}' in SBS line") from e

    return SBSMessage(
        message_type=parts[0].strip(),
        transmission_type=ttype,
        session_id=parts[2].strip(),
        aircraft_id=parts[3].strip(),
        hex_ident=parts[4].strip().upper(),
        flight_id=parts[5].strip(),
        date_gen=parts[6].strip(),
        time_gen=parts[7].strip(),
        date_log=parts[8].strip(),
        time_log=parts[9].strip(),
        callsign=parts[10].strip(),
        altitude=_opt_int(parts[11]),
        ground_speed=_opt_float(parts[12]),
        track=_opt_float(parts[13]),
        lat=_opt_float(parts[14]),
        lon=_opt_float(parts[15]),
        vertical_rate=_opt_int(parts[16]),
        squawk=parts[17].strip(),
        alert=_opt_int(parts[18]),
        emergency=_opt_int(parts[19]),
        spi=_opt_int(parts[20]),
        is_on_ground=_opt_int(parts[21]),
    )


def parse_sbs_stream(stream: str) -> list[SBSMessage]:
    """Parse a multi-line SBS-1 BaseStation CSV stream into a list of SBSMessages.

    Empty lines and lines that fail to parse are safely ignored.
    """
    messages: list[SBSMessage] = []
    for line in stream.splitlines():
        trimmed = line.strip()
        if not trimmed:
            continue
        try:
            messages.append(parse_sbs_message(trimmed))
        except ValueError:
            continue
    return messages


__all__ = [
    "SBSMessage",
    "parse_sbs_message",
    "parse_sbs_stream",
    "serialize_sbs_message",
]
