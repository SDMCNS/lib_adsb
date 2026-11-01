"""Comm-B Data Link (BDS) register decoders and classifiers.

Authoritative reference: ICAO Annex 10 Vol IV, RTCA DO-260B/C,
and docs/02_PROTOCOL_SPECIFICATION.md Section 7.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from adsb.core.bits import extract_bits

# 64-character AIS charmap for callsign decoding
AIS_CHARMAP = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######"


@dataclass(frozen=True, slots=True)
class BDS10Data:
    """BDS 1,0: Data Link Capability Report."""

    bds: tuple[int, int] = (1, 0)
    subnetwork_version: int = 0
    transponder_enhanced: bool = False
    overlay_command_capability: bool = False
    acas_status: int = 0
    mode_s_subnetwork_version: int = 0
    raw_mb: bytes = b""


BDS10DataLinkCapability = BDS10Data


@dataclass(frozen=True, slots=True)
class BDS17Data:
    """BDS 1,7: Common Usage GICB Capability Report."""

    bds: tuple[int, int] = (1, 7)
    active_bds_bitmap: int = 0
    raw_mb: bytes = b""


BDS17GICBCapability = BDS17Data


@dataclass(frozen=True, slots=True)
class BDS20Data:
    """BDS 2,0: Aircraft Identification."""

    bds: tuple[int, int] = (2, 0)
    callsign: str = ""
    raw_mb: bytes = b""


BDS20AircraftIdentification = BDS20Data


@dataclass(frozen=True, slots=True)
class BDS40Data:
    """BDS 4,0: Selected Vertical Intention."""

    bds: tuple[int, int] = (4, 0)
    mcp_altitude_ft: int | None = None
    fms_altitude_ft: int | None = None
    baro_setting_hpa: float | None = None
    vnav_mode: bool = False
    alt_hold_mode: bool = False
    approach_mode: bool = False
    target_alt_source: int = 0
    raw_mb: bytes = b""


BDS40SelectedVerticalIntention = BDS40Data


@dataclass(frozen=True, slots=True)
class BDS50Data:
    """BDS 5,0: Track and Turn Report."""

    bds: tuple[int, int] = (5, 0)
    roll_angle_deg: float | None = None
    true_track_deg: float | None = None
    groundspeed_kt: float | None = None
    track_angle_rate_dps: float | None = None
    true_airspeed_kt: float | None = None
    raw_mb: bytes = b""


BDS50TrackAndTurnReport = BDS50Data


@dataclass(frozen=True, slots=True)
class BDS60Data:
    """BDS 6,0: Heading and Speed Report."""

    bds: tuple[int, int] = (6, 0)
    magnetic_heading_deg: float | None = None
    indicated_airspeed_kt: float | None = None
    mach: float | None = None
    baro_vertical_rate_fpm: int | None = None
    inertial_vertical_rate_fpm: int | None = None
    raw_mb: bytes = b""


BDS60HeadingAndSpeedReport = BDS60Data


def decode_bds_10(mb_bytes: bytes) -> BDS10Data:
    """Decode BDS 1,0 Data Link Capability Report."""
    val = int.from_bytes(mb_bytes[:7], "big")
    subnetwork_version = extract_bits(val, 56, 10, 16)
    transponder_enhanced = bool(extract_bits(val, 56, 17, 17))
    overlay = bool(extract_bits(val, 56, 25, 25))
    acas = extract_bits(val, 56, 26, 28)
    subnetwork_v = extract_bits(val, 56, 33, 40)
    return BDS10Data(
        subnetwork_version=subnetwork_version,
        transponder_enhanced=transponder_enhanced,
        overlay_command_capability=overlay,
        acas_status=acas,
        mode_s_subnetwork_version=subnetwork_v,
        raw_mb=mb_bytes,
    )


def decode_bds_17(mb_bytes: bytes) -> BDS17Data:
    """Decode BDS 1,7 Common Usage GICB Capability Report."""
    val = int.from_bytes(mb_bytes[:7], "big")
    return BDS17Data(
        active_bds_bitmap=val,
        raw_mb=mb_bytes,
    )


def decode_bds_20(mb_bytes: bytes) -> BDS20Data:
    """Decode BDS 2,0 Aircraft Identification."""
    val = int.from_bytes(mb_bytes[:7], "big")
    # Bits 9..56 contain 8 characters (6 bits each)
    chars: list[str] = []
    for i in range(8):
        c_code = extract_bits(val, 56, 9 + i * 6, 14 + i * 6)
        chars.append(AIS_CHARMAP[c_code])
    callsign = "".join(chars).rstrip()
    return BDS20Data(callsign=callsign, raw_mb=mb_bytes)


def decode_bds_40(mb_bytes: bytes) -> BDS40Data:
    """Decode BDS 4,0 Selected Vertical Intention."""
    val = int.from_bytes(mb_bytes[:7], "big")

    # MCP selected altitude: bit 1 status, bits 2..13 value (scale 16 ft)
    mcp_status = extract_bits(val, 56, 1, 1)
    mcp_alt = extract_bits(val, 56, 2, 13) * 16 if mcp_status else None

    # FMS selected altitude: bit 14 status, bits 15..26 value (scale 16 ft)
    fms_status = extract_bits(val, 56, 14, 14)
    fms_alt = extract_bits(val, 56, 15, 26) * 16 if fms_status else None

    # Baro setting: bit 27 status, bits 28..39 value (scale 0.1 mb + 800 mb)
    baro_status = extract_bits(val, 56, 27, 27)
    baro_setting = 800.0 + extract_bits(val, 56, 28, 39) * 0.1 if baro_status else None

    # Modes: bit 48 status of MCP modes, 49 VNAV, 50 Alt Hold, 51 Approach
    mode_status = extract_bits(val, 56, 48, 48)
    vnav = bool(extract_bits(val, 56, 49, 49)) if mode_status else False
    alt_hold = bool(extract_bits(val, 56, 50, 50)) if mode_status else False
    approach = bool(extract_bits(val, 56, 51, 51)) if mode_status else False

    # Target altitude source: bit 54 status, bits 55..56 value
    target_src_status = extract_bits(val, 56, 54, 54)
    target_src = extract_bits(val, 56, 55, 56) if target_src_status else 0

    return BDS40Data(
        mcp_altitude_ft=mcp_alt,
        fms_altitude_ft=fms_alt,
        baro_setting_hpa=baro_setting,
        vnav_mode=vnav,
        alt_hold_mode=alt_hold,
        approach_mode=approach,
        target_alt_source=target_src,
        raw_mb=mb_bytes,
    )


def decode_bds_50(mb_bytes: bytes) -> BDS50Data:
    """Decode BDS 5,0 Track and Turn Report."""
    val = int.from_bytes(mb_bytes[:7], "big")

    # Roll angle: bit 1 status, bit 2 sign, bits 3..11 value (scale 45/256 deg)
    roll_status = extract_bits(val, 56, 1, 1)
    if roll_status:
        roll_sign = -1.0 if extract_bits(val, 56, 2, 2) else 1.0
        roll_val = extract_bits(val, 56, 3, 11) * (45.0 / 256.0)
        roll_angle: float | None = roll_sign * roll_val
    else:
        roll_angle = None

    # True track angle: bit 12 status, bit 13 sign, bits 14..23 value (scale 90/512 deg)
    track_status = extract_bits(val, 56, 12, 12)
    if track_status:
        track_sign = -1.0 if extract_bits(val, 56, 13, 13) else 1.0
        track_val = extract_bits(val, 56, 14, 23) * (90.0 / 512.0)
        true_track: float | None = (track_sign * track_val) % 360.0
    else:
        true_track = None

    # Ground speed: bit 24 status, bits 25..34 value (scale 2 kt)
    gs_status = extract_bits(val, 56, 24, 24)
    groundspeed = float(extract_bits(val, 56, 25, 34) * 2) if gs_status else None

    # Track angle rate: bit 35 status, bit 36 sign, bits 37..45 value (scale 8/256 deg/s)
    rate_status = extract_bits(val, 56, 35, 35)
    if rate_status:
        rate_sign = -1.0 if extract_bits(val, 56, 36, 36) else 1.0
        rate_val = extract_bits(val, 56, 37, 45) * (8.0 / 256.0)
        track_rate: float | None = rate_sign * rate_val
    else:
        track_rate = None

    # True airspeed: bit 46 status, bits 47..56 value (scale 2 kt)
    tas_status = extract_bits(val, 56, 46, 46)
    tas = float(extract_bits(val, 56, 47, 56) * 2) if tas_status else None

    return BDS50Data(
        roll_angle_deg=roll_angle,
        true_track_deg=true_track,
        groundspeed_kt=groundspeed,
        track_angle_rate_dps=track_rate,
        true_airspeed_kt=tas,
        raw_mb=mb_bytes,
    )


def decode_bds_60(mb_bytes: bytes) -> BDS60Data:
    """Decode BDS 6,0 Heading and Speed Report."""
    val = int.from_bytes(mb_bytes[:7], "big")

    # Magnetic heading: bit 1 status, bit 2 sign, bits 3..12 value (scale 90/512 deg)
    hdg_status = extract_bits(val, 56, 1, 1)
    if hdg_status:
        hdg_sign = -1.0 if extract_bits(val, 56, 2, 2) else 1.0
        hdg_val = extract_bits(val, 56, 3, 12) * (90.0 / 512.0)
        mag_hdg: float | None = (hdg_sign * hdg_val) % 360.0
    else:
        mag_hdg = None

    # Indicated airspeed: bit 13 status, bits 14..23 value (scale 1 kt)
    ias_status = extract_bits(val, 56, 13, 13)
    ias = float(extract_bits(val, 56, 14, 23)) if ias_status else None

    # Mach number: bit 24 status, bits 25..34 value (scale 0.004)
    mach_status = extract_bits(val, 56, 24, 24)
    mach = extract_bits(val, 56, 25, 34) * 0.004 if mach_status else None

    # Barometric altitude rate: bit 35 status, bit 36 sign, bits 37..45 value (scale 32 fpm)
    baro_status = extract_bits(val, 56, 35, 35)
    if baro_status:
        baro_sign = -1 if extract_bits(val, 56, 36, 36) else 1
        baro_rate: int | None = baro_sign * extract_bits(val, 56, 37, 45) * 32
    else:
        baro_rate = None

    # Inertial vertical velocity: bit 46 status, bit 47 sign, bits 48..56 value (scale 32 fpm)
    inertial_status = extract_bits(val, 56, 46, 46)
    if inertial_status:
        inertial_sign = -1 if extract_bits(val, 56, 47, 47) else 1
        inertial_rate: int | None = inertial_sign * extract_bits(val, 56, 48, 56) * 32
    else:
        inertial_rate = None

    return BDS60Data(
        magnetic_heading_deg=mag_hdg,
        indicated_airspeed_kt=ias,
        mach=mach,
        baro_vertical_rate_fpm=baro_rate,
        inertial_vertical_rate_fpm=inertial_rate,
        raw_mb=mb_bytes,
    )


def classify_bds(mb_bytes: bytes) -> tuple[int, int]:
    """Classify the Comm-B BDS register format of a 56-bit MB field.

    Returns (major, minor) BDS code.
    """
    if mb_bytes[0] == 0x10:
        return (1, 0)
    if mb_bytes[0] == 0x20:
        return (2, 0)

    val = int.from_bytes(mb_bytes[:7], "big")

    # Check BDS 4,0 plausibility
    # MCP alt status or Baro status
    mcp_status = extract_bits(val, 56, 1, 1)
    baro_status = extract_bits(val, 56, 27, 27)
    if mcp_status and baro_status:
        baro_val = 800.0 + extract_bits(val, 56, 28, 39) * 0.1
        if 850.0 <= baro_val <= 1100.0:
            return (4, 0)

    # Check BDS 5,0 vs BDS 6,0
    # BDS 5,0 has ground speed at bit 24 (scale 2kt)
    # BDS 6,0 has Mach at bit 24 (scale 0.004)
    hdg_status = extract_bits(val, 56, 1, 1)
    mach_status = extract_bits(val, 56, 24, 24)
    if hdg_status and mach_status:
        mach_val = extract_bits(val, 56, 25, 34) * 0.004
        if 0.1 <= mach_val <= 1.0:
            return (6, 0)

    roll_status = extract_bits(val, 56, 1, 1)
    gs_status = extract_bits(val, 56, 24, 24)
    if roll_status and gs_status:
        gs_val = extract_bits(val, 56, 25, 34) * 2
        if 30 <= gs_val <= 800:
            return (5, 0)

    # Default to BDS 1,7 if non-zero
    if val != 0:
        return (1, 7)

    return (0, 0)


def decode_comm_b(mb_bytes: bytes, bds: tuple[int, int] | None = None) -> Any:
    """Decode a 56-bit Comm-B MB payload into its corresponding BDS data structure."""
    if bds is None:
        bds = classify_bds(mb_bytes)

    if bds == (1, 0):
        return decode_bds_10(mb_bytes)
    elif bds == (1, 7):
        return decode_bds_17(mb_bytes)
    elif bds == (2, 0):
        return decode_bds_20(mb_bytes)
    elif bds == (4, 0):
        return decode_bds_40(mb_bytes)
    elif bds == (5, 0):
        return decode_bds_50(mb_bytes)
    elif bds == (6, 0):
        return decode_bds_60(mb_bytes)
    else:
        return decode_bds_17(mb_bytes)


__all__ = [
    "AIS_CHARMAP",
    "BDS10Data",
    "BDS10DataLinkCapability",
    "BDS17Data",
    "BDS17GICBCapability",
    "BDS20AircraftIdentification",
    "BDS20Data",
    "BDS40Data",
    "BDS40SelectedVerticalIntention",
    "BDS50Data",
    "BDS50TrackAndTurnReport",
    "BDS60Data",
    "BDS60HeadingAndSpeedReport",
    "classify_bds",
    "decode_bds_10",
    "decode_bds_17",
    "decode_bds_20",
    "decode_bds_40",
    "decode_bds_50",
    "decode_bds_60",
    "decode_comm_b",
]
