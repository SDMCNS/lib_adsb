"""Compact Position Reporting (CPR) encoding and decoding.

Authoritative reference: RTCA DO-260B / DO-260C § 2.2.3.2.3,
ICAO Annex 10 Vol IV, and docs/02_PROTOCOL_SPECIFICATION.md Section 5.
"""

from __future__ import annotations

import math

# 59-entry precomputed latitude zone boundaries for NL(lat)
# Table of transition latitudes where NL decreases by 1
_NL_TRANSITION_TABLE: list[float] = []
for _n in range(2, 60):
    _num = 1.0 - math.cos(math.pi / 30.0)
    _den = 1.0 - math.cos(2.0 * math.pi / _n)
    _val = math.sqrt(_num / _den)
    _val = max(0.0, min(1.0, _val))
    _NL_TRANSITION_TABLE.append(math.degrees(math.acos(_val)))

cpr_nl_table = _NL_TRANSITION_TABLE


def nl_lat(lat_deg: float) -> int:
    """Compute the CPR Number of Longitude Zones function NL(phi).

    Properties:
    - NL(phi) is symmetric: NL(phi) == NL(-phi).
    - NL(0) == 59.
    - NL(lat) is monotonically non-increasing from equator to pole.
    - NL(|lat| == 87.0) == 2, NL(|lat| > 87.0) == 1.
    """
    lat = abs(lat_deg)
    if lat == 0.0:
        return 59
    if lat >= 87.0:
        return 1 if lat > 87.0 else 2

    cos_lat = math.cos(math.radians(lat))
    num = 1.0 - math.cos(math.pi / 30.0)
    den = cos_lat**2
    if den == 0.0:
        return 1
    val = 1.0 - (num / den)
    val = max(-1.0, min(1.0, val))
    nl = math.floor(2.0 * math.pi / math.acos(val))
    return max(1, min(59, nl))


def _mods(x: float, y: float) -> float:
    """Modulo function: x - y * floor(x / y) (always non-negative for y > 0)."""
    return x - y * math.floor(x / y)


def encode_cpr_airborne(lat_deg: float, lon_deg: float, cpr_format: int) -> tuple[int, int]:
    """Encode geodetic coordinates into 17-bit airborne CPR fractions (YZ, XZ).

    Parameters:
        lat_deg: Latitude [-90.0, 90.0].
        lon_deg: Longitude [-180.0, 180.0).
        cpr_format: 0 for Even frame, 1 for Odd frame.

    Returns:
        tuple[int, int]: (cpr_lat, cpr_lon) as 17-bit integers in [0, 131071].
    """
    d_lat = 360.0 / (60.0 - cpr_format)
    yz = math.floor(131072.0 * (_mods(lat_deg, d_lat) / d_lat) + 0.5) % 131072

    rlat = d_lat * (yz / 131072.0 + math.floor(lat_deg / d_lat))
    nl = nl_lat(rlat)
    ni = max(nl - cpr_format, 1)
    d_lon = 360.0 / ni
    xz = math.floor(131072.0 * (_mods(lon_deg, d_lon) / d_lon) + 0.5) % 131072

    return int(yz), int(xz)


def encode_cpr_surface(lat_deg: float, lon_deg: float, cpr_format: int) -> tuple[int, int]:
    """Encode geodetic coordinates into 17-bit surface CPR fractions (YZ, XZ).

    Surface CPR uses a 4x finer grid (90-degree span).
    """
    d_lat = 90.0 / (60.0 - cpr_format)
    yz = math.floor(131072.0 * (_mods(lat_deg, d_lat) / d_lat) + 0.5) % 131072

    rlat = d_lat * (yz / 131072.0 + math.floor(lat_deg / d_lat))
    nl = nl_lat(rlat)
    ni = max(nl - cpr_format, 1)
    d_lon = 90.0 / ni
    xz = math.floor(131072.0 * (_mods(lon_deg, d_lon) / d_lon) + 0.5) % 131072

    return int(yz), int(xz)


def decode_cpr_global_airborne(
    lat_cpr0: float,
    lon_cpr0: float,
    lat_cpr1: float,
    lon_cpr1: float,
    most_recent: int = 1,
) -> tuple[float, float]:
    """Decode global airborne position from paired Even (0) and Odd (1) CPR fractions.

    Parameters:
        lat_cpr0, lon_cpr0: Even frame CPR fractions in [0.0, 1.0).
        lat_cpr1, lon_cpr1: Odd frame CPR fractions in [0.0, 1.0).
        most_recent: 0 if Even frame is more recent, 1 if Odd frame is more recent.

    Returns:
        tuple[float, float]: (lat_deg, lon_deg).

    Raises:
        ValueError: If CPR latitude zone straddle is detected (NL(Rlat0) != NL(Rlat1)).
    """
    d_lat0 = 360.0 / 60.0
    d_lat1 = 360.0 / 59.0

    j = math.floor(59.0 * lat_cpr0 - 60.0 * lat_cpr1 + 0.5)
    rlat0 = d_lat0 * ((j % 60) + lat_cpr0)
    rlat1 = d_lat1 * ((j % 59) + lat_cpr1)

    if rlat0 >= 270.0:
        rlat0 -= 360.0
    if rlat1 >= 270.0:
        rlat1 -= 360.0

    # Mandatory Zone Straddle Check
    if nl_lat(rlat0) != nl_lat(rlat1):
        raise ValueError("CPR zone boundary straddle detected")

    lat = rlat1 if most_recent == 1 else rlat0
    nl = nl_lat(lat)

    if most_recent == 0:
        ni = nl
        cpr_lon = lon_cpr0
    else:
        ni = max(nl - 1, 1)
        cpr_lon = lon_cpr1

    m = math.floor(lon_cpr0 * (nl - 1) - lon_cpr1 * nl + 0.5)
    d_lon = 360.0 / ni
    lon = d_lon * ((m % ni) + cpr_lon)

    if lon >= 180.0:
        lon -= 360.0
    elif lon < -180.0:
        lon += 360.0

    return lat, lon


decode_cpr_airborne = decode_cpr_global_airborne


def decode_cpr_global_surface(
    lat_cpr0: float,
    lon_cpr0: float,
    lat_cpr1: float,
    lon_cpr1: float,
    ref_lat: float,
    ref_lon: float,
    most_recent: int = 1,
) -> tuple[float, float]:
    """Decode global surface position from paired Even and Odd frames with reference origin."""
    d_lat0 = 90.0 / 60.0
    d_lat1 = 90.0 / 59.0

    j = math.floor(59.0 * lat_cpr0 - 60.0 * lat_cpr1 + 0.5)
    rlat0 = d_lat0 * ((j % 60) + lat_cpr0)
    rlat1 = d_lat1 * ((j % 59) + lat_cpr1)

    lat_cand = rlat1 if most_recent == 1 else rlat0
    # Disambiguate quadrant using ref_lat
    # Quadrant offsets: 0 or -90
    k_lat = round((ref_lat - lat_cand) / 90.0) * 90.0
    lat = lat_cand + k_lat

    nl = nl_lat(lat)
    ni = max(nl - most_recent, 1)
    cpr_lon = lon_cpr1 if most_recent == 1 else lon_cpr0

    m = math.floor(lon_cpr0 * (nl - 1) - lon_cpr1 * nl + 0.5)
    d_lon = 90.0 / ni
    lon_cand = d_lon * ((m % ni) + cpr_lon)
    k_lon = round((ref_lon - lon_cand) / 90.0) * 90.0
    lon = lon_cand + k_lon

    if lon >= 180.0:
        lon -= 360.0
    elif lon < -180.0:
        lon += 360.0

    return lat, lon


decode_cpr_surface = decode_cpr_global_surface


def decode_cpr_local(
    lat_cpr: float,
    lon_cpr: float,
    cpr_format: int,
    ref_lat: float,
    ref_lon: float,
    surface: bool = False,
) -> tuple[float, float]:
    """Decode single CPR frame using a local reference position within ambiguity window.

    Window: ~180 NM for airborne, ~45 NM for surface.
    """
    span = 90.0 if surface else 360.0
    d_lat = span / (60.0 - cpr_format)
    j = math.floor(ref_lat / d_lat) + math.floor(0.5 + (_mods(ref_lat, d_lat) / d_lat) - lat_cpr)
    lat = d_lat * (j + lat_cpr)

    nl = nl_lat(lat)
    ni = max(nl - cpr_format, 1)
    d_lon = span / ni
    m = math.floor(ref_lon / d_lon) + math.floor(0.5 + (_mods(ref_lon, d_lon) / d_lon) - lon_cpr)
    lon = d_lon * (m + lon_cpr)

    if lon >= 180.0:
        lon -= 360.0
    elif lon < -180.0:
        lon += 360.0

    return lat, lon


__all__ = [
    "cpr_nl_table",
    "decode_cpr_airborne",
    "decode_cpr_global_airborne",
    "decode_cpr_global_surface",
    "decode_cpr_local",
    "decode_cpr_surface",
    "encode_cpr_airborne",
    "encode_cpr_surface",
    "nl_lat",
]
