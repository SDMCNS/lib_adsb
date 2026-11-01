"""Aviation unit conversion constants and functions.

Strict naming conventions apply across all modules:
All variables representing physical quantities must be explicitly suffixed
with their units (_ft, _m, _kt, _mps, _fpm, _deg, _rad, _nm).
"""

from __future__ import annotations

import math

# Exact length conversions
FEET_TO_METERS: float = 0.3048
METERS_TO_FEET: float = 1.0 / 0.3048
NM_TO_METERS: float = 1852.0
METERS_TO_NM: float = 1.0 / 1852.0

# Exact velocity conversions
KNOTS_TO_MPS: float = 1852.0 / 3600.0
MPS_TO_KNOTS: float = 3600.0 / 1852.0
FPM_TO_MPS: float = 0.3048 / 60.0
MPS_TO_FPM: float = 60.0 / 0.3048


def feet_to_meters(ft: float) -> float:
    """Convert feet to meters."""
    return ft * FEET_TO_METERS


def meters_to_feet(m: float) -> float:
    """Convert meters to feet."""
    return m * METERS_TO_FEET


def ft_to_m(ft: float) -> float:
    """Alias for feet_to_meters."""
    return feet_to_meters(ft)


def m_to_ft(m: float) -> float:
    """Alias for meters_to_feet."""
    return meters_to_feet(m)


def knots_to_mps(kt: float) -> float:
    """Convert knots to meters per second."""
    return kt * KNOTS_TO_MPS


def mps_to_knots(mps: float) -> float:
    """Convert meters per second to knots."""
    return mps * MPS_TO_KNOTS


def kt_to_mps(kt: float) -> float:
    """Alias for knots_to_mps."""
    return knots_to_mps(kt)


def mps_to_kt(mps: float) -> float:
    """Alias for mps_to_knots."""
    return mps_to_knots(mps)


def fpm_to_mps(fpm: float) -> float:
    """Convert feet per minute to meters per second."""
    return fpm * FPM_TO_MPS


def mps_to_fpm(mps: float) -> float:
    """Convert meters per second to feet per minute."""
    return mps * MPS_TO_FPM


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    return math.radians(deg)


def rad_to_deg(rad: float) -> float:
    """Convert radians to degrees."""
    return math.degrees(rad)


def nm_to_meters(nm: float) -> float:
    """Convert nautical miles to meters."""
    return nm * NM_TO_METERS


def meters_to_nm(m: float) -> float:
    """Convert meters to nautical miles."""
    return m * METERS_TO_NM


def normalize_angle_deg(deg: float) -> float:
    """Normalize an angle in degrees to [0.0, 360.0)."""
    return deg % 360.0


def normalize_angle_diff_deg(deg: float) -> float:
    """Normalize an angle difference in degrees to [-180.0, 180.0)."""
    val = (deg + 180.0) % 360.0 - 180.0
    return val


__all__ = [
    "FEET_TO_METERS",
    "FPM_TO_MPS",
    "KNOTS_TO_MPS",
    "METERS_TO_FEET",
    "METERS_TO_NM",
    "MPS_TO_FPM",
    "MPS_TO_KNOTS",
    "NM_TO_METERS",
    "deg_to_rad",
    "feet_to_meters",
    "fpm_to_mps",
    "ft_to_m",
    "knots_to_mps",
    "kt_to_mps",
    "m_to_ft",
    "meters_to_feet",
    "meters_to_nm",
    "mps_to_fpm",
    "mps_to_knots",
    "mps_to_kt",
    "nm_to_meters",
    "normalize_angle_deg",
    "normalize_angle_diff_deg",
    "rad_to_deg",
]
