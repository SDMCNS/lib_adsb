"""Coordinate reference frames and transformations (WGS84, ECEF, Local ENU).

All calculations are strictly typed and use double-precision floating point.
Provides Bowring iterative conversion with near-polar fallback (<1e-9 m precision).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

# Physical & Geodetic Constants
SPEED_OF_LIGHT_MPS: float = 299_792_458.0
WGS84_A_M: float = 6_378_137.0  # Semi-major axis
WGS84_F: float = 1.0 / 298.257223563  # Flattening
WGS84_B_M: float = 6_356_752.314245179  # Semi-minor axis: a * (1 - f)
WGS84_E2: float = 6.6943799901413165e-3  # First eccentricity squared: f * (2 - f)
WGS84_E_PRIME2: float = (WGS84_A_M**2 - WGS84_B_M**2) / (WGS84_B_M**2)
EARTH_MEAN_RADIUS_M: float = 6_371_008.8
EFFECTIVE_EARTH_RADIUS_FACTOR: float = 4.0 / 3.0

# Standard Atmosphere & RF Constants
GRAVITY_MPS2: float = 9.80665
ISA_T0_K: float = 288.15
ISA_P0_PA: float = 101_325.0
ISA_RHO0_KGPM3: float = 1.225
ISA_LAPSE_RATE_KPM: float = 0.0065
GAS_CONSTANT_AIR: float = 287.05287
GAMMA_AIR: float = 1.4
FREQ_1090ES_HZ: float = 1_090_000_000.0
FREQ_UAT_HZ: float = 978_000_000.0


@dataclass(frozen=True, slots=True)
class GeodeticPosition:
    """WGS84 Geodetic Position."""

    lat_deg: float
    lon_deg: float
    alt_hae_m: float = 0.0


@dataclass(frozen=True, slots=True)
class ECEFPosition:
    """Earth-Centered Earth-Fixed (ECEF) Cartesian Position."""

    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True, slots=True)
class ENUPosition:
    """Local East-North-Up (ENU) Cartesian Position."""

    east_m: float
    north_m: float
    up_m: float


def geodetic_to_ecef(
    lat_deg: float, lon_deg: float, alt_hae_m: float = 0.0
) -> tuple[float, float, float]:
    """Convert WGS84 Geodetic coordinates to ECEF Cartesian (X, Y, Z in meters)."""
    phi = math.radians(lat_deg)
    lam = math.radians(lon_deg)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sin_lam = math.sin(lam)
    cos_lam = math.cos(lam)

    n_phi = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)

    x_m = (n_phi + alt_hae_m) * cos_phi * cos_lam
    y_m = (n_phi + alt_hae_m) * cos_phi * sin_lam
    z_m = (n_phi * (1.0 - WGS84_E2) + alt_hae_m) * sin_phi

    return (x_m, y_m, z_m)


def ecef_to_geodetic(x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
    """Convert ECEF Cartesian coordinates (meters) to WGS84 Geodetic (lat_deg, lon_deg, alt_hae_m).

    Uses Bowring's method with 2-iteration refinement and polar fallback (<1e-9 m accuracy).
    """
    p = math.hypot(x_m, y_m)

    # Near-polar boundary singularity check
    if p < 1e-6:
        lon_deg = 0.0
        if z_m >= 0.0:
            lat_deg = 90.0
            alt_hae_m = z_m - WGS84_B_M
        else:
            lat_deg = -90.0
            alt_hae_m = -z_m - WGS84_B_M
        return (lat_deg, lon_deg, alt_hae_m)

    lam = math.atan2(y_m, x_m)

    # Bowring initial parametric latitude
    theta0 = math.atan2(z_m * WGS84_A_M, p * WGS84_B_M)
    sin_th0 = math.sin(theta0)
    cos_th0 = math.cos(theta0)

    phi1 = math.atan2(
        z_m + WGS84_E_PRIME2 * WGS84_B_M * (sin_th0**3),
        p - WGS84_E2 * WGS84_A_M * (cos_th0**3),
    )

    # Bowring refinement iteration 1
    theta1 = math.atan2(WGS84_B_M * math.sin(phi1), WGS84_A_M * math.cos(phi1))
    sin_th1 = math.sin(theta1)
    cos_th1 = math.cos(theta1)

    phi = math.atan2(
        z_m + WGS84_E_PRIME2 * WGS84_B_M * (sin_th1**3),
        p - WGS84_E2 * WGS84_A_M * (cos_th1**3),
    )

    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    n_phi = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_phi * sin_phi)

    # Polar fallback for height calculation when cos(phi) approaches 0
    if abs(math.degrees(phi)) >= 89.9:
        alt_hae_m = (z_m / sin_phi) - n_phi * (1.0 - WGS84_E2)
    else:
        alt_hae_m = (p / cos_phi) - n_phi

    lat_deg = math.degrees(phi)
    lon_deg = math.degrees(lam)

    # Normalize lon_deg to [-180, 180)
    lon_deg = (lon_deg + 180.0) % 360.0 - 180.0

    return (lat_deg, lon_deg, alt_hae_m)


def enu_rotation_matrix(lat0_deg: float, lon0_deg: float) -> npt.NDArray[np.float64]:
    """Compute the 3x3 rotation matrix R_ECEF->ENU anchored at origin (lat0_deg, lon0_deg)."""
    phi = math.radians(lat0_deg)
    lam = math.radians(lon0_deg)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sin_lam = math.sin(lam)
    cos_lam = math.cos(lam)

    rot: npt.NDArray[np.float64] = np.array(
        [
            [-sin_lam, cos_lam, 0.0],
            [-sin_phi * cos_lam, -sin_phi * sin_lam, cos_phi],
            [cos_phi * cos_lam, cos_phi * sin_lam, sin_phi],
        ],
        dtype=np.float64,
    )
    return rot


def ecef_to_enu(
    x_m: float,
    y_m: float,
    z_m: float,
    lat0_deg: float,
    lon0_deg: float,
    alt0_hae_m: float = 0.0,
) -> tuple[float, float, float]:
    """Convert ECEF coordinates (meters) to local ENU (East, North, Up meters)
    relative to origin.
    """
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_hae_m)
    dx = x_m - x0
    dy = y_m - y0
    dz = z_m - z0

    rot = enu_rotation_matrix(lat0_deg, lon0_deg)
    delta_r = np.array([dx, dy, dz], dtype=np.float64)
    enu: npt.NDArray[np.float64] = rot @ delta_r
    return (float(enu[0]), float(enu[1]), float(enu[2]))


def enu_to_ecef(
    east_m: float,
    north_m: float,
    up_m: float,
    lat0_deg: float,
    lon0_deg: float,
    alt0_hae_m: float = 0.0,
) -> tuple[float, float, float]:
    """Convert local ENU coordinates (meters) to ECEF Cartesian (meters) relative to origin."""
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_hae_m)
    rot = enu_rotation_matrix(lat0_deg, lon0_deg)
    enu_vec = np.array([east_m, north_m, up_m], dtype=np.float64)
    delta_r: npt.NDArray[np.float64] = rot.T @ enu_vec
    return (float(x0 + delta_r[0]), float(y0 + delta_r[1]), float(z0 + delta_r[2]))


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_hae_m: float,
    lat0_deg: float,
    lon0_deg: float,
    alt0_hae_m: float = 0.0,
) -> tuple[float, float, float]:
    """Convert WGS84 Geodetic coordinates to local ENU coordinates (meters)."""
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_hae_m)
    return ecef_to_enu(x, y, z, lat0_deg, lon0_deg, alt0_hae_m)


def enu_to_geodetic(
    east_m: float,
    north_m: float,
    up_m: float,
    lat0_deg: float,
    lon0_deg: float,
    alt0_hae_m: float = 0.0,
) -> tuple[float, float, float]:
    """Convert local ENU coordinates (meters) to WGS84 Geodetic coordinates."""
    x, y, z = enu_to_ecef(east_m, north_m, up_m, lat0_deg, lon0_deg, alt0_hae_m)
    return ecef_to_geodetic(x, y, z)


__all__ = [
    "EARTH_MEAN_RADIUS_M",
    "EFFECTIVE_EARTH_RADIUS_FACTOR",
    "FREQ_1090ES_HZ",
    "FREQ_UAT_HZ",
    "GAMMA_AIR",
    "GAS_CONSTANT_AIR",
    "GRAVITY_MPS2",
    "ISA_LAPSE_RATE_KPM",
    "ISA_P0_PA",
    "ISA_RHO0_KGPM3",
    "ISA_T0_K",
    "SPEED_OF_LIGHT_MPS",
    "WGS84_A_M",
    "WGS84_B_M",
    "WGS84_E2",
    "WGS84_E_PRIME2",
    "WGS84_F",
    "ECEFPosition",
    "ENUPosition",
    "GeodeticPosition",
    "ecef_to_enu",
    "ecef_to_geodetic",
    "enu_rotation_matrix",
    "enu_to_ecef",
    "enu_to_geodetic",
    "geodetic_to_ecef",
    "geodetic_to_enu",
]
