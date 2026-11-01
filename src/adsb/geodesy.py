"""Geodesy and coordinate transformations (re-exported from adsb.core)."""

from __future__ import annotations

from adsb.core.coordinates import (
    EARTH_MEAN_RADIUS_M,
    EFFECTIVE_EARTH_RADIUS_FACTOR,
    WGS84_A_M,
    WGS84_B_M,
    WGS84_E2,
    WGS84_E_PRIME2,
    WGS84_F,
    ECEFPosition,
    ENUPosition,
    GeodeticPosition,
    ecef_to_enu,
    ecef_to_geodetic,
    enu_rotation_matrix,
    enu_to_ecef,
    enu_to_geodetic,
    geodetic_to_ecef,
    geodetic_to_enu,
)
from adsb.core.geodesy import (
    along_track_distance,
    cross_track_distance,
    destination_point,
    haversine_distance,
    initial_bearing,
)

__all__ = [
    "EARTH_MEAN_RADIUS_M",
    "EFFECTIVE_EARTH_RADIUS_FACTOR",
    "WGS84_A_M",
    "WGS84_B_M",
    "WGS84_E2",
    "WGS84_E_PRIME2",
    "WGS84_F",
    "ECEFPosition",
    "ENUPosition",
    "GeodeticPosition",
    "along_track_distance",
    "cross_track_distance",
    "destination_point",
    "ecef_to_enu",
    "ecef_to_geodetic",
    "enu_rotation_matrix",
    "enu_to_ecef",
    "enu_to_geodetic",
    "geodetic_to_ecef",
    "geodetic_to_enu",
    "haversine_distance",
    "initial_bearing",
]
