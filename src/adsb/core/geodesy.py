"""Spherical geodesy primitives for great-circle navigation.

Uses IUGG mean spherical radius R_E = 6,371,008.8 m.
"""

from __future__ import annotations

import math

from adsb.core.coordinates import EARTH_MEAN_RADIUS_M


def haversine_distance(
    lat1_deg: float,
    lon1_deg: float,
    lat2_deg: float,
    lon2_deg: float,
    radius_m: float = EARTH_MEAN_RADIUS_M,
) -> float:
    """Compute the great-circle distance between two points on a sphere (meters)."""
    phi1 = math.radians(lat1_deg)
    phi2 = math.radians(lat2_deg)
    dphi = math.radians(lat2_deg - lat1_deg)
    dlam = math.radians(lon2_deg - lon1_deg)

    a = (math.sin(dphi / 2.0) ** 2) + math.cos(phi1) * math.cos(phi2) * (math.sin(dlam / 2.0) ** 2)
    a = max(0.0, min(1.0, a))
    delta = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return radius_m * delta


def initial_bearing(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    """Compute initial great-circle bearing (azimuth) from point 1 to point 2
    in degrees [0, 360).
    """
    phi1 = math.radians(lat1_deg)
    phi2 = math.radians(lat2_deg)
    dlam = math.radians(lon2_deg - lon1_deg)

    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)

    bearing_deg = math.degrees(math.atan2(y, x))
    return (bearing_deg + 360.0) % 360.0


def cross_track_distance(
    lat_a_deg: float,
    lon_a_deg: float,
    lat_b_deg: float,
    lon_b_deg: float,
    lat_p_deg: float,
    lon_p_deg: float,
    radius_m: float = EARTH_MEAN_RADIUS_M,
) -> float:
    """Compute signed cross-track distance (meters) from great circle path A->B to point P.

    Positive means point P is to the right of path A->B; negative means to the left.
    """
    d_ap = haversine_distance(lat_a_deg, lon_a_deg, lat_p_deg, lon_p_deg, radius_m)
    delta_ap = d_ap / radius_m

    theta_ap = math.radians(initial_bearing(lat_a_deg, lon_a_deg, lat_p_deg, lon_p_deg))
    theta_ab = math.radians(initial_bearing(lat_a_deg, lon_a_deg, lat_b_deg, lon_b_deg))

    sin_delta_xt = math.sin(delta_ap) * math.sin(theta_ap - theta_ab)
    sin_delta_xt = max(-1.0, min(1.0, sin_delta_xt))
    delta_xt = math.asin(sin_delta_xt)

    return radius_m * delta_xt


def along_track_distance(
    lat_a_deg: float,
    lon_a_deg: float,
    lat_b_deg: float,
    lon_b_deg: float,
    lat_p_deg: float,
    lon_p_deg: float,
    radius_m: float = EARTH_MEAN_RADIUS_M,
) -> float:
    """Compute signed along-track distance (meters) from point A toward point B for point P.

    Returns the projection of path A->P along great circle path A->B.
    """
    d_ap = haversine_distance(lat_a_deg, lon_a_deg, lat_p_deg, lon_p_deg, radius_m)
    delta_ap = d_ap / radius_m

    d_xt = cross_track_distance(
        lat_a_deg, lon_a_deg, lat_b_deg, lon_b_deg, lat_p_deg, lon_p_deg, radius_m
    )
    delta_xt = d_xt / radius_m

    cos_delta_xt = math.cos(delta_xt)
    if abs(cos_delta_xt) < 1e-12:
        return 0.0

    ratio = math.cos(delta_ap) / cos_delta_xt
    ratio = max(-1.0, min(1.0, ratio))
    delta_at = math.acos(ratio)

    theta_ap = math.radians(initial_bearing(lat_a_deg, lon_a_deg, lat_p_deg, lon_p_deg))
    theta_ab = math.radians(initial_bearing(lat_a_deg, lon_a_deg, lat_b_deg, lon_b_deg))

    sign = -1.0 if math.cos(theta_ap - theta_ab) < 0.0 else 1.0
    return sign * radius_m * delta_at


def destination_point(
    lat_deg: float,
    lon_deg: float,
    distance_m: float,
    bearing_deg: float,
    radius_m: float = EARTH_MEAN_RADIUS_M,
) -> tuple[float, float]:
    """Compute destination point given starting point, distance (m), and initial bearing (deg)."""
    phi1 = math.radians(lat_deg)
    lam1 = math.radians(lon_deg)
    theta = math.radians(bearing_deg)
    delta = distance_m / radius_m

    phi2 = math.asin(
        math.sin(phi1) * math.cos(delta) + math.cos(phi1) * math.sin(delta) * math.cos(theta)
    )
    lam2 = lam1 + math.atan2(
        math.sin(theta) * math.sin(delta) * math.cos(phi1),
        math.cos(delta) - math.sin(phi1) * math.sin(phi2),
    )

    dest_lat = math.degrees(phi2)
    dest_lon = math.degrees(lam2)
    dest_lon = (dest_lon + 180.0) % 360.0 - 180.0

    return (dest_lat, dest_lon)


__all__ = [
    "along_track_distance",
    "cross_track_distance",
    "destination_point",
    "haversine_distance",
    "initial_bearing",
]
