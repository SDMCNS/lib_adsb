"""Terminal procedures geometry solver and runway kinematics utilities.

Authoritative reference: RTCA DO-260B / DO-260C, ICAO Doc 8168 (PANS-OPS),
and docs/03_FLIGHT_SYNTHESIS_AND_PHYSICS.md Section 3.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from adsb.constants import FEET_TO_METERS, METERS_TO_FEET, NM_TO_METERS
from adsb.core.geodesy import destination_point
from adsb.synthesis.navigation import Waypoint

if TYPE_CHECKING:
    from adsb.synthesis.airports import Airport, Runway


def compute_departure_fix(
    midpoint_lat_deg: float,
    midpoint_lon_deg: float,
    runway_heading_deg: float,
    distance_nm: float = 5.0,
) -> tuple[float, float]:
    """Compute (lat, lon) along runway heading from runway midpoint reference.

    Per Requirement R3, departures maintain runway heading for 5.0 NM.
    """
    return destination_point(
        midpoint_lat_deg,
        midpoint_lon_deg,
        distance_nm * NM_TO_METERS,
        runway_heading_deg,
    )


def compute_ils_intercept_fix(
    midpoint_lat_deg: float,
    midpoint_lon_deg: float,
    runway_heading_deg: float,
    distance_nm: float = 17.0,
) -> tuple[float, float]:
    """Compute (lat, lon) on extended ILS final approach course from runway midpoint.

    Per Requirement R3, arrivals intercept the ILS localizer course 17.0 NM out.
    """
    reciprocal_hdg = (runway_heading_deg + 180.0) % 360.0
    return destination_point(
        midpoint_lat_deg,
        midpoint_lon_deg,
        distance_nm * NM_TO_METERS,
        reciprocal_hdg,
    )


def compute_departure_waypoint(
    airport: Airport,
    runway: Runway | None = None,
    distance_nm: float = 5.0,
) -> Waypoint:
    """Create a structured Waypoint for the 5 NM departure fix."""
    rwy = runway or airport.primary_runway
    lat, lon = compute_departure_fix(
        rwy.midpoint_lat_deg,
        rwy.midpoint_lon_deg,
        rwy.true_heading_deg,
        distance_nm=distance_nm,
    )
    return Waypoint(
        name=f"{airport.icao}-DEP5",
        lat_deg=lat,
        lon_deg=lon,
        alt_target_ft=3000.0,
        target_alt_ft=3000.0,
        flyover=True,
    )


def compute_ils_intercept_waypoint(
    airport: Airport,
    runway: Runway | None = None,
    distance_nm: float = 17.0,
    altitude_ft: float = 3000.0,
) -> Waypoint:
    """Create a structured Waypoint for the 17 NM ILS final approach intercept fix."""
    rwy = runway or airport.primary_runway
    lat, lon = compute_ils_intercept_fix(
        rwy.midpoint_lat_deg,
        rwy.midpoint_lon_deg,
        rwy.true_heading_deg,
        distance_nm=distance_nm,
    )
    return Waypoint(
        name=f"{airport.icao}-ILS17",
        lat_deg=lat,
        lon_deg=lon,
        alt_target_ft=altitude_ft,
        target_alt_ft=altitude_ft,
        speed_target_kt=180.0,
        flyover=False,
    )


def compute_glide_slope_intercept_distance_m(
    runway_elevation_ft: float,
    intercept_alt_ft: float = 3000.0,
    glide_slope_deg: float = 3.0,
) -> float:
    """Calculate distance in meters from runway midpoint to 3.0-degree glide slope intercept.

    For a sea-level runway, this is ~17,448 m (~9.42 NM).
    """
    delta_h_m = max(0.0, intercept_alt_ft - runway_elevation_ft) * FEET_TO_METERS
    gamma_rad = math.radians(glide_slope_deg)
    return delta_h_m / math.tan(gamma_rad)


def compute_glide_slope_altitude_ft(
    distance_to_touchdown_m: float,
    runway_elevation_ft: float,
    glide_slope_deg: float = 3.0,
) -> float:
    """Compute target altitude (feet MSL) on a 3.0-degree glide slope given distance from touchdown."""
    gamma_rad = math.radians(glide_slope_deg)
    delta_h_ft = (distance_to_touchdown_m * math.tan(gamma_rad)) * METERS_TO_FEET
    return runway_elevation_ft + delta_h_ft


def compute_capped_cruise_altitude(
    route_distance_nm: float,
    requested_alt_ft: float,
    origin_elev_ft: float = 0.0,
    dest_elev_ft: float = 0.0,
    service_ceiling_ft: float = 41000.0,
) -> float:
    """Dynamically cap cruise altitude for short-haul routes to prevent VNAV profile violation.

    Ensures sufficient distance is available for:
    - 5 NM straight departure
    - En-route climb to cruise
    - Level cruise segment (minimum 5-10 NM)
    - Descent to 3,000 ft at standard 3-degree gradient (~318 ft/NM)
    - Level deceleration to glide slope intercept (~9.4 NM)
    - Final approach (17 NM)
    """
    base_alt = max(3000.0, origin_elev_ft + 1500.0, dest_elev_ft + 1500.0)
    fixed_terminal_nm = 22.0  # 5 NM dep + 17 NM arr
    avail_enroute_nm = max(0.0, route_distance_nm - fixed_terminal_nm)

    # Combined climb + descent gradient ~ 180 ft/NM, reserving 10 NM for cruise + decel
    usable_nm = max(0.0, avail_enroute_nm - 10.0)
    delta_h_max = usable_nm * 180.0
    max_alt_ft = base_alt + delta_h_max

    # Cap to service ceiling and round to nearest 1,000 ft
    ceiling_cap = min(requested_alt_ft, service_ceiling_ft, max_alt_ft)
    capped_alt = max(base_alt, math.floor(ceiling_cap / 1000.0) * 1000.0)
    return capped_alt


__all__ = [
    "compute_capped_cruise_altitude",
    "compute_departure_fix",
    "compute_departure_waypoint",
    "compute_glide_slope_altitude_ft",
    "compute_glide_slope_intercept_distance_m",
    "compute_ils_intercept_fix",
    "compute_ils_intercept_waypoint",
]
