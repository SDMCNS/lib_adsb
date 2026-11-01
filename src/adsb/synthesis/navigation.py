"""Navigation algorithms, waypoint definitions, Rhumb lines, fly-by turn lead solver & holding patterns.

Authoritative reference: ICAO Annex 4 (Aeronautical Charts), FAA AIM 5-3-8,
and docs/03_FLIGHT_SYNTHESIS_AND_PHYSICS.md Section 3.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

from adsb.constants import EARTH_MEAN_RADIUS_M, GRAVITY_MPS2


class TurnDirection(StrEnum):
    """Holding or turn direction."""

    RIGHT = "RIGHT"  # Standard right-hand turns
    LEFT = "LEFT"  # Non-standard left-hand turns


class HoldingEntryType(StrEnum):
    """Holding pattern entry procedure per ICAO / FAA AIM 5-3-8."""

    DIRECT = "DIRECT"
    PARALLEL = "PARALLEL"
    TEARDROP = "TEARDROP"


class HoldingLegState(StrEnum):
    """Sub-phase within a holding pattern racetrack."""

    ENTRY = "ENTRY"
    INBOUND_LEG = "INBOUND_LEG"
    OUTBOUND_TURN = "OUTBOUND_TURN"
    OUTBOUND_LEG = "OUTBOUND_LEG"
    INBOUND_TURN = "INBOUND_TURN"


@dataclass(frozen=True, slots=True)
class GeoPoint:
    """Geodetic 3D coordinate point."""

    lat_deg: float
    lon_deg: float
    alt_msl_m: float = 0.0


@dataclass(frozen=True, slots=True)
class HoldingPattern:
    """Specification of an enroute or terminal holding pattern."""

    fix_name: str
    fix_lat_deg: float
    fix_lon_deg: float
    inbound_track_deg: float
    turn_direction: TurnDirection = TurnDirection.RIGHT
    leg_time_s: float | None = None  # 60s <= 14000 ft, 90s > 14000 ft
    leg_length_nm: float | None = None
    target_alt_ft: float | None = None
    speed_limit_kt: float | None = None


@dataclass(frozen=True, slots=True)
class Waypoint:
    """Navigation waypoint along an LNAV route."""

    name: str
    lat_deg: float
    lon_deg: float
    alt_target_ft: float | None = None
    target_alt_ft: float | None = None
    speed_target_kt: float | None = None
    flyover: bool = False
    holding_pattern: HoldingPattern | None = None

    def get_target_alt_ft(self) -> float | None:
        """Return target altitude constraint if specified."""
        if self.alt_target_ft is not None:
            return self.alt_target_ft
        return self.target_alt_ft


@dataclass(frozen=True, slots=True)
class FlightPlan:
    """Flight plan route and mission parameters."""

    icao_hex: str
    callsign: str
    squawk: str
    aircraft_type: str
    origin: GeoPoint
    destination: GeoPoint
    departure_time_s: float
    cruise_alt_ft: float
    cruise_mach: float | None = None
    cruise_tas_kt: float | None = None
    waypoints: list[Waypoint] = field(default_factory=list)


# --- Rhumb Line (Loxodrome) Navigation ---


def rhumb_distance_and_bearing(
    lat1_deg: float,
    lon1_deg: float,
    lat2_deg: float,
    lon2_deg: float,
    radius_m: float = EARTH_MEAN_RADIUS_M,
) -> tuple[float, float]:
    """Compute Rhumb line (loxodrome) distance (meters) and constant bearing (deg)."""
    phi1 = math.radians(max(-89.9, min(89.9, lat1_deg)))
    phi2 = math.radians(max(-89.9, min(89.9, lat2_deg)))
    dphi = phi2 - phi1
    dlam = math.radians((lon2_deg - lon1_deg + 180.0) % 360.0 - 180.0)

    psi1 = math.log(math.tan(math.pi / 4.0 + phi1 / 2.0))
    psi2 = math.log(math.tan(math.pi / 4.0 + phi2 / 2.0))
    dpsi = psi2 - psi1

    q = math.cos(phi1) if abs(dpsi) < 1e-12 else dphi / dpsi

    bearing_deg = (math.degrees(math.atan2(dlam, dpsi)) + 360.0) % 360.0
    dist_m = radius_m * math.sqrt(dphi * dphi + q * q * dlam * dlam)
    return dist_m, bearing_deg


def rhumb_destination_point(
    lat_deg: float,
    lon_deg: float,
    distance_m: float,
    bearing_deg: float,
    radius_m: float = EARTH_MEAN_RADIUS_M,
) -> tuple[float, float]:
    """Compute destination point along a constant Rhumb line course."""
    delta = distance_m / radius_m
    theta = math.radians(bearing_deg)

    phi1 = math.radians(max(-89.9, min(89.9, lat_deg)))
    lam1 = math.radians(lon_deg)

    phi2 = phi1 + delta * math.cos(theta)
    phi2 = max(-math.radians(89.9), min(math.radians(89.9), phi2))

    psi1 = math.log(math.tan(math.pi / 4.0 + phi1 / 2.0))
    psi2 = math.log(math.tan(math.pi / 4.0 + phi2 / 2.0))
    dpsi = psi2 - psi1

    q = math.cos(phi1) if abs(dpsi) < 1e-12 else (phi2 - phi1) / dpsi
    dlam = 0.0 if abs(q) < 1e-12 else (delta * math.sin(theta)) / q

    lam2 = (lam1 + dlam + math.pi) % (2.0 * math.pi) - math.pi

    dest_lat = math.degrees(phi2)
    dest_lon = math.degrees(lam2)
    return dest_lat, (dest_lon + 180.0) % 360.0 - 180.0


# --- Fly-By Turn Anticipation Lead Solver ---


def compute_flyby_lead_distance(
    inbound_track_deg: float,
    outbound_track_deg: float,
    groundspeed_mps: float,
    max_bank_angle_deg: float = 25.0,
    max_roll_rate_dps: float = 5.0,
    leg_length_m: float | None = None,
    gravity_mps2: float = GRAVITY_MPS2,
) -> tuple[float, float, float]:
    """Compute turn radius R_turn (m), lead distance d_lead (m), and bank angle phi_bank (deg).

    Parameters:
        inbound_track_deg: Track of inbound leg.
        outbound_track_deg: Track of outbound leg.
        groundspeed_mps: Aircraft groundspeed (m/s).
        max_bank_angle_deg: Maximum allowable bank angle (deg).
        max_roll_rate_dps: Roll rate limit (deg/s).
        leg_length_m: Optional length of inbound leg (m) for clamp.
        gravity_mps2: Gravitational acceleration (m/s^2).

    Returns:
        tuple[float, float, float]: (d_lead_m, turn_radius_m, bank_angle_deg)
    """
    diff_deg = (outbound_track_deg - inbound_track_deg + 180.0) % 360.0 - 180.0
    abs_diff_deg = abs(diff_deg)

    if abs_diff_deg < 0.5 or groundspeed_mps <= 0.0:
        return 0.0, 0.0, 0.0

    # Standard Rate Turn (Rate 1 = 3 deg/s) limit
    omega_std_radps = math.radians(3.0)
    phi_std_rad = math.atan((groundspeed_mps * omega_std_radps) / gravity_mps2)
    phi_max_rad = math.radians(max_bank_angle_deg)
    phi_bank_rad = min(phi_max_rad, phi_std_rad)
    phi_bank_deg = math.degrees(phi_bank_rad)

    # Steady-turn radius: R = V^2 / (g * tan(phi))
    turn_radius_m = (groundspeed_mps**2) / (gravity_mps2 * math.tan(phi_bank_rad))

    # Capped angle for tangent geometry to prevent divergence at 180 deg
    turn_angle_clamped_deg = min(abs_diff_deg, 120.0)
    half_angle_rad = math.radians(turn_angle_clamped_deg / 2.0)

    # Geometric lead distance: d_geom = R * tan(Delta_theta / 2)
    d_geom = turn_radius_m * math.tan(half_angle_rad)

    # Roll delay compensation: d_roll = 0.5 * V * (phi / roll_rate)
    roll_rate_radps = math.radians(max_roll_rate_dps)
    d_roll = 0.5 * groundspeed_mps * (phi_bank_rad / roll_rate_radps)

    d_lead_m = d_geom + d_roll

    # Safety clamp: do not anticipate more than 50% of the inbound leg length
    if leg_length_m is not None and leg_length_m > 0:
        d_lead_m = min(d_lead_m, 0.5 * leg_length_m)

    return d_lead_m, turn_radius_m, phi_bank_deg


# --- Holding Pattern Entry Sector Classifier ---


def classify_holding_entry(
    approach_track_deg: float,
    inbound_holding_track_deg: float,
    turn_direction: TurnDirection = TurnDirection.RIGHT,
) -> HoldingEntryType:
    """Classify holding pattern entry (Direct, Parallel, or Teardrop) per FAA/ICAO standards."""
    reciprocal_inbound_deg = (inbound_holding_track_deg + 180.0) % 360.0
    relative_angle = (approach_track_deg - reciprocal_inbound_deg + 360.0) % 360.0

    if turn_direction == TurnDirection.RIGHT:
        # Standard right-hand pattern
        if 0.0 <= relative_angle < 110.0:
            return HoldingEntryType.PARALLEL
        elif 290.0 <= relative_angle < 360.0:
            return HoldingEntryType.TEARDROP
        else:
            return HoldingEntryType.DIRECT
    else:
        # Non-standard left-hand pattern (mirrored)
        if 250.0 <= relative_angle < 360.0:
            return HoldingEntryType.PARALLEL
        elif 0.0 <= relative_angle < 70.0:
            return HoldingEntryType.TEARDROP
        else:
            return HoldingEntryType.DIRECT


__all__ = [
    "FlightPlan",
    "GeoPoint",
    "HoldingEntryType",
    "HoldingLegState",
    "HoldingPattern",
    "TurnDirection",
    "Waypoint",
    "classify_holding_entry",
    "compute_flyby_lead_distance",
    "rhumb_destination_point",
    "rhumb_distance_and_bearing",
]
