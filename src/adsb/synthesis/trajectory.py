"""Continuous 4D flight trajectory synthesis, VNAV state machine & kinematic generators.

Authoritative reference: RTCA DO-260B / DO-260C, BADA kinematic envelopes,
and docs/03_FLIGHT_SYNTHESIS_AND_PHYSICS.md Sections 3 and 4.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from adsb.constants import GRAVITY_MPS2, KNOTS_TO_MPS, METERS_TO_FEET
from adsb.core.geodesy import (
    destination_point,
    haversine_distance,
    initial_bearing,
)
from adsb.synthesis.atmosphere import (
    AtmosphereModel,
    compute_atmosphere,
)
from adsb.synthesis.performance import AircraftPerformanceProfile


class FlightPhase(StrEnum):
    """10-phase vertical navigation (VNAV) flight state machine."""

    PREFLIGHT = "PREFLIGHT"
    PREFLIGHT_TAXI = "PREFLIGHT"  # Alias
    PUSHBACK = "PUSHBACK"
    TAKEOFF_ROLL = "TAKEOFF_ROLL"
    INITIAL_CLIMB = "INITIAL_CLIMB"
    ENROUTE_CLIMB = "ENROUTE_CLIMB"
    CLIMB = "ENROUTE_CLIMB"  # Alias
    CRUISE = "CRUISE"
    HOLDING = "HOLDING"
    DESCENT = "DESCENT"
    APPROACH = "APPROACH"
    LANDING_ROLLOUT = "LANDING_ROLLOUT"
    TAXI_IN = "TAXI_IN"
    ARRIVED_GATE = "TAXI_IN"  # Alias


def compute_top_of_descent_distance(
    cruise_alt_m: float,
    dest_alt_m: float,
    gamma_deg: float = 3.0,
    decel_allowance_m: float = 18_520.0,
) -> float:
    """Calculate the along-track distance (meters) from destination to Top of Descent (TOD).

    Standard 3.0-degree descent gradient (~318 ft/NM) plus deceleration allowance.
    """
    approach_alt_m = dest_alt_m + (3000.0 * 0.3048)
    altitude_loss_m = max(0.0, cruise_alt_m - approach_alt_m)
    gamma_rad = math.radians(gamma_deg)

    descent_distance_m = altitude_loss_m / math.tan(gamma_rad)
    return descent_distance_m + decel_allowance_m


@dataclass(slots=True)
class AircraftTruthState:
    """Continuous 4D kinematic simulation truth state vector."""

    timestamp_s: float
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    alt_hae_m: float
    alt_baro_ft: float
    groundspeed_mps: float
    tas_mps: float
    ias_mps: float
    mach: float
    true_track_deg: float
    true_heading_deg: float
    vertical_rate_baro_mps: float
    on_ground: bool = False
    vertical_rate_geom_mps: float = 0.0
    roll_angle_deg: float = 0.0
    track_angle_rate_dps: float = 0.0
    alt_mcp_selected_ft: float = 0.0
    magnetic_heading_deg: float = 0.0
    icao_hex: str = ""
    callsign: str = ""
    squawk: str = "1200"
    category: str = "A3"
    phase: FlightPhase = FlightPhase.CRUISE

    @property
    def gs_mps(self) -> float:
        """Alias for groundspeed in m/s."""
        return self.groundspeed_mps

    @property
    def groundspeed_kt(self) -> float:
        """Groundspeed in knots."""
        return self.groundspeed_mps / KNOTS_TO_MPS

    @property
    def gs_kt(self) -> float:
        """Alias for groundspeed in knots."""
        return self.groundspeed_kt

    @property
    def track_deg(self) -> float:
        """Alias for true ground track in degrees."""
        return self.true_track_deg

    @property
    def heading_deg(self) -> float:
        """Alias for true heading in degrees."""
        return self.true_heading_deg

    @property
    def vr_mps(self) -> float:
        """Alias for barometric vertical rate in m/s."""
        return self.vertical_rate_baro_mps

    @property
    def vertical_rate_fpm(self) -> float:
        """Vertical rate in feet per minute."""
        return self.vertical_rate_baro_mps / 0.00508

    @property
    def vr_fpm(self) -> float:
        """Alias for vertical rate in feet per minute."""
        return self.vertical_rate_fpm

    @property
    def alt_ft(self) -> float:
        """Alias for barometric altitude in feet."""
        return self.alt_baro_ft


KinematicState = AircraftTruthState


def step_kinematics(
    current_state: AircraftTruthState,
    target_alt_m: float,
    target_tas_mps: float,
    target_track_deg: float,
    dt_s: float,
    profile: AircraftPerformanceProfile,
    atmosphere: AtmosphereModel | None = None,
    wind_vector_mps: tuple[float, float] = (0.0, 0.0),
    phase: FlightPhase = FlightPhase.CRUISE,
    on_ground: bool = False,
) -> AircraftTruthState:
    """Perform one discrete kinematic simulation tick Delta t."""
    if atmosphere is None:
        atmosphere = AtmosphereModel()

    # Longitudinal acceleration limit
    a_long_max = profile.enroute_accel_mps2
    a_vert_max = 1.0  # m/s^2 comfort vertical acceleration limit

    # Speed update
    v_diff = target_tas_mps - current_state.tas_mps
    delta_v = max(-a_long_max * dt_s, min(a_long_max * dt_s, v_diff))
    new_tas_mps = max(0.0, current_state.tas_mps + delta_v)

    # Vertical Rate & Altitude update
    if on_ground:
        new_alt_msl_m = current_state.alt_msl_m
        new_vr_mps = 0.0
    else:
        alt_diff = target_alt_m - current_state.alt_msl_m
        if abs(alt_diff) < 2.0:
            target_vr = 0.0
        elif alt_diff > 0:
            target_vr = min(profile.initial_climb_fpm * 0.00508, alt_diff / 5.0)
        else:
            target_vr = max(profile.normal_descent_fpm * 0.00508, alt_diff / 5.0)

        vr_diff = target_vr - current_state.vertical_rate_geom_mps
        delta_vr = max(-a_vert_max * dt_s, min(a_vert_max * dt_s, vr_diff))
        new_vr_mps = current_state.vertical_rate_geom_mps + delta_vr
        new_alt_msl_m = current_state.alt_msl_m + new_vr_mps * dt_s

    # Bank Angle & Turn Rate update
    track_diff = (target_track_deg - current_state.true_track_deg + 180.0) % 360.0 - 180.0
    desired_turn_rate_dps = max(-3.0, min(3.0, track_diff / 3.0))
    desired_bank_rad = math.atan((new_tas_mps * math.radians(desired_turn_rate_dps)) / GRAVITY_MPS2)
    max_bank_rad = math.radians(profile.max_bank_angle_deg)
    cmd_bank_deg = math.degrees(max(-max_bank_rad, min(max_bank_rad, desired_bank_rad)))

    # Roll-in rate limit
    max_roll_step = profile.max_roll_rate_dps * dt_s
    bank_diff = cmd_bank_deg - current_state.roll_angle_deg
    new_bank_deg = current_state.roll_angle_deg + max(-max_roll_step, min(max_roll_step, bank_diff))

    actual_turn_rate_radps = (GRAVITY_MPS2 * math.tan(math.radians(new_bank_deg))) / max(
        1.0, new_tas_mps
    )
    actual_turn_rate_dps = math.degrees(actual_turn_rate_radps)
    new_track_deg = (current_state.true_track_deg + actual_turn_rate_dps * dt_s) % 360.0

    # 3D Wind Triangle & Groundspeed
    w_east, w_north = wind_vector_mps
    track_rad = math.radians(new_track_deg)

    if new_tas_mps > 1e-3:
        cross_wind = w_east * math.cos(track_rad) - w_north * math.sin(track_rad)
        sin_wca = max(-1.0, min(1.0, cross_wind / new_tas_mps))
        wca_rad = math.asin(sin_wca)
        true_heading_rad = (track_rad - wca_rad) % (2.0 * math.pi)
        new_heading_deg = math.degrees(true_heading_rad)
        new_gs_mps = (
            new_tas_mps * math.cos(wca_rad)
            + w_east * math.sin(track_rad)
            + w_north * math.cos(track_rad)
        )
    else:
        new_heading_deg = new_track_deg
        new_gs_mps = math.hypot(w_east, w_north)

    new_gs_mps = max(0.0, new_gs_mps)

    # Position propagation (Spherical Destination Point)
    dist_step_m = new_gs_mps * dt_s
    new_lat_deg, new_lon_deg = destination_point(
        current_state.lat_deg, current_state.lon_deg, dist_step_m, new_track_deg
    )

    # Atmosphere & Altimeter
    atmo = atmosphere.evaluate(new_alt_msl_m)
    ias_mps = new_tas_mps * math.sqrt(atmo.density_kgpm3 / 1.225)
    mach = new_tas_mps / atmo.speed_of_sound_mps
    alt_baro_ft = atmo.pressure_altitude_ft

    return AircraftTruthState(
        timestamp_s=current_state.timestamp_s + dt_s,
        icao_hex=current_state.icao_hex,
        callsign=current_state.callsign,
        squawk=current_state.squawk,
        category=current_state.category,
        phase=phase,
        on_ground=on_ground,
        lat_deg=new_lat_deg,
        lon_deg=new_lon_deg,
        alt_msl_m=new_alt_msl_m,
        alt_hae_m=new_alt_msl_m,
        alt_baro_ft=alt_baro_ft,
        alt_mcp_selected_ft=target_alt_m * METERS_TO_FEET,
        groundspeed_mps=new_gs_mps,
        tas_mps=new_tas_mps,
        ias_mps=ias_mps,
        mach=mach,
        true_track_deg=new_track_deg,
        magnetic_heading_deg=new_heading_deg,
        true_heading_deg=new_heading_deg,
        vertical_rate_baro_mps=new_vr_mps,
        vertical_rate_geom_mps=new_vr_mps,
        roll_angle_deg=new_bank_deg,
        track_angle_rate_dps=actual_turn_rate_dps,
    )


# --- Closed-Loop Factory Generators ---


def generate_simple_climb_trajectory(
    profile: AircraftPerformanceProfile,
    origin_lat: float,
    origin_lon: float,
    target_alt_ft: float,
    duration_s: float,
    dt_s: float = 1.0,
    rng: np.random.Generator | None = None,
    callsign: str = "KLM1023",
    icao_hex: str = "4840D6",
) -> list[AircraftTruthState]:
    """Generate a clean climbing flight trajectory for test_e2e_single_aircraft_clean_closed_loop."""
    num_steps = math.ceil(duration_s / dt_s) + 1
    states: list[AircraftTruthState] = []

    # Initial kinematic parameters
    initial_alt_m = 0.0
    initial_tas_mps = profile.v_initial_climb_ias_kt * KNOTS_TO_MPS
    climb_rate_mps = profile.initial_climb_fpm * 0.00508  # ~15.24 m/s for 3000 fpm
    target_alt_m = target_alt_ft * 0.3048
    heading_deg = 60.0  # Departure heading (degrees true)

    cur_lat = origin_lat
    cur_lon = origin_lon
    cur_alt_m = initial_alt_m
    cur_tas_mps = initial_tas_mps

    for step in range(num_steps):
        t = step * dt_s
        if t > duration_s:
            break

        # Linear climb up to target altitude
        if cur_alt_m < target_alt_m:
            cur_alt_m = min(target_alt_m, initial_alt_m + climb_rate_mps * t)
            cur_vr_mps = climb_rate_mps
        else:
            cur_vr_mps = 0.0

        # Atmosphere at current altitude
        atmo = compute_atmosphere(cur_alt_m)
        alt_baro_ft = atmo.pressure_altitude_ft
        mach = cur_tas_mps / atmo.speed_of_sound_mps
        ias_mps = cur_tas_mps * math.sqrt(atmo.density_kgpm3 / 1.225)
        groundspeed_mps = cur_tas_mps

        state = AircraftTruthState(
            timestamp_s=float(t),
            lat_deg=cur_lat,
            lon_deg=cur_lon,
            alt_msl_m=cur_alt_m,
            alt_hae_m=cur_alt_m,
            alt_baro_ft=alt_baro_ft,
            groundspeed_mps=groundspeed_mps,
            tas_mps=cur_tas_mps,
            ias_mps=ias_mps,
            mach=mach,
            true_track_deg=heading_deg,
            true_heading_deg=heading_deg,
            magnetic_heading_deg=heading_deg,
            vertical_rate_baro_mps=cur_vr_mps,
            vertical_rate_geom_mps=cur_vr_mps,
            on_ground=False,
            icao_hex=icao_hex,
            callsign=callsign,
            category=str(profile.category),
            phase=FlightPhase.INITIAL_CLIMB,
        )
        states.append(state)

        # Propagate position to next step
        cur_lat, cur_lon = destination_point(
            cur_lat, cur_lon, groundspeed_mps * dt_s, heading_deg
        )

    return states


def generate_straight_flight(
    start_lat: float,
    start_lon: float,
    heading_deg: float,
    speed_kt: float,
    alt_ft: float,
    duration_s: float,
    dt_s: float = 1.0,
    rng: np.random.Generator | None = None,
    callsign: str = "TEST01",
    icao_hex: str = "A00001",
) -> list[AircraftTruthState]:
    """Generate straight constant-velocity flight across geodetic boundaries."""
    num_steps = math.ceil(duration_s / dt_s) + 1
    states: list[AircraftTruthState] = []

    speed_mps = speed_kt * KNOTS_TO_MPS
    alt_m = alt_ft * 0.3048
    atmo = compute_atmosphere(alt_m)
    alt_baro_ft = atmo.pressure_altitude_ft
    mach = speed_mps / atmo.speed_of_sound_mps
    ias_mps = speed_mps * math.sqrt(atmo.density_kgpm3 / 1.225)

    cur_lat = start_lat
    cur_lon = start_lon

    for step in range(num_steps):
        t = step * dt_s
        if t > duration_s:
            break

        state = AircraftTruthState(
            timestamp_s=float(t),
            lat_deg=cur_lat,
            lon_deg=cur_lon,
            alt_msl_m=alt_m,
            alt_hae_m=alt_m,
            alt_baro_ft=alt_baro_ft,
            groundspeed_mps=speed_mps,
            tas_mps=speed_mps,
            ias_mps=ias_mps,
            mach=mach,
            true_track_deg=heading_deg,
            true_heading_deg=heading_deg,
            magnetic_heading_deg=heading_deg,
            vertical_rate_baro_mps=0.0,
            vertical_rate_geom_mps=0.0,
            on_ground=False,
            icao_hex=icao_hex,
            callsign=callsign,
            category="A3",
            phase=FlightPhase.CRUISE,
        )
        states.append(state)

        # Advance position along heading
        cur_lat, cur_lon = destination_point(cur_lat, cur_lon, speed_mps * dt_s, heading_deg)

    return states


def generate_maneuvering_flight(
    start_lat: float,
    start_lon: float,
    speed_kt: float,
    alt_ft: float,
    straight_time_1_s: float,
    bank_angle_deg: float,
    turn_duration_s: float,
    straight_time_2_s: float,
    dt_s: float = 1.0,
    rng: np.random.Generator | None = None,
    callsign: str = "MANEUVR",
    icao_hex: str = "C00003",
) -> list[AircraftTruthState]:
    """Generate 3-stage maneuvering flight (Straight -> Coordinated Turn -> Straight)."""
    total_duration_s = straight_time_1_s + turn_duration_s + straight_time_2_s
    num_steps = math.ceil(total_duration_s / dt_s) + 1
    states: list[AircraftTruthState] = []

    speed_mps = speed_kt * KNOTS_TO_MPS
    alt_m = alt_ft * 0.3048
    atmo = compute_atmosphere(alt_m)
    alt_baro_ft = atmo.pressure_altitude_ft
    mach = speed_mps / atmo.speed_of_sound_mps
    ias_mps = speed_mps * math.sqrt(atmo.density_kgpm3 / 1.225)

    # Coordinated turn rate: omega = (g * tan(bank)) / V
    turn_rate_radps = (GRAVITY_MPS2 * math.tan(math.radians(bank_angle_deg))) / max(1.0, speed_mps)
    turn_rate_dps = math.degrees(turn_rate_radps)

    cur_lat = start_lat
    cur_lon = start_lon
    cur_track_deg = 90.0  # Initial heading due East

    for step in range(num_steps):
        t = step * dt_s
        if t > total_duration_s:
            break

        if t < straight_time_1_s:
            # Stage 1: Straight cruise
            cur_bank = 0.0
            cur_turn_rate = 0.0
        elif t <= straight_time_1_s + turn_duration_s:
            # Stage 2: Coordinated turn
            cur_bank = bank_angle_deg
            cur_turn_rate = turn_rate_dps
        else:
            # Stage 3: Straight cruise
            cur_bank = 0.0
            cur_turn_rate = 0.0

        state = AircraftTruthState(
            timestamp_s=float(t),
            lat_deg=cur_lat,
            lon_deg=cur_lon,
            alt_msl_m=alt_m,
            alt_hae_m=alt_m,
            alt_baro_ft=alt_baro_ft,
            groundspeed_mps=speed_mps,
            tas_mps=speed_mps,
            ias_mps=ias_mps,
            mach=mach,
            true_track_deg=cur_track_deg,
            true_heading_deg=cur_track_deg,
            magnetic_heading_deg=cur_track_deg,
            vertical_rate_baro_mps=0.0,
            vertical_rate_geom_mps=0.0,
            roll_angle_deg=cur_bank,
            track_angle_rate_dps=cur_turn_rate,
            on_ground=False,
            icao_hex=icao_hex,
            callsign=callsign,
            category="A3",
            phase=FlightPhase.CRUISE,
        )
        states.append(state)

        # Update position and track for next step
        cur_lat, cur_lon = destination_point(cur_lat, cur_lon, speed_mps * dt_s, cur_track_deg)
        if cur_turn_rate != 0.0:
            cur_track_deg = (cur_track_deg + cur_turn_rate * dt_s) % 360.0

    return states


def generate_terminal_routed_trajectory(
    origin_icao: str,
    dest_icao: str,
    aircraft_type: str,
    callsign: str = "AUTO01",
    cruise_alt_ft: float | None = None,
    squawk: str = "1200",
    icao_hex: str = "",
    departure_time_s: float = 0.0,
    dt_s: float = 1.0,
    rng: np.random.Generator | None = None,
) -> list[AircraftTruthState]:
    """Generate a complete 4D flight trajectory adhering to R3 realistic terminal kinematics.

    Terminal Constraints:
    - Departure: Start at departure runway midpoint, maintain runway heading (delta heading < 0.1 deg)
      for 5.0 NM after takeoff before turning.
    - Arrival: Intercept the extended ILS final approach course 17.0 NM out at 3,000 ft altitude
      aligned with runway heading (delta track < 0.1 deg), decelerate level to ~9.43 NM glide slope
      intercept, and follow a 3.0 deg descent gradient down to touchdown at the arrival runway midpoint.
    - Cruise altitude is automatically capped for short-haul flights to prevent VNAV overshoot.
    """
    from adsb.synthesis.airports import get_airport
    from adsb.synthesis.performance import get_aircraft_profile
    from adsb.synthesis.terminal import (
        compute_capped_cruise_altitude,
        compute_departure_fix,
        compute_glide_slope_intercept_distance_m,
        compute_ils_intercept_fix,
    )

    origin = get_airport(origin_icao)
    dest = get_airport(dest_icao)
    dep_rwy = origin.primary_runway
    arr_rwy = dest.primary_runway
    profile = get_aircraft_profile(aircraft_type)

    if not icao_hex:
        if rng is not None:
            icao_hex = f"{rng.integers(0x100000, 0xEFFFFF):06X}"
        else:
            seed_int = hash((callsign, origin_icao, dest_icao)) & 0xFFFFFF
            icao_hex = f"{seed_int:06X}"

    # Geodetic reference fixes
    p_dep_mid = (dep_rwy.midpoint_lat_deg, dep_rwy.midpoint_lon_deg)
    p_dep_5nm = compute_departure_fix(
        dep_rwy.midpoint_lat_deg,
        dep_rwy.midpoint_lon_deg,
        dep_rwy.true_heading_deg,
        distance_nm=5.0,
    )
    p_arr_17nm = compute_ils_intercept_fix(
        arr_rwy.midpoint_lat_deg,
        arr_rwy.midpoint_lon_deg,
        arr_rwy.true_heading_deg,
        distance_nm=17.0,
    )
    p_arr_mid = (arr_rwy.midpoint_lat_deg, arr_rwy.midpoint_lon_deg)
    recip_arr = (arr_rwy.true_heading_deg + 180.0) % 360.0

    total_route_nm = (
        haversine_distance(p_dep_mid[0], p_dep_mid[1], p_arr_mid[0], p_arr_mid[1]) / 1852.0
    )
    req_alt = cruise_alt_ft if cruise_alt_ft is not None else profile.default_cruise_alt_ft
    effective_cruise_alt_ft = compute_capped_cruise_altitude(
        route_distance_nm=total_route_nm,
        requested_alt_ft=req_alt,
        origin_elev_ft=origin.elevation_ft,
        dest_elev_ft=dest.elevation_ft,
        service_ceiling_ft=profile.service_ceiling_ft,
    )
    effective_cruise_alt_m = effective_cruise_alt_ft * 0.3048

    # Speeds and vertical rates
    v_rot_mps = profile.v_rotate_ias_kt * KNOTS_TO_MPS
    v_init_mps = profile.v_initial_climb_ias_kt * KNOTS_TO_MPS
    v_cruise_mps = profile.v_cruise_tas_kt * KNOTS_TO_MPS
    v_int_app_mps = 180.0 * KNOTS_TO_MPS
    v_app_mps = profile.v_approach_ias_kt * KNOTS_TO_MPS
    v_taxi_mps = profile.v_taxi_kt * KNOTS_TO_MPS

    rocd_init_mps = profile.initial_climb_fpm * 0.00508
    rocd_climb_mps = profile.mid_climb_fpm * 0.00508

    states: list[AircraftTruthState] = []
    t = departure_time_s

    # --- PHASE 1: Takeoff Ground Roll ---
    v_cur = 0.0
    d_dep_m = 0.0
    cur_lat, cur_lon = p_dep_mid
    cur_alt_m = origin.elevation_m

    while v_cur < v_rot_mps:
        atmo = compute_atmosphere(cur_alt_m)
        states.append(
            AircraftTruthState(
                timestamp_s=t,
                lat_deg=cur_lat,
                lon_deg=cur_lon,
                alt_msl_m=cur_alt_m,
                alt_hae_m=cur_alt_m,
                alt_baro_ft=atmo.pressure_altitude_ft,
                alt_mcp_selected_ft=effective_cruise_alt_ft,
                groundspeed_mps=v_cur,
                tas_mps=v_cur,
                ias_mps=v_cur,
                mach=v_cur / atmo.speed_of_sound_mps if atmo.speed_of_sound_mps > 0 else 0.0,
                true_track_deg=dep_rwy.true_heading_deg,
                true_heading_deg=dep_rwy.true_heading_deg,
                magnetic_heading_deg=dep_rwy.true_heading_deg,
                vertical_rate_baro_mps=0.0,
                vertical_rate_geom_mps=0.0,
                on_ground=True,
                roll_angle_deg=0.0,
                track_angle_rate_dps=0.0,
                icao_hex=icao_hex,
                callsign=callsign,
                squawk=squawk,
                category=str(profile.category),
                phase=FlightPhase.TAKEOFF_ROLL,
            )
        )
        step_m = v_cur * dt_s + 0.5 * profile.takeoff_accel_mps2 * (dt_s**2)
        v_cur += profile.takeoff_accel_mps2 * dt_s
        d_dep_m += step_m
        cur_lat, cur_lon = destination_point(
            p_dep_mid[0], p_dep_mid[1], d_dep_m, dep_rwy.true_heading_deg
        )
        t += dt_s

    # --- PHASE 2: Initial Climb (Strict Runway Heading for 5.0 NM) ---
    while d_dep_m < 5.0 * 1852.0:
        atmo = compute_atmosphere(cur_alt_m)
        states.append(
            AircraftTruthState(
                timestamp_s=t,
                lat_deg=cur_lat,
                lon_deg=cur_lon,
                alt_msl_m=cur_alt_m,
                alt_hae_m=cur_alt_m,
                alt_baro_ft=atmo.pressure_altitude_ft,
                alt_mcp_selected_ft=effective_cruise_alt_ft,
                groundspeed_mps=v_cur,
                tas_mps=v_cur,
                ias_mps=v_cur,
                mach=v_cur / atmo.speed_of_sound_mps,
                true_track_deg=dep_rwy.true_heading_deg,
                true_heading_deg=dep_rwy.true_heading_deg,
                magnetic_heading_deg=dep_rwy.true_heading_deg,
                vertical_rate_baro_mps=rocd_init_mps,
                vertical_rate_geom_mps=rocd_init_mps,
                on_ground=False,
                roll_angle_deg=0.0,
                track_angle_rate_dps=0.0,
                icao_hex=icao_hex,
                callsign=callsign,
                squawk=squawk,
                category=str(profile.category),
                phase=FlightPhase.INITIAL_CLIMB,
            )
        )
        if v_cur < v_init_mps:
            v_cur = min(v_init_mps, v_cur + profile.enroute_accel_mps2 * dt_s)
        step_m = v_cur * dt_s
        d_dep_m += step_m
        cur_lat, cur_lon = destination_point(
            p_dep_mid[0], p_dep_mid[1], min(5.0 * 1852.0, d_dep_m), dep_rwy.true_heading_deg
        )
        cur_alt_m += rocd_init_mps * dt_s
        t += dt_s

    # --- PHASE 3: En-Route (Climb, Cruise, Descent to 17 NM Intercept) ---
    d_enroute_total_m = haversine_distance(
        p_dep_5nm[0], p_dep_5nm[1], p_arr_17nm[0], p_arr_17nm[1]
    )
    brg_enroute = initial_bearing(p_dep_5nm[0], p_dep_5nm[1], p_arr_17nm[0], p_arr_17nm[1])

    target_int_alt_m = 3000.0 * 0.3048
    delta_h_desc_m = max(0.0, effective_cruise_alt_m - target_int_alt_m)
    d_desc_calc_m = (delta_h_desc_m / math.tan(math.radians(3.0))) + 8.0 * 1852.0
    d_desc_m = min(d_enroute_total_m * 0.65, d_desc_calc_m)

    s_m = 0.0
    cur_track = dep_rwy.true_heading_deg

    while s_m < d_enroute_total_m:
        d_to_arr17_m = d_enroute_total_m - s_m

        # Guidance: Turn toward arrival runway heading when within 5 NM of 17 NM fix
        tgt_track = arr_rwy.true_heading_deg if d_to_arr17_m < 5.0 * 1852.0 else brg_enroute

        track_diff = (tgt_track - cur_track + 180.0) % 360.0 - 180.0
        turn_step = max(-3.0 * dt_s, min(3.0 * dt_s, track_diff))
        cur_track = (cur_track + turn_step) % 360.0

        if d_to_arr17_m <= d_desc_m:
            phase = FlightPhase.DESCENT
            fraction = max(0.0, min(1.0, (d_desc_m - d_to_arr17_m) / d_desc_m))
            target_alt = effective_cruise_alt_m - fraction * delta_h_desc_m
            cur_alt_m = max(target_int_alt_m, target_alt)
            if v_cur > v_int_app_mps:
                v_cur = max(v_int_app_mps, v_cur - profile.enroute_accel_mps2 * dt_s)
            vr_mps = (
                -(delta_h_desc_m / (d_desc_m / max(1.0, v_cur))) if d_desc_m > 0 else 0.0
            )
        else:
            if cur_alt_m < effective_cruise_alt_m:
                phase = FlightPhase.ENROUTE_CLIMB
                cur_alt_m = min(effective_cruise_alt_m, cur_alt_m + rocd_climb_mps * dt_s)
                if v_cur < v_cruise_mps:
                    v_cur = min(v_cruise_mps, v_cur + profile.enroute_accel_mps2 * dt_s)
                vr_mps = rocd_climb_mps
            else:
                phase = FlightPhase.CRUISE
                cur_alt_m = effective_cruise_alt_m
                v_cur = v_cruise_mps
                vr_mps = 0.0

        cur_lat, cur_lon = destination_point(p_dep_5nm[0], p_dep_5nm[1], s_m, brg_enroute)
        atmo = compute_atmosphere(cur_alt_m)
        states.append(
            AircraftTruthState(
                timestamp_s=t,
                lat_deg=cur_lat,
                lon_deg=cur_lon,
                alt_msl_m=cur_alt_m,
                alt_hae_m=cur_alt_m,
                alt_baro_ft=atmo.pressure_altitude_ft,
                alt_mcp_selected_ft=(
                    effective_cruise_alt_ft if phase != FlightPhase.DESCENT else 3000.0
                ),
                groundspeed_mps=v_cur,
                tas_mps=v_cur,
                ias_mps=v_cur,
                mach=v_cur / atmo.speed_of_sound_mps,
                true_track_deg=cur_track,
                true_heading_deg=cur_track,
                magnetic_heading_deg=cur_track,
                vertical_rate_baro_mps=vr_mps,
                vertical_rate_geom_mps=vr_mps,
                on_ground=False,
                roll_angle_deg=(
                    0.0 if abs(turn_step) < 1e-3 else (15.0 if turn_step > 0 else -15.0)
                ),
                track_angle_rate_dps=turn_step / dt_s,
                icao_hex=icao_hex,
                callsign=callsign,
                squawk=squawk,
                category=str(profile.category),
                phase=phase,
            )
        )

        step_m = v_cur * dt_s
        s_m += step_m
        t += dt_s

    # --- PHASE 4: Final Approach (17 NM out to Touchdown) ---
    d_gs_int_m = compute_glide_slope_intercept_distance_m(
        runway_elevation_ft=dest.elevation_ft,
        intercept_alt_ft=3000.0,
        glide_slope_deg=3.0,
    )
    d_arr_to_mid_m = 17.0 * 1852.0
    v_cur = v_int_app_mps

    while d_arr_to_mid_m > 0:
        lat, lon = destination_point(
            arr_rwy.midpoint_lat_deg, arr_rwy.midpoint_lon_deg, d_arr_to_mid_m, recip_arr
        )
        if d_arr_to_mid_m >= d_gs_int_m:
            alt_ft = 3000.0
            vr_mps = 0.0
            if v_cur > v_app_mps:
                v_cur = max(v_app_mps, v_cur - 0.5 * dt_s)
        else:
            alt_ft = dest.elevation_ft + (d_arr_to_mid_m * math.tan(math.radians(3.0))) / 0.3048
            vr_mps = -v_cur * math.tan(math.radians(3.0))
            v_cur = v_app_mps

        alt_m = alt_ft * 0.3048
        atmo = compute_atmosphere(alt_m)
        states.append(
            AircraftTruthState(
                timestamp_s=t,
                lat_deg=lat,
                lon_deg=lon,
                alt_msl_m=alt_m,
                alt_hae_m=alt_m,
                alt_baro_ft=alt_ft,
                alt_mcp_selected_ft=dest.elevation_ft,
                groundspeed_mps=v_cur,
                tas_mps=v_cur,
                ias_mps=v_cur,
                mach=v_cur / atmo.speed_of_sound_mps,
                true_track_deg=arr_rwy.true_heading_deg,
                true_heading_deg=arr_rwy.true_heading_deg,
                magnetic_heading_deg=arr_rwy.true_heading_deg,
                vertical_rate_baro_mps=vr_mps,
                vertical_rate_geom_mps=vr_mps,
                on_ground=False,
                roll_angle_deg=0.0,
                track_angle_rate_dps=0.0,
                icao_hex=icao_hex,
                callsign=callsign,
                squawk=squawk,
                category=str(profile.category),
                phase=FlightPhase.APPROACH,
            )
        )
        step_m = v_cur * dt_s
        d_arr_to_mid_m = max(0.0, d_arr_to_mid_m - step_m)
        t += dt_s

    # --- PHASE 5: Touchdown & Landing Rollout ---
    v_cur = v_app_mps
    d_roll_m = 0.0
    while v_cur > v_taxi_mps:
        lat, lon = destination_point(
            arr_rwy.midpoint_lat_deg, arr_rwy.midpoint_lon_deg, d_roll_m, arr_rwy.true_heading_deg
        )
        atmo = compute_atmosphere(dest.elevation_m)
        states.append(
            AircraftTruthState(
                timestamp_s=t,
                lat_deg=lat,
                lon_deg=lon,
                alt_msl_m=dest.elevation_m,
                alt_hae_m=dest.elevation_m,
                alt_baro_ft=dest.elevation_ft,
                alt_mcp_selected_ft=dest.elevation_ft,
                groundspeed_mps=v_cur,
                tas_mps=v_cur,
                ias_mps=v_cur,
                mach=v_cur / atmo.speed_of_sound_mps,
                true_track_deg=arr_rwy.true_heading_deg,
                true_heading_deg=arr_rwy.true_heading_deg,
                magnetic_heading_deg=arr_rwy.true_heading_deg,
                vertical_rate_baro_mps=0.0,
                vertical_rate_geom_mps=0.0,
                on_ground=True,
                roll_angle_deg=0.0,
                track_angle_rate_dps=0.0,
                icao_hex=icao_hex,
                callsign=callsign,
                squawk=squawk,
                category=str(profile.category),
                phase=FlightPhase.LANDING_ROLLOUT,
            )
        )
        v_cur = max(v_taxi_mps, v_cur + profile.landing_decel_mps2 * dt_s)
        step_m = v_cur * dt_s
        d_roll_m += step_m
        t += dt_s

    return states


__all__ = [
    "AircraftTruthState",
    "FlightPhase",
    "KinematicState",
    "compute_top_of_descent_distance",
    "generate_maneuvering_flight",
    "generate_simple_climb_trajectory",
    "generate_straight_flight",
    "generate_terminal_routed_trajectory",
    "step_kinematics",
]
