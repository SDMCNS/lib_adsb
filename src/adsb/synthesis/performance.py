"""Parametric aircraft performance profiles for flight trajectory synthesis.

Authoritative reference: RTCA DO-260B / DO-260C, ICAO Annex 10,
and docs/03_FLIGHT_SYNTHESIS_AND_PHYSICS.md Section 1.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class WakeVortexCategory(StrEnum):
    """Wake vortex / emitter category per RTCA DO-260B and ICAO Annex 10."""

    NO_INFO = "A0"
    LIGHT = "A1"  # < 15,500 lbs (7,030 kg) (e.g. C172, SR22)
    SMALL = "A2"  # 15,500 to 75,000 lbs (e.g. B350, DH8D)
    LARGE = "A3"  # 75,000 to 300,000 lbs (e.g. B738, A320, E75L)
    HIGH_VORTEX_LARGE = "A4"  # e.g. B757
    HEAVY = "A5"  # > 300,000 lbs (e.g. B77W, CONC)
    HIGH_PERFORMANCE = "A6"  # > 5g and > 400 kt (e.g. F16)
    ROTORCRAFT = "A7"  # Helicopter (e.g. R44, EC35)
    GLIDER = "B1"  # Sailplane (e.g. GLID)
    LIGHTER_THAN_AIR = "B2"
    UAV = "B6"
    SURFACE_EMERGENCY_VEHICLE = "C1"
    SURFACE_SERVICE_VEHICLE = "C2"
    SURFACE_OBSTACLE = "C3"


@dataclass(frozen=True, slots=True)
class AircraftPerformanceProfile:
    """Kinematic performance envelope for flight trajectory synthesis (22 core parameters)."""

    type_code: str
    category: WakeVortexCategory

    # Speed Envelopes (Indicated Airspeed in knots & Mach)
    v_taxi_kt: float
    v_rotate_ias_kt: float
    v_initial_climb_ias_kt: float
    v_climb_ias_kt: float
    mach_climb: float
    mach_cruise: float
    v_cruise_tas_kt: float
    mach_descent: float
    v_descent_ias_kt: float
    v_approach_ias_kt: float

    # Altitude & Vertical Rate Envelopes
    service_ceiling_ft: float
    initial_climb_fpm: float
    mid_climb_fpm: float
    high_climb_fpm: float
    normal_descent_fpm: float
    approach_descent_fpm: float

    # Longitudinal & Lateral Accelerations / Turn Limits
    takeoff_accel_mps2: float
    landing_decel_mps2: float
    enroute_accel_mps2: float
    max_bank_angle_deg: float
    max_roll_rate_dps: float = 5.0

    # Metadata
    description: str = ""
    default_cruise_alt_ft: float = 35000.0


# Authoritative parameters for reference aircraft types
_BUILTIN_PROFILES: dict[str, AircraftPerformanceProfile] = {
    "B738": AircraftPerformanceProfile(
        type_code="B738",
        category=WakeVortexCategory.LARGE,
        v_taxi_kt=15.0,
        v_rotate_ias_kt=145.0,
        v_initial_climb_ias_kt=250.0,
        v_climb_ias_kt=290.0,
        mach_climb=0.78,
        mach_cruise=0.79,
        v_cruise_tas_kt=455.0,
        mach_descent=0.78,
        v_descent_ias_kt=280.0,
        v_approach_ias_kt=140.0,
        service_ceiling_ft=41000.0,
        initial_climb_fpm=3000.0,
        mid_climb_fpm=2100.0,
        high_climb_fpm=1100.0,
        normal_descent_fpm=-2000.0,
        approach_descent_fpm=-750.0,
        takeoff_accel_mps2=2.4,
        landing_decel_mps2=-2.8,
        enroute_accel_mps2=0.8,
        max_bank_angle_deg=25.0,
        max_roll_rate_dps=5.0,
        description="Boeing 737-800",
        default_cruise_alt_ft=36000.0,
    ),
    "A320": AircraftPerformanceProfile(
        type_code="A320",
        category=WakeVortexCategory.LARGE,
        v_taxi_kt=15.0,
        v_rotate_ias_kt=140.0,
        v_initial_climb_ias_kt=250.0,
        v_climb_ias_kt=290.0,
        mach_climb=0.78,
        mach_cruise=0.78,
        v_cruise_tas_kt=450.0,
        mach_descent=0.78,
        v_descent_ias_kt=280.0,
        v_approach_ias_kt=135.0,
        service_ceiling_ft=39800.0,
        initial_climb_fpm=2900.0,
        mid_climb_fpm=2000.0,
        high_climb_fpm=1000.0,
        normal_descent_fpm=-2000.0,
        approach_descent_fpm=-720.0,
        takeoff_accel_mps2=2.3,
        landing_decel_mps2=-2.7,
        enroute_accel_mps2=0.8,
        max_bank_angle_deg=25.0,
        max_roll_rate_dps=5.0,
        description="Airbus A320-200",
        default_cruise_alt_ft=36000.0,
    ),
    "B77W": AircraftPerformanceProfile(
        type_code="B77W",
        category=WakeVortexCategory.HEAVY,
        v_taxi_kt=15.0,
        v_rotate_ias_kt=160.0,
        v_initial_climb_ias_kt=250.0,
        v_climb_ias_kt=310.0,
        mach_climb=0.82,
        mach_cruise=0.84,
        v_cruise_tas_kt=485.0,
        mach_descent=0.82,
        v_descent_ias_kt=300.0,
        v_approach_ias_kt=148.0,
        service_ceiling_ft=43100.0,
        initial_climb_fpm=2800.0,
        mid_climb_fpm=2000.0,
        high_climb_fpm=1000.0,
        normal_descent_fpm=-2200.0,
        approach_descent_fpm=-780.0,
        takeoff_accel_mps2=2.2,
        landing_decel_mps2=-2.6,
        enroute_accel_mps2=0.7,
        max_bank_angle_deg=25.0,
        max_roll_rate_dps=4.0,
        description="Boeing 777-300ER",
        default_cruise_alt_ft=38000.0,
    ),
    "C172": AircraftPerformanceProfile(
        type_code="C172",
        category=WakeVortexCategory.LIGHT,
        v_taxi_kt=10.0,
        v_rotate_ias_kt=55.0,
        v_initial_climb_ias_kt=74.0,
        v_climb_ias_kt=80.0,
        mach_climb=0.15,
        mach_cruise=0.20,
        v_cruise_tas_kt=122.0,
        mach_descent=0.18,
        v_descent_ias_kt=110.0,
        v_approach_ias_kt=65.0,
        service_ceiling_ft=14000.0,
        initial_climb_fpm=730.0,
        mid_climb_fpm=450.0,
        high_climb_fpm=200.0,
        normal_descent_fpm=-700.0,
        approach_descent_fpm=-450.0,
        takeoff_accel_mps2=1.5,
        landing_decel_mps2=-1.8,
        enroute_accel_mps2=0.5,
        max_bank_angle_deg=20.0,
        max_roll_rate_dps=10.0,
        description="Cessna 172 Skyhawk",
        default_cruise_alt_ft=7000.0,
    ),
    "B350": AircraftPerformanceProfile(
        type_code="B350",
        category=WakeVortexCategory.SMALL,
        v_taxi_kt=12.0,
        v_rotate_ias_kt=105.0,
        v_initial_climb_ias_kt=140.0,
        v_climb_ias_kt=170.0,
        mach_climb=0.45,
        mach_cruise=0.52,
        v_cruise_tas_kt=310.0,
        mach_descent=0.45,
        v_descent_ias_kt=180.0,
        v_approach_ias_kt=115.0,
        service_ceiling_ft=35000.0,
        initial_climb_fpm=2450.0,
        mid_climb_fpm=1600.0,
        high_climb_fpm=800.0,
        normal_descent_fpm=-1500.0,
        approach_descent_fpm=-600.0,
        takeoff_accel_mps2=2.0,
        landing_decel_mps2=-2.2,
        enroute_accel_mps2=0.7,
        max_bank_angle_deg=25.0,
        max_roll_rate_dps=6.0,
        description="Beechcraft Super King Air 350",
        default_cruise_alt_ft=28000.0,
    ),
    "R44": AircraftPerformanceProfile(
        type_code="R44",
        category=WakeVortexCategory.ROTORCRAFT,
        v_taxi_kt=5.0,
        v_rotate_ias_kt=0.0,
        v_initial_climb_ias_kt=55.0,
        v_climb_ias_kt=65.0,
        mach_climb=0.12,
        mach_cruise=0.18,
        v_cruise_tas_kt=110.0,
        mach_descent=0.15,
        v_descent_ias_kt=70.0,
        v_approach_ias_kt=50.0,
        service_ceiling_ft=14000.0,
        initial_climb_fpm=1000.0,
        mid_climb_fpm=600.0,
        high_climb_fpm=250.0,
        normal_descent_fpm=-800.0,
        approach_descent_fpm=-400.0,
        takeoff_accel_mps2=1.2,
        landing_decel_mps2=-1.5,
        enroute_accel_mps2=0.5,
        max_bank_angle_deg=30.0,
        max_roll_rate_dps=15.0,
        description="Robinson R44",
        default_cruise_alt_ft=3000.0,
    ),
    "F16": AircraftPerformanceProfile(
        type_code="F16",
        category=WakeVortexCategory.HIGH_PERFORMANCE,
        v_taxi_kt=20.0,
        v_rotate_ias_kt=155.0,
        v_initial_climb_ias_kt=350.0,
        v_climb_ias_kt=450.0,
        mach_climb=0.95,
        mach_cruise=1.40,
        v_cruise_tas_kt=800.0,
        mach_descent=0.90,
        v_descent_ias_kt=350.0,
        v_approach_ias_kt=150.0,
        service_ceiling_ft=50000.0,
        initial_climb_fpm=15000.0,
        mid_climb_fpm=10000.0,
        high_climb_fpm=5000.0,
        normal_descent_fpm=-4000.0,
        approach_descent_fpm=-800.0,
        takeoff_accel_mps2=4.5,
        landing_decel_mps2=-3.5,
        enroute_accel_mps2=2.5,
        max_bank_angle_deg=60.0,
        max_roll_rate_dps=30.0,
        description="General Dynamics F-16 Fighting Falcon",
        default_cruise_alt_ft=40000.0,
    ),
    "CONC": AircraftPerformanceProfile(
        type_code="CONC",
        category=WakeVortexCategory.HEAVY,
        v_taxi_kt=20.0,
        v_rotate_ias_kt=195.0,
        v_initial_climb_ias_kt=250.0,
        v_climb_ias_kt=400.0,
        mach_climb=1.70,
        mach_cruise=2.02,
        v_cruise_tas_kt=1160.0,
        mach_descent=1.50,
        v_descent_ias_kt=350.0,
        v_approach_ias_kt=165.0,
        service_ceiling_ft=60000.0,
        initial_climb_fpm=4000.0,
        mid_climb_fpm=3000.0,
        high_climb_fpm=1500.0,
        normal_descent_fpm=-3000.0,
        approach_descent_fpm=-850.0,
        takeoff_accel_mps2=3.0,
        landing_decel_mps2=-3.2,
        enroute_accel_mps2=1.2,
        max_bank_angle_deg=30.0,
        max_roll_rate_dps=6.0,
        description="Aérospatiale/BAC Concorde",
        default_cruise_alt_ft=55000.0,
    ),
    "GLID": AircraftPerformanceProfile(
        type_code="GLID",
        category=WakeVortexCategory.GLIDER,
        v_taxi_kt=0.0,
        v_rotate_ias_kt=45.0,
        v_initial_climb_ias_kt=55.0,
        v_climb_ias_kt=45.0,
        mach_climb=0.08,
        mach_cruise=0.12,
        v_cruise_tas_kt=60.0,
        mach_descent=0.10,
        v_descent_ias_kt=55.0,
        v_approach_ias_kt=50.0,
        service_ceiling_ft=15000.0,
        initial_climb_fpm=400.0,
        mid_climb_fpm=300.0,
        high_climb_fpm=150.0,
        normal_descent_fpm=-250.0,
        approach_descent_fpm=-500.0,
        takeoff_accel_mps2=1.8,
        landing_decel_mps2=-1.5,
        enroute_accel_mps2=0.4,
        max_bank_angle_deg=45.0,
        max_roll_rate_dps=15.0,
        description="Standard Class Glider",
        default_cruise_alt_ft=5000.0,
    ),
    "E75L": AircraftPerformanceProfile(
        type_code="E75L",
        category=WakeVortexCategory.LARGE,
        v_taxi_kt=15.0,
        v_rotate_ias_kt=135.0,
        v_initial_climb_ias_kt=250.0,
        v_climb_ias_kt=280.0,
        mach_climb=0.76,
        mach_cruise=0.78,
        v_cruise_tas_kt=450.0,
        mach_descent=0.76,
        v_descent_ias_kt=280.0,
        v_approach_ias_kt=130.0,
        service_ceiling_ft=41000.0,
        initial_climb_fpm=2800.0,
        mid_climb_fpm=1900.0,
        high_climb_fpm=1000.0,
        normal_descent_fpm=-2000.0,
        approach_descent_fpm=-700.0,
        takeoff_accel_mps2=2.3,
        landing_decel_mps2=-2.7,
        enroute_accel_mps2=0.8,
        max_bank_angle_deg=25.0,
        max_roll_rate_dps=5.0,
        description="Embraer E175",
        default_cruise_alt_ft=35000.0,
    ),
    "DH8D": AircraftPerformanceProfile(
        type_code="DH8D",
        category=WakeVortexCategory.LARGE,
        v_taxi_kt=15.0,
        v_rotate_ias_kt=120.0,
        v_initial_climb_ias_kt=185.0,
        v_climb_ias_kt=210.0,
        mach_climb=0.50,
        mach_cruise=0.55,
        v_cruise_tas_kt=360.0,
        mach_descent=0.50,
        v_descent_ias_kt=220.0,
        v_approach_ias_kt=125.0,
        service_ceiling_ft=27000.0,
        initial_climb_fpm=2200.0,
        mid_climb_fpm=1500.0,
        high_climb_fpm=800.0,
        normal_descent_fpm=-1600.0,
        approach_descent_fpm=-650.0,
        takeoff_accel_mps2=2.1,
        landing_decel_mps2=-2.4,
        enroute_accel_mps2=0.7,
        max_bank_angle_deg=25.0,
        max_roll_rate_dps=6.0,
        description="De Havilland Canada Dash 8-400",
        default_cruise_alt_ft=24000.0,
    ),
    "SR22": AircraftPerformanceProfile(
        type_code="SR22",
        category=WakeVortexCategory.LIGHT,
        v_taxi_kt=12.0,
        v_rotate_ias_kt=75.0,
        v_initial_climb_ias_kt=100.0,
        v_climb_ias_kt=120.0,
        mach_climb=0.25,
        mach_cruise=0.30,
        v_cruise_tas_kt=185.0,
        mach_descent=0.25,
        v_descent_ias_kt=140.0,
        v_approach_ias_kt=80.0,
        service_ceiling_ft=17500.0,
        initial_climb_fpm=1200.0,
        mid_climb_fpm=800.0,
        high_climb_fpm=400.0,
        normal_descent_fpm=-1000.0,
        approach_descent_fpm=-500.0,
        takeoff_accel_mps2=1.8,
        landing_decel_mps2=-2.0,
        enroute_accel_mps2=0.6,
        max_bank_angle_deg=25.0,
        max_roll_rate_dps=10.0,
        description="Cirrus SR22",
        default_cruise_alt_ft=10000.0,
    ),
    "EC35": AircraftPerformanceProfile(
        type_code="EC35",
        category=WakeVortexCategory.ROTORCRAFT,
        v_taxi_kt=5.0,
        v_rotate_ias_kt=0.0,
        v_initial_climb_ias_kt=60.0,
        v_climb_ias_kt=75.0,
        mach_climb=0.15,
        mach_cruise=0.22,
        v_cruise_tas_kt=135.0,
        mach_descent=0.18,
        v_descent_ias_kt=80.0,
        v_approach_ias_kt=60.0,
        service_ceiling_ft=15000.0,
        initial_climb_fpm=1500.0,
        mid_climb_fpm=1000.0,
        high_climb_fpm=400.0,
        normal_descent_fpm=-1000.0,
        approach_descent_fpm=-500.0,
        takeoff_accel_mps2=1.5,
        landing_decel_mps2=-1.8,
        enroute_accel_mps2=0.6,
        max_bank_angle_deg=30.0,
        max_roll_rate_dps=15.0,
        description="Airbus Helicopters H135 / EC135",
        default_cruise_alt_ft=4000.0,
    ),
}

_ACTIVE_PROFILES: dict[str, AircraftPerformanceProfile] = dict(_BUILTIN_PROFILES)


def get_aircraft_profile(type_code: str) -> AircraftPerformanceProfile:
    """Retrieve aircraft performance profile by ICAO type designator (case-insensitive)."""
    code = type_code.strip().upper()
    if code in _ACTIVE_PROFILES:
        return _ACTIVE_PROFILES[code]
    raise KeyError(
        f"Unknown aircraft type '{type_code}'. Available types: {list_aircraft_profiles()}"
    )


def register_aircraft_profile(profile: AircraftPerformanceProfile) -> None:
    """Register or overwrite an aircraft performance profile."""
    _ACTIVE_PROFILES[profile.type_code.strip().upper()] = profile


def list_aircraft_profiles() -> list[str]:
    """Return sorted list of registered aircraft profile type codes."""
    return sorted(_ACTIVE_PROFILES.keys())


def has_aircraft_profile(type_code: str) -> bool:
    """Check if profile is registered for type code."""
    return type_code.strip().upper() in _ACTIVE_PROFILES


__all__ = [
    "AircraftPerformanceProfile",
    "WakeVortexCategory",
    "get_aircraft_profile",
    "has_aircraft_profile",
    "list_aircraft_profiles",
    "register_aircraft_profile",
]
