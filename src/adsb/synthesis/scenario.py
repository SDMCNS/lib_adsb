"""Airspace scenario definitions and pre-configured traffic generators.

Authoritative reference: docs/03_FLIGHT_SYNTHESIS_AND_PHYSICS.md Section 5,
and tests/integration/test_e2e_closed_loop.py Tier 4.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from adsb.synthesis.performance import AircraftPerformanceProfile, get_aircraft_profile
from adsb.synthesis.trajectory import (
    AircraftTruthState,
    generate_simple_climb_trajectory,
    generate_straight_flight,
)


@dataclass(frozen=True, slots=True)
class ScenarioAircraft:
    """An individual aircraft participating in an airspace simulation scenario."""

    icao24: int
    callsign: str
    states: list[AircraftTruthState]
    profile: AircraftPerformanceProfile | None = None


@dataclass(frozen=True, slots=True)
class Scenario:
    """Multi-aircraft airspace simulation scenario."""

    name: str
    aircraft: list[ScenarioAircraft]
    duration_s: float
    ref_lat: float = 52.3105
    ref_lon: float = 4.7683


def _build_clean_tma_10_aircraft(
    rng: np.random.Generator,
    duration_s: float = 3600.0,
) -> Scenario:
    """Build clean 10-aircraft terminal maneuvering area scenario (Schiphol)."""
    b738 = get_aircraft_profile("B738")
    a320 = get_aircraft_profile("A320")
    b77w = get_aircraft_profile("B77W")
    c172 = get_aircraft_profile("C172")
    b350 = get_aircraft_profile("B350")
    ec35 = get_aircraft_profile("EC35")
    configs = [
        # (icao, callsign, profile, lat, lon, heading, speed_kt, alt_ft, is_climb)
        (0x484001, "KLM101", b738, 52.3105, 4.7683, 40.0, 250.0, 10000.0, True),
        (0x484002, "KLM102", a320, 52.3200, 4.7500, 60.0, 250.0, 12000.0, True),
        (0x484003, "KLM103", b77w, 52.3000, 4.7800, 90.0, 260.0, 14000.0, True),
        (0x484004, "PH172A", c172, 52.3500, 4.8200, 180.0, 110.0, 3000.0, False),
        (0x484005, "LIFELN", ec35, 52.2800, 4.7100, 270.0, 120.0, 1500.0, False),
        (0x484006, "KLM106", b738, 52.4000, 4.6000, 120.0, 280.0, 8000.0, False),
        (0x484007, "EZY107", a320, 52.2500, 4.8500, 340.0, 250.0, 9000.0, True),
        (0x484008, "TRA108", b738, 52.4500, 4.9000, 220.0, 260.0, 6000.0, False),
        (0x484009, "PH350B", b350, 52.2000, 4.6500, 10.0, 240.0, 7000.0, False),
        (0x48400A, "PH172B", c172, 52.3300, 4.8800, 300.0, 105.0, 2500.0, False),
    ]

    aircraft_list: list[ScenarioAircraft] = []
    for icao, callsign, profile, lat, lon, hdg, spd, alt, is_climb in configs:
        if is_climb:
            states = generate_simple_climb_trajectory(
                profile=profile,
                origin_lat=lat,
                origin_lon=lon,
                target_alt_ft=alt,
                duration_s=duration_s,
                dt_s=1.0,
                rng=rng,
                callsign=callsign,
                icao_hex=f"{icao:06X}",
            )
        else:
            states = generate_straight_flight(
                start_lat=lat,
                start_lon=lon,
                heading_deg=hdg,
                speed_kt=spd,
                alt_ft=alt,
                duration_s=duration_s,
                dt_s=1.0,
                rng=rng,
                callsign=callsign,
                icao_hex=f"{icao:06X}",
            )
        aircraft_list.append(
            ScenarioAircraft(
                icao24=icao,
                callsign=callsign,
                states=states,
                profile=profile,
            )
        )

    return Scenario(
        name="clean_tma_10_aircraft",
        aircraft=aircraft_list,
        duration_s=duration_s,
    )


def _build_congested_tma_25_aircraft(
    rng: np.random.Generator,
    duration_s: float = 3600.0,
) -> Scenario:
    """Build congested 25-aircraft scenario with high traffic density for FRUIT/Garble testing."""
    profiles = [
        get_aircraft_profile("B738"),
        get_aircraft_profile("A320"),
        get_aircraft_profile("B77W"),
        get_aircraft_profile("B350"),
        get_aircraft_profile("C172"),
    ]

    aircraft_list: list[ScenarioAircraft] = []
    for i in range(25):
        icao = 0x485001 + i
        callsign = f"TMA{i+1:02d}"
        profile = profiles[i % len(profiles)]

        # Radial distribution around TMA
        angle_deg = (i * 360.0 / 25.0) % 360.0
        angle_rad = np.radians(angle_deg)
        dist_nm = 10.0 + (i % 5) * 4.0
        dist_m = dist_nm * 1852.0

        lat = 52.3105 + (dist_m * np.cos(angle_rad)) / 111120.0
        lon = 4.7683 + (dist_m * np.sin(angle_rad)) / (111120.0 * np.cos(np.radians(52.3105)))
        hdg = (angle_deg + 180.0) % 360.0  # Inbound towards center
        alt_ft = 3000.0 + (i % 10) * 1000.0
        spd_kt = 180.0 + (i % 4) * 30.0

        states = generate_straight_flight(
            start_lat=lat,
            start_lon=lon,
            heading_deg=hdg,
            speed_kt=spd_kt,
            alt_ft=alt_ft,
            duration_s=duration_s,
            dt_s=1.0,
            rng=rng,
            callsign=callsign,
            icao_hex=f"{icao:06X}",
        )
        aircraft_list.append(
            ScenarioAircraft(
                icao24=icao,
                callsign=callsign,
                states=states,
                profile=profile,
            )
        )

    return Scenario(
        name="congested_tma_25_aircraft",
        aircraft=aircraft_list,
        duration_s=duration_s,
    )


def _build_fault_and_spoofing_test(
    rng: np.random.Generator,
    duration_s: float = 3600.0,
) -> Scenario:
    """Build scenario for avionics fault injection and adversarial spoofing tests."""
    b738 = get_aircraft_profile("B738")
    a320 = get_aircraft_profile("A320")

    # Baseline legitimate aircraft
    configs = [
        (0x486001, "LEGIT1", b738, 52.3105, 4.7683, 50.0, 250.0, 10000.0, True),
        (0x486002, "LEGIT2", a320, 52.3500, 4.6000, 110.0, 260.0, 8000.0, False),
        (0x486003, "LEGIT3", b738, 52.2000, 4.8000, 350.0, 240.0, 6000.0, False),
    ]

    aircraft_list: list[ScenarioAircraft] = []
    for icao, callsign, profile, lat, lon, hdg, spd, alt, is_climb in configs:
        if is_climb:
            states = generate_simple_climb_trajectory(
                profile=profile,
                origin_lat=lat,
                origin_lon=lon,
                target_alt_ft=alt,
                duration_s=duration_s,
                dt_s=1.0,
                rng=rng,
                callsign=callsign,
                icao_hex=f"{icao:06X}",
            )
        else:
            states = generate_straight_flight(
                start_lat=lat,
                start_lon=lon,
                heading_deg=hdg,
                speed_kt=spd,
                alt_ft=alt,
                duration_s=duration_s,
                dt_s=1.0,
                rng=rng,
                callsign=callsign,
                icao_hex=f"{icao:06X}",
            )
        aircraft_list.append(
            ScenarioAircraft(
                icao24=icao,
                callsign=callsign,
                states=states,
                profile=profile,
            )
        )

    return Scenario(
        name="fault_and_spoofing_test",
        aircraft=aircraft_list,
        duration_s=duration_s,
    )


def load_scenario(
    name: str,
    rng: np.random.Generator,
    duration_s: float = 3600.0,
) -> Scenario:
    """Factory loader for predefined airspace traffic scenarios.

    Parameters:
        name: Predefined scenario identifier ('clean_tma_10_aircraft',
            'congested_tma_25_aircraft', or 'fault_and_spoofing_test').
        rng: Deterministic numpy random generator.
        duration_s: Total trajectory duration in seconds (default 3600.0 / 1 hour).
    """
    cleaned_name = name.strip().lower()
    if cleaned_name == "clean_tma_10_aircraft":
        return _build_clean_tma_10_aircraft(rng, duration_s=duration_s)
    elif cleaned_name == "congested_tma_25_aircraft":
        return _build_congested_tma_25_aircraft(rng, duration_s=duration_s)
    elif cleaned_name == "fault_and_spoofing_test":
        return _build_fault_and_spoofing_test(rng, duration_s=duration_s)
    else:
        raise ValueError(
            f"Unknown scenario '{name}'. Available scenarios: "
            "['clean_tma_10_aircraft', 'congested_tma_25_aircraft', 'fault_and_spoofing_test']"
        )


__all__ = [
    "Scenario",
    "ScenarioAircraft",
    "load_scenario",
]
