"""Dual-layer International Standard Atmosphere (ISA), altimetry, airspeeds & 3D wind triangle.

Authoritative reference: ICAO Doc 7488/3 (Manual of the ICAO Standard Atmosphere),
RTCA DO-260B / DO-260C, and docs/03_FLIGHT_SYNTHESIS_AND_PHYSICS.md Section 2.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from adsb.constants import (
    EARTH_MEAN_RADIUS_M,
    GAMMA_AIR,
    GAS_CONSTANT_AIR,
    GRAVITY_MPS2,
    ISA_LAPSE_RATE_KPM,
    ISA_P0_PA,
    ISA_RHO0_KGPM3,
    ISA_T0_K,
)

TROPOPAUSE_ALT_M: float = 11000.0
STRATOSPHERE_MAX_ALT_M: float = 20000.0
T_TROPOPAUSE_K: float = ISA_T0_K - ISA_LAPSE_RATE_KPM * TROPOPAUSE_ALT_M  # 216.65 K
_EXPONENT_PRESSURE: float = GRAVITY_MPS2 / (GAS_CONSTANT_AIR * ISA_LAPSE_RATE_KPM)  # ~5.25588
P_TROPOPAUSE_PA: float = ISA_P0_PA * math.pow(T_TROPOPAUSE_K / ISA_T0_K, _EXPONENT_PRESSURE)  # ~22632.06 Pa


@dataclass(frozen=True, slots=True)
class AtmosphereState:
    """Atmospheric state bundle at a specified altitude."""

    alt_m: float
    temperature_k: float
    pressure_pa: float
    density_kgpm3: float
    speed_of_sound_mps: float
    delta_isa_k: float = 0.0
    qnh_hpa: float = 1013.25

    @property
    def pressure_altitude_m(self) -> float:
        """Pressure altitude in meters referenced to standard datum 1013.25 hPa."""
        return pressure_to_pressure_altitude_m(self.pressure_pa)

    @property
    def pressure_altitude_ft(self) -> float:
        """Pressure altitude in feet referenced to standard datum 1013.25 hPa."""
        return pressure_to_pressure_altitude_ft(self.pressure_pa)


class AtmosphereModel:
    """Configurable atmospheric model instance."""

    def __init__(self, delta_isa_k: float = 0.0, qnh_hpa: float = 1013.25) -> None:
        self.delta_isa_k = delta_isa_k
        self.qnh_hpa = qnh_hpa

    def evaluate(self, alt_m: float) -> AtmosphereState:
        """Compute atmosphere state at given geometric altitude (meters MSL)."""
        return compute_atmosphere(alt_m, self.delta_isa_k, self.qnh_hpa)


# --- Dual-Layer ISA Atmosphere ---


def isa_temperature_k(alt_m: float, delta_isa_k: float = 0.0) -> float:
    """Compute ambient temperature (K) at altitude (m MSL)."""
    h = max(0.0, alt_m)
    t_isa = ISA_T0_K - ISA_LAPSE_RATE_KPM * h if h <= TROPOPAUSE_ALT_M else T_TROPOPAUSE_K
    return t_isa + delta_isa_k


def isa_pressure_pa(
    alt_m: float, delta_isa_k: float = 0.0, qnh_hpa: float = 1013.25
) -> float:
    """Compute ambient pressure (Pa) at altitude (m MSL)."""
    h = max(0.0, alt_m)
    p_base = qnh_hpa * 100.0

    if h <= TROPOPAUSE_ALT_M:
        temp_ratio = 1.0 - (ISA_LAPSE_RATE_KPM * h) / ISA_T0_K
        return p_base * math.pow(max(0.0, temp_ratio), _EXPONENT_PRESSURE)
    else:
        temp_ratio = T_TROPOPAUSE_K / ISA_T0_K
        p_trop = p_base * math.pow(temp_ratio, _EXPONENT_PRESSURE)
        dh = min(h, STRATOSPHERE_MAX_ALT_M) - TROPOPAUSE_ALT_M
        decay = math.exp(-GRAVITY_MPS2 * dh / (GAS_CONSTANT_AIR * T_TROPOPAUSE_K))
        return p_trop * decay


def isa_density_kgpm3(
    alt_m: float, delta_isa_k: float = 0.0, qnh_hpa: float = 1013.25
) -> float:
    """Compute air density (kg/m^3) at altitude (m MSL)."""
    t = isa_temperature_k(alt_m, delta_isa_k=delta_isa_k)
    p = isa_pressure_pa(alt_m, delta_isa_k=delta_isa_k, qnh_hpa=qnh_hpa)
    return p / (GAS_CONSTANT_AIR * t)


def isa_speed_of_sound_mps(alt_m: float, delta_isa_k: float = 0.0) -> float:
    """Compute local speed of sound (m/s) at altitude (m MSL)."""
    t = isa_temperature_k(alt_m, delta_isa_k=delta_isa_k)
    return math.sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * t)


def compute_atmosphere(
    alt_m: float, delta_isa_k: float = 0.0, qnh_hpa: float = 1013.25
) -> AtmosphereState:
    """Compute full atmospheric state bundle at altitude (m MSL)."""
    t = isa_temperature_k(alt_m, delta_isa_k)
    p = isa_pressure_pa(alt_m, delta_isa_k, qnh_hpa)
    rho = p / (GAS_CONSTANT_AIR * t)
    sos = math.sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * t)
    return AtmosphereState(
        alt_m=alt_m,
        temperature_k=t,
        pressure_pa=p,
        density_kgpm3=rho,
        speed_of_sound_mps=sos,
        delta_isa_k=delta_isa_k,
        qnh_hpa=qnh_hpa,
    )


# --- Altimetry Conversions ---


def geometric_to_geopotential_m(alt_geom_m: float) -> float:
    """Convert geometric altitude to geopotential altitude (meters)."""
    r_e = EARTH_MEAN_RADIUS_M
    return (r_e * alt_geom_m) / (r_e + alt_geom_m)


def geopotential_to_geometric_m(alt_geop_m: float) -> float:
    """Convert geopotential altitude to geometric altitude (meters)."""
    r_e = EARTH_MEAN_RADIUS_M
    return (r_e * alt_geop_m) / (r_e - alt_geop_m)


def pressure_to_pressure_altitude_m(pressure_pa: float) -> float:
    """Convert ambient static pressure (Pa) to standard pressure altitude (m)."""
    p = max(1.0, pressure_pa)
    inv_exponent = (GAS_CONSTANT_AIR * ISA_LAPSE_RATE_KPM) / GRAVITY_MPS2

    if p >= P_TROPOPAUSE_PA:
        # Troposphere
        h = (ISA_T0_K / ISA_LAPSE_RATE_KPM) * (1.0 - math.pow(p / ISA_P0_PA, inv_exponent))
        return h
    else:
        # Stratosphere
        dh = -(GAS_CONSTANT_AIR * T_TROPOPAUSE_K / GRAVITY_MPS2) * math.log(p / P_TROPOPAUSE_PA)
        return TROPOPAUSE_ALT_M + dh


def pressure_to_pressure_altitude_ft(pressure_pa: float) -> float:
    """Convert ambient static pressure (Pa) to standard pressure altitude (ft)."""
    return pressure_to_pressure_altitude_m(pressure_pa) / 0.3048


def altitude_to_indicated_altitude_ft(
    alt_m: float, qnh_hpa: float, delta_isa_k: float = 0.0
) -> float:
    """Calculate cockpit indicated barometric altitude in feet given QNH."""
    p_ambient = isa_pressure_pa(alt_m, delta_isa_k=delta_isa_k, qnh_hpa=qnh_hpa)
    p_ref = qnh_hpa * 100.0
    inv_exponent = (GAS_CONSTANT_AIR * ISA_LAPSE_RATE_KPM) / GRAVITY_MPS2
    h_ind_m = (ISA_T0_K / ISA_LAPSE_RATE_KPM) * (1.0 - math.pow(p_ambient / p_ref, inv_exponent))
    return h_ind_m / 0.3048


def geo_minus_baro_ft(alt_hae_m: float, alt_baro_ft: float) -> float:
    """Compute difference (GNSS HAE - Baro Alt) in feet for TC 19 Airborne Velocity."""
    return (alt_hae_m / 0.3048) - alt_baro_ft


# --- Airspeed Conversions (CAS, TAS, EAS, Mach) ---


def tas_to_mach(tas_mps: float, alt_m: float, delta_isa_k: float = 0.0) -> float:
    """Convert True Airspeed (m/s) to Mach number at altitude."""
    sos = isa_speed_of_sound_mps(alt_m, delta_isa_k)
    return tas_mps / sos


def mach_to_tas(mach: float, alt_m: float, delta_isa_k: float = 0.0) -> float:
    """Convert Mach number to True Airspeed (m/s) at altitude."""
    sos = isa_speed_of_sound_mps(alt_m, delta_isa_k)
    return mach * sos


def tas_to_eas(
    tas_mps: float, alt_m: float, delta_isa_k: float = 0.0, qnh_hpa: float = 1013.25
) -> float:
    """Convert True Airspeed (m/s) to Equivalent Airspeed (m/s)."""
    rho = isa_density_kgpm3(alt_m, delta_isa_k, qnh_hpa)
    return tas_mps * math.sqrt(rho / ISA_RHO0_KGPM3)


def eas_to_tas(
    eas_mps: float, alt_m: float, delta_isa_k: float = 0.0, qnh_hpa: float = 1013.25
) -> float:
    """Convert Equivalent Airspeed (m/s) to True Airspeed (m/s)."""
    rho = isa_density_kgpm3(alt_m, delta_isa_k, qnh_hpa)
    return eas_mps * math.sqrt(ISA_RHO0_KGPM3 / rho)


def cas_to_tas(
    cas_mps: float, alt_m: float, delta_isa_k: float = 0.0, qnh_hpa: float = 1013.25
) -> float:
    """Convert Calibrated Airspeed (m/s) to True Airspeed (m/s) using Saint-Venant equation."""
    if cas_mps <= 0.0:
        return 0.0
    a0 = math.sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * ISA_T0_K)
    p0 = ISA_P0_PA
    p = isa_pressure_pa(alt_m, delta_isa_k, qnh_hpa)

    # Impact pressure qc from CAS
    qc = p0 * (
        math.pow(
            1.0 + ((GAMMA_AIR - 1.0) / 2.0) * math.pow(cas_mps / a0, 2),
            GAMMA_AIR / (GAMMA_AIR - 1.0),
        )
        - 1.0
    )

    # Mach from qc and ambient p
    mach_sq = (2.0 / (GAMMA_AIR - 1.0)) * (
        math.pow((qc / p) + 1.0, (GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0
    )
    mach = math.sqrt(max(0.0, mach_sq))
    return mach_to_tas(mach, alt_m, delta_isa_k)


def tas_to_cas(
    tas_mps: float, alt_m: float, delta_isa_k: float = 0.0, qnh_hpa: float = 1013.25
) -> float:
    """Convert True Airspeed (m/s) to Calibrated Airspeed (m/s) using Saint-Venant equation."""
    if tas_mps <= 0.0:
        return 0.0
    a0 = math.sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * ISA_T0_K)
    p0 = ISA_P0_PA
    p = isa_pressure_pa(alt_m, delta_isa_k, qnh_hpa)
    mach = tas_to_mach(tas_mps, alt_m, delta_isa_k)

    # Impact pressure qc from Mach and ambient p
    qc = p * (
        math.pow(
            1.0 + ((GAMMA_AIR - 1.0) / 2.0) * math.pow(mach, 2),
            GAMMA_AIR / (GAMMA_AIR - 1.0),
        )
        - 1.0
    )

    # CAS from qc at standard sea-level
    cas_sq = (2.0 * math.pow(a0, 2) / (GAMMA_AIR - 1.0)) * (
        math.pow((qc / p0) + 1.0, (GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0
    )
    return math.sqrt(max(0.0, cas_sq))


def crossover_altitude_m(cas_mps: float, mach: float) -> float:
    """Compute the crossover altitude (meters) where constant CAS matches climb Mach."""
    a0 = math.sqrt(GAMMA_AIR * GAS_CONSTANT_AIR * ISA_T0_K)
    p0 = ISA_P0_PA
    qc = p0 * (
        math.pow(
            1.0 + ((GAMMA_AIR - 1.0) / 2.0) * math.pow(cas_mps / a0, 2),
            GAMMA_AIR / (GAMMA_AIR - 1.0),
        )
        - 1.0
    )
    denom = (
        math.pow(
            1.0 + ((GAMMA_AIR - 1.0) / 2.0) * math.pow(mach, 2),
            GAMMA_AIR / (GAMMA_AIR - 1.0),
        )
        - 1.0
    )
    if denom <= 0.0:
        return 0.0
    p_cross = qc / denom
    return pressure_to_pressure_altitude_m(p_cross)


# --- 3D Wind Triangle ---


def wind_components_uv(wind_speed_mps: float, wind_dir_from_deg: float) -> tuple[float, float]:
    """Convert meteorological wind (speed, direction FROM in degrees) to (w_east, w_north) in m/s."""
    dir_rad = math.radians(wind_dir_from_deg)
    w_east = -wind_speed_mps * math.sin(dir_rad)
    w_north = -wind_speed_mps * math.cos(dir_rad)
    return (w_east, w_north)


def solve_wind_triangle_heading(
    track_deg: float,
    tas_mps: float,
    wind_speed_mps: float,
    wind_dir_from_deg: float,
) -> tuple[float, float, float]:
    """Given desired True Track, True Airspeed, and Wind, solve for (heading_deg, gs_mps, wca_deg).

    Raises:
        ValueError: If crosswind component exceeds TAS (impossible to maintain track).
    """
    w_east, w_north = wind_components_uv(wind_speed_mps, wind_dir_from_deg)
    chi = math.radians(track_deg)

    w_cross = w_east * math.cos(chi) - w_north * math.sin(chi)
    w_along = w_east * math.sin(chi) + w_north * math.cos(chi)

    if tas_mps <= 0.0:
        raise ValueError("True Airspeed must be positive to solve wind triangle")

    sin_wca = w_cross / tas_mps
    if abs(sin_wca) > 1.0:
        raise ValueError(
            f"Crosswind ({w_cross:.1f} m/s) exceeds True Airspeed ({tas_mps:.1f} m/s): cannot maintain track"
        )

    wca_rad = math.asin(sin_wca)
    wca_deg = math.degrees(wca_rad)

    heading_rad = (chi - wca_rad) % (2.0 * math.pi)
    heading_deg = math.degrees(heading_rad)

    gs_mps = tas_mps * math.cos(wca_rad) + w_along
    if gs_mps < 0.0:
        gs_mps = 0.0  # Excessive headwind

    return (heading_deg % 360.0, gs_mps, wca_deg)


def solve_wind_triangle_track(
    heading_deg: float,
    tas_mps: float,
    wind_speed_mps: float,
    wind_dir_from_deg: float,
) -> tuple[float, float, float]:
    """Given True Heading, True Airspeed, and Wind, solve for (track_deg, gs_mps, drift_deg)."""
    w_east, w_north = wind_components_uv(wind_speed_mps, wind_dir_from_deg)
    psi = math.radians(heading_deg)

    vg_east = tas_mps * math.sin(psi) + w_east
    vg_north = tas_mps * math.cos(psi) + w_north

    gs_mps = math.hypot(vg_east, vg_north)
    if gs_mps < 1e-6:
        track_deg = heading_deg
        drift_deg = 0.0
    else:
        track_deg = math.degrees(math.atan2(vg_east, vg_north)) % 360.0
        drift_deg = (track_deg - heading_deg + 180.0) % 360.0 - 180.0

    return (track_deg, gs_mps, drift_deg)


def groundspeed_vector_3d(
    heading_deg: float,
    tas_mps: float,
    vr_mps: float,
    wind_speed_mps: float,
    wind_dir_from_deg: float,
    w_up_mps: float = 0.0,
) -> tuple[float, float, float]:
    """Compute 3D ground velocity vector [V_east, V_north, V_up] in m/s."""
    w_east, w_north = wind_components_uv(wind_speed_mps, wind_dir_from_deg)
    psi = math.radians(heading_deg)

    vg_east = tas_mps * math.sin(psi) + w_east
    vg_north = tas_mps * math.cos(psi) + w_north
    vg_up = vr_mps + w_up_mps

    return (vg_east, vg_north, vg_up)


__all__ = [
    "P_TROPOPAUSE_PA",
    "STRATOSPHERE_MAX_ALT_M",
    "TROPOPAUSE_ALT_M",
    "T_TROPOPAUSE_K",
    "AtmosphereModel",
    "AtmosphereState",
    "altitude_to_indicated_altitude_ft",
    "cas_to_tas",
    "compute_atmosphere",
    "crossover_altitude_m",
    "eas_to_tas",
    "geo_minus_baro_ft",
    "geometric_to_geopotential_m",
    "geopotential_to_geometric_m",
    "groundspeed_vector_3d",
    "isa_density_kgpm3",
    "isa_pressure_pa",
    "isa_speed_of_sound_mps",
    "isa_temperature_k",
    "mach_to_tas",
    "pressure_to_pressure_altitude_ft",
    "pressure_to_pressure_altitude_m",
    "solve_wind_triangle_heading",
    "solve_wind_triangle_track",
    "tas_to_cas",
    "tas_to_eas",
    "tas_to_mach",
    "wind_components_uv",
]
