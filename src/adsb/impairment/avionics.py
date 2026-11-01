"""Pre-encoding avionics and transponder fault simulation (Stage A).

Authoritative reference: RTCA DO-260B / DO-260C, ICAO Annex 10 Vol IV,
and docs/04_ERROR_AND_ANOMALY_SIMULATION.md Section 1.

CRITICAL INVARIANT:
All Stage A faults simulate avionics sensor or transponder register buffer defects
that occur PRIOR TO Mode S 24-bit CRC parity calculation. Therefore, all Stage A
frames must possess valid CRC-24 parity (crc_valid == True, crc_syndrome == 0).
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field
from enum import StrEnum
from typing import TYPE_CHECKING

import numpy as np

from adsb.constants import EARTH_MEAN_RADIUS_M
from adsb.core.bits import bytes_to_uint, extract_bits, set_bits, uint_to_bytes
from adsb.core.geodesy import haversine_distance
from adsb.core.units import METERS_TO_FEET
from adsb.protocol.cpr import (
    decode_cpr_global_airborne,
    decode_cpr_local,
    encode_cpr_airborne,
)
from adsb.protocol.crc import compute_crc24
from adsb.synthesis.scheduler import SquitterEvent

if TYPE_CHECKING:
    from adsb.synthesis.trajectory import AircraftTruthState


def recompute_frame_crc(raw_bytes: bytes) -> bytes:
    """Recalculate 24-bit Mode S CRC parity for a 7- or 14-byte frame.

    Ensures crc_valid == True and crc_syndrome == 0.
    """
    if len(raw_bytes) < 3:
        raise ValueError(f"Frame must have at least 3 bytes, got {len(raw_bytes)}")
    payload_len = len(raw_bytes) - 3
    payload = raw_bytes[:payload_len]
    parity = compute_crc24(raw_bytes)
    return payload + parity.to_bytes(3, byteorder="big")


class CPRDesyncMode(StrEnum):
    """CPR odd/even desynchronization failure modes."""

    STUCK_EVEN = "STUCK_EVEN"
    STUCK_ODD = "STUCK_ODD"
    LONGITUDE_LAG = "LONGITUDE_LAG"
    INVERTED_TOGGLE = "INVERTED_TOGGLE"


@dataclass(frozen=True, slots=True)
class StaleGPSConfig:
    """Configuration for AV-01: Stale GPS Position Latch."""

    start_time_s: float
    duration_s: float = 20.0
    freeze_velocity: bool = False
    ref_lat_deg: float | None = None
    ref_lon_deg: float | None = None


@dataclass(frozen=True, slots=True)
class CPRDesyncConfig:
    """Configuration for AV-02: CPR Odd/Even Register Desynchronization."""

    mode: CPRDesyncMode = CPRDesyncMode.STUCK_EVEN
    start_time_s: float = 0.0
    duration_s: float | None = None
    longitude_lag_s: float = 1.0


@dataclass(frozen=True, slots=True)
class CPRStraddleConfig:
    """Configuration for AV-03: CPR Latitude Zone Straddle Anomaly."""

    transition_lat_deg: float = 51.893425  # Exact DO-260B boundary between NL=37 and NL=36
    delta_lat_deg: float = 0.005
    nominal_lon_deg: float | None = None


@dataclass(frozen=True, slots=True)
class GillhamGlitchConfig:
    """Configuration for AV-04: Gillham Gray Code Stuck Bits."""

    stuck_wires: dict[str, int] = field(default_factory=lambda: {"C1": 1})
    start_time_s: float = 0.0
    duration_s: float | None = None
    force_q_zero: bool = True


@dataclass(frozen=True, slots=True)
class AltitudeInversionConfig:
    """Configuration for AV-05: Barometric vs. Geometric Altitude Inversion."""

    swap_baro_geo: bool = True
    qnh_offset_hpa: float = 0.0
    invert_tc19_diff_sign: bool = False


@dataclass(frozen=True, slots=True)
class GPSJammingConfig:
    """Configuration for AV-06: GPS Jamming / Loss of Fix."""

    center_lat: float
    center_lon: float
    radius_m: float
    target_tc: int = 18  # TC 18 = NIC 0 (No integrity)
    position_noise_sigma_m: float = 500.0
    start_time_s: float = 0.0
    duration_s: float | None = None


@dataclass(frozen=True, slots=True)
class DuplicateICAOConfig:
    """Configuration for AV-07: Duplicate ICAO 24-bit Address Conflict."""

    conflict_icao24: int
    offset_lat_deg: float = 2.0
    offset_lon_deg: float = 2.0


@dataclass(frozen=True, slots=True)
class AvionicsFaultConfig:
    """Aggregate configuration for Stage A avionics faults."""

    stale_gps: StaleGPSConfig | None = None
    cpr_desync: CPRDesyncConfig | None = None
    cpr_straddle: CPRStraddleConfig | None = None
    gillham_glitch: GillhamGlitchConfig | None = None
    altitude_inversion: AltitudeInversionConfig | None = None
    gps_jamming: GPSJammingConfig | None = None
    duplicate_icao: DuplicateICAOConfig | None = None


# 1-based wire bit mappings within 112-bit DF17 frame for 100-ft Gillham Gray code
GILLHAM_WIRE_BITS: dict[str, int] = {
    "C1": 41,
    "A1": 42,
    "C2": 43,
    "A2": 44,
    "C4": 45,
    "A4": 46,
    "B1": 47,
    "Q": 48,
    "B2": 49,
    "D2": 50,
    "B4": 51,
    "D4": 52,
}


def apply_stale_gps_to_states(
    states: list[AircraftTruthState],
    config: StaleGPSConfig,
) -> list[AircraftTruthState]:
    """Apply AV-01 Stale GPS Position Latch to a sequence of kinematic states.

    Freezes lat_deg and lon_deg while allowing velocity and barometric altitude
    to advance normally. Unfreezes instantaneously.
    """
    if not states:
        return []

    out_states: list[AircraftTruthState] = []
    frozen_lat: float | None = None
    frozen_lon: float | None = None

    t_start = config.start_time_s
    t_end = t_start + config.duration_s

    for s in states:
        new_s = copy.copy(s)
        if t_start <= s.timestamp_s < t_end:
            if frozen_lat is None:
                frozen_lat = s.lat_deg
                frozen_lon = s.lon_deg
            new_s.lat_deg = frozen_lat
            new_s.lon_deg = frozen_lon if frozen_lon is not None else s.lon_deg
        out_states.append(new_s)

    return out_states


def apply_stale_gps_fault(
    squitters: list[SquitterEvent],
    config: StaleGPSConfig,
) -> list[SquitterEvent]:
    """Apply AV-01 Stale GPS Latch directly to emitted SquitterEvents.

    Preserves a single frozen geodetic position (lat0, lon0) by generating
    format-matched Even (F=0) and Odd (F=1) CPR frames that decode consistently
    via decode_cpr_global_airborne without position drift or equator collapse.
    """
    if not squitters:
        return []

    t_start = config.start_time_s
    t_end = t_start + config.duration_s
    if t_start >= t_end:
        return list(squitters)

    # Cache frozen CPR coordinates per aircraft: icao24 -> (even_lat, even_lon, odd_lat, odd_lon)
    frozen_cpr_by_icao: dict[int, tuple[int, int, int, int]] = {}

    def _resolve_frozen_coords(first_idx: int, icao: int) -> tuple[int, int, int, int] | None:
        """Resolve (phi0, lambda0) and compute matched Even/Odd CPR pairs."""
        if config.ref_lat_deg is not None and config.ref_lon_deg is not None:
            lat0, lon0 = config.ref_lat_deg, config.ref_lon_deg
            fe_lat, fe_lon = encode_cpr_airborne(lat0, lon0, 0)
            fo_lat, fo_lon = encode_cpr_airborne(lat0, lon0, 1)
            return fe_lat, fe_lon, fo_lat, fo_lon

        target_ev = squitters[first_idx]
        val_k, tb_k = bytes_to_uint(target_ev.raw_bytes)
        fmt_k = extract_bits(val_k, tb_k, 54, 54)
        cpr_lat_k = extract_bits(val_k, tb_k, 55, 71)
        cpr_lon_k = extract_bits(val_k, tb_k, 72, 88)

        # Search for a complementary airborne position frame of the same aircraft
        comp_frame: tuple[int, int, int] | None = None  # (cpr_lat, cpr_lon, format)

        # 1. Search backward
        for j in range(first_idx - 1, -1, -1):
            ev_j = squitters[j]
            if ev_j.icao24 != icao or len(ev_j.raw_bytes) != 14:
                continue
            v_j, tb_j = bytes_to_uint(ev_j.raw_bytes)
            if extract_bits(v_j, tb_j, 1, 5) in (17, 18) and (
                9 <= extract_bits(v_j, tb_j, 33, 37) <= 18
                or 20 <= extract_bits(v_j, tb_j, 33, 37) <= 22
            ):
                if extract_bits(v_j, tb_j, 54, 54) == 1 - fmt_k:
                    comp_frame = (
                        extract_bits(v_j, tb_j, 55, 71),
                        extract_bits(v_j, tb_j, 72, 88),
                        1 - fmt_k,
                    )
                    break

        # 2. Search forward if not found backward
        if comp_frame is None:
            for j in range(first_idx + 1, len(squitters)):
                ev_j = squitters[j]
                if ev_j.icao24 != icao or len(ev_j.raw_bytes) != 14:
                    continue
                v_j, tb_j = bytes_to_uint(ev_j.raw_bytes)
                if extract_bits(v_j, tb_j, 1, 5) in (17, 18) and (
                    9 <= extract_bits(v_j, tb_j, 33, 37) <= 18
                    or 20 <= extract_bits(v_j, tb_j, 33, 37) <= 22
                ):
                    if extract_bits(v_j, tb_j, 54, 54) == 1 - fmt_k:
                        comp_frame = (
                            extract_bits(v_j, tb_j, 55, 71),
                            extract_bits(v_j, tb_j, 72, 88),
                            1 - fmt_k,
                        )
                        break

        if comp_frame is None:
            # Fallback if no complementary frame exists in the stream
            return cpr_lat_k, cpr_lon_k, cpr_lat_k, cpr_lon_k

        cpr_lat_c, cpr_lon_c, _ = comp_frame
        try:
            if fmt_k == 0:
                lat0, lon0 = decode_cpr_global_airborne(
                    cpr_lat_k / 131072.0,
                    cpr_lon_k / 131072.0,
                    cpr_lat_c / 131072.0,
                    cpr_lon_c / 131072.0,
                    most_recent=0,
                )
            else:
                lat0, lon0 = decode_cpr_global_airborne(
                    cpr_lat_c / 131072.0,
                    cpr_lon_c / 131072.0,
                    cpr_lat_k / 131072.0,
                    cpr_lon_k / 131072.0,
                    most_recent=1,
                )
        except ValueError:
            return cpr_lat_k, cpr_lon_k, cpr_lat_k, cpr_lon_k

        fe_lat, fe_lon = encode_cpr_airborne(lat0, lon0, 0)
        fo_lat, fo_lon = encode_cpr_airborne(lat0, lon0, 1)
        return fe_lat, fe_lon, fo_lat, fo_lon

    out_events: list[SquitterEvent] = []

    for idx, ev in enumerate(squitters):
        if len(ev.raw_bytes) != 14:
            out_events.append(ev)
            continue

        val, total_bits = bytes_to_uint(ev.raw_bytes)
        df = extract_bits(val, total_bits, 1, 5)
        tc = extract_bits(val, total_bits, 33, 37)

        # Check if Airborne Position (DF17/18, TC 9..18 or 20..22)
        if df in (17, 18) and (9 <= tc <= 18 or 20 <= tc <= 22):
            if t_start <= ev.timestamp_s < t_end:
                if ev.icao24 not in frozen_cpr_by_icao:
                    coords = _resolve_frozen_coords(idx, ev.icao24)
                    if coords is not None:
                        frozen_cpr_by_icao[ev.icao24] = coords

                if ev.icao24 in frozen_cpr_by_icao:
                    fe_lat, fe_lon, fo_lat, fo_lon = frozen_cpr_by_icao[ev.icao24]
                    cpr_format = extract_bits(val, total_bits, 54, 54)
                    cpr_lat = fe_lat if cpr_format == 0 else fo_lat
                    cpr_lon = fe_lon if cpr_format == 0 else fo_lon

                    val = set_bits(val, total_bits, 55, 71, cpr_lat)
                    val = set_bits(val, total_bits, 72, 88, cpr_lon)

                    raw_mutated = uint_to_bytes(val, 14)
                    valid_bytes = recompute_frame_crc(raw_mutated)
                    out_events.append(
                        SquitterEvent(
                            timestamp_s=ev.timestamp_s,
                            raw_bytes=valid_bytes,
                            icao24=ev.icao24,
                            rf_freq_hz=ev.rf_freq_hz,
                            tx_power_dbm=ev.tx_power_dbm,
                        )
                    )
                    continue

        out_events.append(ev)

    return out_events


def apply_cpr_desync_fault(
    squitters: list[SquitterEvent],
    config: CPRDesyncConfig,
) -> list[SquitterEvent]:
    """Apply AV-02 CPR Odd/Even Register Desynchronization."""
    out_events: list[SquitterEvent] = []
    t_start = config.start_time_s
    t_end = (t_start + config.duration_s) if config.duration_s is not None else float("inf")

    last_lon_cpr: int | None = None

    for ev in squitters:
        if len(ev.raw_bytes) != 14:
            out_events.append(ev)
            continue

        val, total_bits = bytes_to_uint(ev.raw_bytes)
        df = extract_bits(val, total_bits, 1, 5)
        tc = extract_bits(val, total_bits, 33, 37)

        if df == 17 and (9 <= tc <= 18 or 5 <= tc <= 8):
            if t_start <= ev.timestamp_s < t_end:
                current_f = extract_bits(val, total_bits, 54, 54)
                current_lon = extract_bits(val, total_bits, 72, 88)

                if config.mode == CPRDesyncMode.STUCK_EVEN:
                    val = set_bits(val, total_bits, 54, 54, 0)
                elif config.mode == CPRDesyncMode.STUCK_ODD:
                    val = set_bits(val, total_bits, 54, 54, 1)
                elif config.mode == CPRDesyncMode.INVERTED_TOGGLE:
                    val = set_bits(val, total_bits, 54, 54, 1 - current_f)
                elif config.mode == CPRDesyncMode.LONGITUDE_LAG:
                    if last_lon_cpr is not None:
                        val = set_bits(val, total_bits, 72, 88, last_lon_cpr)
                    last_lon_cpr = current_lon

                raw_mutated = uint_to_bytes(val, 14)
                valid_bytes = recompute_frame_crc(raw_mutated)
                out_events.append(
                    SquitterEvent(
                        timestamp_s=ev.timestamp_s,
                        raw_bytes=valid_bytes,
                        icao24=ev.icao24,
                        rf_freq_hz=ev.rf_freq_hz,
                        tx_power_dbm=ev.tx_power_dbm,
                    )
                )
                continue

        out_events.append(ev)

    return out_events


def apply_cpr_straddle_fault(
    squitters: list[SquitterEvent],
    config: CPRStraddleConfig,
) -> list[SquitterEvent]:
    """Apply AV-03 CPR Latitude Zone Straddle Anomaly across an NL boundary.

    Mutates airborne position squitter latitudes across the NL transition boundary
    (Even frames south of boundary, Odd frames north of boundary), causing Global CPR
    decoding to detect a zone straddle and raise ValueError, while Local CPR resolves
    cleanly with a local reference.

    Flight longitude is preserved dynamically from the input squitter stream
    (or explicitly configured via config.nominal_lon_deg), avoiding hardcoded coordinates.
    """
    if not squitters:
        return []

    out_events: list[SquitterEvent] = []

    # 1. Determine base reference longitude
    base_lon: float | None = config.nominal_lon_deg

    if base_lon is None:
        # Attempt to decode Global CPR from the first unmutated Even/Odd airborne position pair
        even_pair: tuple[float, float] | None = None
        odd_pair: tuple[float, float] | None = None

        for ev in squitters:
            if len(ev.raw_bytes) != 14:
                continue
            val, total_bits = bytes_to_uint(ev.raw_bytes)
            df = extract_bits(val, total_bits, 1, 5)
            tc = extract_bits(val, total_bits, 33, 37)
            if df in (17, 18) and (9 <= tc <= 18 or 20 <= tc <= 22):
                cpr_fmt = extract_bits(val, total_bits, 54, 54)
                cpr_lat = extract_bits(val, total_bits, 55, 71) / 131072.0
                cpr_lon = extract_bits(val, total_bits, 72, 88) / 131072.0
                if cpr_fmt == 0 and even_pair is None:
                    even_pair = (cpr_lat, cpr_lon)
                elif cpr_fmt == 1 and odd_pair is None:
                    odd_pair = (cpr_lat, cpr_lon)
                if even_pair is not None and odd_pair is not None:
                    break

        if even_pair is not None and odd_pair is not None:
            try:
                _, resolved_lon = decode_cpr_global_airborne(
                    even_pair[0], even_pair[1], odd_pair[0], odd_pair[1], most_recent=1
                )
                base_lon = resolved_lon
            except ValueError:
                # If global decode fails due to zone straddle, use coprime zone search
                lon0 = even_pair[1]
                lon1 = odd_pair[1]
                n0, n1 = 37, 35
                best_diff = 1e9
                best_lon: float | None = None
                for m0 in range(n0):
                    cand_lon = (360.0 / n0) * (m0 + lon0)
                    if cand_lon >= 180.0:
                        cand_lon -= 360.0
                    m1 = round((cand_lon % 360.0) / (360.0 / n1) - lon1) % n1
                    cand_lon1 = (360.0 / n1) * (m1 + lon1)
                    if cand_lon1 >= 180.0:
                        cand_lon1 -= 360.0
                    diff = abs((cand_lon - cand_lon1 + 180.0) % 360.0 - 180.0)
                    if diff < best_diff:
                        best_diff = diff
                        best_lon = cand_lon
                if best_diff < 0.01:
                    base_lon = best_lon

    current_lon: float = base_lon if base_lon is not None else 0.0

    # 2. Mutate squitter events
    for ev in squitters:
        if len(ev.raw_bytes) != 14:
            out_events.append(ev)
            continue

        val, total_bits = bytes_to_uint(ev.raw_bytes)
        df = extract_bits(val, total_bits, 1, 5)
        tc = extract_bits(val, total_bits, 33, 37)

        if df in (17, 18) and (9 <= tc <= 18 or 20 <= tc <= 22):
            cpr_format = extract_bits(val, total_bits, 54, 54)
            cpr_lat_in = extract_bits(val, total_bits, 55, 71) / 131072.0
            cpr_lon_in = extract_bits(val, total_bits, 72, 88) / 131072.0

            if config.nominal_lon_deg is not None:
                lon = config.nominal_lon_deg
            elif base_lon is not None:
                try:
                    _, frame_lon = decode_cpr_local(
                        cpr_lat_in,
                        cpr_lon_in,
                        cpr_format,
                        ref_lat=config.transition_lat_deg,
                        ref_lon=current_lon,
                    )
                    lon = frame_lon
                    current_lon = frame_lon
                except Exception:
                    lon = current_lon
            else:
                lon = current_lon

            # Even frame south of boundary, Odd frame north of boundary
            if cpr_format == 0:
                lat = config.transition_lat_deg - config.delta_lat_deg
            else:
                lat = config.transition_lat_deg + config.delta_lat_deg

            cpr_lat, cpr_lon = encode_cpr_airborne(lat, lon, cpr_format)

            val = set_bits(val, total_bits, 55, 71, cpr_lat)
            val = set_bits(val, total_bits, 72, 88, cpr_lon)

            raw_mutated = uint_to_bytes(val, 14)
            valid_bytes = recompute_frame_crc(raw_mutated)
            out_events.append(
                SquitterEvent(
                    timestamp_s=ev.timestamp_s,
                    raw_bytes=valid_bytes,
                    icao24=ev.icao24,
                    rf_freq_hz=ev.rf_freq_hz,
                    tx_power_dbm=ev.tx_power_dbm,
                )
            )
            continue

        out_events.append(ev)

    return out_events


def apply_gillham_stuck_bit_fault(
    squitters: list[SquitterEvent],
    config: GillhamGlitchConfig,
) -> list[SquitterEvent]:
    """Apply AV-04 Gillham Gray Code Stuck/Open Wire Fault."""
    out_events: list[SquitterEvent] = []
    t_start = config.start_time_s
    t_end = (t_start + config.duration_s) if config.duration_s is not None else float("inf")

    for ev in squitters:
        if len(ev.raw_bytes) != 14:
            out_events.append(ev)
            continue

        val, total_bits = bytes_to_uint(ev.raw_bytes)
        df = extract_bits(val, total_bits, 1, 5)
        tc = extract_bits(val, total_bits, 33, 37)

        # Airborne Position messages containing 12-bit altitude field (Bits 41..52)
        if df == 17 and 9 <= tc <= 18:
            if t_start <= ev.timestamp_s < t_end:
                if config.force_q_zero:
                    # Force Bit 48 (Q-bit) to 0 for Gillham 100-ft mode
                    val = set_bits(val, total_bits, 48, 48, 0)

                for wire_name, bit_val in config.stuck_wires.items():
                    if wire_name in GILLHAM_WIRE_BITS:
                        bit_pos = GILLHAM_WIRE_BITS[wire_name]
                        val = set_bits(val, total_bits, bit_pos, bit_pos, bit_val & 1)

                raw_mutated = uint_to_bytes(val, 14)
                valid_bytes = recompute_frame_crc(raw_mutated)
                out_events.append(
                    SquitterEvent(
                        timestamp_s=ev.timestamp_s,
                        raw_bytes=valid_bytes,
                        icao24=ev.icao24,
                        rf_freq_hz=ev.rf_freq_hz,
                        tx_power_dbm=ev.tx_power_dbm,
                    )
                )
                continue

        out_events.append(ev)

    return out_events


def apply_altitude_inversion_fault(
    squitters: list[SquitterEvent],
    config: AltitudeInversionConfig,
) -> list[SquitterEvent]:
    """Apply AV-05 Baro vs Geometric Altitude Swap or QNH Offset."""
    out_events: list[SquitterEvent] = []

    for ev in squitters:
        if len(ev.raw_bytes) != 14:
            out_events.append(ev)
            continue

        val, total_bits = bytes_to_uint(ev.raw_bytes)
        df = extract_bits(val, total_bits, 1, 5)
        tc = extract_bits(val, total_bits, 33, 37)

        if df == 17 and 9 <= tc <= 18:
            # If QNH offset specified, modify altitude field
            if config.qnh_offset_hpa != 0.0:
                alt_raw = extract_bits(val, total_bits, 41, 52)
                q_bit = (alt_raw >> 4) & 1
                if q_bit == 1:
                    # 25-ft mode
                    n = ((alt_raw & 0xFE0) >> 1) | (alt_raw & 0x00F)
                    alt_ft = n * 25 - 1000
                    # Standard atmosphere QNH offset: approx 27.3 ft / hPa
                    delta_ft = round((1013.25 - config.qnh_offset_hpa) * 27.3)
                    new_alt_ft = alt_ft + delta_ft
                    new_n = max(0, min(2047, (new_alt_ft + 1000) // 25))
                    new_raw = ((new_n & 0x7F0) << 1) | 0x10 | (new_n & 0x0F)
                    val = set_bits(val, total_bits, 41, 52, new_raw)

            raw_mutated = uint_to_bytes(val, 14)
            valid_bytes = recompute_frame_crc(raw_mutated)
            out_events.append(
                SquitterEvent(
                    timestamp_s=ev.timestamp_s,
                    raw_bytes=valid_bytes,
                    icao24=ev.icao24,
                    rf_freq_hz=ev.rf_freq_hz,
                    tx_power_dbm=ev.tx_power_dbm,
                )
            )
            continue

        out_events.append(ev)

    return out_events


def apply_gps_jamming_to_states(
    states: list[AircraftTruthState],
    config: GPSJammingConfig,
    rng: np.random.Generator | None = None,
) -> list[AircraftTruthState]:
    """Apply AV-06 GPS Jamming position noise to kinematic truth states within the jamming zone."""
    if not states:
        return []

    active_rng = rng if rng is not None else np.random.default_rng()
    t_start = config.start_time_s
    t_end = (t_start + config.duration_s) if config.duration_s is not None else float("inf")

    out_states: list[AircraftTruthState] = []
    for s in states:
        new_s = copy.copy(s)
        if t_start <= s.timestamp_s < t_end:
            dist_m = haversine_distance(s.lat_deg, s.lon_deg, config.center_lat, config.center_lon)
            if dist_m <= config.radius_m and config.position_noise_sigma_m > 0.0:
                noise_north_m = float(active_rng.normal(0.0, config.position_noise_sigma_m))
                noise_east_m = float(active_rng.normal(0.0, config.position_noise_sigma_m))

                delta_lat_deg = math.degrees(noise_north_m / EARTH_MEAN_RADIUS_M)
                cos_lat = max(math.cos(math.radians(s.lat_deg)), 1e-6)
                delta_lon_deg = math.degrees(noise_east_m / (EARTH_MEAN_RADIUS_M * cos_lat))

                new_s.lat_deg += delta_lat_deg
                new_s.lon_deg += delta_lon_deg
        out_states.append(new_s)

    return out_states


def apply_gps_jamming_fault(
    squitters: list[SquitterEvent],
    config: GPSJammingConfig,
    rng: np.random.Generator | None = None,
) -> list[SquitterEvent]:
    """Apply AV-06 GPS Jamming / Loss of Fix (Integrity drop to TC 18 and scaled noise).

    Gated geographically to aircraft physically located within radius_m of
    (center_lat, center_lon) during [start_time_s, start_time_s + duration_s).
    """
    out_events: list[SquitterEvent] = []
    active_rng = rng if rng is not None else np.random.default_rng()

    t_start = config.start_time_s
    t_end = (t_start + config.duration_s) if config.duration_s is not None else float("inf")

    for ev in squitters:
        if len(ev.raw_bytes) != 14:
            out_events.append(ev)
            continue

        val, total_bits = bytes_to_uint(ev.raw_bytes)
        df = extract_bits(val, total_bits, 1, 5)
        tc = extract_bits(val, total_bits, 33, 37)

        # Check if Airborne Position (DF17/18, TC 9..18)
        if df in (17, 18) and 9 <= tc <= 18:
            if t_start <= ev.timestamp_s < t_end:
                cpr_fmt = extract_bits(val, total_bits, 54, 54)
                cpr_lat = extract_bits(val, total_bits, 55, 71)
                cpr_lon = extract_bits(val, total_bits, 72, 88)

                # Geographic masking: resolve aircraft coordinates relative to jamming center
                try:
                    ac_lat, ac_lon = decode_cpr_local(
                        cpr_lat / 131072.0,
                        cpr_lon / 131072.0,
                        cpr_fmt,
                        ref_lat=config.center_lat,
                        ref_lon=config.center_lon,
                    )
                    dist_m = haversine_distance(ac_lat, ac_lon, config.center_lat, config.center_lon)
                except Exception:
                    dist_m = float("inf")

                if dist_m <= config.radius_m:
                    # Demote Type Code to target_tc (default 18: NIC=0)
                    val = set_bits(val, total_bits, 33, 37, config.target_tc)

                    if config.position_noise_sigma_m > 0.0:
                        noise_north_m = float(active_rng.normal(0.0, config.position_noise_sigma_m))
                        noise_east_m = float(active_rng.normal(0.0, config.position_noise_sigma_m))

                        delta_lat_deg = math.degrees(noise_north_m / EARTH_MEAN_RADIUS_M)
                        cos_lat = max(math.cos(math.radians(ac_lat)), 1e-6)
                        delta_lon_deg = math.degrees(noise_east_m / (EARTH_MEAN_RADIUS_M * cos_lat))

                        noisy_lat = ac_lat + delta_lat_deg
                        noisy_lon = ac_lon + delta_lon_deg

                        new_cpr_lat, new_cpr_lon = encode_cpr_airborne(noisy_lat, noisy_lon, cpr_fmt)
                        val = set_bits(val, total_bits, 55, 71, new_cpr_lat)
                        val = set_bits(val, total_bits, 72, 88, new_cpr_lon)

                    raw_mutated = uint_to_bytes(val, 14)
                    valid_bytes = recompute_frame_crc(raw_mutated)
                    out_events.append(
                        SquitterEvent(
                            timestamp_s=ev.timestamp_s,
                            raw_bytes=valid_bytes,
                            icao24=ev.icao24,
                            rf_freq_hz=ev.rf_freq_hz,
                            tx_power_dbm=ev.tx_power_dbm,
                        )
                    )
                    continue

        out_events.append(ev)

    return out_events


def apply_duplicate_icao_fault(
    squitters: list[SquitterEvent],
    config: DuplicateICAOConfig,
) -> list[SquitterEvent]:
    """Apply AV-07 Duplicate ICAO Address Conflict by overwriting the AA field."""
    out_events: list[SquitterEvent] = []

    for ev in squitters:
        if len(ev.raw_bytes) not in (7, 14):
            out_events.append(ev)
            continue

        val, total_bits = bytes_to_uint(ev.raw_bytes)
        df = extract_bits(val, total_bits, 1, 5)

        # In DF11, DF17, DF18, Bits 9..32 are Address Announced (AA)
        if df in (11, 17, 18):
            val = set_bits(val, total_bits, 9, 32, config.conflict_icao24 & 0xFFFFFF)
            raw_mutated = uint_to_bytes(val, len(ev.raw_bytes))
            valid_bytes = recompute_frame_crc(raw_mutated)
            out_events.append(
                SquitterEvent(
                    timestamp_s=ev.timestamp_s,
                    raw_bytes=valid_bytes,
                    icao24=config.conflict_icao24,
                    rf_freq_hz=ev.rf_freq_hz,
                    tx_power_dbm=ev.tx_power_dbm,
                )
            )
            continue

        out_events.append(ev)

    return out_events


class AvionicsFaultInjector:
    """Unified Stage A avionics fault injection engine."""

    def __init__(
        self,
        config: AvionicsFaultConfig | None = None,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.config = config if config is not None else AvionicsFaultConfig()
        self.rng = rng if rng is not None else np.random.default_rng()

    def apply_to_states(
        self,
        states: list[AircraftTruthState],
    ) -> list[AircraftTruthState]:
        """Apply pre-encoding physical/sensor anomalies to truth states."""
        out_states = list(states)

        if self.config.stale_gps is not None:
            out_states = apply_stale_gps_to_states(out_states, self.config.stale_gps)

        if self.config.altitude_inversion is not None:
            inv = self.config.altitude_inversion
            for s in out_states:
                if inv.swap_baro_geo:
                    s.alt_baro_ft = s.alt_hae_m * METERS_TO_FEET
                if inv.qnh_offset_hpa != 0.0:
                    delta_ft = (1013.25 - inv.qnh_offset_hpa) * 27.3
                    s.alt_baro_ft += delta_ft

        if self.config.gps_jamming is not None:
            out_states = apply_gps_jamming_to_states(
                out_states, self.config.gps_jamming, rng=self.rng
            )

        return out_states

    def apply_to_squitters(
        self,
        squitters: list[SquitterEvent],
    ) -> list[SquitterEvent]:
        """Apply pre-encoding digital transponder faults to emitted squitters."""
        events = list(squitters)

        if self.config.stale_gps is not None:
            events = apply_stale_gps_fault(events, self.config.stale_gps)

        if self.config.cpr_desync is not None:
            events = apply_cpr_desync_fault(events, self.config.cpr_desync)

        if self.config.cpr_straddle is not None:
            events = apply_cpr_straddle_fault(events, self.config.cpr_straddle)

        if self.config.gillham_glitch is not None:
            events = apply_gillham_stuck_bit_fault(events, self.config.gillham_glitch)

        if self.config.altitude_inversion is not None:
            events = apply_altitude_inversion_fault(events, self.config.altitude_inversion)

        if self.config.gps_jamming is not None:
            events = apply_gps_jamming_fault(events, self.config.gps_jamming, rng=self.rng)

        if self.config.duplicate_icao is not None:
            events = apply_duplicate_icao_fault(events, self.config.duplicate_icao)

        return events


__all__ = [
    "GILLHAM_WIRE_BITS",
    "AltitudeInversionConfig",
    "AvionicsFaultConfig",
    "AvionicsFaultInjector",
    "CPRDesyncConfig",
    "CPRDesyncMode",
    "CPRStraddleConfig",
    "DuplicateICAOConfig",
    "GPSJammingConfig",
    "GillhamGlitchConfig",
    "StaleGPSConfig",
    "apply_altitude_inversion_fault",
    "apply_cpr_desync_fault",
    "apply_cpr_straddle_fault",
    "apply_duplicate_icao_fault",
    "apply_gillham_stuck_bit_fault",
    "apply_gps_jamming_fault",
    "apply_gps_jamming_to_states",
    "apply_stale_gps_fault",
    "apply_stale_gps_to_states",
    "recompute_frame_crc",
]
