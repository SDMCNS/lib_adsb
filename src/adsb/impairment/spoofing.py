"""Adversarial spoofing and electronic warfare squitter injection generators.

Authoritative reference: docs/04_ERROR_AND_ANOMALY_SIMULATION.md Section 4
and tests/integration/test_e2e_closed_loop.py lines 369-386.
"""

from __future__ import annotations

import contextlib
import math

import numpy as np

from adsb.core.bits import bytes_to_uint, extract_bits, set_bits, uint_to_bytes
from adsb.core.geodesy import destination_point, haversine_distance, initial_bearing
from adsb.core.units import MPS_TO_KNOTS
from adsb.impairment.avionics import recompute_frame_crc
from adsb.protocol.cpr import (
    decode_cpr_global_airborne,
    decode_cpr_local,
    encode_cpr_airborne,
)
from adsb.protocol.gillham import decode_gillham_altitude, encode_gillham_altitude
from adsb.protocol.modes import (
    decode_mode_s,
    encode_1090es_airborne_position,
    encode_1090es_airborne_velocity,
    encode_1090es_df11,
    encode_1090es_identification,
)
from adsb.synthesis.scheduler import SquitterEvent


class GhostTrackInjector:
    """Injects a high-speed synthetic phantom squitter stream.

    Supports generating physically impossible kinematic trajectories (e.g. 800 m/s)
    to trigger downstream tracker anomaly detectors such as 'KINEMATIC_TELEPORT'
    or 'RF_PHYSICAL_SPOOF'.
    """

    def __init__(
        self,
        target_icao: int = 0xFA11ED,
        speed_mps: float = 800.0,
        origin_lat: float = 52.3105,
        origin_lon: float = 4.7683,
        origin_alt_ft: float = 10000.0,
        track_deg: float = 90.0,
        callsign: str = "GHOST01",
        tx_power_dbm: float = 51.0,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.target_icao = target_icao
        self.speed_mps = speed_mps
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt_ft = origin_alt_ft
        self.track_deg = track_deg
        self.callsign = callsign
        self.tx_power_dbm = tx_power_dbm
        self.rng = rng if rng is not None else np.random.default_rng()

    def generate_ghost_squitters(
        self,
        duration_s: float = 60.0,
        start_time_s: float = 0.0,
    ) -> list[SquitterEvent]:
        """Generate time-ordered Mode S squitters for the ghost flight."""
        events: list[SquitterEvent] = []
        if duration_s <= 0.0:
            return events

        track_rad = math.radians(self.track_deg)
        speed_kt = self.speed_mps * MPS_TO_KNOTS
        v_ew_kt = speed_kt * math.sin(track_rad)
        v_ns_kt = speed_kt * math.cos(track_rad)

        # 1. Identification squitters every 5.0 seconds
        id_bytes = encode_1090es_identification(self.target_icao, self.callsign)
        t_id = start_time_s
        while t_id < start_time_s + duration_s:
            events.append(
                SquitterEvent(
                    timestamp_s=t_id,
                    raw_bytes=id_bytes,
                    icao24=self.target_icao,
                    tx_power_dbm=self.tx_power_dbm,
                )
            )
            t_id += 5.0

        # 2. Airborne velocity squitters at 2 Hz
        vel_bytes = encode_1090es_airborne_velocity(
            icao24=self.target_icao,
            v_ew_kt=v_ew_kt,
            v_ns_kt=v_ns_kt,
            vertical_rate_fpm=0,
        )
        t_vel = start_time_s + 0.1
        while t_vel < start_time_s + duration_s:
            events.append(
                SquitterEvent(
                    timestamp_s=t_vel,
                    raw_bytes=vel_bytes,
                    icao24=self.target_icao,
                    tx_power_dbm=self.tx_power_dbm,
                )
            )
            t_vel += 0.5

        # 3. Airborne position squitters at 2 Hz, alternating Even (0) and Odd (1)
        t_pos = start_time_s + 0.25
        cpr_toggle = 0
        while t_pos < start_time_s + duration_s:
            dt = t_pos - start_time_s
            dist_m = self.speed_mps * dt

            # Geodetic displacement
            d_lat = (dist_m * math.cos(track_rad)) / 111_139.0
            d_lon = (dist_m * math.sin(track_rad)) / (
                111_139.0 * math.cos(math.radians(self.origin_lat))
            )
            lat = self.origin_lat + d_lat
            lon = self.origin_lon + d_lon

            pos_bytes = encode_1090es_airborne_position(
                icao24=self.target_icao,
                altitude_ft=int(self.origin_alt_ft),
                lat_deg=lat,
                lon_deg=lon,
                cpr_format=cpr_toggle,
            )
            events.append(
                SquitterEvent(
                    timestamp_s=t_pos,
                    raw_bytes=pos_bytes,
                    icao24=self.target_icao,
                    tx_power_dbm=self.tx_power_dbm,
                )
            )

            cpr_toggle = 1 - cpr_toggle
            t_pos += 0.5

        events.sort(key=lambda e: e.timestamp_s)
        return events


class ShadowTrackHijacker:
    """Clones a legitimate aircraft's squitter stream with higher power and diverging kinematics."""

    def __init__(
        self,
        target_icao: int,
        divergence_track_deg: float = 15.0,
        alt_rate_fpm: float = 2000.0,
        tx_power_dbm: float = 57.0,  # +6 dB capture advantage
        rng: np.random.Generator | None = None,
    ) -> None:
        self.target_icao = target_icao
        self.divergence_track_deg = divergence_track_deg
        self.alt_rate_fpm = alt_rate_fpm
        self.tx_power_dbm = tx_power_dbm
        self.rng = rng if rng is not None else np.random.default_rng()

    def generate_shadow_squitters(
        self,
        base_events: list[SquitterEvent],
        divergence_start_s: float = 10.0,
    ) -> list[SquitterEvent]:
        """Produce diverging shadow squitters from base events."""
        shadow_events: list[SquitterEvent] = []
        if not base_events:
            return shadow_events

        # 1. Establish reference position from first Even/Odd pair of target_icao
        even_cpr: tuple[float, float] | None = None
        odd_cpr: tuple[float, float] | None = None

        for ev in base_events:
            if ev.icao24 == self.target_icao and len(ev.raw_bytes) == 14:
                val, tb = bytes_to_uint(ev.raw_bytes)
                df = extract_bits(val, tb, 1, 5)
                tc = extract_bits(val, tb, 33, 37)
                if df in (17, 18) and (9 <= tc <= 18 or 20 <= tc <= 22):
                    fmt = extract_bits(val, tb, 54, 54)
                    c_lat = extract_bits(val, tb, 55, 71) / 131072.0
                    c_lon = extract_bits(val, tb, 72, 88) / 131072.0
                    if fmt == 0 and even_cpr is None:
                        even_cpr = (c_lat, c_lon)
                    elif fmt == 1 and odd_cpr is None:
                        odd_cpr = (c_lat, c_lon)
                    if even_cpr is not None and odd_cpr is not None:
                        break

        ref_lat: float | None = None
        ref_lon: float | None = None
        if even_cpr is not None and odd_cpr is not None:
            with contextlib.suppress(ValueError):
                ref_lat, ref_lon = decode_cpr_global_airborne(
                    even_cpr[0], even_cpr[1], odd_cpr[0], odd_cpr[1], most_recent=1
                )

        # 2. Track base kinematics (speed, track, position)
        speed_mps: float = 200.0  # nominal default ~388 kt
        base_track_deg: float = 0.0
        prev_pos: tuple[float, float, float] | None = None  # (t, lat, lon)

        # Pre-scan for initial kinematics if TC 19 or multiple positions exist
        for ev in base_events:
            if ev.icao24 != self.target_icao:
                continue
            if len(ev.raw_bytes) == 14:
                val, tb = bytes_to_uint(ev.raw_bytes)
                df = extract_bits(val, tb, 1, 5)
                tc = extract_bits(val, tb, 33, 37)
                if df in (17, 18) and tc == 19:
                    m = decode_mode_s(ev.raw_bytes)
                    if m.groundspeed_kt is not None and m.ground_track_deg is not None:
                        speed_mps = m.groundspeed_kt / MPS_TO_KNOTS
                        base_track_deg = m.ground_track_deg
                        break
                elif (
                    df in (17, 18)
                    and (9 <= tc <= 18 or 20 <= tc <= 22)
                    and ref_lat is not None
                    and ref_lon is not None
                ):
                    fmt = extract_bits(val, tb, 54, 54)
                    c_lat = extract_bits(val, tb, 55, 71) / 131072.0
                    c_lon = extract_bits(val, tb, 72, 88) / 131072.0
                    plat, plon = decode_cpr_local(c_lat, c_lon, fmt, ref_lat, ref_lon)
                    ref_lat, ref_lon = plat, plon
                    if prev_pos is None:
                        prev_pos = (ev.timestamp_s, plat, plon)
                    else:
                        dt = ev.timestamp_s - prev_pos[0]
                        if dt >= 0.5:
                            dist = haversine_distance(prev_pos[1], prev_pos[2], plat, plon)
                            speed_mps = dist / dt
                            base_track_deg = initial_bearing(prev_pos[1], prev_pos[2], plat, plon)
                            break

        shadow_track_deg = (base_track_deg + self.divergence_track_deg) % 360.0
        cur_ref_lat = ref_lat
        cur_ref_lon = ref_lon

        # 3. Process events and apply divergence
        for ev in base_events:
            if ev.icao24 != self.target_icao or ev.timestamp_s < divergence_start_s:
                continue

            dt = ev.timestamp_s - divergence_start_s
            raw_out = ev.raw_bytes

            if len(ev.raw_bytes) == 14:
                val, tb = bytes_to_uint(ev.raw_bytes)
                df = extract_bits(val, tb, 1, 5)
                tc = extract_bits(val, tb, 33, 37)

                # Airborne Position frame: diverge position and altitude
                if df in (17, 18) and (9 <= tc <= 18 or 20 <= tc <= 22):
                    fmt = extract_bits(val, tb, 54, 54)
                    c_lat = extract_bits(val, tb, 55, 71) / 131072.0
                    c_lon = extract_bits(val, tb, 72, 88) / 131072.0
                    alt_code = extract_bits(val, tb, 41, 52)
                    base_alt = decode_gillham_altitude(alt_code)

                    if cur_ref_lat is not None and cur_ref_lon is not None:
                        base_lat, base_lon = decode_cpr_local(
                            c_lat, c_lon, fmt, cur_ref_lat, cur_ref_lon
                        )
                        cur_ref_lat, cur_ref_lon = base_lat, base_lon

                        # Lateral divergence projection
                        rad_base = math.radians(base_track_deg)
                        rad_shadow = math.radians(shadow_track_deg)
                        d_veast = speed_mps * (math.sin(rad_shadow) - math.sin(rad_base))
                        d_vnorth = speed_mps * (math.cos(rad_shadow) - math.cos(rad_base))
                        dx_m = d_veast * dt
                        dy_m = d_vnorth * dt
                        d_lat = dy_m / 111_139.0
                        cos_lat = max(math.cos(math.radians(base_lat)), 1e-6)
                        d_lon = dx_m / (111_139.0 * cos_lat)

                        shadow_lat = max(-90.0, min(90.0, base_lat + d_lat))
                        shadow_lon = (base_lon + d_lon + 180.0) % 360.0 - 180.0

                        # Vertical divergence projection
                        d_alt_ft = self.alt_rate_fpm * (dt / 60.0)
                        shadow_alt = round(base_alt + d_alt_ft) if base_alt is not None else 10000

                        # Re-encode CPR and altitude
                        new_c_lat, new_c_lon = encode_cpr_airborne(shadow_lat, shadow_lon, fmt)
                        new_alt_code = encode_gillham_altitude(shadow_alt, resolution_25ft=True)

                        new_val = set_bits(val, tb, 41, 52, new_alt_code)
                        new_val = set_bits(new_val, tb, 55, 71, new_c_lat)
                        new_val = set_bits(new_val, tb, 72, 88, new_c_lon)
                        raw_out = recompute_frame_crc(uint_to_bytes(new_val, 14))

                # Airborne Velocity frame: diverge velocity vector
                elif df in (17, 18) and tc == 19:
                    speed_kt = speed_mps * MPS_TO_KNOTS
                    rad_shadow = math.radians(shadow_track_deg)
                    v_ew_shad = speed_kt * math.sin(rad_shadow)
                    v_ns_shad = speed_kt * math.cos(rad_shadow)
                    vr_shad = round(self.alt_rate_fpm)
                    raw_out = encode_1090es_airborne_velocity(
                        icao24=ev.icao24,
                        v_ew_kt=v_ew_shad,
                        v_ns_kt=v_ns_shad,
                        vertical_rate_fpm=vr_shad,
                    )

            shadow_events.append(
                SquitterEvent(
                    timestamp_s=ev.timestamp_s + 0.001,
                    raw_bytes=raw_out,
                    icao24=ev.icao24,
                    rf_freq_hz=ev.rf_freq_hz,
                    tx_power_dbm=self.tx_power_dbm,
                )
            )

        return shadow_events


class FloodDenialInjector:
    """Floods receiver and tracker with high-density synthetic squitters.

    Modes:
    1. RF Channel / Receiver Saturation (emit_positions=False, default):
       Emits high-rate 56-bit DF11 All-Call frames (7 bytes) to saturate receiver
       preamble detection, channel ALOHA capacity, and MLAT buffers. The physical
       jammer transmitter is geographically sited at (center_lat, center_lon).
    2. Track Table Exhaustion (emit_positions=True):
       Emits 112-bit DF17 Airborne Position frames (14 bytes) for targets arranged
       in a geometric ring around (center_lat, center_lon) to exhaust tracker state
       association and Kalman filter tracking capacity.
    """

    def __init__(
        self,
        num_targets: int = 50,
        burst_rate_hz: float = 500.0,
        center_lat: float = 52.3105,
        center_lon: float = 4.7683,
        radius_m: float = 15000.0,
        emit_positions: bool = False,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.num_targets = num_targets
        self.burst_rate_hz = burst_rate_hz
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.radius_m = radius_m
        self.emit_positions = emit_positions
        self.rng = rng if rng is not None else np.random.default_rng()

    def generate_flood(
        self,
        duration_s: float = 10.0,
        start_time_s: float = 0.0,
        emit_positions: bool | None = None,
    ) -> list[SquitterEvent]:
        """Generate high-density flood squitters.

        Parameters:
            duration_s: Duration of flood burst in seconds.
            start_time_s: Initial timestamp of flood sequence.
            emit_positions: If True, emits DF17 Airborne Position squitters in a ring
                around (center_lat, center_lon). If False, emits DF11 All-Call replies.
                Defaults to self.emit_positions.
        """
        events: list[SquitterEvent] = []
        if duration_s <= 0.0 or self.burst_rate_hz <= 0.0:
            return events

        use_positions = emit_positions if emit_positions is not None else self.emit_positions

        # Generate unique pool of fake ICAOs
        icaos = [
            int(self.rng.integers(0x100000, 0xEFFFFF))
            for _ in range(self.num_targets)
        ]

        # Precompute geometric ring coordinates if position mode enabled
        target_coords: list[tuple[float, float, int]] = []
        if use_positions:
            for i in range(self.num_targets):
                bearing = (360.0 * i) / max(self.num_targets, 1)
                t_lat, t_lon = destination_point(self.center_lat, self.center_lon, self.radius_m, bearing)
                alt_ft = 5000 + (i * 200) % 25000
                target_coords.append((t_lat, t_lon, alt_ft))

        total_events = round(duration_s * self.burst_rate_hz)
        dt = 1.0 / self.burst_rate_hz
        cpr_toggle: dict[int, int] = dict.fromkeys(icaos, 0)

        for idx in range(total_events):
            t = start_time_s + idx * dt
            t_idx = idx % len(icaos)
            icao = icaos[t_idx]

            if use_positions:
                t_lat, t_lon, alt_ft = target_coords[t_idx]
                cpr_fmt = cpr_toggle[icao]
                cpr_toggle[icao] = 1 - cpr_fmt
                raw_bytes = encode_1090es_airborne_position(icao, alt_ft, t_lat, t_lon, cpr_fmt)
            else:
                raw_bytes = encode_1090es_df11(icao)

            events.append(
                SquitterEvent(
                    timestamp_s=t,
                    raw_bytes=raw_bytes,
                    icao24=icao,
                )
            )

        return events


class ReplayInjector:
    """Retransmits previously recorded squitter sequences with time/spatial offsets."""

    def __init__(
        self,
        delay_s: float = 30.0,
        delta_lat_deg: float = 0.0,
        delta_lon_deg: float = 0.0,
        target_icao: int | None = None,
    ) -> None:
        self.delay_s = delay_s
        self.delta_lat_deg = delta_lat_deg
        self.delta_lon_deg = delta_lon_deg
        self.target_icao = target_icao

    def replay(
        self,
        events: list[SquitterEvent],
    ) -> list[SquitterEvent]:
        """Replay events shifted in time by delay_s and translated spatially."""
        replayed: list[SquitterEvent] = []
        if not events:
            return replayed

        has_spatial_offset = (self.delta_lat_deg != 0.0 or self.delta_lon_deg != 0.0)

        # Pre-resolve reference position per ICAO if spatial translation is active
        ref_positions: dict[int, tuple[float, float]] = {}
        if has_spatial_offset:
            even_pairs: dict[int, tuple[float, float]] = {}
            odd_pairs: dict[int, tuple[float, float]] = {}
            for ev in events:
                if self.target_icao is not None and ev.icao24 != self.target_icao:
                    continue
                if len(ev.raw_bytes) == 14:
                    val, tb = bytes_to_uint(ev.raw_bytes)
                    df = extract_bits(val, tb, 1, 5)
                    tc = extract_bits(val, tb, 33, 37)
                    if df in (17, 18) and (9 <= tc <= 18 or 20 <= tc <= 22):
                        fmt = extract_bits(val, tb, 54, 54)
                        c_lat = extract_bits(val, tb, 55, 71) / 131072.0
                        c_lon = extract_bits(val, tb, 72, 88) / 131072.0
                        if fmt == 0 and ev.icao24 not in even_pairs:
                            even_pairs[ev.icao24] = (c_lat, c_lon)
                        elif fmt == 1 and ev.icao24 not in odd_pairs:
                            odd_pairs[ev.icao24] = (c_lat, c_lon)

            for icao in set(even_pairs.keys()) & set(odd_pairs.keys()):
                with contextlib.suppress(ValueError):
                    ref_positions[icao] = decode_cpr_global_airborne(
                        even_pairs[icao][0],
                        even_pairs[icao][1],
                        odd_pairs[icao][0],
                        odd_pairs[icao][1],
                        most_recent=1,
                    )

        # Replay events
        cur_refs = dict(ref_positions)
        for ev in events:
            if self.target_icao is not None and ev.icao24 != self.target_icao:
                continue

            raw_out = ev.raw_bytes

            if has_spatial_offset and len(ev.raw_bytes) == 14:
                val, tb = bytes_to_uint(ev.raw_bytes)
                df = extract_bits(val, tb, 1, 5)
                tc = extract_bits(val, tb, 33, 37)

                if df in (17, 18) and (9 <= tc <= 18 or 20 <= tc <= 22):
                    fmt = extract_bits(val, tb, 54, 54)
                    c_lat = extract_bits(val, tb, 55, 71) / 131072.0
                    c_lon = extract_bits(val, tb, 72, 88) / 131072.0

                    ref_pos = cur_refs.get(ev.icao24)
                    if ref_pos is not None:
                        orig_lat, orig_lon = decode_cpr_local(
                            c_lat, c_lon, fmt, ref_pos[0], ref_pos[1]
                        )
                        cur_refs[ev.icao24] = (orig_lat, orig_lon)

                        trans_lat = max(-90.0, min(90.0, orig_lat + self.delta_lat_deg))
                        trans_lon = (
                            (orig_lon + self.delta_lon_deg + 180.0) % 360.0 - 180.0
                        )

                        new_c_lat, new_c_lon = encode_cpr_airborne(trans_lat, trans_lon, fmt)
                        new_val = set_bits(val, tb, 55, 71, new_c_lat)
                        new_val = set_bits(new_val, tb, 72, 88, new_c_lon)
                        raw_out = recompute_frame_crc(uint_to_bytes(new_val, 14))

            replayed.append(
                SquitterEvent(
                    timestamp_s=ev.timestamp_s + self.delay_s,
                    raw_bytes=raw_out,
                    icao24=ev.icao24,
                    rf_freq_hz=ev.rf_freq_hz,
                    tx_power_dbm=ev.tx_power_dbm,
                )
            )

        return replayed


__all__ = [
    "FloodDenialInjector",
    "GhostTrackInjector",
    "ReplayInjector",
    "ShadowTrackHijacker",
]
