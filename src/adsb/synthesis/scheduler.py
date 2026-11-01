"""Asynchronous transponder timer engine and DO-260B squitter scheduler.

Authoritative reference: RTCA DO-260B / DO-260C Section 2.2.3,
ICAO Annex 10 Vol IV, and docs/03_FLIGHT_SYNTHESIS_AND_PHYSICS.md Section 4.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from adsb.constants import FEET_TO_METERS
from adsb.protocol.modes import (
    encode_1090es_airborne_position,
    encode_1090es_airborne_velocity,
    encode_1090es_df11,
    encode_1090es_identification,
    encode_1090es_surface_position,
)

if TYPE_CHECKING:
    from adsb.synthesis.performance import AircraftPerformanceProfile
    from adsb.synthesis.scenario import Scenario
    from adsb.synthesis.trajectory import AircraftTruthState


@dataclass(frozen=True, slots=True)
class SquitterEvent:
    """A single RF squitter transmission event."""

    timestamp_s: float
    raw_bytes: bytes
    icao24: int
    rf_freq_hz: int = 1_090_000_000
    tx_power_dbm: float = 51.0
    tx_lat_deg: float | None = None
    tx_lon_deg: float | None = None
    tx_alt_m: float | None = None

    @property
    def timestamp_ns(self) -> int:
        """Timestamp in nanoseconds."""
        return int(self.timestamp_s * 1e9)

    @property
    def tx_lat(self) -> float | None:
        """Alias for tx_lat_deg."""
        return self.tx_lat_deg

    @property
    def tx_lon(self) -> float | None:
        """Alias for tx_lon_deg."""
        return self.tx_lon_deg


def _interpolate_state(
    states: list[AircraftTruthState], t_tx: float
) -> tuple[float, float, float, float, float, float, bool]:
    """Linearly interpolate state attributes at transmission time t_tx.

    Returns:
        tuple: (lat_deg, lon_deg, alt_baro_ft, gs_kt, track_deg, vr_fpm, on_ground)
    """
    if len(states) == 1 or t_tx <= states[0].timestamp_s:
        s0 = states[0]
        return (
            s0.lat_deg,
            s0.lon_deg,
            s0.alt_baro_ft,
            s0.groundspeed_kt,
            s0.true_track_deg,
            s0.vertical_rate_fpm,
            s0.on_ground,
        )
    if t_tx >= states[-1].timestamp_s:
        s_end = states[-1]
        return (
            s_end.lat_deg,
            s_end.lon_deg,
            s_end.alt_baro_ft,
            s_end.groundspeed_kt,
            s_end.true_track_deg,
            s_end.vertical_rate_fpm,
            s_end.on_ground,
        )

    # Binary search for interval
    low = 0
    high = len(states) - 1
    while low < high - 1:
        mid = (low + high) // 2
        if states[mid].timestamp_s <= t_tx:
            low = mid
        else:
            high = mid

    s_k = states[low]
    s_kp1 = states[high]
    dt = s_kp1.timestamp_s - s_k.timestamp_s
    alpha = (t_tx - s_k.timestamp_s) / dt if dt > 1e-9 else 0.0

    lat_deg = s_k.lat_deg + alpha * (s_kp1.lat_deg - s_k.lat_deg)

    # Longitude circular interpolation across anti-meridian
    d_lon = ((s_kp1.lon_deg - s_k.lon_deg + 180.0) % 360.0) - 180.0
    lon_deg = ((s_k.lon_deg + alpha * d_lon + 180.0) % 360.0) - 180.0

    alt_baro_ft = s_k.alt_baro_ft + alpha * (s_kp1.alt_baro_ft - s_k.alt_baro_ft)
    gs_kt = s_k.groundspeed_kt + alpha * (s_kp1.groundspeed_kt - s_k.groundspeed_kt)

    # Track circular interpolation
    d_track = ((s_kp1.true_track_deg - s_k.true_track_deg + 180.0) % 360.0) - 180.0
    track_deg = (s_k.true_track_deg + alpha * d_track) % 360.0

    vr_fpm = s_k.vertical_rate_fpm + alpha * (s_kp1.vertical_rate_fpm - s_k.vertical_rate_fpm)
    on_ground = s_k.on_ground if alpha < 0.5 else s_kp1.on_ground

    return lat_deg, lon_deg, alt_baro_ft, gs_kt, track_deg, vr_fpm, on_ground


class SquitterScheduler:
    """Transponder event scheduler emitting DO-260B squitters with uniform random jitter."""

    def __init__(
        self,
        icao24: int,
        callsign: str,
        rng: np.random.Generator,
        profile: AircraftPerformanceProfile | None = None,
        include_df11: bool = False,
        enable_uat: bool = False,
    ) -> None:
        self.icao24 = icao24
        self.callsign = callsign.strip().upper()
        self.rng = rng
        self.profile = profile
        self.include_df11 = include_df11
        self.enable_uat = enable_uat

    def generate_squitters(self, states: list[AircraftTruthState]) -> list[SquitterEvent]:
        """Generate chronologically sorted squitter transmission events over states duration."""
        if not states:
            return []

        t_start = states[0].timestamp_s
        t_end = states[-1].timestamp_s
        if t_end < t_start:
            return []

        events: list[SquitterEvent] = []

        # Position squitter timer (TC 11 airborne or TC 8 surface): 0.5s +/- 0.1s jitter
        cpr_format = 0  # Strictly alternating CPR (0=Even, 1=Odd)
        t_pos = t_start + float(self.rng.uniform(0.0, 0.2))

        while t_pos <= t_end:
            lat_deg, lon_deg, alt_ft, gs_kt, track_deg, _, on_ground = _interpolate_state(
                states, t_pos
            )

            if on_ground:
                raw_bytes = encode_1090es_surface_position(
                    icao24=self.icao24,
                    lat_deg=lat_deg,
                    lon_deg=lon_deg,
                    cpr_format=cpr_format,
                    groundspeed_kt=gs_kt,
                    ground_track_deg=track_deg,
                )
                interval = (
                    float(self.rng.uniform(0.40, 0.60))
                    if gs_kt > 0.5
                    else float(self.rng.uniform(4.80, 5.20))
                )
            else:
                raw_bytes = encode_1090es_airborne_position(
                    icao24=self.icao24,
                    altitude_ft=round(alt_ft),
                    lat_deg=lat_deg,
                    lon_deg=lon_deg,
                    cpr_format=cpr_format,
                )
                interval = float(self.rng.uniform(0.40, 0.60))

            events.append(
                SquitterEvent(
                    timestamp_s=t_pos,
                    raw_bytes=raw_bytes,
                    icao24=self.icao24,
                    tx_lat_deg=lat_deg,
                    tx_lon_deg=lon_deg,
                    tx_alt_m=alt_ft * FEET_TO_METERS,
                )
            )

            cpr_format = 1 - cpr_format
            t_pos += interval

        # Velocity squitter timer (TC 19 Subtype 1): 0.5s +/- 0.1s jitter
        t_vel = t_start + float(self.rng.uniform(0.1, 0.3))

        while t_vel <= t_end:
            lat_deg, lon_deg, alt_ft, gs_kt, track_deg, vr_fpm, on_ground = _interpolate_state(
                states, t_vel
            )

            if not on_ground:
                track_rad = math.radians(track_deg)
                v_ew_kt = gs_kt * math.sin(track_rad)
                v_ns_kt = gs_kt * math.cos(track_rad)

                raw_bytes = encode_1090es_airborne_velocity(
                    icao24=self.icao24,
                    v_ew_kt=v_ew_kt,
                    v_ns_kt=v_ns_kt,
                    vertical_rate_fpm=round(vr_fpm),
                )
                events.append(
                    SquitterEvent(
                        timestamp_s=t_vel,
                        raw_bytes=raw_bytes,
                        icao24=self.icao24,
                        tx_lat_deg=lat_deg,
                        tx_lon_deg=lon_deg,
                        tx_alt_m=alt_ft * FEET_TO_METERS,
                    )
                )

            t_vel += float(self.rng.uniform(0.40, 0.60))

        # Identification squitter timer (TC 4): 5.0s +/- 0.2s jitter
        t_id = t_start + float(self.rng.uniform(0.0, 0.5))

        while t_id <= t_end:
            lat_deg, lon_deg, alt_ft, gs_kt, _, _, on_ground = _interpolate_state(
                states, t_id
            )
            category_int = 0

            raw_bytes = encode_1090es_identification(
                icao24=self.icao24,
                callsign=self.callsign,
                category=category_int,
            )
            events.append(
                SquitterEvent(
                    timestamp_s=t_id,
                    raw_bytes=raw_bytes,
                    icao24=self.icao24,
                    tx_lat_deg=lat_deg,
                    tx_lon_deg=lon_deg,
                    tx_alt_m=alt_ft * FEET_TO_METERS,
                )
            )

            if on_ground and gs_kt <= 0.5:
                interval = float(self.rng.uniform(9.60, 10.40))
            else:
                interval = float(self.rng.uniform(4.80, 5.20))
            t_id += interval

        # Acquisition Squitter (DF11): 1.0s +/- 0.2s jitter (if enabled)
        if self.include_df11:
            t_df11 = t_start + float(self.rng.uniform(0.0, 1.0))
            while t_df11 <= t_end:
                lat_deg, lon_deg, alt_ft, _, _, _, _ = _interpolate_state(
                    states, t_df11
                )
                raw_bytes = encode_1090es_df11(icao24=self.icao24)
                events.append(
                    SquitterEvent(
                        timestamp_s=t_df11,
                        raw_bytes=raw_bytes,
                        icao24=self.icao24,
                        tx_lat_deg=lat_deg,
                        tx_lon_deg=lon_deg,
                        tx_alt_m=alt_ft * FEET_TO_METERS,
                    )
                )
                t_df11 += float(self.rng.uniform(0.80, 1.20))

        # Sort all events chronologically
        events.sort(key=lambda ev: ev.timestamp_s)
        return events


class MultiAircraftScheduler:
    """Orchestrator scheduling squitter events across multiple aircraft in a scenario."""

    def __init__(self, scenario: Scenario, rng: np.random.Generator) -> None:
        self.scenario = scenario
        self.rng = rng
        self.schedulers = [
            SquitterScheduler(
                icao24=ac.icao24,
                callsign=ac.callsign,
                rng=rng,
                profile=ac.profile,
            )
            for ac in scenario.aircraft
        ]

    def generate_all_events(self) -> list[SquitterEvent]:
        """Generate and emit a single time-ordered stream of all squitter events."""
        all_events: list[SquitterEvent] = []
        for ac, scheduler in zip(self.scenario.aircraft, self.schedulers):
            events = scheduler.generate_squitters(ac.states)
            all_events.extend(events)

        all_events.sort(key=lambda ev: ev.timestamp_s)
        return all_events


__all__ = [
    "MultiAircraftScheduler",
    "SquitterEvent",
    "SquitterScheduler",
]
