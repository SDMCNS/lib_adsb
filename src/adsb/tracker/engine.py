"""Multi-target stateful tracking engine.

Ingests decoded Mode S / ADS-B / UAT messages, manages track lifecycle,
coordinates stateful CPR resolution, estimates smoothed kinematic state
vectors via 7-state local ENU IMM Kalman filtering, and detects avionics
and physical RF anomalies.

Authoritative reference: docs/05_TRACKING_AND_STATE_ESTIMATION.md Section 1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from adsb.core.coordinates import enu_to_geodetic, geodetic_to_enu
from adsb.core.geodesy import haversine_distance
from adsb.core.units import FEET_TO_METERS, FPM_TO_MPS, KNOTS_TO_MPS
from adsb.protocol.modes import DecodedModeSMessage
from adsb.tracker.anomaly import AnomalyDetector
from adsb.tracker.imm_filter import IMMEstimator
from adsb.tracker.lifecycle import TrackLifecycleManager, TrackState
from adsb.tracker.resolver import StatefulCPRResolver


@dataclass(slots=True)
class Track:
    """Stateful tracking context for an individual aircraft."""

    icao24: int
    callsign: str = ""
    state: TrackState = TrackState.TENTATIVE
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_baro_ft: float | None = None
    alt_geom_ft: float | None = None
    groundspeed_kt: float | None = None
    track_deg: float | None = None
    vertical_rate_fpm: float | None = None
    mode_probability_turn: float = 0.1
    anomalies: list[str] = field(default_factory=list)
    last_seen_s: float = 0.0
    has_position: bool = False


class TrackingEngine:
    """Multi-target tracking pipeline maintaining stateful tracking context per ICAO."""

    def __init__(
        self,
        ref_lat: float,
        ref_lon: float,
        ref_alt_m: float = 0.0,
    ) -> None:
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_alt_m = ref_alt_m

        self.cpr_resolver = StatefulCPRResolver(
            station_ref_lat=ref_lat,
            station_ref_lon=ref_lon,
        )
        self.anomaly_detector = AnomalyDetector()

        self._tracks: dict[int, Track] = {}
        self._lifecycles: dict[int, TrackLifecycleManager] = {}
        self._filters: dict[int, IMMEstimator] = {}
        self._current_time_s: float = 0.0

    def compute_ground_distance_m(
        self,
        lat1: float,
        lon1: float,
        lat2: float,
        lon2: float,
    ) -> float:
        """Compute great-circle ground distance between two points in meters."""
        return haversine_distance(lat1, lon1, lat2, lon2)

    def get_track(self, icao: int) -> Track | None:
        """Retrieve tracking context for specified ICAO address, if active."""
        track = self._tracks.get(icao)
        if track and track.state != TrackState.DELETED:
            return track
        return None

    def step_time(self, current_time_s: float) -> None:
        """Advance tracking engine clock and evaluate track timeouts across all active tracks.

        Enables periodic maintenance sweeps independently of packet arrivals.

        Parameters:
            current_time_s: Current simulation timestamp in seconds.
        """
        self._current_time_s = max(self._current_time_s, current_time_s)
        for icao, track in list(self._tracks.items()):
            lifecycle = self._lifecycles.get(icao)
            if lifecycle is not None:
                track.state = lifecycle.evaluate_timeouts(current_time_s)

    def all_tracks(self, include_tentative: bool = False) -> list[Track]:
        """Return list of active tracks.

        Parameters:
            include_tentative: If True, includes newly acquired tentative tracks.
                Defaults to False (returns CONFIRMED and COASTING tracks only).
        """
        active: list[Track] = []
        for t in self._tracks.values():
            if t.state == TrackState.DELETED:
                continue
            if not include_tentative and t.state == TrackState.TENTATIVE:
                continue
            active.append(t)
        return active

    def update(
        self,
        rx_timestamp_s: float,
        decoded_msg: DecodedModeSMessage,
        rssi_dbfs: float = 0.0,
    ) -> None:
        """Ingest a decoded Mode S message and update the corresponding aircraft track."""
        if not decoded_msg.crc_valid or decoded_msg.icao24 == 0:
            return

        self._current_time_s = max(self._current_time_s, rx_timestamp_s)
        icao = decoded_msg.icao24

        # Track creation
        if icao not in self._tracks:
            self._tracks[icao] = Track(icao24=icao, last_seen_s=rx_timestamp_s)
            self._lifecycles[icao] = TrackLifecycleManager()
            self._filters[icao] = IMMEstimator()

        track = self._tracks[icao]
        lifecycle = self._lifecycles[icao]
        imm = self._filters[icao]

        tc = decoded_msg.tc
        df = decoded_msg.df

        # 1. Identification (TC 1..4)
        if df in (17, 18) and 1 <= tc <= 4 and decoded_msg.callsign:
            track.callsign = decoded_msg.callsign

        # 2. Airborne / Surface Position (TC 5..8 surface, 9..18 baro, 20..22 geom)
        is_pos_frame = (df in (17, 18)) and ((5 <= tc <= 8) or (9 <= tc <= 18) or (20 <= tc <= 22))

        if is_pos_frame:
            on_ground = 5 <= tc <= 8

            # Check altitude updates and altitude anomalies (Gillham stuck bit / cliff)
            if decoded_msg.altitude_ft is not None:
                alt_ft = float(decoded_msg.altitude_ft)
                alt_anoms = self.anomaly_detector.check_altitude_anomalies(
                    icao, alt_ft, rx_timestamp_s, track.vertical_rate_fpm
                )
                for a in alt_anoms:
                    if a not in track.anomalies:
                        track.anomalies.append(a)

                # Only accept altitude if not an anomalous cliff jump
                if "ALTITUDE_CLIFF_GILLHAM" not in alt_anoms:
                    if 9 <= tc <= 18 or df in (0, 4, 16, 20):
                        track.alt_baro_ft = decoded_msg.altitude_ft
                    elif 20 <= tc <= 22:
                        track.alt_geom_ft = decoded_msg.altitude_ft

            # Resolve CPR position
            pos = self.cpr_resolver.resolve_position(
                icao24=icao,
                cpr_lat=decoded_msg.cpr_lat,
                cpr_lon=decoded_msg.cpr_lon,
                cpr_format=decoded_msg.cpr_format,
                rx_timestamp_s=rx_timestamp_s,
                on_ground=on_ground,
                track_ref_lat=track.lat_deg if track.has_position else None,
                track_ref_lon=track.lon_deg if track.has_position else None,
            )

            if pos is not None:
                lat, lon = pos
                track.has_position = True

                # Check position anomalies (stale GPS, kinematic teleport, zone straddles)
                zone_straddle = self.cpr_resolver.get_cpr_state(icao).zone_straddle_detected
                pos_anoms = self.anomaly_detector.check_position_anomalies(
                    icao24=icao,
                    lat_deg=lat,
                    lon_deg=lon,
                    timestamp_s=rx_timestamp_s,
                    cpr_lat=decoded_msg.cpr_lat,
                    cpr_lon=decoded_msg.cpr_lon,
                    groundspeed_kt=track.groundspeed_kt,
                    zone_straddle=zone_straddle,
                )
                for a in pos_anoms:
                    if a not in track.anomalies:
                        track.anomalies.append(a)

                # Determine current altitude in meters
                alt_m = 0.0
                if track.alt_baro_ft is not None:
                    alt_m = track.alt_baro_ft * FEET_TO_METERS
                elif track.alt_geom_ft is not None:
                    alt_m = track.alt_geom_ft * FEET_TO_METERS

                e_m, n_m, u_m = geodetic_to_enu(
                    lat, lon, alt_m, self.ref_lat, self.ref_lon, self.ref_alt_m
                )

                if imm.last_update_time_s is None:
                    # Initial position fix
                    ve0 = 0.0
                    vn0 = 0.0
                    vu0 = 0.0
                    if track.groundspeed_kt is not None and track.track_deg is not None:
                        spd_mps = track.groundspeed_kt * KNOTS_TO_MPS
                        rad = math.radians(track.track_deg)
                        ve0 = spd_mps * math.sin(rad)
                        vn0 = spd_mps * math.cos(rad)
                    if track.vertical_rate_fpm is not None:
                        vu0 = track.vertical_rate_fpm * FPM_TO_MPS

                    imm.x1[:6] = [e_m, n_m, u_m, ve0, vn0, vu0]
                    imm.x2[:6] = [e_m, n_m, u_m, ve0, vn0, vu0]
                    imm.last_update_time_s = rx_timestamp_s
                    track.lat_deg = lat
                    track.lon_deg = lon
                else:
                    dt = rx_timestamp_s - imm.last_update_time_s
                    if dt > 0.0:
                        imm.predict(dt)
                        imm.last_update_time_s = rx_timestamp_s
                    imm.update_position(e_m, n_m, u_m)

                    # Extract smoothed coordinates from filter
                    st = imm.state
                    lat_s, lon_s, _ = enu_to_geodetic(
                        st[0], st[1], st[2], self.ref_lat, self.ref_lon, self.ref_alt_m
                    )
                    track.lat_deg = lat_s
                    track.lon_deg = lon_s

                track.mode_probability_turn = imm.mode_probability_turn

        # 3. Airborne Velocity (TC 19 Subtype 1 & 3)
        elif df in (17, 18) and tc == 19:
            if decoded_msg.groundspeed_kt is not None:
                track.groundspeed_kt = decoded_msg.groundspeed_kt
            if decoded_msg.ground_track_deg is not None:
                track.track_deg = decoded_msg.ground_track_deg
            elif decoded_msg.magnetic_heading_deg is not None and track.track_deg is None:
                track.track_deg = decoded_msg.magnetic_heading_deg

            if decoded_msg.vertical_rate_fpm is not None:
                track.vertical_rate_fpm = decoded_msg.vertical_rate_fpm

            # Check velocity anomalies (supersonic speed, non-physical acceleration)
            if track.groundspeed_kt is not None:
                spd_mps = track.groundspeed_kt * KNOTS_TO_MPS
                vel_anoms = self.anomaly_detector.check_velocity_anomalies(
                    icao, spd_mps, rx_timestamp_s
                )
                for a in vel_anoms:
                    if a not in track.anomalies:
                        track.anomalies.append(a)

            # Update IMM filter velocity if position is already established
            if (
                imm.last_update_time_s is not None
                and decoded_msg.v_ew_kt is not None
                and decoded_msg.v_ns_kt is not None
            ):
                ve_mps = decoded_msg.v_ew_kt * KNOTS_TO_MPS
                vn_mps = decoded_msg.v_ns_kt * KNOTS_TO_MPS
                vu_mps = (decoded_msg.vertical_rate_fpm or 0) * FPM_TO_MPS

                dt = rx_timestamp_s - imm.last_update_time_s
                if dt > 0.0:
                    imm.predict(dt)
                    imm.last_update_time_s = rx_timestamp_s
                imm.update_velocity(ve_mps, vn_mps, vu_mps)
                track.mode_probability_turn = imm.mode_probability_turn

        # 4. Altitude Surveillance Replies (DF 0, 4, 16, 20)
        elif df in (0, 4, 16, 20) and decoded_msg.altitude_ft is not None:
            alt_ft = float(decoded_msg.altitude_ft)
            alt_anoms = self.anomaly_detector.check_altitude_anomalies(
                icao, alt_ft, rx_timestamp_s, track.vertical_rate_fpm
            )
            for a in alt_anoms:
                if a not in track.anomalies:
                    track.anomalies.append(a)

            if "ALTITUDE_CLIFF_GILLHAM" not in alt_anoms:
                track.alt_baro_ft = decoded_msg.altitude_ft
                if imm.last_update_time_s is not None:
                    dt = rx_timestamp_s - imm.last_update_time_s
                    if dt > 0.0:
                        imm.predict(dt)
                        imm.last_update_time_s = rx_timestamp_s
                    imm.update_altitude(alt_ft * FEET_TO_METERS)

        # 5. Track Lifecycle Update
        has_established_pos = track.has_position
        new_state = lifecycle.register_hit(rx_timestamp_s, has_established_pos)
        track.state = new_state
        track.last_seen_s = rx_timestamp_s

        # 6. Periodic inactivity sweep across tracks
        for other_icao, other_track in list(self._tracks.items()):
            other_life = self._lifecycles[other_icao]
            other_state = other_life.evaluate_timeouts(rx_timestamp_s)
            other_track.state = other_state


__all__ = [
    "Track",
    "TrackingEngine",
]
