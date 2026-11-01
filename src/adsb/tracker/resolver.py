"""Stateful Compact Position Reporting (CPR) position resolver.

Maintains sliding-window even and odd CPR squitter history per aircraft,
evaluates global CPR decode validity (10s airborne window, 25s surface window,
zone straddle validation), and provides seamless single-frame local CPR fallback.

Authoritative reference: docs/05_TRACKING_AND_STATE_ESTIMATION.md Section 2
and RTCA DO-260B § 2.2.3.2.3.
"""

from __future__ import annotations

from dataclasses import dataclass

from adsb.core.geodesy import haversine_distance
from adsb.protocol.cpr import (
    decode_cpr_global_airborne,
    decode_cpr_global_surface,
    decode_cpr_local,
)


@dataclass(slots=True)
class _CPREntry:
    lat_frac: float
    lon_frac: float
    timestamp_s: float
    on_ground: bool


@dataclass(slots=True)
class _AircraftCPRState:
    even_frame: _CPREntry | None = None
    odd_frame: _CPREntry | None = None
    established_lat: float | None = None
    established_lon: float | None = None
    last_fix_time_s: float | None = None
    has_established_position: bool = False
    zone_straddle_detected: bool = False


class StatefulCPRResolver:
    """Stateful multi-aircraft Compact Position Reporting (CPR) coordinate resolver."""

    def __init__(self, station_ref_lat: float | None = None, station_ref_lon: float | None = None) -> None:
        self.station_ref_lat = station_ref_lat
        self.station_ref_lon = station_ref_lon
        self._aircraft_states: dict[int, _AircraftCPRState] = {}

    def get_cpr_state(self, icao24: int) -> _AircraftCPRState:
        """Get or initialize CPR state for given ICAO24 address."""
        if icao24 not in self._aircraft_states:
            self._aircraft_states[icao24] = _AircraftCPRState()
        return self._aircraft_states[icao24]

    def has_position(self, icao24: int) -> bool:
        """Return True if ICAO has an established reference position."""
        state = self._aircraft_states.get(icao24)
        return state.has_established_position if state else False

    def reset_track(self, icao24: int) -> None:
        """Reset CPR history for dropped or re-assigned track."""
        self._aircraft_states.pop(icao24, None)

    def resolve_position(
        self,
        icao24: int,
        cpr_lat: int,
        cpr_lon: int,
        cpr_format: int,
        rx_timestamp_s: float,
        on_ground: bool = False,
        track_ref_lat: float | None = None,
        track_ref_lon: float | None = None,
    ) -> tuple[float, float] | None:
        """Process incoming CPR position squitter and attempt position resolution.

        Parameters:
            icao24: 24-bit ICAO aircraft address.
            cpr_lat: 17-bit raw encoded latitude fraction [0..131071].
            cpr_lon: 17-bit raw encoded longitude fraction [0..131071].
            cpr_format: 0 for Even, 1 for Odd.
            rx_timestamp_s: Reception timestamp in seconds.
            on_ground: True for surface position (TC 5..8), False for airborne (TC 9..18, 20..22).
            track_ref_lat: Optional current estimated track latitude.
            track_ref_lon: Optional current estimated track longitude.

        Returns:
            tuple[float, float] | None: (lat_deg, lon_deg) if resolved, else None.
        """
        state = self.get_cpr_state(icao24)
        state.zone_straddle_detected = False

        lat_frac = cpr_lat / 131072.0
        lon_frac = cpr_lon / 131072.0
        new_entry = _CPREntry(
            lat_frac=lat_frac,
            lon_frac=lon_frac,
            timestamp_s=rx_timestamp_s,
            on_ground=on_ground,
        )

        # Update cache for incoming parity
        if cpr_format == 0:
            state.even_frame = new_entry
        else:
            state.odd_frame = new_entry

        # Reference position to use for local decode (track ref > established state > station ref)
        ref_lat = track_ref_lat if track_ref_lat is not None else state.established_lat
        ref_lon = track_ref_lon if track_ref_lon is not None else state.established_lon

        # Case A: Track has an established position -> Local CPR with global cross-check
        if state.has_established_position and ref_lat is not None and ref_lon is not None:
            # 1. Local CPR decode
            local_lat, local_lon = decode_cpr_local(
                lat_frac,
                lon_frac,
                cpr_format,
                ref_lat=ref_lat,
                ref_lon=ref_lon,
                surface=on_ground,
            )

            # 2. Attempt global decode if opposite parity frame is within time window
            opp_entry = state.odd_frame if cpr_format == 0 else state.even_frame
            global_pos: tuple[float, float] | None = None

            if opp_entry and opp_entry.on_ground == on_ground:
                dt = abs(rx_timestamp_s - opp_entry.timestamp_s)
                max_dt = 25.0 if on_ground else 10.0
                if dt <= max_dt:
                    e_frame = state.even_frame if cpr_format == 0 else opp_entry
                    o_frame = opp_entry if cpr_format == 0 else state.odd_frame
                    if e_frame and o_frame:
                        try:
                            if on_ground:
                                s_ref_lat = ref_lat if ref_lat is not None else (self.station_ref_lat or 0.0)
                                s_ref_lon = ref_lon if ref_lon is not None else (self.station_ref_lon or 0.0)
                                global_pos = decode_cpr_global_surface(
                                    e_frame.lat_frac,
                                    e_frame.lon_frac,
                                    o_frame.lat_frac,
                                    o_frame.lon_frac,
                                    ref_lat=s_ref_lat,
                                    ref_lon=s_ref_lon,
                                    most_recent=cpr_format,
                                )
                            else:
                                global_pos = decode_cpr_global_airborne(
                                    e_frame.lat_frac,
                                    e_frame.lon_frac,
                                    o_frame.lat_frac,
                                    o_frame.lon_frac,
                                    most_recent=cpr_format,
                                )
                        except ValueError:
                            # Zone straddle in global decode: cleanly use local CPR!
                            state.zone_straddle_detected = True
                            global_pos = None

            # Cross-check global vs local
            if global_pos is not None:
                d_diff = haversine_distance(local_lat, local_lon, global_pos[0], global_pos[1])
                # If they agree within 5 km, either is fine; prefer global or local
                chosen_pos = global_pos if d_diff < 5000.0 else (local_lat, local_lon)
            else:
                chosen_pos = (local_lat, local_lon)

            state.established_lat = chosen_pos[0]
            state.established_lon = chosen_pos[1]
            state.last_fix_time_s = rx_timestamp_s
            return chosen_pos

        # Case B: Initial position resolution (Track has no established position yet)
        opp_entry = state.odd_frame if cpr_format == 0 else state.even_frame
        if not opp_entry or opp_entry.on_ground != on_ground:
            return None

        dt = abs(rx_timestamp_s - opp_entry.timestamp_s)
        max_dt = 25.0 if on_ground else 10.0
        if dt > max_dt:
            return None

        e_frame = state.even_frame if cpr_format == 0 else opp_entry
        o_frame = opp_entry if cpr_format == 0 else state.odd_frame
        if not e_frame or not o_frame:
            return None

        try:
            if on_ground:
                s_ref_lat = ref_lat if ref_lat is not None else (self.station_ref_lat or 0.0)
                s_ref_lon = ref_lon if ref_lon is not None else (self.station_ref_lon or 0.0)
                pos = decode_cpr_global_surface(
                    e_frame.lat_frac,
                    e_frame.lon_frac,
                    o_frame.lat_frac,
                    o_frame.lon_frac,
                    ref_lat=s_ref_lat,
                    ref_lon=s_ref_lon,
                    most_recent=cpr_format,
                )
            else:
                pos = decode_cpr_global_airborne(
                    e_frame.lat_frac,
                    e_frame.lon_frac,
                    o_frame.lat_frac,
                    o_frame.lon_frac,
                    most_recent=cpr_format,
                )
        except ValueError:
            state.zone_straddle_detected = True
            return None

        # Validate range from reference station if known (must be < 600 km)
        if self.station_ref_lat is not None and self.station_ref_lon is not None:
            dist_to_station = haversine_distance(
                pos[0], pos[1], self.station_ref_lat, self.station_ref_lon
            )
            if dist_to_station > 1_500_000.0:  # 1500 km max line of sight / realistic limit
                return None

        state.established_lat = pos[0]
        state.established_lon = pos[1]
        state.last_fix_time_s = rx_timestamp_s
        state.has_established_position = True
        return pos


__all__ = [
    "StatefulCPRResolver",
]
