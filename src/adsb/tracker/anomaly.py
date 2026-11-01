"""Kinematic anomaly and adversarial spoofing detectors.

Detectors:
    - KINEMATIC_TELEPORT: Velocity > 600 m/s or spatial jump with non-physical implied speed.
    - STALE_GPS_LATCH: Advancing timestamps with frozen identical CPR coordinates.
    - ALTITUDE_CLIFF_GILLHAM: Step altitude jump > 2000 ft inconsistent with vertical rate.
    - RF_PHYSICAL_SPOOF: Physically impossible kinematics (speed > 600 m/s or acceleration).
    - CPR_PARITY_OR_ZONE_GLITCH: CPR zone straddle or local/global divergence > 5 km.

Authoritative reference: docs/05_TRACKING_AND_STATE_ESTIMATION.md Section 5.
"""

from __future__ import annotations

from dataclasses import dataclass

from adsb.core.geodesy import haversine_distance


@dataclass(slots=True)
class AnomalyTrackerState:
    """Historical state maintained per track for anomaly evaluation."""

    last_cpr_lat: int | None = None
    last_cpr_lon: int | None = None
    consecutive_identical_cpr: int = 0
    last_pos: tuple[float, float] | None = None  # (lat_deg, lon_deg)
    last_pos_time_s: float | None = None
    last_alt_ft: float | None = None
    last_alt_time_s: float | None = None
    last_speed_mps: float | None = None
    last_speed_time_s: float | None = None


class AnomalyDetector:
    """Evaluates incoming message kinematics and tracker state against anomaly rules."""

    def __init__(self) -> None:
        self._states: dict[int, AnomalyTrackerState] = {}

    def get_state(self, icao24: int) -> AnomalyTrackerState:
        """Get or initialize anomaly state for track."""
        if icao24 not in self._states:
            self._states[icao24] = AnomalyTrackerState()
        return self._states[icao24]

    def reset_track(self, icao24: int) -> None:
        """Reset historical state for track."""
        self._states.pop(icao24, None)

    def check_position_anomalies(
        self,
        icao24: int,
        lat_deg: float,
        lon_deg: float,
        timestamp_s: float,
        cpr_lat: int,
        cpr_lon: int,
        groundspeed_kt: float | None = None,
        zone_straddle: bool = False,
    ) -> list[str]:
        """Evaluate position update for KINEMATIC_TELEPORT, STALE_GPS_LATCH, CPR glitches."""
        anomalies: list[str] = []
        st = self.get_state(icao24)

        # 1. Stale GPS Latch (AV-01): >= 4 identical CPR coordinates with advancing time
        if st.last_cpr_lat == cpr_lat and st.last_cpr_lon == cpr_lon:
            st.consecutive_identical_cpr += 1
            if st.consecutive_identical_cpr >= 4 and (
                groundspeed_kt is None or groundspeed_kt > 30.0
            ):
                anomalies.append("STALE_GPS_LATCH")
        else:
            st.consecutive_identical_cpr = 1
            st.last_cpr_lat = cpr_lat
            st.last_cpr_lon = cpr_lon

        # 2. CPR Zone Straddle
        if zone_straddle:
            anomalies.append("CPR_PARITY_OR_ZONE_GLITCH")

        # 3. Kinematic Teleportation via position jump
        if st.last_pos is not None and st.last_pos_time_s is not None:
            dt = timestamp_s - st.last_pos_time_s
            if dt > 0.05:
                dist_m = haversine_distance(st.last_pos[0], st.last_pos[1], lat_deg, lon_deg)
                implied_speed_mps = dist_m / dt
                if implied_speed_mps > 600.0:
                    anomalies.append("KINEMATIC_TELEPORT")
                    anomalies.append("RF_PHYSICAL_SPOOF")

        st.last_pos = (lat_deg, lon_deg)
        st.last_pos_time_s = timestamp_s
        return anomalies

    def check_velocity_anomalies(
        self,
        icao24: int,
        speed_mps: float,
        timestamp_s: float,
    ) -> list[str]:
        """Evaluate velocity update for non-physical speed or spoofing."""
        anomalies: list[str] = []
        st = self.get_state(icao24)

        # Speed > 600 m/s is non-physical for civil air traffic / indicates RF spoofing
        if speed_mps > 600.0:
            anomalies.append("RF_PHYSICAL_SPOOF")
            anomalies.append("KINEMATIC_TELEPORT")

        # Acceleration check (> 50 m/s^2 or ~5g sustained)
        if st.last_speed_mps is not None and st.last_speed_time_s is not None:
            dt = timestamp_s - st.last_speed_time_s
            if 0.05 < dt < 5.0:
                accel = abs(speed_mps - st.last_speed_mps) / dt
                if accel > 60.0:
                    anomalies.append("RF_PHYSICAL_SPOOF")

        st.last_speed_mps = speed_mps
        st.last_speed_time_s = timestamp_s
        return anomalies

    def check_altitude_anomalies(
        self,
        icao24: int,
        alt_ft: float,
        timestamp_s: float,
        vertical_rate_fpm: float | None = None,
    ) -> list[str]:
        """Evaluate altitude update for Gillham bit stick or cliff jump (AV-04)."""
        anomalies: list[str] = []
        st = self.get_state(icao24)

        if st.last_alt_ft is not None and st.last_alt_time_s is not None:
            dt = timestamp_s - st.last_alt_time_s
            if 0.01 <= dt <= 3.0:
                d_alt = abs(alt_ft - st.last_alt_ft)
                # Jump > 2000 ft in short period or |dh/dt| > 12000 fpm inconsistent with VR
                implied_vr_fpm = (d_alt / dt) * 60.0
                if d_alt >= 2000.0 or (d_alt >= 300.0 and implied_vr_fpm > 12000.0):
                    if vertical_rate_fpm is None or abs(vertical_rate_fpm) < 3000.0:
                        anomalies.append("ALTITUDE_CLIFF_GILLHAM")

        st.last_alt_ft = alt_ft
        st.last_alt_time_s = timestamp_s
        return anomalies


__all__ = [
    "AnomalyDetector",
    "AnomalyTrackerState",
]
