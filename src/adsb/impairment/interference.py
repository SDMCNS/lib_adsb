"""RF Interference and jamming physics modeling for ADS-B / Mode S channels.

Authoritative reference: RTCA DO-260B / DO-260C, ICAO Annex 10 Vol IV,
and docs/04_ERROR_AND_ANOMALY_SIMULATION.md Section 2.3.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace
from enum import StrEnum
from typing import Any

from adsb.constants import FREQ_1090ES_HZ
from adsb.impairment.channel import (
    FadingModel,
    ReceiverSite,
    compute_fspl,
    compute_ppm_ber,
    compute_preamble_drop_prob,
    compute_radio_horizon,
    compute_slant_range,
)


class JammingModulation(StrEnum):
    """Modulation type of RF interference transmitter."""

    CW = "CW"        # Continuous wave unmodulated tone
    NOISE = "NOISE"  # Gaussian noise barrage
    PULSE = "PULSE"  # Swept/pulsed radar interference


@dataclass(frozen=True, slots=True)
class InterferenceSource:
    """RF Interference / Jamming transmitter domain model.

    Attributes:
        source_id: Unique identifier for the jammer (e.g. 'JAM-EHAM-01').
        name: Descriptive human-readable label.
        lat_deg: WGS84 Geodetic Latitude in decimal degrees [-90.0, +90.0].
        lon_deg: WGS84 Geodetic Longitude in decimal degrees [-180.0, +180.0).
        alt_msl_m: Transmitter antenna height above Mean Sea Level in meters.
        tx_power_dbm: Equivalent Isotropically Radiated Power (EIRP) in dBm.
        frequency_hz: Center emission frequency in Hertz (default 1090ES: 1.09 GHz).
        bandwidth_mhz: 3-dB RF emission bandwidth in MHz (default 2.0 MHz).
        modulation: Jammer signal modulation type (default NOISE).
        is_active: Operational state flag (True = radiating, False = standby).
    """

    source_id: str
    name: str = "RF Jammer"
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_msl_m: float = 10.0
    tx_power_dbm: float = 50.0  # 100 W EIRP
    frequency_hz: float = FREQ_1090ES_HZ
    bandwidth_mhz: float = 2.0
    modulation: JammingModulation = JammingModulation.NOISE
    is_active: bool = True

    @property
    def tx_power_w(self) -> float:
        """Transmit power in Watts."""
        return float(10.0 ** ((self.tx_power_dbm - 30.0) / 10.0))

    @property
    def effective_range_km(self) -> float:
        """Estimated 3 dB noise floor elevation radius in km against -90 dBm noise floor.

        Derived from FSPL at 1090 MHz: FSPL(d) = 20*log10(d_km) + 93.196 dB.
        """
        # Distance at which received jammer power equals -90 dBm with 5 dBi rx antenna and 2 dB loss:
        # P_rx = P_tx + G_rx - FSPL - L = P_tx + 5.0 - (20*log10(d) + 93.2) - 2.0 = -90
        # 20*log10(d) = P_tx - 0.2
        val = (self.tx_power_dbm - 0.2) / 20.0
        return max(1.0, round(float(10.0 ** val), 1))

    def with_active(self, is_active: bool) -> InterferenceSource:
        """Return a copy with updated active state."""
        return replace(self, is_active=is_active)

    def with_power(self, tx_power_dbm: float) -> InterferenceSource:
        """Return a copy with updated transmit power."""
        return replace(self, tx_power_dbm=tx_power_dbm)

    def to_dict(self) -> dict[str, Any]:
        """Convert interference source to serializable dictionary."""
        return {
            "source_id": self.source_id,
            "name": self.name,
            "lat": self.lat_deg,
            "lon": self.lon_deg,
            "lat_deg": self.lat_deg,
            "lon_deg": self.lon_deg,
            "alt_msl_m": self.alt_msl_m,
            "tx_power_dbm": self.tx_power_dbm,
            "tx_power_w": round(self.tx_power_w, 2),
            "frequency_hz": self.frequency_hz,
            "bandwidth_mhz": self.bandwidth_mhz,
            "modulation": str(self.modulation),
            "is_active": self.is_active,
            "effective_range_km": self.effective_range_km,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> InterferenceSource:
        """Construct InterferenceSource from dictionary."""
        source_id = str(data.get("source_id", data.get("id", "JAMMER_01")))
        name = str(data.get("name", "RF Jammer"))
        lat = float(data.get("lat_deg", data.get("lat", 0.0)))
        lon = float(data.get("lon_deg", data.get("lon", 0.0)))
        alt = float(data.get("alt_msl_m", data.get("alt_m", 10.0)))
        power = float(data.get("tx_power_dbm", data.get("power_dbm", 50.0)))
        freq = float(data.get("frequency_hz", FREQ_1090ES_HZ))
        bw = float(data.get("bandwidth_mhz", 2.0))
        mod_str = str(data.get("modulation", "NOISE"))
        mod = (
            JammingModulation(mod_str)
            if mod_str in JammingModulation.__members__
            else JammingModulation.NOISE
        )
        active = bool(data.get("is_active", True))
        return cls(
            source_id=source_id,
            name=name,
            lat_deg=lat,
            lon_deg=lon,
            alt_msl_m=alt,
            tx_power_dbm=power,
            frequency_hz=freq,
            bandwidth_mhz=bw,
            modulation=mod,
            is_active=active,
        )


def compute_spectral_overlap_factor(
    f_tx_hz: float,
    bw_tx_mhz: float,
    f_rx_hz: float = FREQ_1090ES_HZ,
    bw_rx_mhz: float = 2.0,
) -> float:
    """Compute fraction of transmitter power falling within receiver IF filter."""
    if bw_tx_mhz <= 0.0 or bw_rx_mhz <= 0.0:
        return 0.0

    bw_tx_hz = bw_tx_mhz * 1e6
    bw_rx_hz = bw_rx_mhz * 1e6

    tx_min = f_tx_hz - (bw_tx_hz / 2.0)
    tx_max = f_tx_hz + (bw_tx_hz / 2.0)
    rx_min = f_rx_hz - (bw_rx_hz / 2.0)
    rx_max = f_rx_hz + (bw_rx_hz / 2.0)

    overlap_hz = max(0.0, min(tx_max, rx_max) - max(tx_min, rx_min))
    return float(overlap_hz / bw_tx_hz)


def compute_received_interference(
    source: InterferenceSource,
    receiver: ReceiverSite,
    cable_loss_db: float = 2.0,
    check_horizon: bool = True,
) -> float:
    """Compute received interference power in dBm at receiver from a single jammer.

    Returns -150.0 dBm if inactive or beyond radio horizon.
    """
    if not source.is_active:
        return -150.0

    d_slant = compute_slant_range(
        source.lat_deg,
        source.lon_deg,
        source.alt_msl_m,
        receiver.lat_deg,
        receiver.lon_deg,
        receiver.effective_alt_m,
    )

    if check_horizon:
        d_horizon = compute_radio_horizon(source.alt_msl_m, receiver.effective_alt_m)
        if d_slant > d_horizon:
            return -150.0

    fspl_db = compute_fspl(d_slant, source.frequency_hz)
    g_rx = receiver.antenna_gain_peak_dbi

    overlap = compute_spectral_overlap_factor(
        source.frequency_hz, source.bandwidth_mhz, FREQ_1090ES_HZ, 2.0
    )
    if overlap <= 0.0:
        return -150.0

    l_bw_db = 10.0 * math.log10(overlap)
    p_rx_dbm = source.tx_power_dbm + g_rx - fspl_db - cable_loss_db + l_bw_db
    return max(-150.0, float(p_rx_dbm))


def compute_aggregate_interference(
    sources: Sequence[InterferenceSource],
    receiver: ReceiverSite,
    cable_loss_db: float = 2.0,
) -> float:
    """Compute total aggregate interference power in dBm across all active jammers."""
    p_lin_sum = 0.0
    for s in sources:
        if not s.is_active:
            continue
        i_dbm = compute_received_interference(s, receiver, cable_loss_db)
        if i_dbm > -140.0:
            p_lin_sum += 10.0 ** (i_dbm / 10.0)

    if p_lin_sum <= 0.0:
        return -150.0
    return float(10.0 * math.log10(p_lin_sum))


def compute_effective_noise_floor(
    quiescent_noise_floor_dbm: float,
    aggregate_interference_dbm: float,
) -> float:
    """Compute effective noise floor combining thermal noise and interference in dBm."""
    p_noise_lin = 10.0 ** (quiescent_noise_floor_dbm / 10.0)
    p_interf_lin = (
        10.0 ** (aggregate_interference_dbm / 10.0)
        if aggregate_interference_dbm > -140.0
        else 0.0
    )
    return float(10.0 * math.log10(p_noise_lin + p_interf_lin))


def compute_sinr(
    rx_power_dbm: float,
    effective_noise_floor_dbm: float,
) -> float:
    """Compute Signal-to-Interference-plus-Noise Ratio in decibels."""
    return float(rx_power_dbm - effective_noise_floor_dbm)


def compute_frame_drop_probability(
    sinr_db: float,
    num_bits: int = 112,
    fading_model: FadingModel = FadingModel.AWGN,
) -> float:
    """Compute total frame drop probability (preamble sync failure + bit errors)."""
    p_preamble_drop = compute_preamble_drop_prob(sinr_db, threshold_snr_db=6.0)
    ber = compute_ppm_ber(sinr_db, fading_model)

    if ber >= 1.0:
        return 1.0
    if ber <= 0.0:
        return float(p_preamble_drop)

    p_payload_clean = (1.0 - ber) ** num_bits
    return float(1.0 - ((1.0 - p_preamble_drop) * p_payload_clean))


@dataclass(slots=True)
class ReceiverStatus:
    """Dynamic operational metrics and telemetry for a ground receiver station."""

    rx_id: str
    name: str
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    quiescent_noise_floor_dbm: float = -90.0
    current_noise_floor_dbm: float = -90.0
    noise_elevation_db: float = 0.0
    current_sinr_db: float = 30.0
    instantaneous_ber: float = 0.0
    cumulative_ber: float = 0.0
    total_frames_received: int = 0
    total_frames_dropped: int = 0
    packet_loss_rate_pct: float = 0.0
    is_jammed: bool = False
    last_updated_s: float = 0.0

    def record_reception(
        self, sinr_db: float, ber: float, is_dropped: bool, timestamp_s: float
    ) -> None:
        """Update live telemetry counters with latest squitter arrival event."""
        self.current_sinr_db = round(sinr_db, 2)
        self.instantaneous_ber = float(ber)
        self.last_updated_s = timestamp_s

        if is_dropped:
            self.total_frames_dropped += 1
        else:
            self.total_frames_received += 1

        total = self.total_frames_received + self.total_frames_dropped
        if total > 0:
            self.packet_loss_rate_pct = round(
                100.0 * (self.total_frames_dropped / total), 2
            )
            alpha = 1.0 / min(total, 1000)
            self.cumulative_ber = (1.0 - alpha) * self.cumulative_ber + alpha * ber

    def to_dict(self) -> dict[str, Any]:
        """Convert receiver status to dictionary."""
        return {
            "rx_id": self.rx_id,
            "name": self.name,
            "lat_deg": self.lat_deg,
            "lon_deg": self.lon_deg,
            "alt_msl_m": self.alt_msl_m,
            "quiescent_noise_floor_dbm": self.quiescent_noise_floor_dbm,
            "current_noise_floor_dbm": self.current_noise_floor_dbm,
            "noise_elevation_db": self.noise_elevation_db,
            "current_sinr_db": self.current_sinr_db,
            "instantaneous_ber": self.instantaneous_ber,
            "cumulative_ber": self.cumulative_ber,
            "total_frames_received": self.total_frames_received,
            "total_frames_dropped": self.total_frames_dropped,
            "packet_loss_rate_pct": self.packet_loss_rate_pct,
            "is_jammed": self.is_jammed,
            "last_updated_s": self.last_updated_s,
        }


class InterferenceEngine:
    """Evaluates multi-source RF interference environments across receiver networks."""

    def __init__(self, cable_loss_db: float = 2.0) -> None:
        self.cable_loss_db: float = cable_loss_db
        self._sources: dict[str, InterferenceSource] = {}

    def add_source(self, source: InterferenceSource) -> None:
        """Register or replace an interference source."""
        self._sources[source.source_id] = source

    def remove_source(self, source_id: str) -> bool:
        """Remove an interference source by ID."""
        return self._sources.pop(source_id, None) is not None

    def get_source(self, source_id: str) -> InterferenceSource | None:
        """Get an interference source by ID."""
        return self._sources.get(source_id)

    def list_sources(self) -> list[InterferenceSource]:
        """List all registered interference sources."""
        return list(self._sources.values())

    def toggle_source(self, source_id: str) -> InterferenceSource | None:
        """Toggle an interference source's active state."""
        src = self._sources.get(source_id)
        if src is None:
            return None
        updated = src.with_active(not src.is_active)
        self._sources[source_id] = updated
        return updated

    def update_source(
        self,
        source_id: str,
        is_active: bool | None = None,
        tx_power_dbm: float | None = None,
        bandwidth_mhz: float | None = None,
    ) -> InterferenceSource | None:
        """Update properties of an existing interference source."""
        src = self._sources.get(source_id)
        if src is None:
            return None
        new_active = src.is_active if is_active is None else is_active
        new_power = src.tx_power_dbm if tx_power_dbm is None else tx_power_dbm
        new_bw = src.bandwidth_mhz if bandwidth_mhz is None else bandwidth_mhz
        updated = replace(
            src,
            is_active=new_active,
            tx_power_dbm=new_power,
            bandwidth_mhz=new_bw,
        )
        self._sources[source_id] = updated
        return updated

    def evaluate_receiver_noise(
        self, receiver: ReceiverSite
    ) -> tuple[float, float, float]:
        """Compute (effective_noise_floor_dbm, noise_elevation_db, aggregate_interf_dbm)."""
        i_agg = compute_aggregate_interference(
            list(self._sources.values()), receiver, self.cable_loss_db
        )
        n_eff = compute_effective_noise_floor(receiver.noise_floor_dbm, i_agg)
        elevation = max(0.0, n_eff - receiver.noise_floor_dbm)
        return (n_eff, elevation, i_agg)

    def find_dominant_jammer(
        self, receiver: ReceiverSite
    ) -> tuple[str | None, float | None]:
        """Find the interference source contributing the strongest power at the receiver."""
        max_power = -150.0
        dominant_id: str | None = None
        for s in self._sources.values():
            if not s.is_active:
                continue
            p = compute_received_interference(s, receiver, self.cable_loss_db)
            if p > max_power:
                max_power = p
                dominant_id = s.source_id
        if dominant_id is not None and max_power > -140.0:
            return dominant_id, round(max_power, 1)
        return None, None
