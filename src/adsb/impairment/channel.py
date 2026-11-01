"""Physical RF channel simulation, propagation geometry, link budget, and reception pipeline.

Authoritative reference: RTCA DO-260B / DO-260C, ICAO Annex 10 Vol IV,
and docs/04_ERROR_AND_ANOMALY_SIMULATION.md Sections 2.1-2.4.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

import numpy as np

from adsb.constants import (
    EARTH_MEAN_RADIUS_M,
    EFFECTIVE_EARTH_RADIUS_FACTOR,
    FREQ_1090ES_HZ,
    SPEED_OF_LIGHT_MPS,
)
from adsb.core.coordinates import ecef_to_enu, geodetic_to_ecef
from adsb.impairment.noise import (
    PoissonFruitGenerator,
    PulseCollisionEngine,
    ReceptionEvent,
)
from adsb.synthesis.scheduler import SquitterEvent


class AntennaPatternType(StrEnum):
    """Antenna elevation radiation pattern types."""

    ISOTROPIC = "ISOTROPIC"
    DIPOLE = "DIPOLE"
    COLLINEAR = "COLLINEAR"


class FadingModel(StrEnum):
    """Multipath fading channel distributions."""

    AWGN = "AWGN"
    RAYLEIGH = "RAYLEIGH"


class DropoutReason(StrEnum):
    """Reason a frame was not successfully received at a receiver station."""

    NONE = "NONE"
    RADIO_HORIZON = "RADIO_HORIZON"
    TERRAIN_MASK = "TERRAIN_MASK"
    PREAMBLE_MISSED = "PREAMBLE_MISSED"


@dataclass(frozen=True, slots=True)
class ReceiverSite:
    """Ground-based Mode S / ADS-B receiver installation.

    Attributes:
        lat_deg: WGS84 Geodetic Latitude in decimal degrees [-90, +90].
        lon_deg: WGS84 Geodetic Longitude in decimal degrees [-180, +180).
        alt_hae_m: Antenna height above WGS84 ellipsoid in meters.
        alt_msl_m: Antenna height above Mean Sea Level in meters (default equal to alt_hae_m).
        rx_id: Unique station identifier.
        name: Station descriptive name.
        noise_floor_dbm: Receiver thermal + environmental noise floor in dBm (default -90.0 dBm).
        antenna_gain_peak_dbi: Maximum antenna gain at horizon in dBi.
        sensitivity_dbm: Minimum triggering signal level in dBm (default -84.0 dBm).
        antenna_pattern: Antenna radiation pattern evaluator.
        clock_drift_ppm: Linear oscillator drift rate in parts per million [-20, +20].
        clock_offset_s: Initial receiver clock phase offset in seconds.
        terrain_mask_profile: Azimuth-elevation horizon mask as tuple of (azimuth_deg, min_elevation_deg).
    """

    lat_deg: float
    lon_deg: float
    alt_hae_m: float = 0.0
    alt_msl_m: float = 0.0
    rx_id: str = "DEFAULT_RX"
    name: str = "DEFAULT_RX"
    noise_floor_dbm: float = -90.0
    antenna_gain_peak_dbi: float = 5.0
    sensitivity_dbm: float = -84.0
    antenna_pattern: AntennaPatternType = AntennaPatternType.DIPOLE
    clock_drift_ppm: float = 0.0
    clock_offset_s: float = 0.0
    terrain_mask_profile: tuple[tuple[float, float], ...] | None = None

    @property
    def effective_alt_m(self) -> float:
        """Return the effective antenna altitude for radio horizon calculations."""
        if self.alt_msl_m != 0.0:
            return self.alt_msl_m
        return self.alt_hae_m

    def to_dict(self) -> dict[str, Any]:
        """Convert receiver site configuration to serializable dictionary."""
        return {
            "rx_id": self.rx_id,
            "name": self.name,
            "lat": self.lat_deg,
            "lon": self.lon_deg,
            "lat_deg": self.lat_deg,
            "lon_deg": self.lon_deg,
            "alt_msl_m": self.alt_msl_m,
            "alt_hae_m": self.alt_hae_m,
            "antenna_gain_peak_dbi": self.antenna_gain_peak_dbi,
            "sensitivity_dbm": self.sensitivity_dbm,
            "noise_floor_dbm": self.noise_floor_dbm,
            "antenna_pattern": str(self.antenna_pattern),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ReceiverSite:
        """Construct ReceiverSite from dictionary representation."""
        lat = data.get("lat_deg", data.get("lat", 0.0))
        lon = data.get("lon_deg", data.get("lon", 0.0))
        alt = data.get("alt_msl_m", data.get("alt_m", data.get("alt_hae_m", 0.0)))
        rx_id = data.get("rx_id", data.get("id", "RX_DEFAULT"))
        name = data.get("name", rx_id)
        gain = data.get("antenna_gain_peak_dbi", data.get("antenna_gain_dbi", 5.0))
        sens = data.get("sensitivity_dbm", data.get("sensitivity", -84.0))
        noise = data.get("noise_floor_dbm", -90.0)
        pattern_str = data.get("antenna_pattern", "DIPOLE")
        pattern = (
            AntennaPatternType(pattern_str)
            if pattern_str in AntennaPatternType.__members__
            else AntennaPatternType.DIPOLE
        )
        return cls(
            lat_deg=float(lat),
            lon_deg=float(lon),
            alt_hae_m=float(alt),
            alt_msl_m=float(alt),
            rx_id=str(rx_id),
            name=str(name),
            antenna_gain_peak_dbi=float(gain),
            sensitivity_dbm=float(sens),
            noise_floor_dbm=float(noise),
            antenna_pattern=pattern,
        )


@dataclass(frozen=True, slots=True)
class ChannelConfig:
    """RF propagation channel and physical simulation parameters."""

    carrier_freq_hz: float = FREQ_1090ES_HZ
    fading_model: FadingModel = FadingModel.AWGN
    enable_horizon_check: bool = True
    enable_terrain_masking: bool = True
    enable_preamble_dropout: bool = True
    enable_bit_errors: bool = True
    enable_clock_drift: bool = True
    jitter_std_s: float = 15e-9  # 15 ns Gaussian jitter
    cable_loss_db: float = 2.0
    atm_loss_db: float = 0.5
    preamble_threshold_snr_db: float = 6.0
    adc_clip_power_dbm: float = 0.0


@dataclass(frozen=True, slots=True)
class ChannelPropagationResult:
    """Comprehensive result of propagating an RF transmission to a receiver."""

    received: bool
    dropout_reason: DropoutReason
    slant_range_m: float
    azimuth_deg: float
    elevation_deg: float
    fspl_db: float
    rx_power_dbm: float
    snr_db: float
    rssi_dbfs: float
    ber: float
    bit_flips: int
    rx_timestamp_s: float
    beast_mlat_counter: int
    corrupted_bytes: bytes


def compute_slant_range(
    tx_lat_deg: float,
    tx_lon_deg: float,
    tx_alt_m: float,
    rx_lat_deg: float,
    rx_lon_deg: float,
    rx_alt_m: float,
) -> float:
    """Compute true 3D Euclidean distance (slant range) in WGS84 ECEF coordinates."""
    x1, y1, z1 = geodetic_to_ecef(tx_lat_deg, tx_lon_deg, tx_alt_m)
    x2, y2, z2 = geodetic_to_ecef(rx_lat_deg, rx_lon_deg, rx_alt_m)
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


def compute_radio_horizon(
    h_tx_m: float,
    h_rx_m: float,
    k_factor: float = EFFECTIVE_EARTH_RADIUS_FACTOR,
) -> float:
    """Compute 4/3-Earth refracted line-of-sight radio horizon distance in meters.

    d_horizon = sqrt(2 * R_eff * max(0, h_tx)) + sqrt(2 * R_eff * max(0, h_rx))
    """
    r_eff = k_factor * EARTH_MEAN_RADIUS_M
    d_tx = math.sqrt(2.0 * r_eff * max(0.0, h_tx_m))
    d_rx = math.sqrt(2.0 * r_eff * max(0.0, h_rx_m))
    return d_tx + d_rx


def compute_fspl(slant_range_m: float, freq_hz: float = FREQ_1090ES_HZ) -> float:
    """Compute Free-Space Path Loss (FSPL) in decibels.

    FSPL = 20*log10(d) + 20*log10(f) - 147.5522 dB
    """
    d_eff = max(1.0, slant_range_m)
    fspl = 20.0 * math.log10(d_eff) + 20.0 * math.log10(freq_hz) - 147.5522
    return max(0.0, fspl)


def compute_antenna_gain(
    el_deg: float,
    peak_gain_dbi: float = 5.0,
    pattern_type: AntennaPatternType = AntennaPatternType.DIPOLE,
) -> float:
    """Compute receiver antenna gain in dBi as a function of elevation angle."""
    if pattern_type == AntennaPatternType.ISOTROPIC:
        return 0.0

    # Half-wave dipole / collinear elevation pattern F(theta) = |cos(pi/2 * sin(theta)) / cos(theta)|^2
    theta = math.radians(el_deg)
    cos_th = math.cos(theta)
    if abs(cos_th) < 1e-6:
        # Directly overhead / cone of silence notch
        return -30.0

    sin_th = math.sin(theta)
    f_val = abs(math.cos(0.5 * math.pi * sin_th) / cos_th) ** 2
    if f_val <= 1e-6:
        return -30.0

    gain = peak_gain_dbi + 10.0 * math.log10(f_val)
    return max(-30.0, gain)


def check_terrain_mask(
    azimuth_deg: float,
    elevation_deg: float,
    profile: tuple[tuple[float, float], ...] | None,
) -> bool:
    """Check if elevation is above terrain obstruction profile at given azimuth.

    Returns True if target is visible (above mask), False if occulted.
    """
    if profile is None or len(profile) == 0:
        return elevation_deg >= 0.0

    az_norm = (azimuth_deg % 360.0 + 360.0) % 360.0
    sorted_p = sorted(profile, key=lambda p: p[0])

    # Circular interpolation
    n = len(sorted_p)
    if n == 1:
        return elevation_deg >= sorted_p[0][1]

    # Find bounding azimuths
    for i in range(n):
        az_curr, el_curr = sorted_p[i]
        next_idx = (i + 1) % n
        az_next, el_next = sorted_p[next_idx]

        if next_idx == 0:  # Wrap-around boundary
            if az_norm >= az_curr or az_norm <= az_next:
                span = (az_next + 360.0) - az_curr
                rel = ((az_norm + 360.0) if az_norm < az_curr else az_norm) - az_curr
                frac = rel / span if span > 0 else 0.0
                min_el = el_curr + frac * (el_next - el_curr)
                return elevation_deg >= min_el
        else:
            if az_curr <= az_norm <= az_next:
                span = az_next - az_curr
                frac = (az_norm - az_curr) / span if span > 0 else 0.0
                min_el = el_curr + frac * (el_next - el_curr)
                return elevation_deg >= min_el

    return elevation_deg >= sorted_p[0][1]


def compute_preamble_drop_prob(snr_db: float, threshold_snr_db: float = 6.0) -> float:
    """Compute preamble detection drop probability via logistic psychometric function."""
    # P_drop = 1.0 / (1.0 + exp((snr - 6.0) / 1.5))
    arg = (snr_db - threshold_snr_db) / 1.5
    if arg > 40.0:
        return 0.0
    if arg < -40.0:
        return 1.0
    return 1.0 / (1.0 + math.exp(arg))


def compute_ppm_ber(
    snr_db: float,
    fading_model: FadingModel = FadingModel.AWGN,
) -> float:
    """Compute theoretical non-coherent 2-PPM Bit Error Rate (BER)."""
    gamma = 10.0 ** (snr_db / 10.0)
    if fading_model == FadingModel.AWGN:
        # P_b = 0.5 * exp(-gamma / 2.0)
        if gamma > 100.0:
            return 0.0
        return float(0.5 * math.exp(-gamma / 2.0))
    else:  # RAYLEIGH
        # P_b = 1.0 / (2.0 + gamma)
        return float(1.0 / (2.0 + gamma))


def apply_bit_errors(
    raw_bytes: bytes,
    ber: float,
    rng: np.random.Generator,
) -> tuple[bytes, int]:
    """Inject stochastic bit flips into raw wire bytes according to BER.

    Returns:
        (corrupted_bytes, bit_flip_count)
    """
    total_bits = len(raw_bytes) * 8
    if ber <= 0.0 or total_bits == 0:
        return raw_bytes, 0
    if ber >= 1.0:
        return bytes(~b & 0xFF for b in raw_bytes), total_bits

    flips = rng.random(total_bits) < ber
    flip_indices = np.nonzero(flips)[0]
    flip_count = len(flip_indices)

    if flip_count == 0:
        return raw_bytes, 0

    byte_arr = bytearray(raw_bytes)
    for bit_idx in flip_indices:
        byte_pos = int(bit_idx // 8)
        bit_in_byte = 7 - int(bit_idx % 8)
        byte_arr[byte_pos] ^= (1 << bit_in_byte)

    return bytes(byte_arr), flip_count


def compute_beast_mlat_counter(rx_timestamp_s: float) -> int:
    """Compute 48-bit 12 MHz Beast MLAT timestamp counter."""
    ticks = round(rx_timestamp_s * 12_000_000)
    return int(ticks % (2**48))


class RFPropagationChannel:
    """Deterministic physical-layer RF propagation engine."""

    def __init__(self, config: ChannelConfig | None = None) -> None:
        self.config = config if config is not None else ChannelConfig()

    def propagate_squitter(
        self,
        raw_bytes: bytes,
        tx_timestamp_s: float,
        tx_lat_deg: float,
        tx_lon_deg: float,
        tx_alt_m: float,
        receiver: ReceiverSite,
        rng: np.random.Generator,
        tx_power_dbm: float = 51.0,
        tx_roll_angle_deg: float = 0.0,
    ) -> ChannelPropagationResult:
        """Simulate physical propagation of a single squitter from aircraft to ground receiver."""
        # Step 1: Slant Range & Propagation Delay
        rx_alt = receiver.effective_alt_m
        d_slant = compute_slant_range(
            tx_lat_deg, tx_lon_deg, tx_alt_m, receiver.lat_deg, receiver.lon_deg, rx_alt
        )
        tau_prop = d_slant / SPEED_OF_LIGHT_MPS

        # Step 2: 4/3-Earth Horizon Check
        d_horizon = compute_radio_horizon(tx_alt_m, rx_alt)
        if self.config.enable_horizon_check and d_slant > d_horizon:
            return ChannelPropagationResult(
                received=False,
                dropout_reason=DropoutReason.RADIO_HORIZON,
                slant_range_m=d_slant,
                azimuth_deg=0.0,
                elevation_deg=-90.0,
                fspl_db=compute_fspl(d_slant, self.config.carrier_freq_hz),
                rx_power_dbm=-150.0,
                snr_db=-60.0,
                rssi_dbfs=-150.0,
                ber=0.5,
                bit_flips=0,
                rx_timestamp_s=tx_timestamp_s + tau_prop,
                beast_mlat_counter=compute_beast_mlat_counter(tx_timestamp_s + tau_prop),
                corrupted_bytes=raw_bytes,
            )

        # Step 3: Local ENU Azimuth and Elevation
        tx_x, tx_y, tx_z = geodetic_to_ecef(tx_lat_deg, tx_lon_deg, tx_alt_m)
        e, n, u = ecef_to_enu(
            tx_x, tx_y, tx_z, receiver.lat_deg, receiver.lon_deg, receiver.alt_hae_m
        )
        rho_horiz = math.hypot(e, n)
        az_deg = (math.degrees(math.atan2(e, n)) + 360.0) % 360.0
        el_deg = math.degrees(math.atan2(u, rho_horiz)) if rho_horiz > 0 or u != 0 else 90.0

        # Step 4: Terrain Masking Check
        if self.config.enable_terrain_masking and not check_terrain_mask(
            az_deg, el_deg, receiver.terrain_mask_profile
        ):
            return ChannelPropagationResult(
                received=False,
                dropout_reason=DropoutReason.TERRAIN_MASK,
                slant_range_m=d_slant,
                azimuth_deg=az_deg,
                elevation_deg=el_deg,
                fspl_db=compute_fspl(d_slant, self.config.carrier_freq_hz),
                rx_power_dbm=-150.0,
                snr_db=-60.0,
                rssi_dbfs=-150.0,
                ber=0.5,
                bit_flips=0,
                rx_timestamp_s=tx_timestamp_s + tau_prop,
                beast_mlat_counter=compute_beast_mlat_counter(tx_timestamp_s + tau_prop),
                corrupted_bytes=raw_bytes,
            )

        # Step 5: Link Budget (FSPL, Antenna Gain, Cable & Bank Losses)
        fspl_db = compute_fspl(d_slant, self.config.carrier_freq_hz)
        g_rx = compute_antenna_gain(
            el_deg, receiver.antenna_gain_peak_dbi, receiver.antenna_pattern
        )
        g_tx = 2.0  # Aircraft quarter-wave blade nominal gain

        # Aircraft belly shadowing during banking
        l_bank = 0.0
        abs_roll = abs(tx_roll_angle_deg)
        if abs_roll > 5.0:
            if abs_roll <= 30.0:
                l_bank = 0.2 * (abs_roll - 5.0)
            else:
                l_bank = min(15.0, 5.0 + 0.5 * (abs_roll - 30.0))

        # Multipath fading
        l_fade = 0.0
        if self.config.fading_model == FadingModel.RAYLEIGH:
            l_fade = float(rng.normal(0.0, 1.5))

        rx_power_dbm = (
            tx_power_dbm
            + g_tx
            + g_rx
            - fspl_db
            - self.config.cable_loss_db
            - self.config.atm_loss_db
            - l_bank
            - l_fade
        )
        snr_db = rx_power_dbm - receiver.noise_floor_dbm
        rssi_dbfs = min(0.0, rx_power_dbm - self.config.adc_clip_power_dbm)

        # Step 6: Preamble Drop Evaluation
        if self.config.enable_preamble_dropout:
            p_drop = compute_preamble_drop_prob(snr_db, self.config.preamble_threshold_snr_db)
            if rng.random() < p_drop:
                return ChannelPropagationResult(
                    received=False,
                    dropout_reason=DropoutReason.PREAMBLE_MISSED,
                    slant_range_m=d_slant,
                    azimuth_deg=az_deg,
                    elevation_deg=el_deg,
                    fspl_db=fspl_db,
                    rx_power_dbm=rx_power_dbm,
                    snr_db=snr_db,
                    rssi_dbfs=rssi_dbfs,
                    ber=compute_ppm_ber(snr_db, self.config.fading_model),
                    bit_flips=0,
                    rx_timestamp_s=tx_timestamp_s + tau_prop,
                    beast_mlat_counter=compute_beast_mlat_counter(tx_timestamp_s + tau_prop),
                    corrupted_bytes=raw_bytes,
                )

        # Step 7: BER and Bit Errors
        ber = compute_ppm_ber(snr_db, self.config.fading_model)
        if self.config.enable_bit_errors:
            corrupted_bytes, bit_flips = apply_bit_errors(raw_bytes, ber, rng)
        else:
            corrupted_bytes, bit_flips = raw_bytes, 0

        # Step 8: Receiver Clock Drift & Timestamp Jitter
        clock_drift = (
            receiver.clock_offset_s + (receiver.clock_drift_ppm * 1e-6 * tx_timestamp_s)
            if self.config.enable_clock_drift
            else 0.0
        )
        jitter = float(rng.normal(0.0, self.config.jitter_std_s))
        rx_timestamp_s = tx_timestamp_s + tau_prop + clock_drift + jitter
        beast_counter = compute_beast_mlat_counter(rx_timestamp_s)

        return ChannelPropagationResult(
            received=True,
            dropout_reason=DropoutReason.NONE,
            slant_range_m=d_slant,
            azimuth_deg=az_deg,
            elevation_deg=el_deg,
            fspl_db=fspl_db,
            rx_power_dbm=rx_power_dbm,
            snr_db=snr_db,
            rssi_dbfs=rssi_dbfs,
            ber=ber,
            bit_flips=bit_flips,
            rx_timestamp_s=rx_timestamp_s,
            beast_mlat_counter=beast_counter,
            corrupted_bytes=corrupted_bytes,
        )


class RFChannelModel:
    """High-level physical RF channel pipeline integrating FRUIT and pulse collisions."""

    def __init__(
        self,
        fruit_rate_hz: float = 0.0,
        garble_enabled: bool = True,
        capture_threshold_db: float = 6.0,
        rx_id: str = "DEFAULT_RX",
        receiver: ReceiverSite | None = None,
        config: ChannelConfig | None = None,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.fruit_rate_hz = fruit_rate_hz
        self.garble_enabled = garble_enabled
        self.capture_threshold_db = capture_threshold_db
        self.rx_id = rx_id
        self.receiver = receiver or ReceiverSite(
            lat_deg=52.3105, lon_deg=4.7683, alt_hae_m=0.0, rx_id=rx_id
        )
        self.config = config or ChannelConfig()
        self.rng = rng if rng is not None else np.random.default_rng()

        self.collision_engine = PulseCollisionEngine(
            capture_threshold_db=self.capture_threshold_db,
            rng=self.rng,
        )
        self.fruit_generator = PoissonFruitGenerator(
            fruit_rate_hz=self.fruit_rate_hz,
            rx_id=self.rx_id,
            rng=self.rng,
        )

    def process_transmissions(
        self,
        transmissions: list[SquitterEvent],
    ) -> list[ReceptionEvent]:
        """Process squitter transmissions through channel:

        1. Map SquitterEvents to ReceptionEvents.
        2. Inject Poisson FRUIT if fruit_rate_hz > 0.
        3. Sort all events chronologically by rx_timestamp_ns.
        4. Resolve pairwise pulse overlap collisions with capture effect if garble_enabled.
        5. Return final sorted list of ReceptionEvents.
        """
        receptions: list[ReceptionEvent] = []

        # 1. Map transmissions to ReceptionEvents
        for ev in transmissions:
            # Map power to realistic RSSI (e.g., 51 dBm -> -15 dBFS)
            rssi_dbfs = min(0.0, max(-45.0, ev.tx_power_dbm - 66.0))
            snr_db = max(0.0, rssi_dbfs - (-45.0))

            receptions.append(
                ReceptionEvent(
                    rx_timestamp_ns=ev.timestamp_ns,
                    rx_id=self.rx_id,
                    raw_bytes=ev.raw_bytes,
                    rssi_dbfs=rssi_dbfs,
                    snr_db=snr_db,
                    is_corrupted=False,
                )
            )

        # 2. Inject background FRUIT if enabled
        if self.fruit_rate_hz > 0.0:
            if transmissions:
                t_min_s = min(e.timestamp_s for e in transmissions)
                t_max_s = max(e.timestamp_s for e in transmissions)
                duration_s = max(1.0, (t_max_s - t_min_s) + 0.5)
                start_ns = int(t_min_s * 1e9)
            else:
                duration_s = 1.0
                start_ns = 0

            fruit_events = self.fruit_generator.generate_fruit(
                duration_s=duration_s,
                start_timestamp_ns=start_ns,
            )
            receptions.extend(fruit_events)

        # 3. Sort chronologically by arrival time
        receptions.sort(key=lambda e: e.rx_timestamp_ns)

        # 4. Resolve pulse collisions
        if self.garble_enabled:
            receptions = self.collision_engine.resolve_collisions(receptions)

        return receptions


__all__ = [
    "AntennaPatternType",
    "ChannelConfig",
    "ChannelPropagationResult",
    "DropoutReason",
    "FadingModel",
    "RFChannelModel",
    "RFPropagationChannel",
    "ReceiverSite",
    "apply_bit_errors",
    "check_terrain_mask",
    "compute_antenna_gain",
    "compute_beast_mlat_counter",
    "compute_fspl",
    "compute_ppm_ber",
    "compute_preamble_drop_prob",
    "compute_radio_horizon",
    "compute_slant_range",
]
