"""Airspace simulation lifecycle manager, squitter scheduling, and WebSocket broadcast hub.

Manages the background simulation time loop, feeds synthesized squitters into
the TrackingEngine, accepts live wire-frame injections, and broadcasts state
vectors to connected telemetry and raw wire WebSocket clients.

Authoritative reference: PROJECT.md M5 & M6 and docs/01_ARCHITECTURE_AND_DESIGN.md.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import threading
from enum import StrEnum
from typing import Any, NamedTuple

import numpy as np
from fastapi import WebSocket
from pydantic import BaseModel, Field

from adsb.core.coordinates import geodetic_to_enu
from adsb.core.units import FEET_TO_METERS, FPM_TO_MPS, KNOTS_TO_MPS
from adsb.impairment.channel import (
    ReceiverSite,
    compute_fspl,
    compute_ppm_ber,
    compute_radio_horizon,
    compute_slant_range,
)
from adsb.impairment.interference import (
    InterferenceEngine,
    InterferenceSource,
    ReceiverStatus,
    compute_frame_drop_probability,
    compute_received_interference,
    compute_sinr,
)
from adsb.protocol.modes import decode_mode_s
from adsb.synthesis.scenario import Scenario, load_scenario
from adsb.synthesis.scheduler import MultiAircraftScheduler, SquitterEvent
from adsb.tracker.engine import Track, TrackingEngine
from adsb.tracker.imm_filter import IMMEstimator
from adsb.tracker.lifecycle import TrackLifecycleManager
from adsb.wire.avr import parse_avr_frame, serialize_avr_frame
from adsb.wire.beast import parse_beast_frame, serialize_beast_frame
from adsb.wire.sbs import parse_sbs_message

logger = logging.getLogger("adsb.api.manager")


class SimulationStatus(StrEnum):
    """Operational state of the simulation engine."""

    STOPPED = "STOPPED"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"


class SimulationConfig(BaseModel):
    """Configuration schema for airspace simulation scenario."""

    scenario_name: str = "clean_tma_10_aircraft"
    speed_multiplier: float = Field(default=1.0, gt=0.0)
    seed: int = 42


class FlightCreateRequest(BaseModel):
    """Payload schema for dynamically creating and launching a custom flight."""

    origin_icao: str = Field(..., description="Origin 4-letter ICAO code (e.g. EHAM)")
    destination_icao: str = Field(..., description="Destination 4-letter ICAO code (e.g. EGLL)")
    aircraft_type: str = Field(..., description="Aircraft performance type designator (e.g. B738)")
    callsign: str | None = Field(default=None, description="Optional flight callsign (e.g. KLM1023)")
    cruise_alt_ft: float | None = Field(default=None, gt=0.0, description="Optional cruise altitude in feet")
    squawk: str | None = Field(default="1200", description="Optional 4-digit octal squawk code")
    icao_hex: str | None = Field(default=None, description="Optional 24-bit ICAO address hex string")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> FlightCreateRequest:
        """Create instance normalizing origin/destination alias keys."""
        normalized = dict(data)
        if "origin" in normalized and "origin_icao" not in normalized:
            normalized["origin_icao"] = normalized["origin"]
        if "destination" in normalized and "destination_icao" not in normalized:
            normalized["destination_icao"] = normalized["destination"]
        return cls(**normalized)


class FlightCreationResult(NamedTuple):
    """Result returned upon successful dynamic flight creation."""

    icao_hex: str
    effective_cruise_alt_ft: float

    def __str__(self) -> str:
        return self.icao_hex


def build_default_european_receivers() -> list[ReceiverSite]:
    """Build default network of 20 primary European ground receiver stations matching European airport hubs."""
    return [
        ReceiverSite(
            rx_id="RX_EHAM",
            name="Amsterdam Schiphol Tower",
            lat_deg=52.3105,
            lon_deg=4.7683,
            alt_msl_m=25.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_EGLL",
            name="London Heathrow Tower",
            lat_deg=51.4700,
            lon_deg=-0.4543,
            alt_msl_m=45.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LFPG",
            name="Paris Charles de Gaulle Tower",
            lat_deg=49.0097,
            lon_deg=2.5479,
            alt_msl_m=130.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_EDDF",
            name="Frankfurt Main Radar",
            lat_deg=50.0379,
            lon_deg=8.5622,
            alt_msl_m=120.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_EDDM",
            name="Munich Franz Josef Strauss",
            lat_deg=48.3537,
            lon_deg=11.7860,
            alt_msl_m=460.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LSZH",
            name="Zurich Kloten Radar",
            lat_deg=47.4582,
            lon_deg=8.5555,
            alt_msl_m=440.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LEMD",
            name="Madrid Barajas Radar",
            lat_deg=40.4839,
            lon_deg=-3.5680,
            alt_msl_m=620.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LIRF",
            name="Rome Fiumicino Tower",
            lat_deg=41.8003,
            lon_deg=12.2389,
            alt_msl_m=20.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LOWW",
            name="Vienna Schwechat Radar",
            lat_deg=48.1103,
            lon_deg=16.5697,
            alt_msl_m=190.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_EKCH",
            name="Copenhagen Kastrup Tower",
            lat_deg=55.6180,
            lon_deg=12.6508,
            alt_msl_m=25.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_EBBR",
            name="Brussels Airport Tower",
            lat_deg=50.9010,
            lon_deg=4.4844,
            alt_msl_m=56.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_EIDW",
            name="Dublin Airport Tower",
            lat_deg=53.4213,
            lon_deg=-6.2701,
            alt_msl_m=74.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LEBL",
            name="Barcelona El Prat Tower",
            lat_deg=41.2974,
            lon_deg=2.0833,
            alt_msl_m=20.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LPPT",
            name="Lisbon Humberto Delgado Tower",
            lat_deg=38.7742,
            lon_deg=-9.1342,
            alt_msl_m=114.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_ENGM",
            name="Oslo Gardermoen Tower",
            lat_deg=60.1975,
            lon_deg=11.1004,
            alt_msl_m=208.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_ESSA",
            name="Stockholm Arlanda Tower",
            lat_deg=59.6498,
            lon_deg=17.9238,
            alt_msl_m=42.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_EFHK",
            name="Helsinki Vantaa Tower",
            lat_deg=60.3172,
            lon_deg=24.9633,
            alt_msl_m=55.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_EPWA",
            name="Warsaw Chopin Tower",
            lat_deg=52.1672,
            lon_deg=20.9679,
            alt_msl_m=110.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LGAV",
            name="Athens Eleftherios Venizelos Tower",
            lat_deg=37.9364,
            lon_deg=23.9445,
            alt_msl_m=94.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
        ReceiverSite(
            rx_id="RX_LTFM",
            name="Istanbul Airport Tower",
            lat_deg=41.2753,
            lon_deg=28.7519,
            alt_msl_m=99.0,
            antenna_gain_peak_dbi=6.0,
            sensitivity_dbm=-88.0,
        ),
    ]


class SimulationManager:
    """Central simulation state coordinator and real-time streaming broadcaster."""

    def __init__(self, config: SimulationConfig | None = None) -> None:
        self.config: SimulationConfig = config or SimulationConfig()
        self.status: SimulationStatus = SimulationStatus.STOPPED
        self.sim_time_s: float = 0.0
        self.speed_multiplier: float = self.config.speed_multiplier

        self.scenario: Scenario | None = None
        self.engine: TrackingEngine | None = None
        self._events: list[SquitterEvent] = []
        self._event_idx: int = 0
        self._cycle_offset_s: float = 0.0
        self._squawks: dict[int, str] = {}

        self._telemetry_clients: set[WebSocket] = set()
        self._wire_clients: dict[WebSocket, str] = {}

        self._sim_task: asyncio.Task[None] | None = None
        self._lock = asyncio.Lock()
        self._injection_lock = threading.Lock()
        self._allocated_icaos: set[int] = set()

        self._receivers: dict[str, ReceiverSite] = {}
        self._receiver_status: dict[str, ReceiverStatus] = {}
        self._interference_engine: InterferenceEngine = InterferenceEngine()
        self._sim_rng: np.random.Generator = np.random.default_rng(self.config.seed)

        self._init_simulation()

    def _init_simulation(self) -> None:
        """Initialize scenario, tracker, and event queues based on current config."""
        rng = np.random.default_rng(self.config.seed)
        self.scenario = load_scenario(self.config.scenario_name, rng)
        self.engine = TrackingEngine(
            ref_lat=self.scenario.ref_lat,
            ref_lon=self.scenario.ref_lon,
        )
        scheduler = MultiAircraftScheduler(self.scenario, rng)
        self._events = scheduler.generate_all_events()
        self._event_idx = 0
        self._cycle_offset_s = 0.0
        self.sim_time_s = 0.0
        self._squawks.clear()
        self._allocated_icaos = {
            a.icao24 for a in (self.scenario.aircraft if self.scenario else [])
        }
        if not self._receivers:
            for rx in build_default_european_receivers():
                self._receivers[rx.rx_id] = rx
                self._receiver_status[rx.rx_id] = ReceiverStatus(
                    rx_id=rx.rx_id,
                    name=rx.name,
                    lat_deg=rx.lat_deg,
                    lon_deg=rx.lon_deg,
                    alt_msl_m=rx.alt_msl_m,
                    quiescent_noise_floor_dbm=rx.noise_floor_dbm,
                    current_noise_floor_dbm=rx.noise_floor_dbm,
                    current_sinr_db=25.0,
                )
        self._update_all_receiver_noise_floors()

    def _update_all_receiver_noise_floors(self) -> None:
        """Recalculate noise floor elevation and jamming status for all receivers."""
        for rx_id, rx in self._receivers.items():
            n_eff, elev, _ = self._interference_engine.evaluate_receiver_noise(rx)
            if rx_id in self._receiver_status:
                status = self._receiver_status[rx_id]
                status.current_noise_floor_dbm = round(n_eff, 2)
                status.noise_elevation_db = round(elev, 2)
                status.is_jammed = elev >= 6.0
                if elev >= 6.0 and status.current_sinr_db > 10.0:
                    status.current_sinr_db = max(-10.0, round(status.current_sinr_db - elev, 2))
                elif elev < 3.0 and status.current_sinr_db < 15.0 and status.total_frames_received == 0:
                    status.current_sinr_db = 25.0

    def _determine_receiver_status_str(self, status: ReceiverStatus) -> str:
        """Evaluate operational health category: NORMAL, DEGRADED, or JAMMED."""
        if status.is_jammed or status.noise_elevation_db >= 6.0 or status.current_sinr_db < 6.0:
            return "JAMMED"
        if status.noise_elevation_db >= 3.0 or status.current_sinr_db < 12.0:
            return "DEGRADED"
        return "NORMAL"

    def _count_active_jammers_in_range(self, rx: ReceiverSite) -> int:
        """Count active jammers contributing significant interference to receiver."""
        count = 0
        for s in self._interference_engine.list_sources():
            if not s.is_active:
                continue
            p = compute_received_interference(s, rx, self._interference_engine.cable_loss_db)
            if p > -140.0 and p > (rx.noise_floor_dbm - 20.0):
                count += 1
        return count

    def _check_signal_reception(self, ev: SquitterEvent, sim_time_s: float) -> bool:
        """Evaluate physical RF line-of-sight and link budget against all placed receivers."""
        if ev.tx_lat_deg is None or ev.tx_lon_deg is None or ev.tx_alt_m is None:
            return True

        if not self._receivers:
            return False

        tx_lat = ev.tx_lat_deg
        tx_lon = ev.tx_lon_deg
        tx_alt = ev.tx_alt_m
        p_tx = ev.tx_power_dbm
        g_tx = 2.0  # Aircraft quarter-wave blade antenna
        freq_hz = ev.rf_freq_hz
        l_misc = 2.5  # 2.0 dB cable loss + 0.5 dB atmospheric loss

        any_detected = False

        for rx_id, rx in self._receivers.items():
            status = self._receiver_status.get(rx_id)
            if status is None:
                continue

            # 1. Spatial pre-filter: discard stations > 500 km away in nanoseconds
            if abs(tx_lat - rx.lat_deg) > 5.0 or abs(tx_lon - rx.lon_deg) > 8.0:
                continue

            # 2. Slant range
            rx_alt = rx.effective_alt_m
            d_slant = compute_slant_range(tx_lat, tx_lon, tx_alt, rx.lat_deg, rx.lon_deg, rx_alt)

            # 3. 4/3-Earth refracted line-of-sight radio horizon
            d_horizon = compute_radio_horizon(tx_alt, rx_alt)
            if d_slant > d_horizon:
                continue

            # 4. Free-space path loss and received power
            fspl_db = compute_fspl(d_slant, freq_hz)
            g_rx = rx.antenna_gain_peak_dbi
            p_rx_dbm = p_tx + g_tx + g_rx - fspl_db - l_misc

            # 5. Sensitivity threshold check (MTL)
            if p_rx_dbm < rx.sensitivity_dbm:
                n_eff = status.current_noise_floor_dbm
                sinr_db = compute_sinr(p_rx_dbm, n_eff)
                status.record_reception(sinr_db=sinr_db, ber=0.5, is_dropped=True, timestamp_s=sim_time_s)
                continue

            # 6. Effective noise floor from thermal + interference
            n_eff = status.current_noise_floor_dbm
            sinr_db = compute_sinr(p_rx_dbm, n_eff)
            ber = compute_ppm_ber(sinr_db)
            p_drop = compute_frame_drop_probability(sinr_db)

            # 7. Stochastic drop (logistic preamble drop + 112-bit packet error)
            is_dropped = bool(self._sim_rng.random() < p_drop)
            status.record_reception(sinr_db=sinr_db, ber=ber, is_dropped=is_dropped, timestamp_s=sim_time_s)

            if not is_dropped:
                any_detected = True

        return any_detected

    def get_state(self) -> dict[str, Any]:
        """Return current simulation status and progress."""
        return {
            "status": self.status.value,
            "sim_time_s": round(self.sim_time_s, 2),
            "speed_multiplier": self.speed_multiplier,
            "scenario_name": self.config.scenario_name,
        }

    def configure(self, config: SimulationConfig | dict[str, Any]) -> dict[str, Any]:
        """Apply new configuration to simulation and reinitialize state."""
        if isinstance(config, dict):
            self.config = SimulationConfig(**config)
        else:
            self.config = config

        self.speed_multiplier = self.config.speed_multiplier
        if self._sim_task and not self._sim_task.done():
            self._sim_task.cancel()
            self._sim_task = None

        self.status = SimulationStatus.STOPPED
        self._init_simulation()
        return self.get_state()

    def start(self) -> dict[str, Any]:
        """Start or resume background simulation execution."""
        if self.status != SimulationStatus.RUNNING:
            self.status = SimulationStatus.RUNNING
            if self._sim_task is None or self._sim_task.done():
                try:
                    loop = asyncio.get_running_loop()
                    self._sim_task = loop.create_task(self._run_loop())
                except RuntimeError:
                    self._sim_task = None
        return self.get_state()

    def pause(self) -> dict[str, Any]:
        """Pause running simulation."""
        self.status = SimulationStatus.PAUSED
        if self._sim_task and not self._sim_task.done():
            self._sim_task.cancel()
            self._sim_task = None
        return self.get_state()

    def reset(self) -> dict[str, Any]:
        """Reset simulation to initial time 0.0, STOPPED status, default receivers, and clear interference."""
        self.pause()
        self.status = SimulationStatus.STOPPED
        self._receivers.clear()
        self._receiver_status.clear()
        self._interference_engine = InterferenceEngine()
        self._init_simulation()
        return self.get_state()

    def set_speed(self, speed_multiplier: float) -> dict[str, Any]:
        """Adjust simulation time compression multiplier."""
        if speed_multiplier <= 0.0:
            raise ValueError(f"Speed multiplier must be positive, got {speed_multiplier}")
        self.speed_multiplier = speed_multiplier
        self.config.speed_multiplier = speed_multiplier
        return self.get_state()

    async def step(self, step_s: float = 1.0) -> dict[str, Any]:
        """Step simulation forward by step_s seconds and process squitter events."""
        if self.engine is None:
            self._init_simulation()
        assert self.engine is not None

        target_time = self.sim_time_s + step_s

        # Feed all events up to target_time with continuous wrap-around replenishment
        while True:
            if self._event_idx >= len(self._events):
                if not self._events:
                    break
                # Cycle offset wrapping: add scenario duration to offset and rewind index
                cycle_duration = (
                    self.scenario.duration_s
                    if self.scenario and self.scenario.duration_s > 0
                    else max(1.0, self._events[-1].timestamp_s)
                )
                self._cycle_offset_s += cycle_duration
                self._event_idx = 0

            ev = self._events[self._event_idx]
            effective_time_s = ev.timestamp_s + self._cycle_offset_s
            if effective_time_s > target_time:
                break

            # Physical signal reception gating across receiver network
            if self._check_signal_reception(ev, effective_time_s):
                try:
                    decoded = decode_mode_s(ev.raw_bytes)
                    self.engine.update(effective_time_s, decoded)
                    if decoded.squawk:
                        self._squawks[decoded.icao24] = decoded.squawk
                except Exception as exc:
                    logger.debug("Failed to decode or update event at t=%.2f: %s", effective_time_s, exc)

                await self.broadcast_wire(ev.raw_bytes)

            self._event_idx += 1

        self.sim_time_s = target_time
        self.engine.step_time(target_time)
        await self.broadcast_telemetry(self.get_telemetry_snapshot())
        return self.get_state()

    async def _run_loop(self) -> None:
        """Asyncio background simulation loop ticking at 20 Hz."""
        tick_interval = 0.05
        try:
            while self.status == SimulationStatus.RUNNING:
                t0 = asyncio.get_running_loop().time()
                step_dt = tick_interval * self.speed_multiplier
                try:
                    await self.step(step_dt)
                except Exception as step_exc:
                    logger.exception(
                        "Unhandled exception during simulation step at sim_time_s=%.2f: %s",
                        self.sim_time_s,
                        step_exc,
                    )
                    raise

                elapsed = asyncio.get_running_loop().time() - t0
                sleep_s = max(0.001, tick_interval - elapsed)
                await asyncio.sleep(sleep_s)
        except asyncio.CancelledError:
            logger.debug("Simulation loop task cancelled at sim_time_s=%.2f", self.sim_time_s)
        except Exception as exc:
            logger.critical(
                "Fatal error in SimulationManager background loop, halting simulation: %s",
                exc,
                exc_info=True,
            )
            self.status = SimulationStatus.STOPPED

    async def inject_frame(self, frame_format: str, raw_data: str | bytes) -> dict[str, Any]:
        """Parse wire-format payload, update tracker, and broadcast to clients.

        Parameters:
            frame_format: Wire format identifier ('AVR', 'BEAST', or 'SBS').
            raw_data: Raw string or binary payload.

        Returns:
            dict containing decoded message summary and track state.

        Raises:
            ValueError: On malformed, truncated, or unsupported wire payload.
        """
        if self.engine is None:
            self._init_simulation()
        assert self.engine is not None

        fmt_upper = frame_format.strip().upper()
        if fmt_upper not in ("AVR", "BEAST", "SBS"):
            raise ValueError(f"Unknown wire format '{frame_format}'. Supported formats: ['AVR', 'BEAST', 'SBS']")

        if fmt_upper == "AVR":
            raw_str = raw_data.decode("ascii", errors="replace") if isinstance(raw_data, bytes) else str(raw_data)
            payload_bytes, mlat_ts = parse_avr_frame(raw_str)
            if len(payload_bytes) not in (7, 14):
                raise ValueError(
                    f"Invalid Mode S frame length: {len(payload_bytes)} bytes (expected 7 or 14)"
                )
            decoded_msg = decode_mode_s(payload_bytes)
            rx_time = (mlat_ts / 12_000_000.0) if mlat_ts is not None else self.sim_time_s
            self.engine.update(rx_time, decoded_msg)
            if decoded_msg.squawk:
                self._squawks[decoded_msg.icao24] = decoded_msg.squawk

            track = self.engine.get_track(decoded_msg.icao24) or self.engine._tracks.get(decoded_msg.icao24)
            callsign = (track.callsign if track and track.callsign else decoded_msg.callsign) or ""
            lat = track.lat_deg if track and track.has_position else None
            lon = track.lon_deg if track and track.has_position else None
            alt = (
                (track.alt_baro_ft if track.alt_baro_ft is not None else track.alt_geom_ft)
                if track
                else decoded_msg.altitude_ft
            )
            if alt is None:
                alt = decoded_msg.altitude_ft
            gs = track.groundspeed_kt if track and track.groundspeed_kt is not None else decoded_msg.groundspeed_kt

            await self.broadcast_wire(payload_bytes, raw_str=raw_str)
            await self.broadcast_telemetry(self.get_telemetry_snapshot(icao24=decoded_msg.icao24))

            return {
                "icao_hex": f"{decoded_msg.icao24:06X}",
                "df": decoded_msg.df,
                "callsign": callsign,
                "crc_valid": decoded_msg.crc_valid,
                "lat": lat,
                "lon": lon,
                "altitude_ft": alt,
                "groundspeed_kt": gs,
            }

        elif fmt_upper == "BEAST":
            if isinstance(raw_data, str):
                s = raw_data.strip()
                try:
                    beast_raw = bytes.fromhex(s)
                except ValueError:
                    beast_raw = s.encode("latin1")
            else:
                beast_raw = bytes(raw_data)

            beast_frame = parse_beast_frame(beast_raw)
            payload_bytes = beast_frame.payload
            if len(payload_bytes) not in (7, 14):
                raise ValueError(
                    f"Invalid Mode S frame length in Beast payload: {len(payload_bytes)} bytes"
                )
            decoded_msg = decode_mode_s(payload_bytes)
            rx_time = (
                beast_frame.mlat_timestamp / 12_000_000.0
                if beast_frame.mlat_timestamp > 0
                else self.sim_time_s
            )
            self.engine.update(rx_time, decoded_msg)
            if decoded_msg.squawk:
                self._squawks[decoded_msg.icao24] = decoded_msg.squawk

            track = self.engine.get_track(decoded_msg.icao24) or self.engine._tracks.get(decoded_msg.icao24)
            callsign = (track.callsign if track and track.callsign else decoded_msg.callsign) or ""
            lat = track.lat_deg if track and track.has_position else None
            lon = track.lon_deg if track and track.has_position else None
            alt = (
                (track.alt_baro_ft if track.alt_baro_ft is not None else track.alt_geom_ft)
                if track
                else decoded_msg.altitude_ft
            )
            if alt is None:
                alt = decoded_msg.altitude_ft
            gs = track.groundspeed_kt if track and track.groundspeed_kt is not None else decoded_msg.groundspeed_kt

            await self.broadcast_wire(payload_bytes)
            await self.broadcast_telemetry(self.get_telemetry_snapshot(icao24=decoded_msg.icao24))

            return {
                "icao_hex": f"{decoded_msg.icao24:06X}",
                "df": decoded_msg.df,
                "callsign": callsign,
                "crc_valid": decoded_msg.crc_valid,
                "lat": lat,
                "lon": lon,
                "altitude_ft": alt,
                "groundspeed_kt": gs,
            }

        else:  # SBS
            raw_str = raw_data.decode("utf-8", errors="replace") if isinstance(raw_data, bytes) else str(raw_data)
            sbs_msg = parse_sbs_message(raw_str)
            if not sbs_msg.hex_ident:
                raise ValueError("SBS message missing hex_ident")
            icao24 = int(sbs_msg.hex_ident, 16)

            if icao24 not in self.engine._tracks:
                self.engine._tracks[icao24] = Track(icao24=icao24, last_seen_s=self.sim_time_s)
                self.engine._lifecycles[icao24] = TrackLifecycleManager()
                self.engine._filters[icao24] = IMMEstimator()

            track = self.engine._tracks[icao24]
            lifecycle = self.engine._lifecycles[icao24]
            imm = self.engine._filters[icao24]

            track.last_seen_s = self.sim_time_s
            if sbs_msg.callsign:
                track.callsign = sbs_msg.callsign
            if sbs_msg.altitude is not None:
                track.alt_baro_ft = float(sbs_msg.altitude)
            if sbs_msg.ground_speed is not None:
                track.groundspeed_kt = sbs_msg.ground_speed
            if sbs_msg.track is not None:
                track.track_deg = sbs_msg.track
            if sbs_msg.vertical_rate is not None:
                track.vertical_rate_fpm = float(sbs_msg.vertical_rate)

            if sbs_msg.lat is not None and sbs_msg.lon is not None:
                track.lat_deg = sbs_msg.lat
                track.lon_deg = sbs_msg.lon
                track.has_position = True

                alt_m = (track.alt_baro_ft or 0.0) * FEET_TO_METERS
                e_m, n_m, u_m = geodetic_to_enu(
                    sbs_msg.lat,
                    sbs_msg.lon,
                    alt_m,
                    self.engine.ref_lat,
                    self.engine.ref_lon,
                    self.engine.ref_alt_m,
                )

                if imm.last_update_time_s is None:
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
                    imm.last_update_time_s = self.sim_time_s
                else:
                    dt = self.sim_time_s - imm.last_update_time_s
                    if dt > 0.0:
                        imm.predict(dt)
                        imm.last_update_time_s = self.sim_time_s
                    imm.update_position(e_m, n_m, u_m)

                track.mode_probability_turn = imm.mode_probability_turn

            elif (
                sbs_msg.ground_speed is not None
                and sbs_msg.track is not None
                and imm.last_update_time_s is not None
            ):
                spd_mps = sbs_msg.ground_speed * KNOTS_TO_MPS
                rad = math.radians(sbs_msg.track)
                ve_mps = spd_mps * math.sin(rad)
                vn_mps = spd_mps * math.cos(rad)
                vu_mps = float(sbs_msg.vertical_rate or 0) * FPM_TO_MPS
                dt = self.sim_time_s - imm.last_update_time_s
                if dt > 0.0:
                    imm.predict(dt)
                    imm.last_update_time_s = self.sim_time_s
                imm.update_velocity(ve_mps, vn_mps, vu_mps)
                track.mode_probability_turn = imm.mode_probability_turn

            elif sbs_msg.altitude is not None and imm.last_update_time_s is not None:
                dt = self.sim_time_s - imm.last_update_time_s
                if dt > 0.0:
                    imm.predict(dt)
                    imm.last_update_time_s = self.sim_time_s
                imm.update_altitude(float(sbs_msg.altitude) * FEET_TO_METERS)

            if sbs_msg.squawk:
                self._squawks[icao24] = sbs_msg.squawk

            track.state = lifecycle.register_hit(self.sim_time_s, track.has_position)

            await self.broadcast_wire(b"", raw_str=raw_str)
            await self.broadcast_telemetry(self.get_telemetry_snapshot(icao24=icao24))

            return {
                "icao_hex": sbs_msg.hex_ident.upper(),
                "df": 17,
                "callsign": sbs_msg.callsign,
                "crc_valid": True,
                "lat": sbs_msg.lat,
                "lon": sbs_msg.lon,
                "altitude_ft": sbs_msg.altitude,
                "groundspeed_kt": sbs_msg.ground_speed,
            }

    def register_telemetry_client(self, ws: WebSocket) -> None:
        """Register active WebSocket for telemetry updates."""
        self._telemetry_clients.add(ws)

    def unregister_telemetry_client(self, ws: WebSocket) -> None:
        """Unregister WebSocket from telemetry updates."""
        self._telemetry_clients.discard(ws)

    def register_wire_client(self, ws: WebSocket, format_name: str = "AVR") -> None:
        """Register active WebSocket for raw wire stream."""
        self._wire_clients[ws] = format_name

    def unregister_wire_client(self, ws: WebSocket) -> None:
        """Unregister WebSocket from raw wire stream."""
        self._wire_clients.pop(ws, None)

    async def broadcast_telemetry(self, data: dict[str, Any]) -> None:
        """Broadcast JSON state vector dictionary to all connected telemetry clients."""
        for ws in list(self._telemetry_clients):
            try:
                await ws.send_json(data)
            except Exception:
                self._telemetry_clients.discard(ws)

    async def broadcast_wire(self, raw_bytes: bytes, raw_str: str | None = None) -> None:
        """Broadcast raw wire encoded messages to connected wire clients."""
        for ws, fmt in list(self._wire_clients.items()):
            try:
                fmt_upper = fmt.upper()
                if fmt_upper == "AVR":
                    if raw_str and raw_str.startswith(("*", "@")):
                        await ws.send_text(raw_str)
                    elif raw_bytes:
                        await ws.send_text(serialize_avr_frame(raw_bytes))
                elif fmt_upper == "BEAST":
                    if raw_bytes:
                        beast_frame_bytes = serialize_beast_frame(
                            frame_type="3" if len(raw_bytes) == 14 else "2",
                            mlat_timestamp=int(self.sim_time_s * 12_000_000) & 0xFFFFFFFFFFFF,
                            rssi_byte=200,
                            payload=raw_bytes,
                        )
                        await ws.send_bytes(beast_frame_bytes)
                elif fmt_upper == "SBS":
                    if raw_str:
                        await ws.send_text(raw_str)
                    elif raw_bytes:
                        text = f"MSG,1,1,1,{raw_bytes[1:4].hex().upper()},1,,,,,,,{raw_bytes.hex().upper()}\r\n"
                        await ws.send_text(text)
                else:
                    if raw_str and raw_str.startswith(("*", "@")):
                        await ws.send_text(raw_str)
                    elif raw_bytes:
                        await ws.send_text(serialize_avr_frame(raw_bytes))
            except Exception:
                self._wire_clients.pop(ws, None)

    def track_to_dict(self, track: Track) -> dict[str, Any]:
        """Serialize Track instance into standardized ATC telemetry dictionary."""
        icao_hex = f"{track.icao24:06X}"
        state_str = track.state.value if hasattr(track.state, "value") else str(track.state)
        squawk = self._squawks.get(track.icao24, "1200")
        lat = round(track.lat_deg, 5) if track.has_position else None
        lon = round(track.lon_deg, 5) if track.has_position else None

        return {
            "icao24": track.icao24,
            "icao_hex": icao_hex,
            "callsign": track.callsign.strip(),
            "squawk": squawk,
            "state": state_str,
            "lat_deg": lat,
            "lon_deg": lon,
            "lat": lat,
            "lon": lon,
            "alt_baro_ft": round(track.alt_baro_ft) if track.alt_baro_ft is not None else None,
            "alt_geom_ft": round(track.alt_geom_ft) if track.alt_geom_ft is not None else None,
            "groundspeed_kt": (
                round(track.groundspeed_kt, 1) if track.groundspeed_kt is not None else None
            ),
            "track_deg": round(track.track_deg, 1) if track.track_deg is not None else None,
            "vertical_rate_fpm": (
                round(track.vertical_rate_fpm) if track.vertical_rate_fpm is not None else None
            ),
            "mode_probability_turn": round(track.mode_probability_turn, 2),
            "anomalies": list(track.anomalies),
            "last_seen_s": round(track.last_seen_s, 2),
            "has_position": track.has_position,
        }

    def get_telemetry_snapshot(self, icao24: int | None = None) -> dict[str, Any]:
        """Build telemetry payload snapshot for websocket broadcast."""
        assert self.engine is not None
        all_tracks = [self.track_to_dict(t) for t in self.engine.all_tracks(include_tentative=True)]
        snapshot: dict[str, Any] = {
            "type": "telemetry",
            "sim_time_s": round(self.sim_time_s, 2),
            "tracks": all_tracks,
            "track_count": len(all_tracks),
            "receivers": self.get_receivers(),
            "interference_sources": self.get_interference_sources(),
        }
        if icao24 is not None:
            snapshot["icao24"] = icao24
        return snapshot

    def add_receiver(self, payload: Any) -> dict[str, Any]:
        """Place and register a ground receiver station."""
        if isinstance(payload, ReceiverSite):
            rx = payload
        else:
            if hasattr(payload, "model_dump"):
                data = payload.model_dump()
            elif isinstance(payload, dict):
                data = dict(payload)
            else:
                data = {
                    "rx_id": getattr(payload, "rx_id", None),
                    "name": getattr(payload, "name", None),
                    "lat_deg": getattr(payload, "lat_deg", getattr(payload, "lat", 0.0)),
                    "lon_deg": getattr(payload, "lon_deg", getattr(payload, "lon", 0.0)),
                    "alt_msl_m": getattr(payload, "alt_msl_m", getattr(payload, "alt_m", 10.0)),
                    "antenna_gain_peak_dbi": getattr(payload, "antenna_gain_peak_dbi", 5.0),
                    "sensitivity_dbm": getattr(payload, "sensitivity_dbm", -84.0),
                    "noise_floor_dbm": getattr(payload, "noise_floor_dbm", -90.0),
                }

            lat = float(data.get("lat_deg", data.get("lat", 0.0)))
            lon = float(data.get("lon_deg", data.get("lon", 0.0)))
            if not (-90.0 <= lat <= 90.0):
                raise ValueError(f"Latitude must be between -90.0 and +90.0, got {lat}")
            if not (-180.0 <= lon <= 180.0):
                raise ValueError(f"Longitude must be between -180.0 and +180.0, got {lon}")

            sens = float(data.get("sensitivity_dbm", data.get("sensitivity", -84.0)))
            if not (-120.0 <= sens <= -40.0):
                raise ValueError(f"Sensitivity must be between -120.0 and -40.0 dBm, got {sens}")

            gain = float(data.get("antenna_gain_peak_dbi", data.get("antenna_gain_dbi", 5.0)))
            noise = float(data.get("noise_floor_dbm", -90.0))
            alt = float(data.get("alt_msl_m", data.get("alt_m", 10.0)))

            rx_id = data.get("rx_id")
            if not rx_id or not str(rx_id).strip():
                rx_id = f"RX_{int(abs(lat)*100):04d}_{int(abs(lon)*100):04d}"
            else:
                rx_id = str(rx_id).strip()

            raw_name = data.get("name")
            name = str(raw_name).strip() if raw_name and str(raw_name).strip() else rx_id

            rx = ReceiverSite(
                lat_deg=lat,
                lon_deg=lon,
                alt_hae_m=alt,
                alt_msl_m=alt,
                rx_id=rx_id,
                name=name,
                antenna_gain_peak_dbi=gain,
                sensitivity_dbm=sens,
                noise_floor_dbm=noise,
            )

        self._receivers[rx.rx_id] = rx
        self._receiver_status[rx.rx_id] = ReceiverStatus(
            rx_id=rx.rx_id,
            name=rx.name,
            lat_deg=rx.lat_deg,
            lon_deg=rx.lon_deg,
            alt_msl_m=rx.alt_msl_m,
            quiescent_noise_floor_dbm=rx.noise_floor_dbm,
            current_noise_floor_dbm=rx.noise_floor_dbm,
            current_sinr_db=25.0,
        )
        self._update_all_receiver_noise_floors()
        result = self.get_receiver(rx.rx_id)
        assert result is not None
        return result

    def remove_receiver(self, rx_id: str) -> bool:
        """Remove placed ground receiver station."""
        removed = self._receivers.pop(rx_id, None) is not None
        self._receiver_status.pop(rx_id, None)
        return removed

    def get_receiver(self, rx_id: str) -> dict[str, Any] | None:
        """Retrieve a specific receiver station dictionary by ID."""
        rx = self._receivers.get(rx_id)
        if rx is None:
            return None
        status = self._receiver_status.get(
            rx_id,
            ReceiverStatus(
                rx_id=rx.rx_id,
                name=rx.name,
                lat_deg=rx.lat_deg,
                lon_deg=rx.lon_deg,
                alt_msl_m=rx.alt_msl_m,
                quiescent_noise_floor_dbm=rx.noise_floor_dbm,
                current_noise_floor_dbm=rx.noise_floor_dbm,
                current_sinr_db=25.0,
            ),
        )
        status_str = self._determine_receiver_status_str(status)
        active_jammers = self._count_active_jammers_in_range(rx)
        cov_km = round(compute_radio_horizon(10000.0, rx.effective_alt_m) / 1000.0, 1)

        return {
            "rx_id": rx.rx_id,
            "name": rx.name,
            "lat_deg": rx.lat_deg,
            "lon_deg": rx.lon_deg,
            "lat": rx.lat_deg,
            "lon": rx.lon_deg,
            "alt_msl_m": rx.alt_msl_m,
            "antenna_gain_peak_dbi": rx.antenna_gain_peak_dbi,
            "sensitivity_dbm": rx.sensitivity_dbm,
            "noise_floor_dbm": rx.noise_floor_dbm,
            "status": status_str,
            "metrics": {
                "sinr_db": status.current_sinr_db,
                "noise_floor_eff_dbm": status.current_noise_floor_dbm,
                "instantaneous_ber": status.instantaneous_ber,
                "average_ber": round(status.cumulative_ber, 6),
                "total_frames_detected": status.total_frames_received,
                "dropped_frames_count": status.total_frames_dropped,
                "packet_loss_rate": round(status.packet_loss_rate_pct / 100.0, 4),
                "active_jammers_in_range": active_jammers,
            },
            "coverage_radius_km": cov_km,
        }

    def get_receivers(self) -> list[dict[str, Any]]:
        """Retrieve all placed ground receiver stations and their live metrics."""
        return [
            rx_dict
            for rx_id in list(self._receivers.keys())
            if (rx_dict := self.get_receiver(rx_id)) is not None
        ]

    def get_receiver_status(self, rx_id: str) -> dict[str, Any] | None:
        """Retrieve live RF health diagnostics and degradation metrics for a receiver."""
        base = self.get_receiver(rx_id)
        if base is None:
            return None
        rx = self._receivers[rx_id]
        dominant_id, dominant_power = self._interference_engine.find_dominant_jammer(rx)
        active_sources = [
            s.source_id
            for s in self._interference_engine.list_sources()
            if s.is_active
        ]
        return {
            "rx_id": base["rx_id"],
            "name": base["name"],
            "lat_deg": base["lat_deg"],
            "lon_deg": base["lon_deg"],
            "alt_msl_m": base["alt_msl_m"],
            "status": base["status"],
            "metrics": base["metrics"],
            "dominant_jammer_id": dominant_id,
            "dominant_jammer_power_dbm": dominant_power,
            "active_interference_sources": active_sources,
        }

    def add_interference_source(self, payload: Any) -> dict[str, Any]:
        """Place and deploy an RF interference source / jammer."""
        if isinstance(payload, InterferenceSource):
            source = payload
        else:
            if hasattr(payload, "model_dump"):
                data = payload.model_dump()
            elif isinstance(payload, dict):
                data = dict(payload)
            else:
                data = {
                    "source_id": getattr(payload, "source_id", None),
                    "name": getattr(payload, "name", None),
                    "lat_deg": getattr(payload, "lat_deg", getattr(payload, "lat", 0.0)),
                    "lon_deg": getattr(payload, "lon_deg", getattr(payload, "lon", 0.0)),
                    "alt_msl_m": getattr(payload, "alt_msl_m", getattr(payload, "alt_m", 10.0)),
                    "tx_power_dbm": getattr(payload, "tx_power_dbm", getattr(payload, "power_dbm", 40.0)),
                    "frequency_hz": getattr(payload, "frequency_hz", 1090.0e6),
                    "bandwidth_mhz": getattr(payload, "bandwidth_mhz", 2.0),
                    "is_active": getattr(payload, "is_active", True),
                }

            lat = float(data.get("lat_deg", data.get("lat", 0.0)))
            lon = float(data.get("lon_deg", data.get("lon", 0.0)))
            if not (-90.0 <= lat <= 90.0):
                raise ValueError(f"Latitude must be between -90.0 and +90.0, got {lat}")
            if not (-180.0 <= lon <= 180.0):
                raise ValueError(f"Longitude must be between -180.0 and +180.0, got {lon}")

            power = float(data.get("tx_power_dbm", data.get("power_dbm", 40.0)))
            if not (0.0 <= power <= 80.0):
                raise ValueError(f"Transmit power must be between 0.0 and 80.0 dBm, got {power}")

            freq = float(data.get("frequency_hz", 1090.0e6))
            bw = float(data.get("bandwidth_mhz", 2.0))
            active = bool(data.get("is_active", True))

            source_id = data.get("source_id")
            if not source_id or not str(source_id).strip():
                source_id = f"JAMMER_{len(self._interference_engine.list_sources())+1:02d}"
            else:
                source_id = str(source_id).strip()

            raw_name = data.get("name")
            name = str(raw_name).strip() if raw_name and str(raw_name).strip() else f"Jammer {source_id}"

            source = InterferenceSource(
                source_id=source_id,
                name=name,
                lat_deg=lat,
                lon_deg=lon,
                alt_msl_m=float(data.get("alt_msl_m", 10.0)),
                tx_power_dbm=power,
                frequency_hz=freq,
                bandwidth_mhz=bw,
                is_active=active,
            )

        self._interference_engine.add_source(source)
        self._update_all_receiver_noise_floors()
        result = self.get_interference_source(source.source_id)
        assert result is not None
        return result

    def get_interference_sources(self) -> list[dict[str, Any]]:
        """Retrieve all configured RF interference sources."""
        return [
            self._interference_to_dict(s)
            for s in self._interference_engine.list_sources()
        ]

    def get_interference_source(self, source_id: str) -> dict[str, Any] | None:
        """Retrieve a specific RF interference source dictionary by ID."""
        s = self._interference_engine.get_source(source_id)
        if s is None:
            return None
        return self._interference_to_dict(s)

    def _interference_to_dict(self, s: InterferenceSource) -> dict[str, Any]:
        """Convert InterferenceSource to serializable dict with calculated properties."""
        return {
            "source_id": s.source_id,
            "name": s.name,
            "lat_deg": s.lat_deg,
            "lon_deg": s.lon_deg,
            "lat": s.lat_deg,
            "lon": s.lon_deg,
            "alt_msl_m": s.alt_msl_m,
            "tx_power_dbm": s.tx_power_dbm,
            "tx_power_w": round(s.tx_power_w, 2),
            "frequency_hz": s.frequency_hz,
            "bandwidth_mhz": s.bandwidth_mhz,
            "is_active": s.is_active,
            "effective_range_km": s.effective_range_km,
        }

    def update_interference_source(self, source_id: str, payload: Any) -> dict[str, Any] | None:
        """Update active state or power level for an RF interference source."""
        if hasattr(payload, "model_dump"):
            data = payload.model_dump(exclude_unset=True)
        elif isinstance(payload, dict):
            data = payload
        else:
            data = {}
            if hasattr(payload, "is_active") and payload.is_active is not None:
                data["is_active"] = payload.is_active
            if hasattr(payload, "tx_power_dbm") and payload.tx_power_dbm is not None:
                data["tx_power_dbm"] = payload.tx_power_dbm
            if hasattr(payload, "bandwidth_mhz") and payload.bandwidth_mhz is not None:
                data["bandwidth_mhz"] = payload.bandwidth_mhz

        updated = self._interference_engine.update_source(
            source_id=source_id,
            is_active=data.get("is_active"),
            tx_power_dbm=data.get("tx_power_dbm"),
            bandwidth_mhz=data.get("bandwidth_mhz"),
        )
        if updated is None:
            return None
        self._update_all_receiver_noise_floors()
        return self.get_interference_source(source_id)

    def toggle_interference_source(self, source_id: str) -> dict[str, Any] | None:
        """Toggle an RF interference source on or off."""
        updated = self._interference_engine.toggle_source(source_id)
        if updated is None:
            return None
        self._update_all_receiver_noise_floors()
        return self.get_interference_source(source_id)

    def remove_interference_source(self, source_id: str) -> bool:
        """Remove an RF interference source."""
        removed = self._interference_engine.remove_source(source_id)
        if removed:
            self._update_all_receiver_noise_floors()
        return removed

    def add_flight(self, request: FlightCreateRequest) -> FlightCreationResult:
        """Dynamically generate and inject a custom flight into the active simulation.

        Parameters:
            request: FlightCreateRequest specification.

        Returns:
            FlightCreationResult: Object containing 24-bit ICAO address hex string
                and effective capped cruise altitude in feet.

        Raises:
            ValueError: If airport codes or aircraft type are invalid.
        """
        from adsb.synthesis.airports import has_airport
        from adsb.synthesis.performance import get_aircraft_profile, has_aircraft_profile
        from adsb.synthesis.scenario import ScenarioAircraft
        from adsb.synthesis.scheduler import SquitterScheduler
        from adsb.synthesis.trajectory import generate_terminal_routed_trajectory

        if self.engine is None:
            self._init_simulation()
        assert self.engine is not None

        origin_code = request.origin_icao.strip().upper()
        dest_code = request.destination_icao.strip().upper()
        ac_type = request.aircraft_type.strip().upper()

        if not has_airport(origin_code):
            raise ValueError(f"Unknown origin airport ICAO code: '{origin_code}'")
        if not has_airport(dest_code):
            raise ValueError(f"Unknown destination airport ICAO code: '{dest_code}'")
        if origin_code == dest_code:
            raise ValueError("Origin and destination airports must be different")
        if not has_aircraft_profile(ac_type):
            raise ValueError(f"Unknown aircraft type: '{ac_type}'")

        profile = get_aircraft_profile(ac_type)

        with self._injection_lock:
            # 1. Resolve 24-bit ICAO address
            if request.icao_hex and request.icao_hex.strip():
                clean_hex = request.icao_hex.strip().upper()
                if clean_hex.startswith("0X"):
                    clean_hex = clean_hex[2:]
                try:
                    icao24 = int(clean_hex, 16)
                except ValueError as err:
                    raise ValueError(f"Invalid ICAO hex string: '{request.icao_hex}'") from err
                icao_hex = f"{icao24:06X}"
                self._allocated_icaos.add(icao24)
            else:
                # Atomic deterministic allocation avoiding collisions with existing scenario aircraft, tracks & concurrent allocations
                existing_icaos = self._allocated_icaos | {
                    a.icao24 for a in (self.scenario.aircraft if self.scenario else [])
                } | set(self.engine._tracks.keys())
                candidate = 0x484100
                while candidate in existing_icaos:
                    candidate += 1
                icao24 = candidate
                icao_hex = f"{icao24:06X}"
                self._allocated_icaos.add(icao24)

            # 2. Resolve Callsign
            if request.callsign and request.callsign.strip():
                callsign = request.callsign.strip().upper()
            else:
                callsign = f"SIM{icao24 & 0xFFF:03X}"

            # 3. Resolve Squawk & Cruise Altitude
            squawk = request.squawk.strip() if request.squawk and request.squawk.strip() else "1200"
            self._squawks[icao24] = squawk

            cruise_alt_ft = (
                request.cruise_alt_ft
                if request.cruise_alt_ft is not None
                else profile.default_cruise_alt_ft
            )

            # 4. Departure time aligned with active cycle offset
            t_start_s = max(0.0, self.sim_time_s - self._cycle_offset_s)

            # 5. Deterministic RNG for flight trajectory and squitters
            flight_idx = len(self._allocated_icaos)
            rng = np.random.default_rng(self.config.seed + 1000 + flight_idx)

        # 6. Generate 4D trajectory states & DO-260B squitters outside lock
        try:
            states = generate_terminal_routed_trajectory(
                origin_icao=origin_code,
                dest_icao=dest_code,
                aircraft_type=ac_type,
                callsign=callsign,
                cruise_alt_ft=cruise_alt_ft,
                squawk=squawk,
                icao_hex=icao_hex,
                departure_time_s=t_start_s,
                dt_s=1.0,
                rng=rng,
            )

            # 7. Generate DO-260B squitters
            scheduler = SquitterScheduler(
                icao24=icao24,
                callsign=callsign,
                rng=rng,
                profile=profile,
                include_df11=True,
            )
            new_events = scheduler.generate_squitters(states)
        except Exception:
            with self._injection_lock:
                self._allocated_icaos.discard(icao24)
            raise

        # 8. Register in scenario and 9. Chronologically merge events inside lock
        with self._injection_lock:
            if self.scenario is not None:
                self.scenario.aircraft.append(
                    ScenarioAircraft(
                        icao24=icao24,
                        callsign=callsign,
                        states=states,
                        profile=profile,
                    )
                )
                # Extend scenario duration if flight exceeds existing limit
                if states and states[-1].timestamp_s > self.scenario.duration_s:
                    object.__setattr__(self.scenario, "duration_s", states[-1].timestamp_s + 60.0)

            # Chronologically merge events without race condition
            past_events = self._events[: self._event_idx]
            future_events = self._events[self._event_idx :] + new_events
            future_events.sort(key=lambda ev: ev.timestamp_s)
            self._events = past_events + future_events

        logger.info(
            "Created custom flight %s (%s) %s -> %s (%d squitters, duration=%.1fs)",
            callsign,
            icao_hex,
            origin_code,
            dest_code,
            len(new_events),
            states[-1].timestamp_s - states[0].timestamp_s if states else 0.0,
        )

        effective_cruise_alt = (
            states[0].alt_mcp_selected_ft if states else cruise_alt_ft
        )
        return FlightCreationResult(
            icao_hex=icao_hex,
            effective_cruise_alt_ft=effective_cruise_alt,
        )

    async def shutdown(self) -> None:
        """Gracefully cancel tasks and clean up connections."""
        self.pause()
        for ws in list(self._telemetry_clients):
            with contextlib.suppress(Exception):
                await ws.close(code=1000, reason="Server shutting down")
        self._telemetry_clients.clear()

        for ws in list(self._wire_clients.keys()):
            with contextlib.suppress(Exception):
                await ws.close(code=1000, reason="Server shutting down")
        self._wire_clients.clear()


_manager_instance: SimulationManager | None = None


def get_simulation_manager() -> SimulationManager:
    """Retrieve process-wide singleton SimulationManager."""
    global _manager_instance
    if _manager_instance is None:
        _manager_instance = SimulationManager()
    return _manager_instance


def reset_simulation_manager() -> SimulationManager:
    """Reset or recreate singleton SimulationManager."""
    global _manager_instance
    if _manager_instance is not None:
        _manager_instance.reset()
    else:
        _manager_instance = SimulationManager()
    return _manager_instance


__all__ = [
    "FlightCreateRequest",
    "FlightCreationResult",
    "SimulationConfig",
    "SimulationManager",
    "SimulationStatus",
    "get_simulation_manager",
    "reset_simulation_manager",
]
