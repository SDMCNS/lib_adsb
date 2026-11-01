"""Noise models, pulse collision physics, capture effect, and background FRUIT generator.

Authoritative reference: RTCA DO-260B / DO-260C, ICAO Annex 10 Vol IV,
and docs/04_ERROR_AND_ANOMALY_SIMULATION.md Section 2.4.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from adsb.protocol.modes import encode_1090es_df11, encode_1090es_identification

# Mode S / ADS-B physical timing constants (in nanoseconds)
MODE_S_PREAMBLE_DURATION_NS: int = 8_000  # 8.0 microseconds
MODE_S_BIT_DURATION_NS: int = 1_000  # 1.0 microsecond (1 Mbps PPM)
MODE_S_SHORT_FRAME_DURATION_NS: int = 64_000  # 56-bit: 8 us + 56 us = 64 us
MODE_S_LONG_FRAME_DURATION_NS: int = 120_000  # 112-bit: 8 us + 112 us = 120 us
MODE_AC_FRAME_DURATION_NS: int = 20_300  # Mode A/C framing F1-F2: 20.3 us


@dataclass(frozen=True, slots=True)
class ReceptionEvent:
    """Immutable physical RF reception event at an antenna/receiver site.

    Attributes:
        rx_timestamp_ns: Integer arrival timestamp at receiver in nanoseconds.
        rx_id: Identifier of the receiving station (e.g., 'DEFAULT_RX').
        raw_bytes: Demodulated wire bytes (7 or 14 bytes), potentially corrupted.
        rssi_dbfs: Received Signal Strength Indicator in dBFS.
        snr_db: Signal-to-Noise Ratio in dB.
        is_corrupted: True if packet suffered uncorrected garble or bit error.
    """

    rx_timestamp_ns: int
    rx_id: str
    raw_bytes: bytes
    rssi_dbfs: float
    snr_db: float
    is_corrupted: bool

    @property
    def rx_timestamp_s(self) -> float:
        """Arrival timestamp converted to float seconds for downstream consumers."""
        return self.rx_timestamp_ns / 1e9


class PulseCollisionEngine:
    """Simulates 1090 MHz PPM pulse collisions, RF capture effect, and destructive garble."""

    def __init__(
        self,
        capture_threshold_db: float = 6.0,
        garble_bit_flip_prob: float = 0.35,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.capture_threshold_db = capture_threshold_db
        self.garble_bit_flip_prob = garble_bit_flip_prob
        self.rng = rng if rng is not None else np.random.default_rng()

    @staticmethod
    def get_frame_duration_ns(frame_len_bytes: int) -> int:
        """Return the physical air duration in nanoseconds for a frame of given byte length."""
        if frame_len_bytes == 14:
            return MODE_S_LONG_FRAME_DURATION_NS
        elif frame_len_bytes == 7:
            return MODE_S_SHORT_FRAME_DURATION_NS
        elif frame_len_bytes <= 2:
            return MODE_AC_FRAME_DURATION_NS
        else:
            return MODE_S_PREAMBLE_DURATION_NS + (frame_len_bytes * 8 * MODE_S_BIT_DURATION_NS)

    @staticmethod
    def check_overlap(
        t_a_ns: int, dur_a_ns: int, t_b_ns: int, dur_b_ns: int
    ) -> tuple[bool, int, int]:
        """Check if two pulse trains overlap in time.

        Returns:
            (is_overlapping, overlap_start_ns, overlap_end_ns)
        """
        t_start = max(t_a_ns, t_b_ns)
        t_end = min(t_a_ns + dur_a_ns, t_b_ns + dur_b_ns)
        if t_start < t_end:
            return (True, t_start, t_end)
        return (False, 0, 0)

    @staticmethod
    def get_overlapping_bit_indices(
        frame_start_ns: int, total_bits: int, overlap_start_ns: int, overlap_end_ns: int
    ) -> list[int]:
        """Identify which 0-based data bit indices overlap with the collision interval."""
        affected: list[int] = []
        data_start_ns = frame_start_ns + MODE_S_PREAMBLE_DURATION_NS
        for k in range(total_bits):
            bit_start = data_start_ns + k * MODE_S_BIT_DURATION_NS
            bit_end = bit_start + MODE_S_BIT_DURATION_NS
            if max(bit_start, overlap_start_ns) < min(bit_end, overlap_end_ns):
                affected.append(k)
        return affected

    def apply_garble(
        self,
        raw_bytes: bytes,
        affected_bits: list[int],
        flip_prob: float | None = None,
        rng: np.random.Generator | None = None,
    ) -> bytes:
        """Corrupt bits at specified indices within raw_bytes."""
        if not affected_bits or not raw_bytes:
            return raw_bytes

        prob = flip_prob if flip_prob is not None else self.garble_bit_flip_prob
        active_rng = rng if rng is not None else self.rng

        byte_arr = bytearray(raw_bytes)
        flips_applied = 0
        total_bits = len(raw_bytes) * 8

        for bit_idx in affected_bits:
            if bit_idx >= total_bits:
                continue
            if active_rng.random() < prob:
                # Protect Mode S DF field (bits 0..4) to avoid morphing into DF 4/5/20/21
                if total_bits in (56, 112) and bit_idx < 5:
                    target_idx = (total_bits - 1) - bit_idx
                else:
                    target_idx = bit_idx
                byte_pos = target_idx // 8
                bit_pos = 7 - (target_idx % 8)
                byte_arr[byte_pos] ^= (1 << bit_pos)
                flips_applied += 1

        # Guarantee at least 1 bit flip if an overlap occurred but random trials produced 0
        if flips_applied == 0 and affected_bits:
            first_idx = affected_bits[0]
            target_idx = (
                total_bits - 1
                if total_bits in (56, 112) and first_idx < 5
                else first_idx
            )
            if target_idx < total_bits:
                byte_pos = target_idx // 8
                bit_pos = 7 - (target_idx % 8)
                byte_arr[byte_pos] ^= (1 << bit_pos)

        return bytes(byte_arr)

    def apply_collision(
        self,
        frame_a: ReceptionEvent,
        frame_b: ReceptionEvent,
    ) -> tuple[ReceptionEvent, ReceptionEvent]:
        """Apply capture effect or destructive garble between two overlapping reception events."""
        dur_a = self.get_frame_duration_ns(len(frame_a.raw_bytes))
        dur_b = self.get_frame_duration_ns(len(frame_b.raw_bytes))
        overlap, t_start, t_end = self.check_overlap(
            frame_a.rx_timestamp_ns, dur_a, frame_b.rx_timestamp_ns, dur_b
        )

        if not overlap:
            return (frame_a, frame_b)

        delta_rssi = frame_a.rssi_dbfs - frame_b.rssi_dbfs

        # Case 1: Frame A captures cleanly (>= +6.0 dB)
        if delta_rssi >= self.capture_threshold_db:
            b_bits = self.get_overlapping_bit_indices(
                frame_b.rx_timestamp_ns, len(frame_b.raw_bytes) * 8, t_start, t_end
            )
            if not b_bits and len(frame_b.raw_bytes) > 0:
                b_bits = [len(frame_b.raw_bytes) * 8 - 1]
            b_corrupted = self.apply_garble(frame_b.raw_bytes, b_bits)
            frame_b_out = ReceptionEvent(
                rx_timestamp_ns=frame_b.rx_timestamp_ns,
                rx_id=frame_b.rx_id,
                raw_bytes=b_corrupted,
                rssi_dbfs=frame_b.rssi_dbfs,
                snr_db=frame_b.snr_db,
                is_corrupted=True,
            )
            return (frame_a, frame_b_out)

        # Case 2: Frame B captures cleanly (<= -6.0 dB)
        if delta_rssi <= -self.capture_threshold_db:
            a_bits = self.get_overlapping_bit_indices(
                frame_a.rx_timestamp_ns, len(frame_a.raw_bytes) * 8, t_start, t_end
            )
            if not a_bits and len(frame_a.raw_bytes) > 0:
                a_bits = [len(frame_a.raw_bytes) * 8 - 1]
            a_corrupted = self.apply_garble(frame_a.raw_bytes, a_bits)
            frame_a_out = ReceptionEvent(
                rx_timestamp_ns=frame_a.rx_timestamp_ns,
                rx_id=frame_a.rx_id,
                raw_bytes=a_corrupted,
                rssi_dbfs=frame_a.rssi_dbfs,
                snr_db=frame_a.snr_db,
                is_corrupted=True,
            )
            return (frame_a_out, frame_b)

        # Case 3: Destructive Garble (|Delta RSSI| < 6.0 dB)
        a_bits = self.get_overlapping_bit_indices(
            frame_a.rx_timestamp_ns, len(frame_a.raw_bytes) * 8, t_start, t_end
        )
        if not a_bits and len(frame_a.raw_bytes) > 0:
            a_bits = [len(frame_a.raw_bytes) * 8 - 1]
        a_corrupted = self.apply_garble(frame_a.raw_bytes, a_bits)

        b_bits = self.get_overlapping_bit_indices(
            frame_b.rx_timestamp_ns, len(frame_b.raw_bytes) * 8, t_start, t_end
        )
        if not b_bits and len(frame_b.raw_bytes) > 0:
            b_bits = [len(frame_b.raw_bytes) * 8 - 1]
        b_corrupted = self.apply_garble(frame_b.raw_bytes, b_bits)

        frame_a_out = ReceptionEvent(
            rx_timestamp_ns=frame_a.rx_timestamp_ns,
            rx_id=frame_a.rx_id,
            raw_bytes=a_corrupted,
            rssi_dbfs=frame_a.rssi_dbfs,
            snr_db=frame_a.snr_db,
            is_corrupted=True,
        )
        frame_b_out = ReceptionEvent(
            rx_timestamp_ns=frame_b.rx_timestamp_ns,
            rx_id=frame_b.rx_id,
            raw_bytes=b_corrupted,
            rssi_dbfs=frame_b.rssi_dbfs,
            snr_db=frame_b.snr_db,
            is_corrupted=True,
        )
        return (frame_a_out, frame_b_out)

    def resolve_collisions(self, receptions: list[ReceptionEvent]) -> list[ReceptionEvent]:
        """Resolve all pairwise pulse collisions in a time-ordered sequence of reception events."""
        if len(receptions) <= 1:
            return list(receptions)

        # Ensure sorted by arrival timestamp
        sorted_events = sorted(receptions, key=lambda e: e.rx_timestamp_ns)
        events = list(sorted_events)
        n = len(events)

        for i in range(n):
            ev_i = events[i]
            dur_i = self.get_frame_duration_ns(len(ev_i.raw_bytes))
            end_i = ev_i.rx_timestamp_ns + dur_i

            for j in range(i + 1, n):
                ev_j = events[j]
                if ev_j.rx_timestamp_ns >= end_i:
                    # Beyond overlap horizon for event i
                    break

                # Apply collision between event i and event j
                ev_i_new, ev_j_new = self.apply_collision(events[i], events[j])
                events[i] = ev_i_new
                events[j] = ev_j_new

        return events


class PoissonFruitGenerator:
    """Generates asynchronous background False Replies Unsynchronized In Time (FRUIT)."""

    def __init__(
        self,
        fruit_rate_hz: float = 500.0,
        rx_id: str = "DEFAULT_RX",
        noise_floor_dbfs: float = -45.0,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.fruit_rate_hz = fruit_rate_hz
        self.rx_id = rx_id
        self.noise_floor_dbfs = noise_floor_dbfs
        self.rng = rng if rng is not None else np.random.default_rng()

    def generate_fruit(
        self,
        duration_s: float,
        start_timestamp_ns: int = 0,
    ) -> list[ReceptionEvent]:
        """Generate time-ordered Poisson FRUIT events over the specified duration."""
        if duration_s <= 0.0 or self.fruit_rate_hz <= 0.0:
            return []

        events: list[ReceptionEvent] = []
        current_t_s = 0.0

        while True:
            # Exponential inter-arrival time
            dt_s = float(self.rng.exponential(1.0 / self.fruit_rate_hz))
            current_t_s += dt_s
            if current_t_s >= duration_s:
                break

            t_ns = start_timestamp_ns + round(current_t_s * 1e9)

            # Mix of 50% DF11, 30% DF17, 20% Mode A/C
            u = float(self.rng.random())
            if u < 0.50:
                # Mode S DF11 Acquisition Squitter (56 bits, 7 bytes)
                icao = int(self.rng.integers(1, 0xFFFFFF))
                raw_bytes = encode_1090es_df11(icao)
            elif u < 0.80:
                # Mode S DF17 Extended Squitter (112 bits, 14 bytes)
                icao = int(self.rng.integers(1, 0xFFFFFF))
                callsign = f"FRT{icao % 10000:04d}"
                raw_bytes = encode_1090es_identification(icao, callsign)
            else:
                # Mode A/C framing (2 bytes)
                mode_ac_val = int(self.rng.integers(0, 0x1000))
                raw_bytes = mode_ac_val.to_bytes(2, "big")

            # Amplitude distribution: N(-22.0, 7.0) dBFS clamped to [-40.0, -5.0]
            rssi_sample = float(self.rng.normal(-22.0, 7.0))
            rssi_dbfs = max(-40.0, min(-5.0, rssi_sample))
            snr_db = max(0.0, rssi_dbfs - self.noise_floor_dbfs)

            events.append(
                ReceptionEvent(
                    rx_timestamp_ns=t_ns,
                    rx_id=self.rx_id,
                    raw_bytes=raw_bytes,
                    rssi_dbfs=rssi_dbfs,
                    snr_db=snr_db,
                    is_corrupted=False,
                )
            )

        return events


__all__ = [
    "MODE_AC_FRAME_DURATION_NS",
    "MODE_S_BIT_DURATION_NS",
    "MODE_S_LONG_FRAME_DURATION_NS",
    "MODE_S_PREAMBLE_DURATION_NS",
    "MODE_S_SHORT_FRAME_DURATION_NS",
    "PoissonFruitGenerator",
    "PulseCollisionEngine",
    "ReceptionEvent",
]
