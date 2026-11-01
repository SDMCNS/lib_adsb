"""Re-export alias for Stage A avionics and transponder fault simulation.

Provides concordance with docs/04_ERROR_AND_ANOMALY_SIMULATION.md Section 1.
"""

from __future__ import annotations

from adsb.impairment.avionics import (
    GILLHAM_WIRE_BITS,
    AltitudeInversionConfig,
    AvionicsFaultConfig,
    AvionicsFaultInjector,
    CPRDesyncConfig,
    CPRDesyncMode,
    CPRStraddleConfig,
    DuplicateICAOConfig,
    GillhamGlitchConfig,
    GPSJammingConfig,
    StaleGPSConfig,
    apply_altitude_inversion_fault,
    apply_cpr_desync_fault,
    apply_cpr_straddle_fault,
    apply_duplicate_icao_fault,
    apply_gillham_stuck_bit_fault,
    apply_gps_jamming_fault,
    apply_stale_gps_fault,
    apply_stale_gps_to_states,
    recompute_frame_crc,
)

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
    "apply_stale_gps_fault",
    "apply_stale_gps_to_states",
    "recompute_frame_crc",
]
