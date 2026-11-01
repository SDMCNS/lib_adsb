"""Re-export alias for RF channel simulation components.

Provides direct concordance with docs/01_ARCHITECTURE_AND_DESIGN.md
and tests/integration/test_e2e_closed_loop.py.
"""

from __future__ import annotations

from adsb.impairment.channel import (
    AntennaPatternType,
    ChannelConfig,
    ChannelPropagationResult,
    DropoutReason,
    FadingModel,
    ReceiverSite,
    RFChannelModel,
    RFPropagationChannel,
    apply_bit_errors,
    check_terrain_mask,
    compute_antenna_gain,
    compute_beast_mlat_counter,
    compute_fspl,
    compute_ppm_ber,
    compute_preamble_drop_prob,
    compute_radio_horizon,
    compute_slant_range,
)

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
