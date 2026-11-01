"""adsb — High-Fidelity ADS-B / Mode S / UAT ATM Library."""

from __future__ import annotations

from adsb.core.bits import (
    BitBuffer,
    bytes_to_uint,
    extract_bits,
    set_bits,
    uint_to_bytes,
)
from adsb.core.coordinates import (
    ECEFPosition,
    ENUPosition,
    GeodeticPosition,
    ecef_to_enu,
    ecef_to_geodetic,
    enu_rotation_matrix,
    enu_to_ecef,
    enu_to_geodetic,
    geodetic_to_ecef,
    geodetic_to_enu,
)
from adsb.core.geodesy import (
    along_track_distance,
    cross_track_distance,
    destination_point,
    haversine_distance,
    initial_bearing,
)
from adsb.core.units import (
    deg_to_rad,
    feet_to_meters,
    fpm_to_mps,
    ft_to_m,
    knots_to_mps,
    kt_to_mps,
    m_to_ft,
    meters_to_feet,
    meters_to_nm,
    mps_to_fpm,
    mps_to_knots,
    mps_to_kt,
    nm_to_meters,
    rad_to_deg,
)
from adsb.protocol import (
    decode_frame,
    decode_mode_s,
    decode_uat,
    verify_crc24,
)

__version__ = "0.1.0"

__all__ = [
    "BitBuffer",
    "ECEFPosition",
    "ENUPosition",
    "GeodeticPosition",
    "along_track_distance",
    "bytes_to_uint",
    "cross_track_distance",
    "decode_frame",
    "decode_mode_s",
    "decode_uat",
    "deg_to_rad",
    "destination_point",
    "ecef_to_enu",
    "ecef_to_geodetic",
    "enu_rotation_matrix",
    "enu_to_ecef",
    "enu_to_geodetic",
    "extract_bits",
    "feet_to_meters",
    "fpm_to_mps",
    "ft_to_m",
    "geodetic_to_ecef",
    "geodetic_to_enu",
    "haversine_distance",
    "initial_bearing",
    "knots_to_mps",
    "kt_to_mps",
    "m_to_ft",
    "meters_to_feet",
    "meters_to_nm",
    "mps_to_fpm",
    "mps_to_knots",
    "mps_to_kt",
    "nm_to_meters",
    "rad_to_deg",
    "set_bits",
    "uint_to_bytes",
    "verify_crc24",
]
