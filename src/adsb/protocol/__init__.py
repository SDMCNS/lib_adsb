"""Protocol codecs for Mode S, 1090ES Extended Squitter, Comm-B BDS, and 978 MHz UAT."""

from __future__ import annotations

from typing import Any

from adsb.protocol.bds import (
    AIS_CHARMAP,
    BDS10Data,
    BDS10DataLinkCapability,
    BDS17Data,
    BDS17GICBCapability,
    BDS20AircraftIdentification,
    BDS20Data,
    BDS40Data,
    BDS40SelectedVerticalIntention,
    BDS50Data,
    BDS50TrackAndTurnReport,
    BDS60Data,
    BDS60HeadingAndSpeedReport,
    classify_bds,
    decode_bds_10,
    decode_bds_17,
    decode_bds_20,
    decode_bds_40,
    decode_bds_50,
    decode_bds_60,
    decode_comm_b,
)
from adsb.protocol.cpr import (
    cpr_nl_table,
    decode_cpr_airborne,
    decode_cpr_global_airborne,
    decode_cpr_global_surface,
    decode_cpr_local,
    decode_cpr_surface,
    encode_cpr_airborne,
    encode_cpr_surface,
    nl_lat,
)
from adsb.protocol.crc import (
    CRC24_POLY,
    CRC24_TABLE,
    SYNDROME_TABLE,
    calculate_crc24,
    compute_crc24,
    compute_syndrome,
    correct_crc_errors,
    correct_syndrome_errors,
    modes_syndrome,
    verify_crc24,
)
from adsb.protocol.gillham import (
    binary_to_gray,
    decode_altitude,
    decode_gillham_altitude,
    decode_mode_s_altitude,
    decode_mode_s_identity,
    decode_squawk,
    encode_altitude,
    encode_gillham_altitude,
    encode_squawk,
    encode_squawk_code,
    gray_to_binary,
)
from adsb.protocol.modes import (
    DecodedModeSMessage,
    ESAirbornePositionMsg,
    ESAirborneVelocityMsg,
    ESAircraftStatusMsg,
    ESIdentificationMsg,
    ESOperationalStatusMsg,
    ESSurfacePositionMsg,
    ESTargetStateStatusMsg,
    decode_1090es,
    decode_mode_s,
    encode_1090es_airborne_position,
    encode_1090es_airborne_velocity,
    encode_1090es_identification,
    encode_mode_s,
)
from adsb.protocol.uat import (
    AltitudeSource,
    ReedSolomonCodec,
    UATAuxStateVector,
    UATHeader,
    UATMessage,
    UATModeStatus,
    UATReedSolomonError,
    UATStateVector,
    decode_uat,
    decode_uat_auxsv,
    decode_uat_basic,
    decode_uat_frame,
    decode_uat_header,
    decode_uat_long,
    decode_uat_ms,
    decode_uat_sv,
    encode_uat,
    encode_uat_auxsv,
    encode_uat_basic,
    encode_uat_frame,
    encode_uat_header,
    encode_uat_long,
    encode_uat_ms,
    encode_uat_sv,
)


def decode_frame(raw_bytes: bytes, **kwargs: Any) -> Any:
    """Unified polymorphic frame dispatcher.

    Routes raw binary bytes to the appropriate decoder based on frame length:
    - 2 bytes (16 bits): Mode A/C octal squawk
    - 7 bytes (56 bits): Mode S Short (DF 0, 4, 5, 11)
    - 14 bytes (112 bits): Mode S Long (DF 16, 17, 18, 20, 21, 24)
    - 18 bytes (144 bits): UAT Basic payload (uncorrected)
    - 30 bytes (240 bits): UAT Basic frame with RS(30,18) FEC
    - 34 bytes (272 bits): UAT Long payload (uncorrected)
    - 48 bytes (384 bits): UAT Long frame with RS(48,34) FEC
    """
    length = len(raw_bytes)
    if length in (7, 14):
        return decode_mode_s(raw_bytes, **kwargs)
    elif length in (18, 30, 34, 48):
        return decode_uat(raw_bytes, **kwargs)
    elif length == 2:
        val = int.from_bytes(raw_bytes, "big")
        return decode_squawk(val)
    else:
        raise ValueError(f"Unrecognized frame length: {length} bytes")


__all__ = [
    "AIS_CHARMAP",
    "CRC24_POLY",
    "CRC24_TABLE",
    "SYNDROME_TABLE",
    "AltitudeSource",
    "BDS10Data",
    "BDS10DataLinkCapability",
    "BDS17Data",
    "BDS17GICBCapability",
    "BDS20AircraftIdentification",
    "BDS20Data",
    "BDS40Data",
    "BDS40SelectedVerticalIntention",
    "BDS50Data",
    "BDS50TrackAndTurnReport",
    "BDS60Data",
    "BDS60HeadingAndSpeedReport",
    "DecodedModeSMessage",
    "ESAirbornePositionMsg",
    "ESAirborneVelocityMsg",
    "ESAircraftStatusMsg",
    "ESIdentificationMsg",
    "ESOperationalStatusMsg",
    "ESSurfacePositionMsg",
    "ESTargetStateStatusMsg",
    "ReedSolomonCodec",
    "UATAuxStateVector",
    "UATHeader",
    "UATMessage",
    "UATModeStatus",
    "UATReedSolomonError",
    "UATStateVector",
    "binary_to_gray",
    "calculate_crc24",
    "classify_bds",
    "compute_crc24",
    "compute_syndrome",
    "correct_crc_errors",
    "correct_syndrome_errors",
    "cpr_nl_table",
    "decode_1090es",
    "decode_altitude",
    "decode_bds_10",
    "decode_bds_17",
    "decode_bds_20",
    "decode_bds_40",
    "decode_bds_50",
    "decode_bds_60",
    "decode_comm_b",
    "decode_cpr_airborne",
    "decode_cpr_global_airborne",
    "decode_cpr_global_surface",
    "decode_cpr_local",
    "decode_cpr_surface",
    "decode_frame",
    "decode_gillham_altitude",
    "decode_mode_s",
    "decode_mode_s_altitude",
    "decode_mode_s_identity",
    "decode_squawk",
    "decode_uat",
    "decode_uat_auxsv",
    "decode_uat_basic",
    "decode_uat_frame",
    "decode_uat_header",
    "decode_uat_long",
    "decode_uat_ms",
    "decode_uat_sv",
    "encode_1090es_airborne_position",
    "encode_1090es_airborne_velocity",
    "encode_1090es_identification",
    "encode_altitude",
    "encode_cpr_airborne",
    "encode_cpr_surface",
    "encode_gillham_altitude",
    "encode_mode_s",
    "encode_squawk",
    "encode_squawk_code",
    "encode_uat",
    "encode_uat_auxsv",
    "encode_uat_basic",
    "encode_uat_frame",
    "encode_uat_header",
    "encode_uat_long",
    "encode_uat_ms",
    "encode_uat_sv",
    "gray_to_binary",
    "modes_syndrome",
    "nl_lat",
    "verify_crc24",
]
