"""Mode S and 1090 MHz Extended Squitter (DF17/DF18) decoder and encoders.

Authoritative reference: ICAO Annex 10 Vol IV, RTCA DO-260B / DO-260C,
and docs/02_PROTOCOL_SPECIFICATION.md Sections 3 and 6.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from adsb.core.bits import extract_bits
from adsb.protocol.bds import AIS_CHARMAP
from adsb.protocol.cpr import encode_cpr_airborne, encode_cpr_surface
from adsb.protocol.crc import compute_crc24, compute_syndrome
from adsb.protocol.gillham import (
    decode_gillham_altitude,
    decode_mode_s_altitude,
    decode_mode_s_identity,
    encode_gillham_altitude,
)


@dataclass(slots=True)
class DecodedModeSMessage:
    """Base and polymorphic container for decoded Mode S messages."""

    df: int
    raw_bytes: bytes = b""
    tc: int = 0
    icao24: int = 0
    ca: int = 0
    callsign: str = ""
    altitude_ft: int | None = None
    cpr_format: int = 0
    cpr_lat: int = 0
    cpr_lon: int = 0
    subtype: int = 0
    v_ew_kt: float | None = None
    v_ns_kt: float | None = None
    groundspeed_kt: float | None = None
    ground_track_deg: float | None = None
    vertical_rate_fpm: int | None = None
    magnetic_heading_deg: float | None = None
    airspeed_kt: float | None = None
    airspeed_is_tas: bool = False
    is_verified: bool = True
    crc_valid: bool = True
    crc_syndrome: int = 0
    emergency_state: int = 0
    squawk: str = ""


# Alias message types for architectural completeness
ESIdentificationMsg = DecodedModeSMessage
ESAirbornePositionMsg = DecodedModeSMessage
ESSurfacePositionMsg = DecodedModeSMessage
ESAirborneVelocityMsg = DecodedModeSMessage
ESAircraftStatusMsg = DecodedModeSMessage
ESTargetStateStatusMsg = DecodedModeSMessage
ESOperationalStatusMsg = DecodedModeSMessage


def decode_mode_s(
    raw_bytes: bytes, active_whitelist: set[int] | None = None
) -> DecodedModeSMessage:
    """Stateless Mode S frame decoder.

    Decodes DF 0, 4, 5, 11, 16, 17, 18, 20, 21.
    For DF 4/5/20/21 Address/Parity frames, validates against active_whitelist.
    """
    length = len(raw_bytes)
    if length not in (7, 14):
        raise ValueError(f"Invalid Mode S frame length: {length} bytes (expected 7 or 14)")

    val = int.from_bytes(raw_bytes, "big")
    total_bits = length * 8
    df = extract_bits(val, total_bits, 1, 5)

    syndrome = compute_syndrome(raw_bytes)
    crc_valid = syndrome == 0

    # DF 17 / DF 18 Extended Squitter (14 bytes)
    if df in (17, 18):
        ca = extract_bits(val, 112, 6, 8)
        icao24 = extract_bits(val, 112, 9, 32)
        tc = extract_bits(val, 112, 33, 37)

        msg = DecodedModeSMessage(
            df=df,
            raw_bytes=raw_bytes,
            tc=tc,
            icao24=icao24,
            ca=ca,
            crc_valid=crc_valid,
            crc_syndrome=syndrome,
            is_verified=crc_valid,
        )

        # TC 1..4: Aircraft Identification and Category
        if 1 <= tc <= 4:
            chars: list[str] = []
            for i in range(8):
                c_code = extract_bits(val, 112, 41 + i * 6, 46 + i * 6)
                chars.append(AIS_CHARMAP[c_code])
            msg.callsign = "".join(chars)
            # Category from ME bits 6..8 (Frame bits 38..40)
            msg.ca = extract_bits(val, 112, 38, 40)

        # TC 5..8: Surface Position
        elif 5 <= tc <= 8:
            mov_code = extract_bits(val, 112, 38, 44)
            trk_status = extract_bits(val, 112, 45, 45)
            trk_raw = extract_bits(val, 112, 46, 52)
            cpr_f = extract_bits(val, 112, 54, 54)
            cpr_lat = extract_bits(val, 112, 55, 71)
            cpr_lon = extract_bits(val, 112, 72, 88)

            msg.cpr_format = cpr_f
            msg.cpr_lat = cpr_lat
            msg.cpr_lon = cpr_lon
            if trk_status:
                msg.ground_track_deg = trk_raw * (360.0 / 128.0)
            if mov_code == 1:
                msg.groundspeed_kt = 0.0
            elif mov_code > 1:
                msg.groundspeed_kt = float(mov_code)

        # TC 9..18 and 20..22: Airborne Position (Baro / GNSS HAE)
        elif 9 <= tc <= 18 or 20 <= tc <= 22:
            alt_code = extract_bits(val, 112, 41, 52)
            cpr_f = extract_bits(val, 112, 54, 54)
            cpr_lat = extract_bits(val, 112, 55, 71)
            cpr_lon = extract_bits(val, 112, 72, 88)

            msg.cpr_format = cpr_f
            msg.cpr_lat = cpr_lat
            msg.cpr_lon = cpr_lon

            msg.altitude_ft = decode_gillham_altitude(alt_code)

        # TC 19: Airborne Velocity
        elif tc == 19:
            subtype = extract_bits(val, 112, 38, 40)
            msg.subtype = subtype

            # Common vertical rate (ME bits 36..46, Frame bits 68..78)
            s_vr = extract_bits(val, 112, 69, 69)
            vr_raw = extract_bits(val, 112, 70, 78)
            if vr_raw != 0:
                vr_sign = -1 if s_vr else 1
                msg.vertical_rate_fpm = vr_sign * 64 * (vr_raw - 1)

            # Subtypes 1 & 2: Groundspeed vector
            if subtype in (1, 2):
                scale = 1.0 if subtype == 1 else 4.0
                s_ew = extract_bits(val, 112, 46, 46)
                v_ew_raw = extract_bits(val, 112, 47, 56)
                s_ns = extract_bits(val, 112, 57, 57)
                v_ns_raw = extract_bits(val, 112, 58, 67)

                if v_ew_raw != 0:
                    sign_ew = -1.0 if s_ew else 1.0
                    msg.v_ew_kt = sign_ew * scale * (v_ew_raw - 1)
                if v_ns_raw != 0:
                    sign_ns = -1.0 if s_ns else 1.0
                    msg.v_ns_kt = sign_ns * scale * (v_ns_raw - 1)

                if msg.v_ew_kt is not None and msg.v_ns_kt is not None:
                    msg.groundspeed_kt = math.hypot(msg.v_ew_kt, msg.v_ns_kt)
                    trk = math.degrees(math.atan2(msg.v_ew_kt, msg.v_ns_kt))
                    msg.ground_track_deg = (trk + 360.0) % 360.0

            # Subtypes 3 & 4: Heading & Airspeed
            elif subtype in (3, 4):
                scale = 1.0 if subtype == 3 else 4.0
                s_hdg = extract_bits(val, 112, 46, 46)
                hdg_raw = extract_bits(val, 112, 47, 56)
                if s_hdg:
                    msg.magnetic_heading_deg = hdg_raw * (360.0 / 1024.0)

                as_type = extract_bits(val, 112, 57, 57)
                msg.airspeed_is_tas = bool(as_type)
                as_raw = extract_bits(val, 112, 58, 67)
                if as_raw != 0:
                    msg.airspeed_kt = scale * (as_raw - 1)

        # TC 28: Emergency Priority & Squawk
        elif tc == 28:
            msg.emergency_state = extract_bits(val, 112, 41, 43)
            id_code = extract_bits(val, 112, 44, 56)
            msg.squawk = decode_mode_s_identity(id_code)

        return msg

    # DF 11: Mode S All-Call Reply (7 bytes)
    elif df == 11:
        ca = extract_bits(val, 56, 6, 8)
        icao24 = extract_bits(val, 56, 9, 32)
        return DecodedModeSMessage(
            df=df,
            raw_bytes=raw_bytes,
            icao24=icao24,
            ca=ca,
            crc_valid=crc_valid,
            crc_syndrome=syndrome,
            is_verified=crc_valid,
        )

    # DF 4 / DF 5: Surveillance Replies (7 bytes with AP)
    elif df in (4, 5):
        # Syndrome contains the recovered 24-bit ICAO address
        recovered_icao = syndrome
        verified = True
        if active_whitelist is not None:
            verified = recovered_icao in active_whitelist

        msg = DecodedModeSMessage(
            df=df,
            raw_bytes=raw_bytes,
            icao24=recovered_icao,
            crc_valid=verified,
            crc_syndrome=syndrome,
            is_verified=verified,
        )
        if df == 4:
            ac_code = extract_bits(val, 56, 20, 32)
            msg.altitude_ft = decode_mode_s_altitude(ac_code)
        else:
            id_code = extract_bits(val, 56, 20, 32)
            msg.squawk = decode_mode_s_identity(id_code)
        return msg

    # DF 20 / DF 21: Comm-B Altitude/Identity Replies (14 bytes with AP)
    elif df in (20, 21):
        recovered_icao = syndrome
        verified = True
        if active_whitelist is not None:
            verified = recovered_icao in active_whitelist

        msg = DecodedModeSMessage(
            df=df,
            raw_bytes=raw_bytes,
            icao24=recovered_icao,
            crc_valid=verified,
            crc_syndrome=syndrome,
            is_verified=verified,
        )
        if df == 20:
            ac_code = extract_bits(val, 112, 20, 32)
            msg.altitude_ft = decode_mode_s_altitude(ac_code)
        else:
            id_code = extract_bits(val, 112, 20, 32)
            msg.squawk = decode_mode_s_identity(id_code)
        return msg

    # DF 0 / DF 16: ACAS Surveillance
    elif df in (0, 16):
        recovered_icao = syndrome
        ac_code = extract_bits(val, total_bits, 20, 32)
        alt = decode_mode_s_altitude(ac_code)
        return DecodedModeSMessage(
            df=df,
            raw_bytes=raw_bytes,
            icao24=recovered_icao,
            altitude_ft=alt,
            crc_valid=crc_valid,
            crc_syndrome=syndrome,
            is_verified=crc_valid,
        )

    # Other formats
    return DecodedModeSMessage(
        df=df,
        raw_bytes=raw_bytes,
        crc_valid=crc_valid,
        crc_syndrome=syndrome,
        is_verified=crc_valid,
    )


decode_1090es = decode_mode_s


def encode_1090es_identification(
    icao24: int, callsign: str, category: int = 0, ca: int = 5
) -> bytes:
    """Encode a 1090ES DF17 TC4 Aircraft Identification message."""
    clean_callsign = callsign.ljust(8)[:8].upper()
    val = (17 << 107) | (ca << 104) | ((icao24 & 0xFFFFFF) << 80) | (4 << 75) | (category << 72)

    charmap_inv = {ch: idx for idx, ch in enumerate(AIS_CHARMAP) if ch != "#"}
    charmap_inv[" "] = 32

    for i, ch in enumerate(clean_callsign):
        code = charmap_inv.get(ch, 32)
        shift = 66 - i * 6
        val |= (code & 0x3F) << shift

    # Compute parity and append
    frame_bytes = val.to_bytes(14, "big")
    parity = compute_crc24(frame_bytes)
    final_val = val | (parity & 0xFFFFFF)
    return final_val.to_bytes(14, "big")


def encode_1090es_airborne_position(
    icao24: int,
    altitude_ft: int,
    lat_deg: float,
    lon_deg: float,
    cpr_format: int,
    ca: int = 5,
) -> bytes:
    """Encode a 1090ES DF17 Airborne Position message (TC 11)."""
    tc = 11
    alt_code = encode_gillham_altitude(altitude_ft, resolution_25ft=True)
    cpr_lat, cpr_lon = encode_cpr_airborne(lat_deg, lon_deg, cpr_format)

    val = (
        (17 << 107)
        | (ca << 104)
        | ((icao24 & 0xFFFFFF) << 80)
        | (tc << 75)
        | ((alt_code & 0xFFF) << 60)
        | (1 << 59)  # time sync T=1
        | (cpr_format << 58)
        | ((cpr_lat & 0x1FFFF) << 41)
        | (cpr_lon & 0x1FFFF) << 24
    )

    frame_bytes = val.to_bytes(14, "big")
    parity = compute_crc24(frame_bytes)
    final_val = val | (parity & 0xFFFFFF)
    return final_val.to_bytes(14, "big")


def encode_1090es_airborne_velocity(
    icao24: int,
    v_ew_kt: float,
    v_ns_kt: float,
    vertical_rate_fpm: int = 0,
    ca: int = 5,
) -> bytes:
    """Encode a 1090ES DF17 Airborne Velocity Subtype 1 message."""
    tc = 19
    subtype = 1

    s_ew = 1 if v_ew_kt < 0 else 0
    v_ew_raw = min(1023, round(abs(v_ew_kt)) + 1)

    s_ns = 1 if v_ns_kt < 0 else 0
    v_ns_raw = min(1023, round(abs(v_ns_kt)) + 1)

    s_vr = 1 if vertical_rate_fpm < 0 else 0
    vr_raw = min(511, round(abs(vertical_rate_fpm) / 64.0) + 1)

    val = (
        (17 << 107)
        | (ca << 104)
        | ((icao24 & 0xFFFFFF) << 80)
        | (tc << 75)
        | (subtype << 72)
        | (s_ew << 66)
        | ((v_ew_raw & 0x3FF) << 56)
        | (s_ns << 55)
        | ((v_ns_raw & 0x3FF) << 45)
        | (s_vr << 43)
        | ((vr_raw & 0x1FF) << 34)
    )

    frame_bytes = val.to_bytes(14, "big")
    parity = compute_crc24(frame_bytes)
    final_val = val | (parity & 0xFFFFFF)
    return final_val.to_bytes(14, "big")


def encode_1090es_surface_position(
    icao24: int,
    lat_deg: float,
    lon_deg: float,
    cpr_format: int,
    groundspeed_kt: float = 0.0,
    ground_track_deg: float | None = None,
    ca: int = 5,
) -> bytes:
    """Encode a 1090ES DF17 Surface Position message (TC 8)."""
    tc = 8
    mov_code = 1 if groundspeed_kt <= 0.125 else min(127, max(2, round(groundspeed_kt)))

    if ground_track_deg is not None:
        trk_status = 1
        trk_raw = round((ground_track_deg % 360.0) * 128.0 / 360.0) & 0x7F
    else:
        trk_status = 0
        trk_raw = 0

    cpr_lat, cpr_lon = encode_cpr_surface(lat_deg, lon_deg, cpr_format)

    val = (
        (17 << 107)
        | (ca << 104)
        | ((icao24 & 0xFFFFFF) << 80)
        | (tc << 75)
        | ((mov_code & 0x7F) << 68)
        | (trk_status << 67)
        | ((trk_raw & 0x7F) << 60)
        | (1 << 59)  # time sync T=1
        | (cpr_format << 58)
        | ((cpr_lat & 0x1FFFF) << 41)
        | ((cpr_lon & 0x1FFFF) << 24)
    )

    frame_bytes = val.to_bytes(14, "big")
    parity = compute_crc24(frame_bytes)
    final_val = val | (parity & 0xFFFFFF)
    return final_val.to_bytes(14, "big")


def encode_1090es_df11(icao24: int, ca: int = 5) -> bytes:
    """Encode a 56-bit Mode S DF11 Acquisition Squitter (All-Call Reply) with II=0."""
    val = (11 << 51) | (ca << 48) | ((icao24 & 0xFFFFFF) << 24)
    frame_bytes = val.to_bytes(7, "big")
    parity = compute_crc24(frame_bytes)
    final_val = val | (parity & 0xFFFFFF)
    return final_val.to_bytes(7, "big")


def encode_mode_s(msg: Any) -> bytes:
    """Polymorphic encoder for Mode S messages."""
    if hasattr(msg, "raw_bytes") and msg.raw_bytes:
        return bytes(msg.raw_bytes)
    raise NotImplementedError("Direct object encoding not yet supported for this message type")


__all__ = [
    "DecodedModeSMessage",
    "ESAirbornePositionMsg",
    "ESAirborneVelocityMsg",
    "ESAircraftStatusMsg",
    "ESIdentificationMsg",
    "ESOperationalStatusMsg",
    "ESSurfacePositionMsg",
    "ESTargetStateStatusMsg",
    "decode_1090es",
    "decode_mode_s",
    "encode_1090es_airborne_position",
    "encode_1090es_airborne_velocity",
    "encode_1090es_df11",
    "encode_1090es_identification",
    "encode_1090es_surface_position",
    "encode_mode_s",
]
