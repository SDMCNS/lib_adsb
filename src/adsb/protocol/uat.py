"""978 MHz Universal Access Transceiver (UAT) Frame Codec and Reed-Solomon FEC.

Authoritative reference: RTCA DO-282B, ICAO Annex 10 Vol IV,
and docs/02_PROTOCOL_SPECIFICATION.md Section 8.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

# ==============================================================================
# 1. Pure-Python GF(2^8) Reed-Solomon Engine (RTCA DO-282B)
# ==============================================================================

# DO-282B: Primitive polynomial p(x) = x^8 + x^7 + x^2 + x + 1 = 0x187 (391)
PRIM_POLY: int = 0x187

# Precompute log and antilog (exp) tables
GF_EXP: list[int] = [0] * 512
GF_LOG: list[int] = [0] * 256

_x = 1
for _i in range(255):
    GF_EXP[_i] = _x
    GF_EXP[_i + 255] = _x
    GF_LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= PRIM_POLY
GF_LOG[0] = 0


def _gf_mul(x: int, y: int) -> int:
    return GF_EXP[GF_LOG[x] + GF_LOG[y]] if (x and y) else 0


def _gf_div(x: int, y: int) -> int:
    if y == 0:
        raise ZeroDivisionError("GF(2^8) division by zero")
    if x == 0:
        return 0
    return GF_EXP[(GF_LOG[x] - GF_LOG[y] + 255) % 255]


def _gf_poly_mul(p: list[int], q: list[int]) -> list[int]:
    r = [0] * (len(p) + len(q) - 1)
    for i, pi in enumerate(p):
        for j, qj in enumerate(q):
            r[i + j] ^= _gf_mul(pi, qj)
    return r


class UATReedSolomonError(Exception):
    """Raised when uncorrectable Reed-Solomon errors are detected."""


class ReedSolomonCodec:
    """Reed-Solomon FEC over GF(2^8) configured for RTCA DO-282B UAT."""

    def __init__(self, nsym: int, fcr: int = 120) -> None:
        self.nsym = nsym
        self.fcr = fcr
        self.gen = self._init_generator()

    def _init_generator(self) -> list[int]:
        g = [1]
        for i in range(self.nsym):
            root = GF_EXP[self.fcr + i]
            g = _gf_poly_mul(g, [1, root])
        return g

    def encode(self, msg: bytes) -> bytes:
        """Encode message bytes systematically into parity bytes."""
        out = list(msg) + [0] * self.nsym
        for i in range(len(msg)):
            coef = out[i]
            if coef != 0:
                for j in range(1, len(self.gen)):
                    out[i + j] ^= _gf_mul(coef, self.gen[j])
        return bytes(out[len(msg) :])

    def calc_syndromes(self, codeword: bytes | bytearray) -> list[int]:
        """Evaluate syndromes over received codeword."""
        syn = []
        for i in range(self.nsym):
            root = GF_EXP[self.fcr + i]
            s = 0
            for b in codeword:
                s = _gf_mul(s, root) ^ b
            syn.append(s)
        return syn

    def decode(self, codeword: bytes) -> tuple[bytes, int]:
        """Decode and correct up to nsym // 2 byte errors in codeword.

        Returns:
            tuple[bytes, int]: (corrected_message, error_count)
        """
        k = len(codeword) - self.nsym
        if k <= 0:
            raise ValueError(f"Codeword too short: {len(codeword)} bytes")

        syndromes = self.calc_syndromes(codeword)
        if all(s == 0 for s in syndromes):
            return codeword[:k], 0

        # Berlekamp-Massey algorithm
        c = [1]
        b = [1]
        length = 0
        m = 1
        b_val = 1

        for n in range(self.nsym):
            d = syndromes[n]
            for i in range(1, len(c)):
                d ^= _gf_mul(c[i], syndromes[n - i])
            if d == 0:
                m += 1
            elif 2 * length <= n:
                t = list(c)
                scale = _gf_div(d, b_val)
                pad = [0] * m + [_gf_mul(scale, val) for val in b]
                while len(c) < len(pad):
                    c.append(0)
                for i in range(len(pad)):
                    c[i] ^= pad[i]
                length = n + 1 - length
                b = t
                b_val = d
                m = 1
            else:
                scale = _gf_div(d, b_val)
                pad = [0] * m + [_gf_mul(scale, val) for val in b]
                while len(c) < len(pad):
                    c.append(0)
                for i in range(len(pad)):
                    c[i] ^= pad[i]
                m += 1

        # Chien search
        err_pos: list[int] = []
        for i in range(len(codeword)):
            p_val = len(codeword) - 1 - i
            x_inv = GF_EXP[(255 - (p_val % 255)) % 255]
            val = 0
            term = 1
            for coef in c:
                val ^= _gf_mul(coef, term)
                term = _gf_mul(term, x_inv)
            if val == 0:
                err_pos.append(i)

        if len(err_pos) != length:
            raise UATReedSolomonError(
                f"Uncorrectable errors: found {len(err_pos)} roots for degree {length}"
            )

        # Forney error magnitude calculation
        cw_mut = bytearray(codeword)
        locator_poly = [1]
        for p in err_pos:
            k_val = len(codeword) - 1 - p
            x_k = GF_EXP[k_val % 255]
            locator_poly = _gf_poly_mul(locator_poly, [1, x_k])

        omega = _gf_poly_mul(syndromes, locator_poly)[: self.nsym]

        for p in err_pos:
            k_val = len(codeword) - 1 - p
            x_k = GF_EXP[k_val % 255]
            x_inv = GF_EXP[(255 - (k_val % 255)) % 255]

            omega_val = 0
            term = 1
            for coef in omega:
                omega_val ^= _gf_mul(coef, term)
                term = _gf_mul(term, x_inv)

            lambda_prime = 0
            term = 1
            for j in range(1, len(locator_poly), 2):
                lambda_prime ^= _gf_mul(locator_poly[j], term)
                term = _gf_mul(term, _gf_mul(x_inv, x_inv))

            exp_k = ((1 - self.fcr) % 255 + 255) % 255
            x_k_factor = GF_EXP[(GF_LOG[x_k] * exp_k) % 255] if x_k else 0
            mag = _gf_mul(x_k_factor, _gf_div(omega_val, lambda_prime))
            cw_mut[p] ^= mag

        # Verify corrected syndromes
        post_syn = self.calc_syndromes(cw_mut)
        if any(s != 0 for s in post_syn):
            raise UATReedSolomonError("Post-correction syndrome verification failed")

        return bytes(cw_mut[:k]), len(err_pos)


# Pre-instantiated codecs for Basic RS(30,18) and Long RS(48,34)
UAT_BASIC_RS = ReedSolomonCodec(nsym=12, fcr=120)
UAT_LONG_RS = ReedSolomonCodec(nsym=14, fcr=120)


# ==============================================================================
# 2. UAT Domain Models and Enums
# ==============================================================================


class AltitudeSource(StrEnum):
    BAROMETRIC = "BARO"
    GNSS_HAE = "GNSS_HAE"


BASE40_ALPHABET = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ  .."


@dataclass(frozen=True, slots=True)
class UATHeader:
    """UAT 4-byte header (bits 1..32)."""

    hdr_type: int
    addr_qual: int
    address: int
    icao_hex: str


@dataclass(frozen=True, slots=True)
class UATStateVector:
    """UAT State Vector (SV, bytes 5..17, bits 33..136)."""

    lat_deg: float
    lon_deg: float
    altitude_ft: int | None
    altitude_source: AltitudeSource
    nic: int
    airground_state: int
    v_ns_kt: float | None = None
    v_ew_kt: float | None = None
    groundspeed_kt: float | None = None
    ground_track_deg: float | None = None
    vertical_rate_fpm: int | None = None
    vertical_rate_source: AltitudeSource = AltitudeSource.BAROMETRIC
    utc_coupled: bool = False
    tisb_site_id: int = 0


@dataclass(frozen=True, slots=True)
class UATModeStatus:
    """UAT Mode Status (MS, bytes 18..29)."""

    emitter_category: int
    callsign: str
    emergency_status: int = 0
    uat_version: int = 2
    sil: int = 3
    transmit_mso: int = 0
    nac_p: int = 9
    nac_v: int = 2
    nic_baro: int = 0
    has_cdti: bool = False
    has_acas: bool = False
    acas_ra_active: bool = False
    ident_active: bool = False
    atc_services: bool = False
    heading_type: str = "TRUE"
    callsign_type: str = "CALLSIGN"


@dataclass(frozen=True, slots=True)
class UATAuxStateVector:
    """UAT Auxiliary State Vector (AUX SV, bytes 30..31)."""

    sec_altitude_ft: int | None
    sec_altitude_source: AltitudeSource


@dataclass(frozen=True, slots=True)
class UATMessage:
    """Complete decoded UAT message."""

    raw_bytes: bytes
    header: UATHeader
    state_vector: UATStateVector | None = None
    mode_status: UATModeStatus | None = None
    aux_state_vector: UATAuxStateVector | None = None
    errors_corrected: int = 0

    @property
    def icao24(self) -> int:
        return self.header.address

    @property
    def icao_hex(self) -> str:
        return self.header.icao_hex

    @property
    def altitude_ft(self) -> int | None:
        return self.state_vector.altitude_ft if self.state_vector else None

    @property
    def lat_deg(self) -> float | None:
        return self.state_vector.lat_deg if self.state_vector else None

    @property
    def lon_deg(self) -> float | None:
        return self.state_vector.lon_deg if self.state_vector else None

    @property
    def callsign(self) -> str:
        return self.mode_status.callsign if self.mode_status else ""


# ==============================================================================
# 3. UAT Decoders & Encoders
# ==============================================================================


def decode_uat_header(frame: bytes) -> UATHeader:
    """Decode 4-byte UAT header."""
    hdr_type = (frame[0] >> 3) & 0x1F
    addr_qual = frame[0] & 0x07
    address = (frame[1] << 16) | (frame[2] << 8) | frame[3]
    return UATHeader(
        hdr_type=hdr_type,
        addr_qual=addr_qual,
        address=address,
        icao_hex=f"{address:06X}",
    )


def encode_uat_header(header: UATHeader) -> bytes:
    """Encode 4-byte UAT header."""
    b0 = ((header.hdr_type & 0x1F) << 3) | (header.addr_qual & 0x07)
    b1 = (header.address >> 16) & 0xFF
    b2 = (header.address >> 8) & 0xFF
    b3 = header.address & 0xFF
    return bytes([b0, b1, b2, b3])


def decode_uat_sv(frame: bytes) -> UATStateVector:
    """Decode 13-byte State Vector (bytes 4..16)."""
    raw_lat = (frame[4] << 15) | (frame[5] << 7) | (frame[6] >> 1)
    if raw_lat & 0x400000:
        lat_deg = ((raw_lat - (1 << 23)) * 180.0) / (1 << 23)
    else:
        lat_deg = (raw_lat * 180.0) / (1 << 23)

    raw_lon = ((frame[6] & 0x01) << 23) | (frame[7] << 15) | (frame[8] << 7) | (frame[9] >> 1)
    if raw_lon & 0x800000:
        lon_deg = ((raw_lon - (1 << 24)) * 360.0) / (1 << 24)
    else:
        lon_deg = (raw_lon * 360.0) / (1 << 24)

    alt_type_bit = frame[9] & 0x01
    alt_source = AltitudeSource.GNSS_HAE if alt_type_bit else AltitudeSource.BAROMETRIC

    raw_alt = (frame[10] << 4) | (frame[11] >> 4)
    alt_ft = ((raw_alt - 1) * 25 - 1000) if raw_alt != 0 else None

    nic = frame[11] & 0x0F
    ag_state = (frame[12] >> 6) & 0x03

    v_ns_kt: float | None = None
    v_ew_kt: float | None = None
    gs_kt: float | None = None
    trk_deg: float | None = None
    vr_fpm: int | None = None
    vr_source = AltitudeSource.BAROMETRIC

    if ag_state in (0, 1):
        scale = 1.0 if ag_state == 0 else 4.0
        # N/S Velocity
        raw_ns = ((frame[12] & 0x1F) << 6) | ((frame[13] & 0xFC) >> 2)
        s_ns = (raw_ns >> 10) & 1
        ns_mag = raw_ns & 0x3FF
        if ns_mag != 0:
            v_ns_kt = (-1.0 if s_ns else 1.0) * scale * (ns_mag - 1)

        # E/W Velocity
        raw_ew = ((frame[13] & 0x03) << 9) | (frame[14] << 1) | ((frame[15] & 0x80) >> 7)
        s_ew = (raw_ew >> 10) & 1
        ew_mag = raw_ew & 0x3FF
        if ew_mag != 0:
            v_ew_kt = (-1.0 if s_ew else 1.0) * scale * (ew_mag - 1)

        if v_ns_kt is not None and v_ew_kt is not None:
            gs_kt = math.hypot(v_ew_kt, v_ns_kt)
            trk = math.degrees(math.atan2(v_ew_kt, v_ns_kt))
            trk_deg = (trk + 360.0) % 360.0

        # Vertical rate
        vr_source = AltitudeSource.BAROMETRIC if (frame[15] & 0x40) else AltitudeSource.GNSS_HAE
        s_vr = (frame[15] >> 5) & 1
        raw_vr = ((frame[15] & 0x1F) << 4) | (frame[16] >> 4)
        if raw_vr != 0:
            vr_sign = -1 if s_vr else 1
            vr_fpm = vr_sign * 64 * (raw_vr - 1)

    elif ag_state == 2:
        # On ground
        raw_gs = ((frame[12] & 0x1F) << 5) | (frame[13] >> 3)
        gs_kt = float((raw_gs & 0x3FF) - 1) if raw_gs != 0 else 0.0
        raw_trk = ((frame[13] & 0x07) << 6) | (frame[14] >> 2)
        trk_deg = raw_trk * (360.0 / 512.0)

    utc_coupled = bool(frame[16] & 0x08)
    tisb_site = frame[16] & 0x0F

    return UATStateVector(
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        altitude_ft=alt_ft,
        altitude_source=alt_source,
        nic=nic,
        airground_state=ag_state,
        v_ns_kt=v_ns_kt,
        v_ew_kt=v_ew_kt,
        groundspeed_kt=gs_kt,
        ground_track_deg=trk_deg,
        vertical_rate_fpm=vr_fpm,
        vertical_rate_source=vr_source,
        utc_coupled=utc_coupled,
        tisb_site_id=tisb_site,
    )


def encode_uat_sv(sv: UATStateVector) -> bytes:
    """Encode 13-byte State Vector (bytes 4..16)."""
    raw_lat = round(sv.lat_deg * (1 << 23) / 180.0) & 0x7FFFFF
    raw_lon = round(sv.lon_deg * (1 << 24) / 360.0) & 0xFFFFFF

    alt_type_bit = 1 if sv.altitude_source == AltitudeSource.GNSS_HAE else 0
    if sv.altitude_ft is None:
        raw_alt = 0
    else:
        raw_alt = max(1, min(4095, round((sv.altitude_ft + 1000) / 25.0) + 1))

    b4 = (raw_lat >> 15) & 0xFF
    b5 = (raw_lat >> 7) & 0xFF
    b6 = ((raw_lat & 0x7F) << 1) | ((raw_lon >> 23) & 0x01)
    b7 = (raw_lon >> 15) & 0xFF
    b8 = (raw_lon >> 7) & 0xFF
    b9 = ((raw_lon & 0x7F) << 1) | alt_type_bit
    b10 = (raw_alt >> 4) & 0xFF
    b11 = ((raw_alt & 0x0F) << 4) | (sv.nic & 0x0F)

    ag_state = sv.airground_state & 0x03
    b12 = ag_state << 6
    b13 = 0
    b14 = 0
    b15 = 0
    b16 = 0

    if ag_state in (0, 1):
        scale = 1.0 if ag_state == 0 else 4.0
        v_ns = sv.v_ns_kt or 0.0
        s_ns = 1 if v_ns < 0 else 0
        ns_raw = min(1023, round(abs(v_ns) / scale) + 1)
        ns_full = (s_ns << 10) | ns_raw

        v_ew = sv.v_ew_kt or 0.0
        s_ew = 1 if v_ew < 0 else 0
        ew_raw = min(1023, round(abs(v_ew) / scale) + 1)
        ew_full = (s_ew << 10) | ew_raw

        b12 |= (ns_full >> 6) & 0x1F
        b13 = ((ns_full & 0x3F) << 2) | ((ew_full >> 9) & 0x03)
        b14 = (ew_full >> 1) & 0xFF

        vr_bit = 0x40 if sv.vertical_rate_source == AltitudeSource.BAROMETRIC else 0
        vr = sv.vertical_rate_fpm or 0
        s_vr = 1 if vr < 0 else 0
        vr_raw = min(511, round(abs(vr) / 64.0) + 1)
        b15 = ((ew_full & 0x01) << 7) | vr_bit | (s_vr << 5) | ((vr_raw >> 4) & 0x1F)
        b16 = (vr_raw & 0x0F) << 4

    b16 |= 0x08 if sv.utc_coupled else 0
    b16 |= sv.tisb_site_id & 0x07

    return bytes([b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15, b16])


def decode_uat_ms(frame: bytes) -> UATModeStatus:
    """Decode 12-byte Mode Status (bytes 17..28)."""
    v0 = (frame[17] << 8) | frame[18]
    cat = (v0 // 1600) % 40
    c0 = BASE40_ALPHABET[(v0 // 40) % 40]
    c1 = BASE40_ALPHABET[v0 % 40]

    v1 = (frame[19] << 8) | frame[20]
    c2 = BASE40_ALPHABET[(v1 // 1600) % 40]
    c3 = BASE40_ALPHABET[(v1 // 40) % 40]
    c4 = BASE40_ALPHABET[v1 % 40]

    v2 = (frame[21] << 8) | frame[22]
    c5 = BASE40_ALPHABET[(v2 // 1600) % 40]
    c6 = BASE40_ALPHABET[(v2 // 40) % 40]
    c7 = BASE40_ALPHABET[v2 % 40]

    callsign = "".join([c0, c1, c2, c3, c4, c5, c6, c7]).rstrip()

    em_status = (frame[23] >> 5) & 7
    uat_ver = (frame[23] >> 2) & 7
    sil = frame[23] & 3
    mso = (frame[24] >> 2) & 0x3F
    nac_p = (frame[25] >> 4) & 15
    nac_v = (frame[25] >> 1) & 7
    nic_baro = frame[25] & 1

    return UATModeStatus(
        emitter_category=cat,
        callsign=callsign,
        emergency_status=em_status,
        uat_version=uat_ver,
        sil=sil,
        transmit_mso=mso,
        nac_p=nac_p,
        nac_v=nac_v,
        nic_baro=nic_baro,
        heading_type="MAGNETIC" if (frame[26] & 0x04) else "TRUE",
        callsign_type="CALLSIGN" if (frame[26] & 0x02) else "SQUAWK",
    )


def encode_uat_ms(ms: UATModeStatus) -> bytes:
    """Encode 12-byte Mode Status (bytes 17..28)."""
    inv = {ch: idx for idx, ch in enumerate(BASE40_ALPHABET)}
    inv[" "] = 36
    cs = ms.callsign.ljust(8)[:8].upper()

    v0 = ms.emitter_category * 1600 + inv.get(cs[0], 36) * 40 + inv.get(cs[1], 36)
    v1 = inv.get(cs[2], 36) * 1600 + inv.get(cs[3], 36) * 40 + inv.get(cs[4], 36)
    v2 = inv.get(cs[5], 36) * 1600 + inv.get(cs[6], 36) * 40 + inv.get(cs[7], 36)

    b17 = (v0 >> 8) & 0xFF
    b18 = v0 & 0xFF
    b19 = (v1 >> 8) & 0xFF
    b20 = v1 & 0xFF
    b21 = (v2 >> 8) & 0xFF
    b22 = v2 & 0xFF

    b23 = ((ms.emergency_status & 7) << 5) | ((ms.uat_version & 7) << 2) | (ms.sil & 3)
    b24 = (ms.transmit_mso & 0x3F) << 2
    b25 = ((ms.nac_p & 15) << 4) | ((ms.nac_v & 7) << 1) | (ms.nic_baro & 1)
    b26 = (0x04 if ms.heading_type == "MAGNETIC" else 0) | (
        0x02 if ms.callsign_type == "CALLSIGN" else 0
    )
    b27 = 0
    b28 = 0

    return bytes([b17, b18, b19, b20, b21, b22, b23, b24, b25, b26, b27, b28])


def decode_uat_auxsv(frame: bytes, primary_alt_src: AltitudeSource) -> UATAuxStateVector:
    """Decode 2-byte Auxiliary State Vector (bytes 29..30)."""
    raw_alt = (frame[29] << 4) | (frame[30] >> 4)
    alt_ft = ((raw_alt - 1) * 25 - 1000) if raw_alt != 0 else None
    sec_source = (
        AltitudeSource.BAROMETRIC
        if primary_alt_src == AltitudeSource.GNSS_HAE
        else AltitudeSource.GNSS_HAE
    )
    return UATAuxStateVector(sec_altitude_ft=alt_ft, sec_altitude_source=sec_source)


def encode_uat_auxsv(aux: UATAuxStateVector) -> bytes:
    """Encode 2-byte Auxiliary State Vector (bytes 29..30)."""
    if aux.sec_altitude_ft is None:
        raw_alt = 0
    else:
        raw_alt = max(1, min(4095, round((aux.sec_altitude_ft + 1000) / 25.0) + 1))
    return bytes([(raw_alt >> 4) & 0xFF, (raw_alt & 0x0F) << 4])


def decode_uat(raw_bytes: bytes) -> UATMessage:
    """Decode a 978 MHz UAT frame (Basic 18B/30B or Long 34B/48B).

    Applies Reed-Solomon forward error correction if FEC bytes are present:
    - 30 bytes: RS(30,18) Basic frame (corrects up to 6 byte errors)
    - 48 bytes: RS(48,34) Long frame (corrects up to 7 byte errors)
    """
    length = len(raw_bytes)
    err_count = 0

    if length == 30:
        payload_bytes, err_count = UAT_BASIC_RS.decode(raw_bytes)
    elif length == 48:
        payload_bytes, err_count = UAT_LONG_RS.decode(raw_bytes)
    elif length in (18, 34):
        payload_bytes = raw_bytes
    else:
        raise ValueError(f"Invalid UAT frame length: {length} bytes (expected 18, 30, 34, or 48)")

    header = decode_uat_header(payload_bytes)
    sv = decode_uat_sv(payload_bytes)

    ms: UATModeStatus | None = None
    aux_sv: UATAuxStateVector | None = None

    if len(payload_bytes) >= 34:
        ms = decode_uat_ms(payload_bytes)
        aux_sv = decode_uat_auxsv(payload_bytes, sv.altitude_source)

    return UATMessage(
        raw_bytes=raw_bytes,
        header=header,
        state_vector=sv,
        mode_status=ms,
        aux_state_vector=aux_sv,
        errors_corrected=err_count,
    )


decode_uat_basic = decode_uat
decode_uat_long = decode_uat
decode_uat_frame = decode_uat


def encode_uat(msg: UATMessage, include_fec: bool = True) -> bytes:
    """Encode UATMessage to raw binary bytes (with optional Reed-Solomon FEC)."""
    hdr_bytes = encode_uat_header(msg.header)
    sv_bytes = encode_uat_sv(msg.state_vector) if msg.state_vector else bytes(13)

    if msg.header.hdr_type == 0 or len(msg.raw_bytes) in (18, 30):
        # Basic 18 bytes payload (4B header + 13B SV + 1B pad)
        payload = hdr_bytes + sv_bytes + b"\x00"
        if include_fec:
            parity = UAT_BASIC_RS.encode(payload)
            return payload + parity
        return payload
    else:
        # Long 34 bytes payload (4B header + 13B SV + 12B MS + 2B AUX + 3B pad)
        ms_bytes = encode_uat_ms(msg.mode_status) if msg.mode_status else bytes(12)
        aux_bytes = encode_uat_auxsv(msg.aux_state_vector) if msg.aux_state_vector else bytes(2)
        payload = hdr_bytes + sv_bytes + ms_bytes + aux_bytes + b"\x00\x00\x00"
        if include_fec:
            parity = UAT_LONG_RS.encode(payload)
            return payload + parity
        return payload


def encode_uat_basic(
    icao24: int,
    lat_deg: float,
    lon_deg: float,
    altitude_ft: int | None = None,
    altitude_source: AltitudeSource = AltitudeSource.BAROMETRIC,
    nic: int = 8,
    airground_state: int = 0,
    v_ns_kt: float | None = None,
    v_ew_kt: float | None = None,
    vertical_rate_fpm: int | None = None,
    include_fec: bool = True,
) -> bytes:
    """Convenience helper to encode a Basic 18-byte UAT message (or 30B with FEC)."""
    header = UATHeader(hdr_type=0, addr_qual=0, address=icao24, icao_hex=f"{icao24:06X}")
    sv = UATStateVector(
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        altitude_ft=altitude_ft,
        altitude_source=altitude_source,
        nic=nic,
        airground_state=airground_state,
        v_ns_kt=v_ns_kt,
        v_ew_kt=v_ew_kt,
        vertical_rate_fpm=vertical_rate_fpm,
    )
    msg = UATMessage(raw_bytes=b"", header=header, state_vector=sv)
    return encode_uat(msg, include_fec=include_fec)


def encode_uat_long(
    icao24: int,
    lat_deg: float,
    lon_deg: float,
    callsign: str,
    altitude_ft: int | None = None,
    altitude_source: AltitudeSource = AltitudeSource.BAROMETRIC,
    nic: int = 8,
    emitter_category: int = 1,
    include_fec: bool = True,
) -> bytes:
    """Convenience helper to encode a Long 34-byte UAT message (or 48B with FEC)."""
    header = UATHeader(hdr_type=1, addr_qual=0, address=icao24, icao_hex=f"{icao24:06X}")
    sv = UATStateVector(
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        altitude_ft=altitude_ft,
        altitude_source=altitude_source,
        nic=nic,
        airground_state=0,
    )
    ms = UATModeStatus(emitter_category=emitter_category, callsign=callsign)
    msg = UATMessage(raw_bytes=b"", header=header, state_vector=sv, mode_status=ms)
    return encode_uat(msg, include_fec=include_fec)


encode_uat_frame = encode_uat


__all__ = [
    "AltitudeSource",
    "ReedSolomonCodec",
    "UATAuxStateVector",
    "UATHeader",
    "UATMessage",
    "UATModeStatus",
    "UATReedSolomonError",
    "UATStateVector",
    "decode_uat",
    "decode_uat_auxsv",
    "decode_uat_basic",
    "decode_uat_frame",
    "decode_uat_header",
    "decode_uat_long",
    "decode_uat_ms",
    "decode_uat_sv",
    "encode_uat",
    "encode_uat_auxsv",
    "encode_uat_basic",
    "encode_uat_frame",
    "encode_uat_header",
    "encode_uat_long",
    "encode_uat_ms",
    "encode_uat_sv",
]
