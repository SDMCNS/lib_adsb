"""Mode S 24-bit Cyclic Redundancy Check (CRC-24) and Error Correction.

Authoritative reference: ICAO Annex 10 Vol IV, RTCA DO-260B/C,
and docs/02_PROTOCOL_SPECIFICATION.md Section 2.

Generator polynomial:
G(x) = x^24 + x^23 + x^22 + x^21 + x^20 + x^19 + x^18 + x^17 + x^16 +
       x^15 + x^14 + x^13 + x^12 + x^10 + x^3 + 1
     = 0x1FFF409 (degree 24)
Truncated constant: 0xFFF409
"""

from __future__ import annotations

CRC24_POLY: int = 0xFFF409

# 256-entry lookup table for O(bytes) CRC computation
CRC24_TABLE: list[int] = [0] * 256
for _i in range(256):
    _c = _i << 16
    for _ in range(8):
        _c = ((_c << 1) ^ CRC24_POLY) & 0xFFFFFF if _c & 0x800000 else (_c << 1) & 0xFFFFFF
    CRC24_TABLE[_i] = _c


def compute_crc24(data_bytes: bytes) -> int:
    """Compute the 24-bit Mode S CRC parity over data bytes.

    If data_bytes is a full frame (len >= 3), the trailing 3 parity bytes are excluded.
    """
    if len(data_bytes) < 3:
        raise ValueError(f"Input must have at least 3 bytes, got {len(data_bytes)}")
    n_bytes = len(data_bytes) - 3
    crc = 0
    for b in data_bytes[:n_bytes]:
        idx = ((crc >> 16) ^ b) & 0xFF
        crc = ((crc << 8) & 0xFFFFFF) ^ CRC24_TABLE[idx]
    return crc


calculate_crc24 = compute_crc24


def compute_syndrome(frame_bytes: bytes) -> int:
    """Compute the 24-bit syndrome of a received 7- or 14-byte frame.

    For DF 17/18 (and DF 11 with II=0), an error-free frame yields syndrome == 0.
    For DF 0/4/5/16/20/21, syndrome == (ICAO_24bit_address) if error-free.
    """
    if len(frame_bytes) < 3:
        raise ValueError(f"Frame must have at least 3 bytes, got {len(frame_bytes)}")
    expected_crc = compute_crc24(frame_bytes)
    received_tail = int.from_bytes(frame_bytes[-3:], "big")
    return expected_crc ^ received_tail


modes_syndrome = compute_syndrome


def verify_crc24(raw_bytes: bytes) -> bool:
    """Verify whether raw frame has valid 24-bit CRC parity (syndrome == 0)."""
    return compute_syndrome(raw_bytes) == 0


# Precompute syndrome lookup tables:
# 112-bit frames (14 bytes): 112 single-bit error syndromes + 111 adjacent 2-bit burst error syndromes (223 total)
SYNDROME_TABLE_112: dict[int, tuple[int, int]] = {}

for _p in range(112):
    _err_int = 1 << _p
    _err_bytes = _err_int.to_bytes(14, "big")
    _syn = compute_syndrome(_err_bytes)
    SYNDROME_TABLE_112[_syn] = (1 << _p, 1)

for _p in range(111):
    _err_int = 3 << _p
    _err_bytes = _err_int.to_bytes(14, "big")
    _syn = compute_syndrome(_err_bytes)
    SYNDROME_TABLE_112[_syn] = (3 << _p, 2)

# 56-bit frames (7 bytes): 56 single-bit error syndromes + 55 adjacent 2-bit burst error syndromes (111 total)
SYNDROME_TABLE_56: dict[int, tuple[int, int]] = {}

for _p in range(56):
    _err_int = 1 << _p
    _err_bytes = _err_int.to_bytes(7, "big")
    _syn = compute_syndrome(_err_bytes)
    SYNDROME_TABLE_56[_syn] = (1 << _p, 1)

for _p in range(55):
    _err_int = 3 << _p
    _err_bytes = _err_int.to_bytes(7, "big")
    _syn = compute_syndrome(_err_bytes)
    SYNDROME_TABLE_56[_syn] = (3 << _p, 2)

# Default alias for 112-bit extended squitter frames
SYNDROME_TABLE: dict[int, tuple[int, int]] = SYNDROME_TABLE_112


def correct_syndrome_errors(corrupted_frame: bytes) -> tuple[bytes, int]:
    """Correct single-bit or adjacent 2-bit burst errors using syndrome lookup.

    Supports both 7-byte (56-bit) and 14-byte (112-bit) Mode S frames.

    Returns:
        tuple[bytes, int]: (corrected_bytes, error_count)
        If the syndrome is 0, returns (corrupted_frame, 0).
        If the syndrome is not in the respective table, returns (corrupted_frame, 0).
    """
    frame_len = len(corrupted_frame)
    if frame_len not in (7, 14):
        return corrupted_frame, 0

    syndrome = compute_syndrome(corrupted_frame)
    if syndrome == 0:
        return corrupted_frame, 0

    table = SYNDROME_TABLE_56 if frame_len == 7 else SYNDROME_TABLE_112

    if syndrome in table:
        mask, err_count = table[syndrome]
        frame_int = int.from_bytes(corrupted_frame, "big")
        corrected_int = frame_int ^ mask
        return corrected_int.to_bytes(frame_len, "big"), err_count

    return corrupted_frame, 0


correct_crc_errors = correct_syndrome_errors


__all__ = [
    "CRC24_POLY",
    "CRC24_TABLE",
    "SYNDROME_TABLE",
    "SYNDROME_TABLE_56",
    "SYNDROME_TABLE_112",
    "calculate_crc24",
    "compute_crc24",
    "compute_syndrome",
    "correct_crc_errors",
    "correct_syndrome_errors",
    "modes_syndrome",
    "verify_crc24",
]
