"""Mode S and Mode A/C Gillham Gray code altitude and squawk code codecs.

Authoritative reference: ICAO Annex 10 Vol IV, RTCA DO-260B/C,
and docs/02_PROTOCOL_SPECIFICATION.md Section 4.
"""

from __future__ import annotations

# Inverse mapping for 100-ft Gray code steps: step (1..5) -> G_100 code
_STEP_TO_G100: dict[int, int] = {
    1: 1,  # 001_2
    2: 3,  # 011_2
    3: 2,  # 010_2
    4: 6,  # 110_2
    5: 4,  # 100_2
}

_G100_TO_STEP: dict[int, int] = {
    1: 1,
    3: 2,
    2: 3,
    6: 4,
    4: 5,
}


def gray_to_binary(gray: int) -> int:
    """Convert an arbitrary Gray code integer to binary integer."""
    b = gray
    b ^= b >> 16
    b ^= b >> 8
    b ^= b >> 4
    b ^= b >> 2
    b ^= b >> 1
    return b


def binary_to_gray(num: int) -> int:
    """Convert binary integer to Gray code."""
    return num ^ (num >> 1)


def decode_gillham_altitude(alt_code: int) -> int | None:
    """Decode a 12-bit Mode S/ADS-B altitude code (ALT field) to feet.

    Handles:
    - alt_code == 0: Returns None (altitude not available).
    - Q-bit == 1: 25-ft increments (-1000 ft to +50175 ft).
    - Q-bit == 0: 100-ft Gillham Gray code pulses. Invalid pulse codes
      where G_100 in {0, 5, 7} return None.
    """
    if alt_code == 0:
        return None

    # Bit 4 from LSB is the Q-bit (ME bit 16, Frame bit 48)
    q_bit = (alt_code >> 4) & 1

    if q_bit == 1:
        # 25-foot increments
        n = ((alt_code >> 5) << 4) | (alt_code & 0x0F)
        return n * 25 - 1000

    # 100-foot Gillham Gray code (Q = 0)
    c1 = (alt_code >> 11) & 1
    a1 = (alt_code >> 10) & 1
    c2 = (alt_code >> 9) & 1
    a2 = (alt_code >> 8) & 1
    c4 = (alt_code >> 7) & 1
    a4 = (alt_code >> 6) & 1
    b1 = (alt_code >> 5) & 1
    # bit 4 is Q=0
    b2 = (alt_code >> 3) & 1
    d2 = (alt_code >> 2) & 1
    b4 = (alt_code >> 1) & 1
    d4 = alt_code & 1

    g500 = (d2 << 7) | (d4 << 6) | (a1 << 5) | (a2 << 4) | (a4 << 3) | (b1 << 2) | (b2 << 1) | b4
    b500 = gray_to_binary(g500)

    g100 = (c1 << 2) | (c2 << 1) | c4
    if g100 not in _G100_TO_STEP:
        # Invalid pulse combinations G_100 in {0, 5, 7}
        return None

    c_step = _G100_TO_STEP[g100]
    if b500 % 2 == 1:
        c_step = 6 - c_step

    alt_ft = 500 * b500 + 100 * c_step - 1300
    return alt_ft


decode_altitude = decode_gillham_altitude


def encode_gillham_altitude(alt_ft: int, resolution_25ft: bool = True) -> int:
    """Encode an altitude in feet into a 12-bit Mode S altitude code (ALT field)."""
    if resolution_25ft:
        # 25-ft increment encoding with Q=1
        clamped_ft = max(-1000, min(50175, alt_ft))
        n = round((clamped_ft + 1000) / 25.0)
        alt_code = ((n >> 4) << 5) | (1 << 4) | (n & 0x0F)
        return alt_code

    # 100-ft increment encoding with Q=0
    offset = alt_ft + 1300
    b500 = offset // 500
    rem = offset % 500
    c_step = max(1, min(5, round(rem / 100.0)))

    if b500 % 2 == 1:
        c_step = 6 - c_step

    g100 = _STEP_TO_G100[c_step]
    g500 = binary_to_gray(b500) & 0xFF

    d2 = (g500 >> 7) & 1
    d4 = (g500 >> 6) & 1
    a1 = (g500 >> 5) & 1
    a2 = (g500 >> 4) & 1
    a4 = (g500 >> 3) & 1
    b1 = (g500 >> 2) & 1
    b2 = (g500 >> 1) & 1
    b4 = g500 & 1

    c1 = (g100 >> 2) & 1
    c2 = (g100 >> 1) & 1
    c4 = g100 & 1

    alt_code = (
        (c1 << 11)
        | (a1 << 10)
        | (c2 << 9)
        | (a2 << 8)
        | (c4 << 7)
        | (a4 << 6)
        | (b1 << 5)
        | (0 << 4)  # Q=0
        | (b2 << 3)
        | (d2 << 2)
        | (b4 << 1)
        | d4
    )
    return alt_code


encode_altitude = encode_gillham_altitude


def decode_mode_s_altitude(ac_code: int) -> int | None:
    """Decode a 13-bit Mode S AC field (DF 0, 4, 16, 20) to altitude in feet.

    Bit 6 (from LSB) is the M-bit (metric vs feet).
    When M=0, strips M-bit to form 12-bit ALT code and decodes via decode_gillham_altitude.
    """
    m_bit = (ac_code >> 6) & 1
    if m_bit == 1:
        # Metric altitude mode (rare, 1m resolution)
        n = ((ac_code >> 7) << 6) | (ac_code & 0x3F)
        # Convert meters to feet
        return round(n * 3.28084)

    # Feet mode: remove bit 6
    alt_12 = ((ac_code >> 7) << 6) | (ac_code & 0x3F)
    return decode_gillham_altitude(alt_12)


def decode_mode_s_identity(id_code: int) -> str:
    """Decode a 13-bit Mode S identity/squawk field (DF 5, 21) into a 4-digit octal string."""
    c1 = (id_code >> 12) & 1
    a1 = (id_code >> 11) & 1
    c2 = (id_code >> 10) & 1
    a2 = (id_code >> 9) & 1
    c4 = (id_code >> 8) & 1
    a4 = (id_code >> 7) & 1
    # bit 6 is unused X=0
    b1 = (id_code >> 5) & 1
    d1 = (id_code >> 4) & 1
    b2 = (id_code >> 3) & 1
    d2 = (id_code >> 2) & 1
    b4 = (id_code >> 1) & 1
    d4 = id_code & 1

    a = (a4 << 2) | (a2 << 1) | a1
    b = (b4 << 2) | (b2 << 1) | b1
    c = (c4 << 2) | (c2 << 1) | c1
    d = (d4 << 2) | (d2 << 1) | d1

    return f"{a}{b}{c}{d}"


decode_squawk = decode_mode_s_identity


def encode_squawk_code(squawk_str: str) -> int:
    """Encode a 4-digit octal squawk string (e.g. '7700') into a 13-bit integer."""
    if len(squawk_str) != 4 or not all("0" <= ch <= "7" for ch in squawk_str):
        raise ValueError(f"Invalid squawk code format: '{squawk_str}' (must be 4 octal digits)")

    a = int(squawk_str[0])
    b = int(squawk_str[1])
    c = int(squawk_str[2])
    d = int(squawk_str[3])

    a4 = (a >> 2) & 1
    a2 = (a >> 1) & 1
    a1 = a & 1
    b4 = (b >> 2) & 1
    b2 = (b >> 1) & 1
    b1 = b & 1
    c4 = (c >> 2) & 1
    c2 = (c >> 1) & 1
    c1 = c & 1
    d4 = (d >> 2) & 1
    d2 = (d >> 1) & 1
    d1 = d & 1

    id_code = (
        (c1 << 12)
        | (a1 << 11)
        | (c2 << 10)
        | (a2 << 9)
        | (c4 << 8)
        | (a4 << 7)
        | (0 << 6)  # X=0
        | (b1 << 5)
        | (d1 << 4)
        | (b2 << 3)
        | (d2 << 2)
        | (b4 << 1)
        | d4
    )
    return id_code


encode_squawk = encode_squawk_code


__all__ = [
    "binary_to_gray",
    "decode_altitude",
    "decode_gillham_altitude",
    "decode_mode_s_altitude",
    "decode_mode_s_identity",
    "decode_squawk",
    "encode_altitude",
    "encode_gillham_altitude",
    "encode_squawk",
    "encode_squawk_code",
    "gray_to_binary",
]
