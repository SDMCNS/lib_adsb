"""Bit manipulation utilities using 1-based MSB-first indexing.

Aviation standards (ICAO Annex 10, RTCA DO-260B/C) mandate 1-based MSB-first indexing:
- Bit 1 is the most significant bit (MSB).
- For a 112-bit Extended Squitter frame, bits are numbered 1 to 112.
- For a 56-bit Mode S frame, bits are numbered 1 to 56.
"""

from __future__ import annotations

from typing import Self


def bytes_to_uint(data: bytes) -> tuple[int, int]:
    """Convert big-endian bytes to unsigned integer and total bit count."""
    total_bits = len(data) * 8
    val = int.from_bytes(data, byteorder="big")
    return val, total_bits


def uint_to_bytes(val: int, num_bytes: int) -> bytes:
    """Convert unsigned integer to big-endian bytes of specified length."""
    return val.to_bytes(num_bytes, byteorder="big")


def extract_bits(val: int, total_bits: int, start_1based: int, end_1based: int) -> int:
    """Extract bits in range [start_1based..end_1based] from a total_bits integer.

    Indexing is 1-based MSB-first (Bit 1 is MSB).
    """
    if not (1 <= start_1based <= end_1based <= total_bits):
        raise ValueError(
            f"Invalid bit range [{start_1based}..{end_1based}] for total_bits={total_bits}"
        )
    width = end_1based - start_1based + 1
    shift = total_bits - end_1based
    mask = (1 << width) - 1
    return (val >> shift) & mask


def set_bits(val: int, total_bits: int, start_1based: int, end_1based: int, field_val: int) -> int:
    """Set bits in range [start_1based..end_1based] to field_val within a total_bits integer.

    Indexing is 1-based MSB-first (Bit 1 is MSB).
    """
    if not (1 <= start_1based <= end_1based <= total_bits):
        raise ValueError(
            f"Invalid bit range [{start_1based}..{end_1based}] for total_bits={total_bits}"
        )
    width = end_1based - start_1based + 1
    if not (0 <= field_val < (1 << width)):
        raise ValueError(
            f"Field value {field_val} exceeds maximum for "
            f"bit width {width} (max {(1 << width) - 1})"
        )
    shift = total_bits - end_1based
    mask = ((1 << width) - 1) << shift
    return (val & ~mask) | (field_val << shift)


class BitBuffer:
    """Mutable buffer wrapping a bitstream with 1-based MSB-first indexing."""

    __slots__ = ("_total_bits", "_val")

    def __init__(self, data: bytes | int | str, total_bits: int | None = None) -> None:
        if isinstance(data, bytes):
            self._total_bits = len(data) * 8 if total_bits is None else total_bits
            self._val = int.from_bytes(data, byteorder="big")
        elif isinstance(data, str):
            clean_str = data.strip()
            if clean_str.startswith(("0x", "0X")):
                clean_str = clean_str[2:]
            b = bytes.fromhex(clean_str)
            self._total_bits = len(b) * 8 if total_bits is None else total_bits
            self._val = int.from_bytes(b, byteorder="big")
        elif isinstance(data, int):
            if total_bits is None:
                raise ValueError("total_bits must be specified when initializing from int")
            self._total_bits = total_bits
            self._val = data
        else:
            raise TypeError(f"Unsupported data type for BitBuffer: {type(data)}")

    @classmethod
    def from_bytes(cls, b: bytes, total_bits: int | None = None) -> BitBuffer:
        """Create BitBuffer from raw bytes."""
        return cls(b, total_bits)

    @classmethod
    def from_hex(cls, hex_str: str, total_bits: int | None = None) -> BitBuffer:
        """Create BitBuffer from hex string."""
        return cls(hex_str, total_bits)

    @classmethod
    def from_int(cls, val: int, total_bits: int) -> BitBuffer:
        """Create BitBuffer from integer with bit count."""
        return cls(val, total_bits)

    @property
    def total_bits(self) -> int:
        """Total bit count."""
        return self._total_bits

    @property
    def total_bytes(self) -> int:
        """Total byte count."""
        return (self._total_bits + 7) // 8

    def extract(self, start_1based: int, end_1based: int) -> int:
        """Extract unsigned integer value from bit range [start_1based..end_1based]."""
        return extract_bits(self._val, self._total_bits, start_1based, end_1based)

    def extract_bool(self, bit_1based: int) -> bool:
        """Extract a single bit as a boolean."""
        return bool(extract_bits(self._val, self._total_bits, bit_1based, bit_1based))

    def set(self, start_1based: int, end_1based: int, field_val: int) -> None:
        """Set bit range [start_1based..end_1based] to field_val in place."""
        self._val = set_bits(self._val, self._total_bits, start_1based, end_1based, field_val)

    def to_bytes(self) -> bytes:
        """Export as big-endian bytes."""
        return uint_to_bytes(self._val, self.total_bytes)

    def to_hex(self) -> str:
        """Export as uppercase hex string."""
        b = self.to_bytes()
        return b.hex().upper()

    def to_int(self) -> int:
        """Return raw integer value."""
        return self._val

    def copy(self) -> Self:
        """Create a deep copy of this BitBuffer."""
        return self.__class__(self._val, self._total_bits)

    def __repr__(self) -> str:
        return f"BitBuffer(total_bits={self._total_bits}, hex='{self.to_hex()}')"


__all__ = [
    "BitBuffer",
    "bytes_to_uint",
    "extract_bits",
    "set_bits",
    "uint_to_bytes",
]
