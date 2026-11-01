# 02 — Complete ADS-B, Mode S, UAT & Wire Protocol Specification

> **Purpose:** This document is the single source of truth for bit-level packing, unpacking, CRC parity generation, Compact Position Reporting (CPR) mathematics, Gillham Gray code altitude encoding, 1090ES Type Codes (DO-260B/C), Mode S Comm-B BDS registers, 978 MHz UAT structures, and transport wire formats.

---

## 1. Bit Numbering & Slicing Primitives (`src/adsb/bitutils.py`)

ICAO Annex 10 and RTCA DO-260B/C define bit positions using **1-based MSB-first indexing**:
- A 56-bit (7-byte) short Mode S frame has bits **1 to 56**.
- A 112-bit (14-byte) long Mode S frame has bits **1 to 112**.
- Within a 112-bit Extended Squitter (DF17/DF18) or Comm-B (DF20/DF21) frame, bits **33 to 88** form the 56-bit message field (`ME` for DF17/18, `MB` for DF20/21). Sub-specifications often refer to `ME` bits **1 to 56** (where `ME` bit $k$ equals frame bit $k + 32$).

### 1.1 Canonical Python Bit Utilities

```python
def bytes_to_uint(data: bytes) -> tuple[int, int]:
    """Return (integer_value, total_bits) from big-endian bytes."""
    return int.from_bytes(data, byteorder="big"), len(data) * 8

def extract_bits(val: int, total_bits: int, start_1based: int, end_1based: int) -> int:
    """
    Extract inclusive 1-based MSB-first bit range [start_1based, end_1based].
    Example: extract_bits(frame_int, 112, 1, 5) extracts Downlink Format (DF).
    """
    assert 1 <= start_1based <= end_1based <= total_bits
    width = end_1based - start_1based + 1
    shift = total_bits - end_1based
    mask = (1 << width) - 1
    return (val >> shift) & mask

def set_bits(val: int, total_bits: int, start_1based: int, end_1based: int, field_val: int) -> int:
    """Write field_val into inclusive 1-based MSB-first bit range [start_1based, end_1based]."""
    width = end_1based - start_1based + 1
    shift = total_bits - end_1based
    mask = ((1 << width) - 1) << shift
    return (val & ~mask) | ((field_val << shift) & mask)
```

---

## 2. Mode S 24-Bit CRC Parity & Syndrome Error Correction (`src/adsb/protocol/crc24.py`)

### 2.1 Generator Polynomial
Mode S uses a 24-bit Cyclic Redundancy Check over $\text{GF}(2)$ with degree-24 generator polynomial:
$$G(x) = x^{24} + x^{23} + x^{22} + x^{21} + x^{20} + x^{19} + x^{18} + x^{17} + x^{16} + x^{15} + x^{14} + x^{13} + x^{12} + x^{10} + x^3 + 1$$
- **25-bit Binary Representation:** `111111111111010000001001` (`0x1FFF409`)
- **24-bit Truncated Polynomial Constant (excluding $x^{24}$):** `CRC24_POLY = 0xFFF409`

### 2.2 Reference CRC-24 Calculation Algorithm
For a frame of $N \in \{56, 112\}$ bits:
- The first $N - 24$ bits ($32$ bits for short frames, $88$ bits for long frames) are the data bits.
- The last $24$ bits (bits $N-23 \dots N$) carry either the **Parity/Interrogator Identifier (`PI`)** (for DF 11, 17, 18) or **Address/Parity (`AP`)** (for DF 0, 4, 5, 16, 20, 21, 24).

```python
CRC24_POLY: int = 0xFFF409

def compute_crc24(data_bytes: bytes) -> int:
    """
    Compute the 24-bit Mode S parity over the first (len(data_bytes) - 3) bytes.
    When encoding, pass the full 7- or 14-byte frame with the last 3 bytes zeroed.
    """
    n_data_bits = (len(data_bytes) - 3) * 8
    # Load data bits shifted left by 24 bits (or operate in-place on top 24 bits)
    reg = int.from_bytes(data_bytes[:-3], "big") << 24
    for bit_idx in range(n_data_bits - 1, -1, -1):
        if (reg >> (bit_idx + 24)) & 1:
            reg ^= (CRC24_POLY << bit_idx)
    return reg & 0xFFFFFF

def compute_syndrome(frame_bytes: bytes) -> int:
    """
    Compute the 24-bit syndrome of a received 7- or 14-byte frame.
    For DF17/DF18 (and DF11 with II=0), a valid error-free frame has syndrome == 0.
    For DF0/4/5/16/20/21, syndrome == (ICAO_24bit_address) if error-free.
    """
    expected_crc = compute_crc24(frame_bytes)
    received_tail = int.from_bytes(frame_bytes[-3:], "big")
    return expected_crc ^ received_crc_tail(received_tail)  # i.e. expected_crc ^ received_tail
```
*(In implementation, accelerate `compute_crc24` using a precomputed 256-entry byte lookup table `CRC24_TABLE[256]` for $O(\text{bytes})$ execution.)*

### 2.3 Address/Parity (`AP`) vs. Parity/Interrogator (`PI`) Recovery
- **DF 17 / DF 18:** The 24-bit ICAO address (`AA`) is transmitted explicitly in bits 9..32. The last 24 bits (`PI`, bits 89..112) equal `compute_crc24(frame)` directly. Syndrome $S = \text{compute\_crc24}(\text{frame}) \oplus \text{PI}$.
  - If $S == 0$: Frame is valid with 0 bit errors.
  - If $S \ne 0$: Look up $S$ in the precomputed **Single/Adjacent-2-Bit Error Syndrome Table** (112 single-bit error syndromes + 111 adjacent two-bit burst syndromes). If found and conservative error correction is enabled, flip the corresponding bit(s) and mark `bit_errors_corrected = 1` (or `2`).
- **DF 11 (All-Call Reply):** Contains explicit ICAO `AA` in bits 9..32. `PI` (bits 33..56) is CRC XORed with a 7-bit Interrogator Identifier (`II` in bits 50..56, i.e., $S \in [0, 79]$). A frame is valid if $S \le 79$.
- **DF 0, 4, 5, 16, 20, 21:** Do **not** transmit the ICAO address in the header. Instead, the transponder transmits $\text{AP} = \text{compute\_crc24}(\text{frame}) \oplus \text{ICAO}_{24}$.
  - Therefore, the syndrome $S = \text{compute\_crc24}(\text{frame}) \oplus \text{AP}$ **is the 24-bit ICAO address**!
  - **Crucial Validation Rule:** Because any random bit error in DF 0/4/5/16/20/21 produces a valid-looking 24-bit number $S$, the decoder/tracker **MUST** verify that $S$ belongs to the **Known Active ICAO Whitelist** (populated by CRC-verified DF11/DF17/DF18 frames within the last 180 seconds) before accepting a DF 0/4/5/16/20/21 frame.

---

## 3. Mode S Downlink Formats (DF) Overview

Every Mode S frame begins with a 5-bit **Downlink Format (`DF`)** in bits **1..5**:

| DF | Length | Name | Key Fields (1-based Frame Bits) | Parity Field |
| :---: | :---: | :--- | :--- | :--- |
| **0** | 56 bits (7B) | Short Air-Air Surveillance (ACAS) | `DF`[1..5]=0, `VS`[6], `CC`[7], `SL`[9..11], `RI`[14..17], `AC`[20..32] (13-bit alt), `AP`[33..56] | `AP` (CRC $\oplus$ ICAO) |
| **4** | 56 bits (7B) | Surveillance, Altitude Reply | `DF`[1..5]=4, `FS`[6..8], `DR`[9..13], `UM`[14..19], `AC`[20..32] (13-bit alt), `AP`[33..56] | `AP` (CRC $\oplus$ ICAO) |
| **5** | 56 bits (7B) | Surveillance, Identity Reply | `DF`[1..5]=5, `FS`[6..8], `DR`[9..13], `UM`[14..19], `ID`[20..32] (13-bit squawk), `AP`[33..56] | `AP` (CRC $\oplus$ ICAO) |
| **11** | 56 bits (7B) | Mode S Only All-Call Reply | `DF`[1..5]=11, `CA`[6..8], `AA`[9..32] (24-bit ICAO), `PI`[33..56] | `PI` (CRC $\oplus$ II) |
| **16** | 112 bits (14B) | Long Air-Air Surveillance (ACAS) | `DF`[1..5]=16, `VS`[6], `SL`[9..11], `RI`[14..17], `AC`[20..32], `MV`[33..88], `AP`[89..112] | `AP` (CRC $\oplus$ ICAO) |
| **17** | 112 bits (14B) | Extended Squitter (ADS-B) | `DF`[1..5]=17, `CA`[6..8], `AA`[9..32] (24-bit ICAO), `ME`[33..88] (56-bit ADS-B), `PI`[89..112] | `PI` (Pure CRC) |
| **18** | 112 bits (14B) | Extended Squitter / Non-Transponder (TIS-B / ADS-R) | `DF`[1..5]=18, `CF`[6..8], `AA`[9..32], `ME`[33..88], `PI`[89..112] | `PI` (Pure CRC) |
| **20** | 112 bits (14B) | Comm-B, Altitude Reply | `DF`[1..5]=20, `FS`[6..8], `DR`[9..13], `UM`[14..19], `AC`[20..32], `MB`[33..88] (56-bit BDS), `AP`[89..112] | `AP` (CRC $\oplus$ ICAO) |
| **21** | 112 bits (14B) | Comm-B, Identity Reply | `DF`[1..5]=21, `FS`[6..8], `DR`[9..13], `UM`[14..19], `ID`[20..32], `MB`[33..88] (56-bit BDS), `AP`[89..112] | `AP` (CRC $\oplus$ ICAO) |
| **24** | 112 bits (14B) | Comm-D (ELM) | `DF`[1..5]=24 (bits 1..2 == `11`), `KE`[4], `ND`[5..8], `MD`[9..88], `AP`[89..112] | `AP` (CRC $\oplus$ ICAO) |

### 3.1 Capability (`CA`) Field (Bits 6..8 in DF11 / DF17)
- `0`: Level 1 transponder
- `1..3`: Reserved
- `4`: Level 2+ transponder, on-ground
- `5`: Level 2+ transponder, airborne
- `6`: Level 2+ transponder, either on-ground or airborne
- `7`: Downlink Request value is 0, or Flight Status is 2, 3, 4, or 5

### 3.2 Flight Status (`FS`) Field (Bits 6..8 in DF 4, 5, 20, 21)
- `0`: No alert, no SPI, aircraft is airborne
- `1`: No alert, no SPI, aircraft is on-ground
- `2`: Alert, no SPI, aircraft is airborne
- `3`: Alert, no SPI, aircraft is on-ground
- `4`: Alert and SPI, aircraft is airborne or on-ground
- `5`: No alert, SPI active, aircraft is airborne or on-ground

---

## 4. Altitude (`AC` / `ALT`) & Identity (`ID`) Encodings (`src/adsb/protocol/gillham.py`)

### 4.1 12-Bit ADS-B Altitude Field (`ALT` in DF17/18 `ME` bits 9..20, Frame bits 41..52)
The 12-bit `ALT` field has bits $b_1 b_2 b_3 b_4 b_5 b_6 b_7 Q b_9 b_{10} b_{11} b_{12}$ where bit 8 ($Q$-bit, frame bit 48) determines the resolution:
- **Case 1: $Q = 1$ (25-foot increments, valid for $[-1000\text{ ft}, +50175\text{ ft}]$):**
  Remove bit 8 ($Q$-bit) to form an 11-bit unsigned integer $N$:
  $$N = ((ALT \gg 5) \ll 4) \mid (ALT \ \& \ \text{0x0F})$$
  $$\text{altitude\_ft} = 25 \times N - 1000$$
  - **Encoding ($Q=1$):** Given $\text{altitude\_ft} \in [-1000, 50175]$:
    $$N = \operatorname{round}\!\left(\frac{\text{altitude\_ft} + 1000}{25}\right), \quad ALT = ((N \gg 4) \ll 5) \mid (1 \ll 4) \mid (N \ \& \ \text{0x0F})$$

- **Case 2: $Q = 0$ (100-foot Gillham Gray Code increments, used above $50,175\text{ ft}$ or legacy encoders):**
  The 11 remaining bits map to Gillham pulses:
  $$C_1 \, A_1 \, C_2 \, A_2 \, C_4 \, A_4 \, B_1 \, [Q=0] \, B_2 \, D_2 \, B_4 \, D_4$$
  1. Form the 8-bit 500-ft Gray code word $G_{500} = (D_2 D_4 A_1 A_2 A_4 B_1 B_2 B_4)_2$ and convert Gray to binary:
     $$B_{500} = \text{gray\_to\_binary}(G_{500})$$
  2. Form the 3-bit 100-ft Gray code word $G_{100} = (C_1 C_2 C_4)_2$. Note that $G_{100} \in \{0, 5, 7\}$ are invalid Gillham codes (return `None`). Map valid $G_{100}$ to step $C_{\text{step}}$:
     - $G_{100} = 1 \ (001_2) \rightarrow 1$
     - $G_{100} = 3 \ (011_2) \rightarrow 2$
     - $G_{100} = 2 \ (010_2) \rightarrow 3$
     - $G_{100} = 6 \ (110_2) \rightarrow 4$
     - $G_{100} = 4 \ (100_2) \rightarrow 5$
  3. If $B_{500}$ is odd ($B_{500} \bmod 2 == 1$), reflect the 100-ft step: $C_{\text{step}} \leftarrow 6 - C_{\text{step}}$.
  4. Compute altitude:
     $$\text{altitude\_ft} = 500 \times B_{500} + 100 \times C_{\text{step}} - 1300$$

### 4.2 13-Bit Mode S Altitude Code (`AC` in DF 0, 4, 16, 20, Frame bits 20..32)
Bits 20..32 ($13\text{ bits}$): $b_1 b_2 b_3 b_4 b_5 b_6 M b_8 Q b_{10} b_{11} b_{12} b_{13}$
- Frame bit 26 is the $M$-bit (`0` = feet, `1` = metric).
- When $M = 0$, remove bit 26 ($M$-bit) to obtain the exact 12-bit `ALT` structure above (where frame bit 28 is the $Q$-bit) and decode identically to Section 4.1.

### 4.3 13-Bit Mode S Identity / Squawk Code (`ID` in DF 5, 21, Frame bits 20..32)
The 13 bits (numbered $20..32$) interleave the four octal digits $A B C D$ (each consisting of pulses $4, 2, 1$) plus an unused $X=0$ bit at bit 26:
$$\text{Bits } 20..32 = C_1 \, A_1 \, C_2 \, A_2 \, C_4 \, A_4 \, [X=0] \, B_1 \, D_1 \, B_2 \, D_2 \, B_4 \, D_4$$
Reconstruct each octal digit:
$$\begin{aligned}
A &= (A_4 \ll 2) \mid (A_2 \ll 1) \mid A_1 \\
B &= (B_4 \ll 2) \mid (B_2 \ll 1) \mid B_1 \\
C &= (C_4 \ll 2) \mid (C_2 \ll 1) \mid C_1 \\
D &= (D_4 \ll 2) \mid (D_2 \ll 1) \mid D_1
\end{aligned}$$
Squawk string is `f"{A}{B}{C}{D}"` (e.g., `"7700"` for emergency, `"7600"` for radio failure, `"7500"` for hijacking).

---

## 5. Compact Position Reporting (CPR) Complete Mathematics (`src/adsb/protocol/cpr.py`)

ADS-B encodes latitude and longitude into 17-bit integers ($\text{YZ}, \text{XZ} \in [0, 131071]$) using two alternating grids: **Even ($i = 0$)** and **Odd ($i = 1$)**.

### 5.1 Fundamental CPR Constants & Functions
- Number of latitude zones between equator and pole: $N_Z = 15$
- Modulo operation: Define $\operatorname{mods}(x, y) = x - y \lfloor x / y \rfloor$ (always non-negative, identical to Python's `x % y` when $y > 0$).
- **Exact Latitude Zone Function $\operatorname{NL}(\phi)$** for latitude $\phi$ in degrees:
  - If $\phi == 0$: $\operatorname{NL}(0) = 59$.
  - If $|\phi| == 87^\circ$: $\operatorname{NL}(\pm 87^\circ) = 2$.
  - If $|\phi| > 87^\circ$: $\operatorname{NL}(\phi) = 1$.
  - For $0 < |\phi| < 87^\circ$:
    $$\operatorname{NL}(\phi) = \left\lfloor \frac{2\pi}{\arccos\!\left(1 - \frac{1 - \cos(\pi / (2 N_Z))}{\cos^2(\frac{\pi}{180}\phi)}\right)} \right\rfloor$$
  *(Implementation Optimization: Precompute the 58 transition latitudes $\phi_{\text{trans}}(n)$ for $n \in [2..59]$ using the inverse formula $\phi_{\text{trans}}(n) = \frac{180}{\pi} \arccos\!\sqrt{\frac{1 - \cos(\pi/(2N_Z))}{1 - \cos(2\pi/n)}}$ and use `bisect` for $O(\log 59)$ exact lookup alongside the exact floating-point formula.)*

### 5.2 Airborne CPR Encoding (17-bit $\text{YZ}_i, \text{XZ}_i$)
For frame parity $i \in \{0, 1\}$ (0 = Even, 1 = Odd) and geodetic coordinates $(\phi, \lambda)$ in degrees:
1. Latitude zone size: $\Delta\text{lat}_i = \frac{360^\circ}{4 N_Z - i} = \begin{cases} 6.0^\circ & (i=0) \\ \frac{360^\circ}{59} \approx 6.101695^\circ & (i=1) \end{cases}$
2. Encoded 17-bit Latitude $\text{YZ}_i$:
   $$\text{YZ}_i = \left\lfloor 131072 \times \frac{\operatorname{mods}(\phi, \Delta\text{lat}_i)}{\Delta\text{lat}_i} + \frac{1}{2} \right\rfloor \bmod 131072$$
3. Reconstruct the quantized receiver latitude $\text{Rlat}_i$ (used to compute $\operatorname{NL}$ consistently between encoder and decoder!):
   $$\text{Rlat}_i = \Delta\text{lat}_i \left( \frac{\text{YZ}_i}{131072} + \left\lfloor \frac{\phi}{\Delta\text{lat}_i} \right\rfloor \right)$$
4. Longitude zone size: Let $N_i = \max(\operatorname{NL}(\text{Rlat}_i) - i, \, 1)$. Then $\Delta\text{lon}_i = \frac{360^\circ}{N_i}$.
5. Encoded 17-bit Longitude $\text{XZ}_i$:
   $$\text{XZ}_i = \left\lfloor 131072 \times \frac{\operatorname{mods}(\lambda, \Delta\text{lon}_i)}{\Delta\text{lon}_i} + \frac{1}{2} \right\rfloor \bmod 131072$$

### 5.3 Airborne Global CPR Decoding (Pairing Even + Odd within $\le 10\text{ s}$)
Given normalized fractions $\text{lat}_{\text{cpr},0} = \frac{\text{YZ}_0}{131072}$, $\text{lon}_{\text{cpr},0} = \frac{\text{XZ}_0}{131072}$ (Even) and $\text{lat}_{\text{cpr},1} = \frac{\text{YZ}_1}{131072}$, $\text{lon}_{\text{cpr},1} = \frac{\text{XZ}_1}{131072}$ (Odd):
1. Compute latitude zone index $j$:
   $$j = \left\lfloor 59 \cdot \text{lat}_{\text{cpr},0} - 60 \cdot \text{lat}_{\text{cpr},1} + \frac{1}{2} \right\rfloor$$
2. Compute candidate latitudes $\text{Rlat}_0$ and $\text{Rlat}_1$:
   $$\text{Rlat}_0 = \frac{360^\circ}{60} \left( (j \bmod 60) + \text{lat}_{\text{cpr},0} \right), \qquad \text{Rlat}_1 = \frac{360^\circ}{59} \left( (j \bmod 59) + \text{lat}_{\text{cpr},1} \right)$$
   If $\text{Rlat}_i \ge 270^\circ$, subtract $360^\circ$ so $\text{Rlat}_i \in [-90^\circ, +90^\circ]$.
3. **Zone Straddle Check (Mandatory):**
   Verify $\operatorname{NL}(\text{Rlat}_0) == \operatorname{NL}(\text{Rlat}_1)$. If they differ, the aircraft crossed a latitude zone boundary between the Even and Odd frames—**reject the global decode** and wait for the next frame!
4. Select the most recent frame $i \in \{0, 1\}$ ($i=0$ if $t_{\text{even}} \ge t_{\text{odd}}$, else $i=1$):
   - $\phi = \text{Rlat}_i$
   - $n_l = \operatorname{NL}(\phi)$
   - $N_i = \max(n_l - i, \, 1)$
   - Longitude index $m = \left\lfloor \text{lon}_{\text{cpr},0}(n_l - 1) - \text{lon}_{\text{cpr},1} n_l + \frac{1}{2} \right\rfloor$
   - $\lambda = \frac{360^\circ}{N_i} \left( (m \bmod N_i) + \text{lon}_{\text{cpr},i} \right)$
   - If $\lambda \ge 180^\circ$, set $\lambda \leftarrow \lambda - 360^\circ$.

### 5.4 Airborne Local CPR Decoding (Single Frame + Reference Position $(\phi_{\text{ref}}, \lambda_{\text{ref}})$)
Valid when the reference position (either the aircraft's last confirmed track position or, for surface messages, the receiver position) is within $\pm 3.0^\circ$ ($\approx 180\text{ NM}$ airborne, $\approx 45\text{ NM}$ surface) of the true position:
1. $\Delta\text{lat}_i = \frac{360^\circ}{60 - i}$ (Airborne) or $\frac{90^\circ}{60 - i}$ (Surface)
2. $j = \left\lfloor \frac{\phi_{\text{ref}}}{\Delta\text{lat}_i} \right\rfloor + \left\lfloor \frac{1}{2} + \frac{\operatorname{mods}(\phi_{\text{ref}}, \Delta\text{lat}_i)}{\Delta\text{lat}_i} - \text{lat}_{\text{cpr},i} \right\rfloor$
3. $\phi = \Delta\text{lat}_i \left( j + \text{lat}_{\text{cpr},i} \right)$
4. $N_i = \max(\operatorname{NL}(\phi) - i, \, 1)$
5. $\Delta\text{lon}_i = \frac{360^\circ}{N_i}$ (Airborne) or $\frac{90^\circ}{N_i}$ (Surface)
6. $m = \left\lfloor \frac{\lambda_{\text{ref}}}{\Delta\text{lon}_i} \right\rfloor + \left\lfloor \frac{1}{2} + \frac{\operatorname{mods}(\lambda_{\text{ref}}, \Delta\text{lon}_i)}{\Delta\text{lon}_i} - \text{lon}_{\text{cpr},i} \right\rfloor$
7. $\lambda = \Delta\text{lon}_i \left( m + \text{lon}_{\text{cpr},i} \right)$

### 5.5 Surface CPR Encoding & Decoding (TC 5..8)
Surface CPR uses a $4\times$ finer grid ($90^\circ$ span instead of $360^\circ$):
- Replace $360^\circ$ with $90^\circ$ in $\Delta\text{lat}_i = \frac{90^\circ}{60 - i}$ and $\Delta\text{lon}_i = \frac{90^\circ}{\max(\operatorname{NL}(\text{Rlat}_i) - i, 1)}$.
- Because $90^\circ$ is ambiguous across the 4 quadrants of the globe (Northern/Southern hemisphere and 4 longitude quadrants), **Global Surface CPR decoding always requires a reference position $(\phi_{\text{ref}}, \lambda_{\text{ref}})$** (such as the receiver location or airport coordinates) to pick the correct $90^\circ$ quadrant offset $k_{\text{lat}} \in \{-90^\circ, 0^\circ\}$, $k_{\text{lon}} \in \{-180^\circ, -90^\circ, 0^\circ, +90^\circ\}$ closest to $(\phi_{\text{ref}}, \lambda_{\text{ref}})$.

---

## 6. 1090 MHz Extended Squitter (DF17 / DF18) Type Codes (`src/adsb/protocol/es1090.py`)

In DF17 and DF18, bits **33..88** form the 56-bit `ME` payload. The first 5 bits of `ME` (`ME` bits **1..5**, Frame bits **33..37**) define the **Type Code (`TC` $\in [1..31]$)**:

| Type Code (`TC`) | `ME` Subtype (`ST`) | Message Type |
| :---: | :---: | :--- |
| **1..4** | `ME`[6..8] = Category `CA` | Aircraft Identification and Category (Callsign) |
| **5..8** | None | Surface Position ($90^\circ$ high-res CPR + Movement + Ground Track) |
| **9..18** | None | Airborne Position with Barometric Altitude ($360^\circ$ CPR) |
| **19** | `ME`[6..8] = `1..4` | Airborne Velocity (1/2 = Groundspeed vector, 3/4 = Heading & Airspeed) |
| **20..22** | None | Airborne Position with GNSS Height (HAE) ($360^\circ$ CPR) |
| **23** | `ME`[6..8] = `0..7` | Test Message / National Use |
| **24** | `ME`[6..8] = `1` | Surface System Status |
| **28** | `ME`[6..8] = `1, 2` | Aircraft Status (1 = Emergency/Priority Status & Squawk, 2 = ACAS RA) |
| **29** | `ME`[6..7] = `0, 1` | Target State and Status (Selected Altitude MCP/FCU, QNH, Autopilot Modes) |
| **31** | `ME`[6..8] = `0, 1` | Aircraft Operational Status (0 = Airborne, 1 = Surface; DO-260B/C Version, NIC, NACp, SIL, SDA) |

### 6.1 TC 1..4: Aircraft Identification (`ME` bits 1..56)
- `ME`[1..5]: `TC` (`1` = Category Set D, `2` = Set C, `3` = Set B, `4` = Set A)
- `ME`[6..8]: `CA` (Emitter Category 0..7)
  - For `TC=4` (Set A): `CA=1` $\rightarrow$ `A1` (Light), `CA=2` $\rightarrow$ `A2` (Small), `CA=3` $\rightarrow$ `A3` (Large), `CA=4` $\rightarrow$ `A4` (High Vortex), `CA=5` $\rightarrow$ `A5` (Heavy), `CA=6` $\rightarrow$ `A6` (High Perf), `CA=7` $\rightarrow$ `A7` (Rotorcraft).
- `ME`[9..56]: 8 characters $\times$ 6 bits (`C1..C8`).
  - **AIS 6-bit Character Map (`CHARMAP` of length 64):**
    `"#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######"`
    - Codes `1..26` $\rightarrow$ `'A'..'Z'`
    - Codes `48..57` $\rightarrow$ `'0'..'9'`
    - Code `32` $\rightarrow$ `' '` (Space)

### 6.2 TC 5..8: Surface Position (`ME` bits 1..56)
- `ME`[1..5]: `TC` (5..8)
- `ME`[6..12]: 7-bit Movement (`MOV`):
  - `0`: No information (`None`)
  - `1`: Stopped ($0\text{ kt}$)
  - `2..8`: $0.125\text{ kt}$ to $0.875\text{ kt}$ in $0.125\text{ kt}$ steps: $0.125 + (\text{MOV} - 2) \times 0.125$
  - `9..12`: $1.0\text{ kt}$ to $1.75\text{ kt}$ in $0.25\text{ kt}$ steps: $1.0 + (\text{MOV} - 9) \times 0.25$
  - `13..38`: $2.0\text{ kt}$ to $14.5\text{ kt}$ in $0.5\text{ kt}$ steps: $2.0 + (\text{MOV} - 13) \times 0.5$
  - `39..93`: $15.0\text{ kt}$ to $69.0\text{ kt}$ in $1.0\text{ kt}$ steps: $15.0 + (\text{MOV} - 39) \times 1.0$
  - `94..108`: $70.0\text{ kt}$ to $98.0\text{ kt}$ in $2.0\text{ kt}$ steps: $70.0 + (\text{MOV} - 94) \times 2.0$
  - `109..123`: $100.0\text{ kt}$ to $170.0\text{ kt}$ in $5.0\text{ kt}$ steps: $100.0 + (\text{MOV} - 109) \times 5.0$
  - `124`: $\ge 175.0\text{ kt}$
- `ME`[13]: Ground Track Status (`S`: `1` = valid, `0` = invalid)
- `ME`[14..20]: 7-bit Ground Track (`TRK`): $\text{track\_deg} = \text{TRK} \times \frac{360.0}{128.0}$
- `ME`[21]: Time bit (`T`)
- `ME`[22]: CPR Format (`F`: `0` = Even, `1` = Odd)
- `ME`[23..39]: 17-bit Encoded Latitude (`CPR_LAT`)
- `ME`[40..56]: 17-bit Encoded Longitude (`CPR_LON`)

### 6.3 TC 9..18 & 20..22: Airborne Position (`ME` bits 1..56)
- `ME`[1..5]: `TC` (`9..18` = Baro Altitude, `20..22` = GNSS HAE)
- `ME`[6..7]: Surveillance Status (`SS`: `0` = normal, `1` = permanent alert, `2` = temp alert, `3` = SPI)
- `ME`[8]: Single Antenna Flag (`SAF` / `NIC_B`)
- `ME`[9..20]: 12-bit Altitude (`ALT`, decoded via Section 4.1)
- `ME`[21]: Time Synchronization (`T`)
- `ME`[22]: CPR Format (`F`: `0` = Even, `1` = Odd)
- `ME`[23..39]: 17-bit Encoded Latitude (`CPR_LAT`)
- `ME`[40..56]: 17-bit Encoded Longitude (`CPR_LON`)

### 6.4 TC 19: Airborne Velocity (`ME` bits 1..56)
- `ME`[1..5]: `TC` = `19`
- `ME`[6..8]: Subtype (`ST` $\in \{1, 2, 3, 4\}$)
- `ME`[9]: Intent Change Flag (`IC`)
- `ME`[10]: IFR Capability Flag (`IFR`)
- `ME`[11..13]: Velocity Uncertainty (`NACv`, 0..4)
- **For Subtypes 1 (Subsonic, scale $k=1$) and 2 (Supersonic, scale $k=4$):**
  - `ME`[14]: East-West Sign (`S_EW`: `0` = West $\rightarrow$ East (+), `1` = East $\rightarrow$ West (-))
  - `ME`[15..24]: 10-bit East-West Velocity (`V_EW_RAW`): If `0`, unavailable; else $v_{\text{ew}} = (-1)^{S_{\text{EW}}} \times k \times (\text{V\_EW\_RAW} - 1)\text{ kt}$
  - `ME`[25]: North-South Sign (`S_NS`: `0` = South $\rightarrow$ North (+), `1` = North $\rightarrow$ South (-))
  - `ME`[26..35]: 10-bit North-South Velocity (`V_NS_RAW`): If `0`, unavailable; else $v_{\text{ns}} = (-1)^{S_{\text{NS}}} \times k \times (\text{V\_NS\_RAW} - 1)\text{ kt}$
  - Groundspeed: $\text{GS} = \sqrt{v_{\text{ew}}^2 + v_{\text{ns}}^2}$, True Track: $\theta = \operatorname{atan2}(v_{\text{ew}}, v_{\text{ns}}) \times \frac{180}{\pi} \bmod 360^\circ$
- **For Subtypes 3 (Subsonic, $k=1$) and 4 (Supersonic, $k=4$):**
  - `ME`[14]: Heading Status (`S_HDG`: `1` = available)
  - `ME`[15..24]: 10-bit Magnetic Heading (`HDG_RAW`): $\text{hdg\_deg} = \text{HDG\_RAW} \times \frac{360.0}{1024.0}$
  - `ME`[25]: Airspeed Type (`AS_T`: `0` = IAS, `1` = TAS)
  - `ME`[26..35]: 10-bit Airspeed (`AS_RAW`): If `0`, unavailable; else $\text{AS} = k \times (\text{AS\_RAW} - 1)\text{ kt}$
- **Common Vertical Rate & Geo-Baro Difference (`ME` bits 36..56 for all Subtypes 1..4):**
  - `ME`[36]: Vertical Rate Source (`VrSrc`: `0` = GNSS/Geometric, `1` = Barometric)
  - `ME`[37]: Vertical Rate Sign (`S_VR`: `0` = Up (+), `1` = Down (-))
  - `ME`[38..46]: 9-bit Vertical Rate (`VR_RAW`): If `0`, unavailable; else $\text{VR} = (-1)^{S_{\text{VR}}} \times 64 \times (\text{VR\_RAW} - 1)\text{ ft/min}$
  - `ME`[47..48]: Reserved (`00`)
  - `ME`[49]: Geo-Minus-Baro Sign (`S_DIF`: `0` = GNSS above Baro (+), `1` = GNSS below Baro (-))
  - `ME`[50..56]: 7-bit Geo-Minus-Baro Difference (`DIF_RAW`): If `0`, unavailable; else $\Delta h_{\text{geo-baro}} = (-1)^{S_{\text{DIF}}} \times 25 \times (\text{DIF\_RAW} - 1)\text{ ft}$

### 6.5 TC 28: Aircraft Status (Emergency / Priority & Squawk)
- `ME`[1..5] = `28`, `ME`[6..8] = Subtype `1`
- `ME`[9..11]: 3-bit Emergency/Priority State (`0`=No emergency, `1`=General emergency, `2`=Lifeguard/Medical, `3`=Minimum fuel, `4`=No communications, `5`=Unlawful interference, `6`=Downed aircraft)
- `ME`[12..24]: 13-bit Mode A Squawk Code (decoded identically to Section 4.3)

### 6.6 TC 29: Target State and Status (`ME` Subtype 1, DO-260B/C)
- `ME`[1..5] = `29`, `ME`[6..7] = `1`
- `ME`[8]: SIL Supplement
- `ME`[9]: Selected Altitude Type (`0` = MCP/FCU, `1` = FMS)
- `ME`[10..20]: 11-bit Selected Altitude (`SEL_ALT`): If `0`, unavailable; else $(\text{SEL\_ALT} - 1) \times 32\text{ ft}$
- `ME`[21..29]: 9-bit Barometric Pressure Setting (`BARO`): If `0`, unavailable; else $800.0 + (\text{BARO} - 1) \times 0.8\text{ hPa}$
- `ME`[30]: Selected Heading Status (`1` = valid)
- `ME`[31]: Selected Heading Sign (`1` = negative / West)
- `ME`[32..39]: 8-bit Selected Heading (`SEL_HDG`): $\text{hdg} = \text{SEL\_HDG} \times \frac{180.0}{256.0}$ (add $180^\circ$ if sign bit is 1)
- `ME`[40..41]: `NACp` (bits 40..43), `NIC_baro` (bit 44), `SIL` (bits 45..46)
- `ME`[47]: Status of MCP/FCU Mode bits (`1` = valid)
- `ME`[48]: Autopilot Engaged (`AP`), `ME`[49]: VNAV Mode, `ME`[50]: Altitude Hold, `ME`[52]: Approach Mode, `ME`[53]: TCAS Operational, `ME`[54]: LNAV Mode

### 6.7 TC 31: Aircraft Operational Status (DO-260B/C)
- `ME`[1..5] = `31`, `ME`[6..8] = Subtype (`0` = Airborne, `1` = Surface)
- `ME`[41..43]: ADS-B Version Number (`0` = DO-260, `1` = DO-260A, `2` = DO-260B, `3` = DO-260C)
- `ME`[44]: `NIC_supp_A`
- `ME`[45..48]: 4-bit `NACp` (Navigation Accuracy Category for Position, 0..11)
- `ME`[49..50]: `GVA` (Geometric Vertical Accuracy, airborne)
- `ME`[51..52]: `SIL` (Source Integrity Level, 0..3)
- `ME`[53]: `NIC_baro` (airborne) or Track Angle / Heading flag (surface)
- `ME`[54]: `HRD` (Horizontal Reference Direction: `0` = True North, `1` = Magnetic North)
- `ME`[55]: `SIL_supp`

---

## 7. Mode S Comm-B BDS Registers (DF20 / DF21 `MB` Field, `src/adsb/protocol/bds.py`)

In DF20 and DF21, bits **33..88** form the 56-bit `MB` field (`MB` bits 1..56). Because Mode S downlink frames do not carry an explicit BDS tag in the header (except BDS 1,0 and BDS 2,0), the decoder must support both **explicit BDS decoding** and **heuristic/multi-hypothesis BDS identification**:

1. **BDS 1,0 (Data Link Capability Report):** `MB`[1..8] == `0x10` (`00010000_2`).
2. **BDS 1,7 (Common Usage GICB Capability Report):** Bitmask indicating which BDS registers are active on the aircraft (e.g., bit 1 = 0,5 ES airborne pos, bit 2 = 0,6 ES surface pos, bit 7 = 2,0 Aircraft ID, bit 8 = 4,0 Selected vertical intention, bit 9 = 5,0 Track and turn, bit 10 = 6,0 Heading and speed).
3. **BDS 2,0 (Aircraft Identification):** `MB`[1..8] == `0x20` (`00100000_2`), followed by `MB`[9..56] containing the 8-character 6-bit AIS callsign (same `CHARMAP` as TC 1..4).
4. **BDS 4,0 (Selected Vertical Intention):**
   - `MB`[1] + `MB`[2..13]: MCP/FCU Selected Altitude (12 bits, scale $16\text{ ft}$)
   - `MB`[14] + `MB`[15..26]: FMS Selected Altitude (12 bits, scale $16\text{ ft}$)
   - `MB`[27] + `MB`[28..39]: Barometric Pressure Setting (12 bits, $800.0 + \text{val} \times 0.1\text{ mb}$)
   - `MB`[48]: Status of Mode bits; `MB`[49]: VNAV, `MB`[50]: Alt Hold, `MB`[51]: Approach
   - `MB`[54]: Status of Target Alt Source; `MB`[55..56]: Target Alt Source
5. **BDS 5,0 (Track and Turn Report):**
   - `MB`[1] (status) + `MB`[2] (sign) + `MB`[3..11] (9 bits): Roll Angle (scale $45/256\text{ deg}$, range $[-90^\circ, +90^\circ]$)
   - `MB`[12] (status) + `MB`[13] (sign) + `MB`[14..23] (10 bits): True Track Angle (scale $90/512\text{ deg}$)
   - `MB`[24] (status) + `MB`[25..34] (10 bits): Ground Speed (scale $2\text{ kt}$, range $[0, 2046]\text{ kt}$)
   - `MB`[35] (status) + `MB`[36] (sign) + `MB`[37..45] (9 bits): Track Angle Rate (scale $8/256\text{ deg/s}$)
   - `MB`[46] (status) + `MB`[47..56] (10 bits): True Airspeed (scale $2\text{ kt}$)
6. **BDS 6,0 (Heading and Speed Report):**
   - `MB`[1] (status) + `MB`[2] (sign) + `MB`[3..12] (10 bits): Magnetic Heading (scale $90/512\text{ deg}$)
   - `MB`[13] (status) + `MB`[14..23] (10 bits): Indicated Airspeed (scale $1\text{ kt}$)
   - `MB`[24] (status) + `MB`[25..34] (10 bits): Mach Number (scale $0.004$, range $[0, 4.092]$)
   - `MB`[35] (status) + `MB`[36] (sign) + `MB`[37..45] (9 bits): Barometric Altitude Rate (scale $32\text{ ft/min}$)
   - `MB`[46] (status) + `MB`[47] (sign) + `MB`[48..56] (9 bits): Inertial Vertical Velocity (scale $32\text{ ft/min}$)

---

## 8. 978 MHz UAT (Universal Access Transceiver) Specification (`src/adsb/protocol/uat978.py`)

UAT operates at $978\text{ MHz}$ ($1.041667\text{ Mbps}$ CPFSK) under RTCA DO-282B.

### 8.1 UAT ADS-B Payload Types & Lengths
1. **Basic UAT ADS-B Message:** **18 bytes (144 bits)** user payload + **12 bytes (96 bits)** Reed-Solomon $\text{RS}(30, 18)$ parity over $\text{GF}(2^8)$ (primitive polynomial $p(x) = x^8 + x^7 + x^2 + x + 1 = \text{0x187}$, generator root $\alpha^{120}$, correcting up to 6 symbol/byte errors).
2. **Long UAT ADS-B Message:** **34 bytes (272 bits)** user payload + **14 bytes (112 bits)** Reed-Solomon $\text{RS}(48, 34)$ parity (correcting up to 7 byte errors).

### 8.2 UAT 4-Byte Header (Bytes 1..4, Bits 1..32)
- Bits 1..5 (`HDR_TYPE`, 5 bits): Payload Type Code (`0` = Basic ADS-B 18B; `1..6` = Long ADS-B 34B).
- Bits 6..8 (`ADDR_QUAL`, 3 bits): Address Qualifier (`0` = ICAO 24-bit ADS-B, `1` = Self-assigned temporary/anonymous ID, `2` = TIS-B with ICAO, `3` = TIS-B with track file ID, `4` = Surface vehicle, `5` = Fixed ADS-B beacon).
- Bits 9..32 (`ADDRESS`, 24 bits): 24-bit ICAO or temporary address.

### 8.3 UAT State Vector (`SV`, Bytes 5..17, Bits 33..136)
Unlike 1090ES CPR, UAT transmits **direct 24-bit WGS84 Latitude and 24-bit Longitude** in every single frame (no Even/Odd pairing needed!):
- **Bits 33..55 (23 bits):** Signed 23-bit Latitude (`RAW_LAT`):
  $$\text{lat\_deg} = \text{RAW\_LAT} \times \frac{180.0}{2^{23}} \quad (\text{if bit 33 is 1, sign-extend from 23 bits; resolution } \approx 2.1458 \times 10^{-5}\text{ deg})$$
- **Bits 56..79 (24 bits):** Signed 24-bit Longitude (`RAW_LON`):
  $$\text{lon\_deg} = \text{RAW\_LON} \times \frac{360.0}{2^{24}} \quad (\text{two's complement 24-bit signed})$$
- **Bit 80 (`ALT_TYPE`):** `0` = Barometric Pressure Altitude, `1` = Geometric Altitude.
- **Bits 81..92 (12 bits, `RAW_ALT`):**
  $$\text{alt\_ft} = (\text{RAW\_ALT} - 1) \times 25 - 1000 \quad (\text{if } \text{RAW\_ALT} == 0, \text{ altitude unavailable})$$
- **Bits 93..96 (4 bits):** `NIC` (Navigation Integrity Category, 0..11).
- **Bits 97..98 (2 bits, `AG_STATE`):**
  - `0` (`00_2`): Airborne, Subsonic ($1\text{ kt}$ velocity steps)
  - `1` (`01_2`): Airborne, Supersonic ($4\text{ kt}$ velocity steps)
  - `2` (`10_2`): On Ground (Velocity fields encode Ground Speed + Track/Heading)
- **Airborne Velocity Fields (when `AG_STATE` $\in \{0, 1\}$, scale $k=1$ or $4$):**
  - **Bit 99 (`S_NS`):** `0` = North (+), `1` = South (-)
  - **Bits 100..109 (10 bits, `V_NS_RAW`):** If `0`, unavailable; else $v_{\text{ns}} = (-1)^{S_{\text{NS}}} \times k \times (\text{V\_NS\_RAW} - 1)\text{ kt}$
  - **Bit 110 (`S_EW`):** `0` = East (+), `1` = West (-)
  - **Bits 111..120 (10 bits, `V_EW_RAW`):** If `0`, unavailable; else $v_{\text{ew}} = (-1)^{S_{\text{EW}}} \times k \times (\text{V\_EW\_RAW} - 1)\text{ kt}$
  - **Bit 121 (`VR_SRC`):** `0` = Geometric, `1` = Barometric
  - **Bit 122 (`S_VR`):** `0` = Up (+), `1` = Down (-)
  - **Bits 123..131 (9 bits, `VR_RAW`):** If `0`, unavailable; else $\text{vr\_fpm} = (-1)^{S_{\text{VR}}} \times 64 \times (\text{VR\_RAW} - 1)$
- **Payload Type 1 (Bytes 18..29):** Mode Status (`MS`) containing 8-character Base-40 encoded Callsign/Flight ID, Emitter Category, Emergency Status, `NACp`, `NACv`, `SIL`, `SDA`.
- **Payload Type 3 & 4 (Bytes 18..29):** Target State (`TS`) and Trajectory Change.

---

## 9. Receiver & Transport Wire Formats (`src/adsb/wire/`)

### 9.1 Raw AVR ASCII Format (`src/adsb/wire/avr.py`)
1. **Standard AVR (`*<HEX>;\r\n`):**
   - Example 112-bit: `*8D4840D6202CC371C32CE0576098;` (28 hex chars = 14 bytes)
   - Example 56-bit: `*02E197B867A348;` (14 hex chars = 7 bytes)
2. **Timestamped MLAT AVR (`@<12_HEX_TIMESTAMP><HEX_PAYLOAD>;\r\n`):**
   - `@` followed by 12 hex characters (48-bit 12 MHz MLAT clock counter, 1 tick = $83.\overline{3}\text{ ns}$), followed by 14 or 28 hex characters of payload, terminated by `;`.

### 9.2 Mode S Beast Binary Format (`src/adsb/wire/beast.py`)
Industry-standard binary framing used by `dump1090`, `readsb`, and Radarcape:
- **Start Sequence:** `<esc>` (`0x1A`) followed by Frame Type byte:
  - `'1'` (`0x31`): Mode A/C frame (2 bytes payload)
  - `'2'` (`0x32`): Mode S Short frame (7 bytes / 56 bits payload)
  - `'3'` (`0x33`): Mode S Long frame (14 bytes / 112 bits payload)
- **Frame Body (immediately following `<esc><type>`):**
  1. **6-byte (48-bit) Big-Endian 12 MHz MLAT Timestamp:** $\text{t\_seconds} = \text{counter} / 12,000,000.0$.
  2. **1-byte Signal Level (`RSSI`):** Unsigned byte $L \in [1..255]$ where $\text{RSSI}_{\text{dBFS}} = 20 \log_{10}(L / 255.0)$.
  3. **Payload Bytes:** 2, 7, or 14 bytes.
- **Byte Stuffing / Escaping Rule (CRITICAL):**
  Any occurrence of `0x1A` inside the 6-byte timestamp, 1-byte RSSI, or payload bytes **MUST** be doubled (`0x1A 0x1A`) on encode and un-doubled on decode.

### 9.3 SBS-1 BaseStation CSV Format (`src/adsb/wire/sbs1.py`)
22-column comma-separated ASCII format (`MSG,<TransmissionType>,1,1,<ICAO_HEX>,1,<DateGen>,<TimeGen>,<DateLog>,<TimeLog>,<Callsign>,<Altitude>,<GroundSpeed>,<Track>,<Lat>,<Lon>,<VerticalRate>,<Squawk>,<Alert>,<Emergency>,<SPI>,<IsOnGround>`):
- `MSG,1`: ES Identification and Category (`Callsign`)
- `MSG,2`: ES Surface Position (`Altitude`, `GroundSpeed`, `Track`, `Lat`, `Lon`, `IsOnGround=-1`)
- `MSG,3`: ES Airborne Position (`Altitude`, `Lat`, `Lon`, `Alert`, `Emergency`, `SPI`, `IsOnGround=0`)
- `MSG,4`: ES Airborne Velocity (`GroundSpeed`, `Track`, `VerticalRate`)
- `MSG,5`: Surveillance Altitude Reply (`Altitude`, `Alert`, `SPI`, `IsOnGround`)
- `MSG,6`: Surveillance Identity Reply (`Altitude`, `Squawk`, `Alert`, `Emergency`, `SPI`, `IsOnGround`)
- `MSG,7`: Air-to-Air Reply (`Altitude`, `IsOnGround`)
- `MSG,8`: All-Call Reply (`IsOnGround`)

### 9.4 UAT `dump978` Raw Hex Format (`src/adsb/wire/uat_raw.py`)
- Downlink ADS-B Frame: `-<36_OR_68_HEX_CHARS>;rs=<ERRS>;rssi=<DBFS>;t=<TIMESTAMP>;`
- Uplink FIS-B Frame: `+<864_HEX_CHARS>;...`

---

## 10. Golden Protocol Verification Vectors

Every AI agent implementing `adsb.protocol` **MUST** verify its implementation against these exact bit-level reference vectors:

| Vector ID | Raw Hex (`*...;`) | DF / TC | Expected Decoded Fields |
| :--- | :--- | :---: | :--- |
| **GV-01 (Callsign)** | `8D4840D6202CC371C32CE0576098` | DF=17, TC=4 | `icao_hex="4840D6"`, `crc_valid=True`, `ca=0`, `callsign="KLM1023 "` |
| **GV-02 (Airborne Pos Even)** | `8D40621D58C382D690C8AC2863A7` | DF=17, TC=11 | `icao_hex="40621D"`, `crc_valid=True`, `altitude_ft=38000`, `q_bit=True`, `cpr_format=EVEN (0)`, `cpr_lat=93000`, `cpr_lon=51372` |
| **GV-03 (Airborne Pos Odd)** | `8D40621D58C386435CC412692AD6` | DF=17, TC=11 | `icao_hex="40621D"`, `crc_valid=True`, `altitude_ft=38000`, `q_bit=True`, `cpr_format=ODD (1)`, `cpr_lat=74158`, `cpr_lon=50194` |
| **GV-02+03 (Global CPR Pair)** | Pair `GV-02` ($t=0\text{ s}$) + `GV-03` ($t=1\text{ s}$) | Global CPR | `lat_deg = 52.25720` ($\pm 0.0001^\circ$), `lon_deg = 3.91937` ($\pm 0.0001^\circ$) |
| **GV-04 (Airborne Vel ST=1)** | `8D485020994409940838175B284F` | DF=17, TC=19, ST=1 | `icao_hex="485020"`, `crc_valid=True`, `v_ew_kt=-8`, `v_ns_kt=-159`, `groundspeed_kt=159.20`, `ground_track_deg=182.88`, `vertical_rate_fpm=-832` |
| **GV-05 (Airborne Vel ST=3)** | `8DA05F219B06B6AF189400CBC33F` | DF=17, TC=19, ST=3 | `icao_hex="A05F21"`, `crc_valid=True`, `magnetic_heading_deg=243.98`, `airspeed_kt=375`, `airspeed_is_tas=True`, `vertical_rate_fpm=-2304` |
| **GV-06 (DF11 All-Call)** | `5D4840D6F8740F` | DF=11 | `icao_hex="4840D6"`, `ca=5`, `crc_syndrome=0` (II=0) |

