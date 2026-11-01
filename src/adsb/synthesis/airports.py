"""Authoritative embedded airport database for flight trajectory synthesis and terminal routing.

Contains 31 major global hubs (20 European hubs and 11 International gateway airports)
with exact aerodrome coordinates, elevations, and primary runway geometries.
"""

from __future__ import annotations

from dataclasses import dataclass

from adsb.core.geodesy import destination_point


@dataclass(frozen=True, slots=True)
class Runway:
    """Runway specification with true heading, dimensions, and geodetic coordinates."""

    designation: str
    true_heading_deg: float
    length_m: float
    width_m: float
    threshold_lat: float
    threshold_lon: float
    threshold_elev_m: float
    midpoint_lat: float
    midpoint_lon: float

    @property
    def midpoint_lat_deg(self) -> float:
        """Runway midpoint latitude in decimal degrees WGS84."""
        return self.midpoint_lat

    @property
    def midpoint_lon_deg(self) -> float:
        """Runway midpoint longitude in decimal degrees WGS84."""
        return self.midpoint_lon

    @property
    def threshold_lat_deg(self) -> float:
        """Runway threshold latitude in decimal degrees WGS84."""
        return self.threshold_lat

    @property
    def threshold_lon_deg(self) -> float:
        """Runway threshold longitude in decimal degrees WGS84."""
        return self.threshold_lon

    @property
    def length_ft(self) -> float:
        """Runway length in feet."""
        return self.length_m / 0.3048

    @property
    def width_ft(self) -> float:
        """Runway width in feet."""
        return self.width_m / 0.3048

    @property
    def elevation_m(self) -> float:
        """Runway threshold elevation in meters MSL."""
        return self.threshold_elev_m

    @property
    def elevation_ft(self) -> float:
        """Runway threshold elevation in feet MSL."""
        return self.threshold_elev_m / 0.3048

    @property
    def threshold_elev_ft(self) -> float:
        """Runway threshold elevation in feet MSL."""
        return self.threshold_elev_m / 0.3048


@dataclass(frozen=True, slots=True)
class Airport:
    """Aerodrome record containing location, elevation, and operational runways."""

    icao: str
    iata: str
    name: str
    city: str
    country: str
    lat_deg: float
    lon_deg: float
    elev_m: float
    runways: tuple[Runway, ...]
    primary_runway_id: str

    @property
    def primary_runway(self) -> Runway:
        """Primary operational runway."""
        for rwy in self.runways:
            if rwy.designation == self.primary_runway_id:
                return rwy
        raise KeyError(
            f"Primary runway '{self.primary_runway_id}' not found for airport '{self.icao}'"
        )

    def get_runway(self, designation: str) -> Runway:
        """Retrieve runway by designation (case-insensitive)."""
        desig_clean = designation.strip().upper()
        for rwy in self.runways:
            if rwy.designation.upper() == desig_clean:
                return rwy
        raise KeyError(
            f"Runway '{designation}' not found at airport '{self.icao}'. "
            f"Available runways: {[r.designation for r in self.runways]}"
        )

    @property
    def elevation_m(self) -> float:
        """Airport elevation in meters MSL."""
        return self.elev_m

    @property
    def elevation_ft(self) -> float:
        """Airport elevation in feet MSL."""
        return self.elev_m / 0.3048

    @property
    def elev_ft(self) -> float:
        """Airport elevation in feet MSL (short alias)."""
        return self.elevation_ft


def _make_runway(
    designation: str,
    true_heading_deg: float,
    length_m: float,
    width_m: float,
    midpoint_lat: float,
    midpoint_lon: float,
    threshold_elev_m: float,
) -> Runway:
    """Helper to construct a Runway, computing threshold coordinates from midpoint."""
    reciprocal_heading = (true_heading_deg + 180.0) % 360.0
    thresh_lat, thresh_lon = destination_point(
        midpoint_lat,
        midpoint_lon,
        length_m / 2.0,
        reciprocal_heading,
    )
    return Runway(
        designation=designation,
        true_heading_deg=true_heading_deg,
        length_m=length_m,
        width_m=width_m,
        threshold_lat=thresh_lat,
        threshold_lon=thresh_lon,
        threshold_elev_m=threshold_elev_m,
        midpoint_lat=midpoint_lat,
        midpoint_lon=midpoint_lon,
    )


# ---------------------------------------------------------------------------
# Embedded Global Airport Catalog: 31 Hubs (20 European, 11 International)
# ---------------------------------------------------------------------------

_AIRPORT_RECORDS: tuple[Airport, ...] = (
    # --- 20 European Hubs ---
    Airport(
        icao="EGLL",
        iata="LHR",
        name="London Heathrow Airport",
        city="London",
        country="United Kingdom",
        lat_deg=51.4700,
        lon_deg=-0.4543,
        elev_m=25.3,
        primary_runway_id="27R",
        runways=(
            _make_runway("27R", 269.8, 3902.0, 50.0, 51.4775, -0.4614, 25.3),
        ),
    ),
    Airport(
        icao="LFPG",
        iata="CDG",
        name="Paris Charles de Gaulle Airport",
        city="Paris",
        country="France",
        lat_deg=49.0097,
        lon_deg=2.5479,
        elev_m=119.5,
        primary_runway_id="27L",
        runways=(
            _make_runway("27L", 266.0, 4215.0, 45.0, 49.0036, 2.5694, 119.5),
        ),
    ),
    Airport(
        icao="EHAM",
        iata="AMS",
        name="Amsterdam Airport Schiphol",
        city="Amsterdam",
        country="Netherlands",
        lat_deg=52.3105,
        lon_deg=4.7683,
        elev_m=-3.35,
        primary_runway_id="18R",
        runways=(
            _make_runway("18R", 183.0, 3800.0, 45.0, 52.3150, 4.7110, -3.35),
        ),
    ),
    Airport(
        icao="EDDF",
        iata="FRA",
        name="Frankfurt am Main Airport",
        city="Frankfurt",
        country="Germany",
        lat_deg=50.0379,
        lon_deg=8.5622,
        elev_m=111.0,
        primary_runway_id="25C",
        runways=(
            _make_runway("25C", 249.0, 4000.0, 60.0, 50.0333, 8.5706, 111.0),
        ),
    ),
    Airport(
        icao="LEMD",
        iata="MAD",
        name="Adolfo Suárez Madrid-Barajas Airport",
        city="Madrid",
        country="Spain",
        lat_deg=40.4839,
        lon_deg=-3.5680,
        elev_m=610.0,
        primary_runway_id="32L",
        runways=(
            _make_runway("32L", 324.0, 4100.0, 60.0, 40.4880, -3.5620, 610.0),
        ),
    ),
    Airport(
        icao="LIRF",
        iata="FCO",
        name="Rome Fiumicino Leonardo da Vinci Airport",
        city="Rome",
        country="Italy",
        lat_deg=41.8003,
        lon_deg=12.2389,
        elev_m=4.3,
        primary_runway_id="16R",
        runways=(
            _make_runway("16R", 161.0, 3900.0, 60.0, 41.7990, 12.2280, 4.3),
        ),
    ),
    Airport(
        icao="LOWW",
        iata="VIE",
        name="Vienna International Airport",
        city="Vienna",
        country="Austria",
        lat_deg=48.1103,
        lon_deg=16.5697,
        elev_m=183.0,
        primary_runway_id="29",
        runways=(
            _make_runway("29", 294.0, 3500.0, 45.0, 48.1120, 16.5650, 183.0),
        ),
    ),
    Airport(
        icao="EKCH",
        iata="CPH",
        name="Copenhagen Kastrup Airport",
        city="Copenhagen",
        country="Denmark",
        lat_deg=55.6180,
        lon_deg=12.6508,
        elev_m=5.2,
        primary_runway_id="22L",
        runways=(
            _make_runway("22L", 221.0, 3600.0, 45.0, 55.6140, 12.6580, 5.2),
        ),
    ),
    Airport(
        icao="LSZH",
        iata="ZRH",
        name="Zurich Kloten Airport",
        city="Zurich",
        country="Switzerland",
        lat_deg=47.4582,
        lon_deg=8.5555,
        elev_m=432.0,
        primary_runway_id="16",
        runways=(
            _make_runway("16", 156.0, 3700.0, 60.0, 47.4580, 8.5550, 432.0),
        ),
    ),
    Airport(
        icao="EDDM",
        iata="MUC",
        name="Munich Franz Josef Strauss Airport",
        city="Munich",
        country="Germany",
        lat_deg=48.3537,
        lon_deg=11.7861,
        elev_m=453.0,
        primary_runway_id="26L",
        runways=(
            _make_runway("26L", 262.0, 4000.0, 60.0, 48.3470, 11.7750, 453.0),
        ),
    ),
    Airport(
        icao="EBBR",
        iata="BRU",
        name="Brussels Airport",
        city="Brussels",
        country="Belgium",
        lat_deg=50.9010,
        lon_deg=4.4844,
        elev_m=56.0,
        primary_runway_id="25L",
        runways=(
            _make_runway("25L", 249.0, 3211.0, 45.0, 50.8980, 4.4860, 56.0),
        ),
    ),
    Airport(
        icao="EIDW",
        iata="DUB",
        name="Dublin Airport",
        city="Dublin",
        country="Ireland",
        lat_deg=53.4213,
        lon_deg=-6.2701,
        elev_m=74.0,
        primary_runway_id="28L",
        runways=(
            _make_runway("28L", 278.0, 2637.0, 45.0, 53.4210, -6.2550, 74.0),
        ),
    ),
    Airport(
        icao="LEBL",
        iata="BCN",
        name="Josep Tarradellas Barcelona-El Prat Airport",
        city="Barcelona",
        country="Spain",
        lat_deg=41.2974,
        lon_deg=2.0833,
        elev_m=4.3,
        primary_runway_id="24L",
        runways=(
            _make_runway("24L", 242.0, 3352.0, 45.0, 41.2930, 2.0880, 4.3),
        ),
    ),
    Airport(
        icao="LPPT",
        iata="LIS",
        name="Lisbon Humberto Delgado Airport",
        city="Lisbon",
        country="Portugal",
        lat_deg=38.7742,
        lon_deg=-9.1342,
        elev_m=114.0,
        primary_runway_id="03",
        runways=(
            _make_runway("03", 27.0, 3805.0, 45.0, 38.7750, -9.1350, 114.0),
        ),
    ),
    Airport(
        icao="ENGM",
        iata="OSL",
        name="Oslo Gardermoen Airport",
        city="Oslo",
        country="Norway",
        lat_deg=60.1975,
        lon_deg=11.1004,
        elev_m=208.0,
        primary_runway_id="01R",
        runways=(
            _make_runway("01R", 14.0, 3600.0, 45.0, 60.1980, 11.1100, 208.0),
        ),
    ),
    Airport(
        icao="ESSA",
        iata="ARN",
        name="Stockholm Arlanda Airport",
        city="Stockholm",
        country="Sweden",
        lat_deg=59.6498,
        lon_deg=17.9238,
        elev_m=42.0,
        primary_runway_id="01L",
        runways=(
            _make_runway("01L", 9.0, 3301.0, 45.0, 59.6500, 17.9150, 42.0),
        ),
    ),
    Airport(
        icao="EFHK",
        iata="HEL",
        name="Helsinki Vantaa Airport",
        city="Helsinki",
        country="Finland",
        lat_deg=60.3172,
        lon_deg=24.9633,
        elev_m=55.0,
        primary_runway_id="22R",
        runways=(
            _make_runway("22R", 223.0, 3060.0, 60.0, 60.3180, 24.9580, 55.0),
        ),
    ),
    Airport(
        icao="EPWA",
        iata="WAW",
        name="Warsaw Chopin Airport",
        city="Warsaw",
        country="Poland",
        lat_deg=52.1672,
        lon_deg=20.9679,
        elev_m=110.0,
        primary_runway_id="33",
        runways=(
            _make_runway("33", 328.0, 3690.0, 60.0, 52.1640, 20.9700, 110.0),
        ),
    ),
    Airport(
        icao="LGAV",
        iata="ATH",
        name="Athens Eleftherios Venizelos Airport",
        city="Athens",
        country="Greece",
        lat_deg=37.9364,
        lon_deg=23.9445,
        elev_m=94.0,
        primary_runway_id="03R",
        runways=(
            _make_runway("03R", 34.0, 4000.0, 45.0, 37.9320, 23.9520, 94.0),
        ),
    ),
    Airport(
        icao="LTFM",
        iata="IST",
        name="Istanbul Airport",
        city="Istanbul",
        country="Turkey",
        lat_deg=41.2753,
        lon_deg=28.7519,
        elev_m=99.0,
        primary_runway_id="35L",
        runways=(
            _make_runway("35L", 354.0, 3750.0, 60.0, 41.2650, 28.7400, 99.0),
        ),
    ),
    # --- 11 International Hubs ---
    Airport(
        icao="KJFK",
        iata="JFK",
        name="John F. Kennedy International Airport",
        city="New York",
        country="United States",
        lat_deg=40.6413,
        lon_deg=-73.7781,
        elev_m=4.0,
        primary_runway_id="31L",
        runways=(
            _make_runway("31L", 314.0, 4423.0, 45.0, 40.6398, -73.7789, 4.0),
        ),
    ),
    Airport(
        icao="KLAX",
        iata="LAX",
        name="Los Angeles International Airport",
        city="Los Angeles",
        country="United States",
        lat_deg=33.9416,
        lon_deg=-118.4085,
        elev_m=39.0,
        primary_runway_id="25R",
        runways=(
            _make_runway("25R", 249.0, 3939.0, 45.0, 33.9490, -118.4120, 39.0),
        ),
    ),
    Airport(
        icao="KORD",
        iata="ORD",
        name="Chicago O'Hare International Airport",
        city="Chicago",
        country="United States",
        lat_deg=41.9742,
        lon_deg=-87.9073,
        elev_m=205.0,
        primary_runway_id="28C",
        runways=(
            _make_runway("28C", 273.0, 3292.0, 45.0, 41.9800, -87.9050, 205.0),
        ),
    ),
    Airport(
        icao="KATL",
        iata="ATL",
        name="Hartsfield-Jackson Atlanta International Airport",
        city="Atlanta",
        country="United States",
        lat_deg=33.6407,
        lon_deg=-84.4277,
        elev_m=313.0,
        primary_runway_id="27R",
        runways=(
            _make_runway("27R", 275.0, 3624.0, 45.0, 33.6480, -84.4250, 313.0),
        ),
    ),
    Airport(
        icao="OMDB",
        iata="DXB",
        name="Dubai International Airport",
        city="Dubai",
        country="United Arab Emirates",
        lat_deg=25.2532,
        lon_deg=55.3657,
        elev_m=19.0,
        primary_runway_id="30L",
        runways=(
            _make_runway("30L", 301.0, 4450.0, 60.0, 25.2480, 55.3700, 19.0),
        ),
    ),
    Airport(
        icao="OTHH",
        iata="DOH",
        name="Hamad International Airport",
        city="Doha",
        country="Qatar",
        lat_deg=25.2731,
        lon_deg=51.6081,
        elev_m=4.0,
        primary_runway_id="34L",
        runways=(
            _make_runway("34L", 342.0, 4850.0, 60.0, 25.2650, 51.6150, 4.0),
        ),
    ),
    Airport(
        icao="RJTT",
        iata="HND",
        name="Tokyo Haneda Airport",
        city="Tokyo",
        country="Japan",
        lat_deg=35.5494,
        lon_deg=139.7798,
        elev_m=6.4,
        primary_runway_id="34R",
        runways=(
            _make_runway("34R", 337.0, 3360.0, 60.0, 35.5450, 139.7900, 6.4),
        ),
    ),
    Airport(
        icao="RJAA",
        iata="NRT",
        name="Tokyo Narita Airport",
        city="Tokyo",
        country="Japan",
        lat_deg=35.7647,
        lon_deg=140.3863,
        elev_m=43.0,
        primary_runway_id="16R",
        runways=(
            _make_runway("16R", 158.0, 4000.0, 60.0, 35.7600, 140.3850, 43.0),
        ),
    ),
    Airport(
        icao="VHHH",
        iata="HKG",
        name="Hong Kong International Airport",
        city="Hong Kong",
        country="Hong Kong",
        lat_deg=22.3080,
        lon_deg=113.9185,
        elev_m=8.5,
        primary_runway_id="07L",
        runways=(
            _make_runway("07L", 71.0, 3800.0, 60.0, 22.3160, 113.9100, 8.5),
        ),
    ),
    Airport(
        icao="WSSS",
        iata="SIN",
        name="Singapore Changi Airport",
        city="Singapore",
        country="Singapore",
        lat_deg=1.3644,
        lon_deg=103.9915,
        elev_m=6.7,
        primary_runway_id="02L",
        runways=(
            _make_runway("02L", 23.0, 4000.0, 60.0, 1.3600, 103.9850, 6.7),
        ),
    ),
    Airport(
        icao="YSSY",
        iata="SYD",
        name="Sydney Kingsford Smith Airport",
        city="Sydney",
        country="Australia",
        lat_deg=-33.9461,
        lon_deg=151.1772,
        elev_m=6.4,
        primary_runway_id="16R",
        runways=(
            _make_runway("16R", 155.0, 3962.0, 45.0, -33.9450, 151.1750, 6.4),
        ),
    ),
)

_AIRPORTS_BY_ICAO: dict[str, Airport] = {apt.icao: apt for apt in _AIRPORT_RECORDS}
_AIRPORTS_BY_IATA: dict[str, Airport] = {apt.iata: apt for apt in _AIRPORT_RECORDS}


def get_airport(icao: str) -> Airport:
    """Retrieve an airport by ICAO designator (or 3-letter IATA code, case-insensitive).

    Args:
        icao: 4-letter ICAO code (e.g. 'EHAM', 'EGLL') or 3-letter IATA code (e.g. 'AMS', 'LHR').

    Returns:
        Airport record with runway geometries and elevations.

    Raises:
        ValueError: If code is not a non-empty string.
        KeyError: If airport is not found in the embedded catalog.
    """
    if not isinstance(icao, str) or not icao.strip():
        raise ValueError("Airport code must be a non-empty string")
    code = icao.strip().upper()
    if code in _AIRPORTS_BY_ICAO:
        return _AIRPORTS_BY_ICAO[code]
    if code in _AIRPORTS_BY_IATA:
        return _AIRPORTS_BY_IATA[code]
    raise KeyError(
        f"Airport '{icao}' not found in database. Available ICAO codes: {sorted(_AIRPORTS_BY_ICAO.keys())}"
    )


def has_airport(code: str) -> bool:
    """Check if an airport exists in the embedded catalog by ICAO or IATA code."""
    if not isinstance(code, str) or not code.strip():
        return False
    c = code.strip().upper()
    return c in _AIRPORTS_BY_ICAO or c in _AIRPORTS_BY_IATA


def list_airports() -> list[Airport]:
    """Return all 31 registered airports sorted alphabetically by ICAO code."""
    return sorted(_AIRPORTS_BY_ICAO.values(), key=lambda a: a.icao)


def search_airports(query: str) -> list[Airport]:
    """Search airports by case-insensitive substring match against ICAO, IATA, name, city, or country."""
    if not isinstance(query, str) or not query.strip():
        return list_airports()
    q = query.strip().upper()
    results = [
        apt
        for apt in _AIRPORTS_BY_ICAO.values()
        if q in apt.icao
        or q in apt.iata
        or q in apt.name.upper()
        or q in apt.city.upper()
        or q in apt.country.upper()
    ]
    return sorted(results, key=lambda a: a.icao)


def list_aircraft_types() -> list[str]:
    """Return sorted list of supported aircraft type codes from the performance database."""
    from adsb.synthesis.performance import list_aircraft_profiles

    return list_aircraft_profiles()


__all__ = [
    "Airport",
    "Runway",
    "get_airport",
    "has_airport",
    "list_aircraft_types",
    "list_airports",
    "search_airports",
]
