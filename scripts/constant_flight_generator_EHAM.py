
import random
import time
import string
import requests
from datetime import datetime

# ============================================================
# USER SETTINGS
# ============================================================

API_URL = "http://127.0.0.1:8000/api/v1/flights"

ORIGIN_ICAO = "EHAM"          # Change to any airport in AIRPORTS
MIN_INTERVAL_SECONDS = 60     # Minimum time between departures
MAX_INTERVAL_SECONDS = 180    # Maximum time between departures

MAX_DEPARTURES = None         # None = run indefinitely
STARTUP_DELAY_SECONDS = 5     # Initial delay before first departure

# ============================================================
# AIRPORT DATABASE
# ICAO: (latitude, longitude)
# ============================================================

AIRPORTS = {
    "EBBR": (50.9010, 4.4844),
    "EDDF": (50.0379, 8.5622),
    "EDDM": (48.3537, 11.7861),
    "EFHK": (60.3172, 24.9633),
    "EGLL": (51.4700, -0.4543),
    "EHAM": (52.3105, 4.7683),
    "EIDW": (53.4213, -6.2701),
    "EKCH": (55.6180, 12.6508),
    "ENGM": (60.1975, 11.1004),
    "EPWA": (52.1672, 20.9679),
    "ESSA": (59.6498, 17.9238),
    "KATL": (33.6407, -84.4277),
    "KJFK": (40.6413, -73.7781),
    "KLAX": (33.9416, -118.4085),
    "KORD": (41.9742, -87.9073),
    "LEBL": (41.2974, 2.0833),
    "LEMD": (40.4839, -3.5680),
    "LFPG": (49.0097, 2.5479),
    "LGAV": (37.9364, 23.9445),
    "LIRF": (41.8003, 12.2389),
    "LOWW": (48.1103, 16.5697),
    "LPPT": (38.7742, -9.1342),
    "LSZH": (47.4582, 8.5555),
    "LTFM": (41.2753, 28.7519),
    "OMDB": (25.2532, 55.3657),
    "OTHH": (25.2731, 51.6081),
    "RJAA": (35.7647, 140.3863),
    "RJTT": (35.5494, 139.7798),
    "VHHH": (22.3080, 113.9185),
    "WSSS": (1.3644, 103.9915),
    "YSSY": (-33.9461, 151.1772),
}

# ============================================================
# AIRCRAFT TYPES AND TRAFFIC MIX
# ============================================================

AIRCRAFT_WEIGHTS = {
    "A320": 24,
    "B738": 22,
    "B77W": 8,
    "E75L": 9,
    "DH8D": 7,
    "B350": 5,
    "C172": 7,
    "SR22": 7,
    "EC35": 3,
    "R44": 3,
    "F16": 2,
    "GLID": 2,
    "CONC": 1,
}

AIRCRAFT_RANGE = {
    "A320": (100, 3500),
    "B738": (100, 3100),
    "B77W": (300, 8500),
    "CONC": (700, 3900),
    "E75L": (80, 2200),
    "DH8D": (50, 1100),
    "B350": (50, 1500),
    "C172": (40, 600),
    "SR22": (40, 1000),
    "EC35": (20, 300),
    "F16": (50, 1200),
    "GLID": (80, 300),
    "R44": (20, 250),
}

# ============================================================
# CALLSIGN PREFIXES
# ============================================================

AIRLINE_PREFIXES = [
    "RYR", "EZY", "BAW", "DLH", "AFR", "KLM",
    "SAS", "AUA", "IBE", "WZZ", "LOT", "VLG", "FIN"
]

MILITARY_PREFIXES = ["RCH", "NATO", "GAF", "FAF"]

# ============================================================
# DISTANCE CALCULATION
# ============================================================

from math import radians, sin, cos, sqrt, atan2

def haversine_nm(lat1, lon1, lat2, lon2):
    earth_radius_nm = 3440.065

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)

    a = (
        sin(dlat / 2) ** 2
        + cos(radians(lat1))
        * cos(radians(lat2))
        * sin(dlon / 2) ** 2
    )

    return earth_radius_nm * 2 * atan2(
        sqrt(a), sqrt(1 - a)
    )

# ============================================================
# BUILD FEASIBLE DESTINATIONS FOR SELECTED AIRPORT
# ============================================================

def build_routes(origin):
    routes = {}

    origin_lat, origin_lon = AIRPORTS[origin]

    for destination, (lat, lon) in AIRPORTS.items():
        if destination == origin:
            continue

        distance = haversine_nm(
            origin_lat, origin_lon, lat, lon
        )

        for aircraft, (minimum, maximum) in AIRCRAFT_RANGE.items():
            if minimum <= distance <= maximum:
                routes.setdefault(aircraft, []).append(
                    (destination, round(distance, 1))
                )

    return routes

# ============================================================
# AIRCRAFT SELECTION
# ============================================================

def choose_aircraft(routes):
    available = [
        aircraft for aircraft in AIRCRAFT_WEIGHTS
        if routes.get(aircraft)
    ]

    weights = [
        AIRCRAFT_WEIGHTS[aircraft]
        for aircraft in available
    ]

    return random.choices(
        available, weights=weights, k=1
    )[0]

# ============================================================
# CRUISE ALTITUDE PROFILES
# ============================================================

def cruise_altitude(aircraft, distance):
    profiles = {
        "A320": [(150, 8000, 16000), (500, 18000, 28000),
                 (1200, 28000, 34000), (99999, 34000, 39000)],
        "B738": [(150, 8000, 16000), (500, 18000, 28000),
                 (1200, 28000, 34000), (99999, 34000, 39000)],
        "B77W": [(300, 16000, 24000), (800, 28000, 34000),
                 (2500, 34000, 39000), (99999, 36000, 43000)],
        "CONC": [(1000, 35000, 45000), (2500, 45000, 55000),
                 (99999, 55000, 60000)],
        "E75L": [(150, 10000, 18000), (500, 20000, 30000),
                 (99999, 30000, 37000)],
        "DH8D": [(150, 6000, 12000), (500, 12000, 20000),
                 (99999, 18000, 25000)],
        "B350": [(150, 8000, 14000), (500, 14000, 24000),
                 (99999, 20000, 30000)],
        "C172": [(150, 3000, 6000), (350, 6000, 10000),
                 (99999, 8000, 12000)],
        "SR22": [(150, 5000, 9000), (500, 9000, 13000),
                 (99999, 12000, 17000)],
        "EC35": [(100, 1000, 2500), (99999, 2000, 5000)],
        "R44": [(100, 1000, 2000), (99999, 2000, 4000)],
        "F16": [(150, 10000, 18000), (500, 20000, 30000),
                (99999, 30000, 45000)],
        "GLID": [(150, 2000, 5000), (99999, 4000, 8000)],
    }

    for limit, minimum, maximum in profiles[aircraft]:
        if distance < limit:
            return random.randrange(
                minimum, maximum + 1, 1000
            )

    return 27000

# ============================================================
# IDENTIFIERS
# ============================================================

used_icao_hex = set()

def generate_icao_hex():
    while True:
        address = "".join(
            random.choices("0123456789ABCDEF", k=6)
        )

        if address not in used_icao_hex:
            used_icao_hex.add(address)
            return address

def generate_squawk():
    while True:
        squawk = "".join(
            random.choices("01234567", k=4)
        )

        if squawk not in {
            "0000", "7500", "7600", "7700"
        }:
            return squawk

def generate_callsign(aircraft):
    if aircraft in ("A320", "B738", "B77W", "E75L", "DH8D"):
        prefix = random.choice(AIRLINE_PREFIXES)
        number = random.randint(100, 9999)
        return f"{prefix}{number}"

    if aircraft == "F16":
        prefix = random.choice(MILITARY_PREFIXES)
        return f"{prefix}{random.randint(100, 999)}"

    return f"{aircraft}{random.randint(100, 999)}"

# ============================================================
# GENERATE AND SUBMIT FLIGHT
# ============================================================

def generate_flight(origin, routes, session):
    aircraft = choose_aircraft(routes)

    destination, distance = random.choice(routes[aircraft])

    flight = {
        "origin_icao": origin,
        "destination_icao": destination,
        "aircraft_type": aircraft,
        "callsign": generate_callsign(aircraft),
        "cruise_alt_ft": cruise_altitude(aircraft, distance),
        "squawk": generate_squawk(),
        "icao_hex": generate_icao_hex(),
    }

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    try:
        response = session.post(
            API_URL,
            json=flight,
            timeout=15
        )

        if response.status_code == 201:
            print(
                f"[{timestamp}] CREATED | "
                f"{flight['callsign']:8} | "
                f"{aircraft:5} | "
                f"{origin} -> {destination} | "
                f"{distance:7.1f} NM | "
                f"FL{flight['cruise_alt_ft'] // 100:03}"
            )
            return True

        print(
            f"[{timestamp}] FAILED | "
            f"HTTP {response.status_code} | "
            f"{response.text[:250]}"
        )

    except requests.RequestException as error:
        print(f"[{timestamp}] API ERROR | {error}")

    return False

# ============================================================
# MAIN CONTINUOUS SCHEDULER
# ============================================================

def main():
    if ORIGIN_ICAO not in AIRPORTS:
        raise ValueError(
            f"Unknown airport: {ORIGIN_ICAO}. "
            f"Available airports: {', '.join(AIRPORTS)}"
        )

    if MIN_INTERVAL_SECONDS <= 0:
        raise ValueError("Minimum interval must be positive.")

    if MAX_INTERVAL_SECONDS < MIN_INTERVAL_SECONDS:
        raise ValueError(
            "Maximum interval must be >= minimum interval."
        )

    routes = build_routes(ORIGIN_ICAO)

    if not routes:
        raise RuntimeError(
            f"No feasible destinations for {ORIGIN_ICAO}"
        )

    print("=" * 65)
    print("SINGLE AIRPORT FLIGHT GENERATOR")
    print("=" * 65)
    print(f"Origin airport : {ORIGIN_ICAO}")
    print(f"Aircraft types : {len(routes)}")
    print(f"Destinations   : {len(AIRPORTS) - 1}")
    print(
        f"Departure gap : {MIN_INTERVAL_SECONDS}–"
        f"{MAX_INTERVAL_SECONDS} seconds"
    )
    print(
        f"Max departures: "
        f"{MAX_DEPARTURES if MAX_DEPARTURES else 'Unlimited'}"
    )
    print("Press CTRL+C to stop.")
    print("=" * 65)

    session = requests.Session()

    departures = 0
    successful = 0

    try:
        print(
            f"First departure in {STARTUP_DELAY_SECONDS} seconds..."
        )
        time.sleep(STARTUP_DELAY_SECONDS)

        while (
            MAX_DEPARTURES is None
            or departures < MAX_DEPARTURES
        ):
            success = generate_flight(
                ORIGIN_ICAO, routes, session
            )

            departures += 1
            if success:
                successful += 1

            if (
                MAX_DEPARTURES is not None
                and departures >= MAX_DEPARTURES
            ):
                break

            interval = random.randint(
                MIN_INTERVAL_SECONDS,
                MAX_INTERVAL_SECONDS
            )

            next_time = datetime.now().strftime("%H:%M:%S")

            print(
                f"Next departure in {interval} seconds "
                f"(scheduled from {next_time}).\n"
            )

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nShutdown requested.")

    finally:
        session.close()

        print("\n" + "=" * 65)
        print("GENERATOR STOPPED")
        print(f"Origin airport : {ORIGIN_ICAO}")
        print(f"Attempts       : {departures}")
        print(f"Successful     : {successful}")
        print(f"Failed         : {departures - successful}")
        print("=" * 65)


if __name__ == "__main__":
    main()
