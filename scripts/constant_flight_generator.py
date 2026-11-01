
import random
import time
import math
import heapq
import requests

from datetime import datetime


# ============================================================
# CONFIGURATION
# ============================================================

API_URL = "http://127.0.0.1:8000/api/v1/flights"

STARTUP_STAGGER_SECONDS = 480   # Spread initial departures over 8 minutes
REQUEST_TIMEOUT = 15

# Set a number to stop after that many departures.
# Set to None for continuous operation.
MAX_DEPARTURES = None


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
# AIRPORT DEPARTURE FREQUENCIES
# Values are minimum and maximum minutes between departures.
# These are approximate simulation tiers, not live schedules.
# ============================================================

AIRPORT_INTERVALS = {

    # Major international hubs: 1-3 minutes
    "EGLL": (1, 3),
    "KATL": (1, 3),
    "KJFK": (1, 3),
    "KLAX": (1, 3),
    "KORD": (1, 3),
    "EDDF": (1, 3),
    "EHAM": (1, 3),
    "LFPG": (1, 3),
    "OMDB": (1, 3),
    "WSSS": (1, 3),
    "VHHH": (1, 3),
    "RJTT": (1, 3),
    "RJAA": (1, 3),
    "YSSY": (1, 3),

    # Busy international airports: 2-4 minutes
    "EIDW": (2, 4),
    "EBBR": (2, 4),
    "EDDM": (2, 4),
    "LEMD": (2, 4),
    "LEBL": (2, 4),
    "LPPT": (2, 4),
    "LTFM": (2, 4),
    "LIRF": (2, 4),
    "OTHH": (2, 4),
    "LGAV": (2, 4),

    # Medium traffic airports: 3-6 minutes
    "EFHK": (3, 6),
    "EKCH": (3, 6),
    "ENGM": (3, 6),
    "EPWA": (3, 6),
    "ESSA": (3, 6),
    "LOWW": (3, 6),
    "LSZH": (3, 6),
}


# ============================================================
# AIRCRAFT TYPES AND TRAFFIC MIX
# ============================================================

AIRCRAFT_TYPES = [
    "A320", "B350", "B738", "B77W", "C172",
    "CONC", "DH8D", "E75L", "EC35", "F16",
    "GLID", "R44", "SR22"
]

# Relative selection weights.
# These control the overall synthetic traffic mix.
# The final selection is restricted to aircraft with a
# suitable route available from the departure airport.

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


# Approximate route range in NM.
# Minimum range prevents obviously unsuitable short routes.

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
# CALLSIGNS
# ============================================================

AIRLINE_PREFIXES = [
    "RYR", "EZY", "BAW", "DLH", "AFR",
    "KLM", "SAS", "AUA", "IBE", "TAP",
    "EIN", "WZZ", "LOT", "VLG", "FIN",
    "UAE", "QTR", "ANA", "JAL", "SIA",
    "CPA", "QFA", "DAL", "UAL", "AAL",
]

AIRLINE_TYPES = {
    "A320", "B738", "B77W", "E75L", "DH8D"
}

MILITARY_PREFIXES = ["RCH", "NATO", "BAF", "IAF"]


def generate_callsign(aircraft_type):

    if aircraft_type in AIRLINE_TYPES:
        prefix = random.choice(AIRLINE_PREFIXES)

    elif aircraft_type == "F16":
        prefix = random.choice(MILITARY_PREFIXES)

    else:
        prefix = aircraft_type

    return f"{prefix}{random.randint(100, 9999)}"


# ============================================================
# IDENTIFIERS
# ============================================================

USED_ICAO_ADDRESSES = set()


def generate_icao_hex():

    while True:
        address = "".join(
            random.choices("0123456789ABCDEF", k=6)
        )

        if address not in USED_ICAO_ADDRESSES:
            USED_ICAO_ADDRESSES.add(address)
            return address


def generate_squawk():

    reserved = {"7500", "7600", "7700", "0000"}

    while True:
        squawk = format(random.randint(1, 4095), "04o")

        if squawk not in reserved:
            return squawk


# ============================================================
# DISTANCE CALCULATION
# ============================================================

def calculate_distance_nm(origin, destination):

    lat1, lon1 = AIRPORTS[origin]
    lat2, lon2 = AIRPORTS[destination]

    earth_radius_nm = 3440.065

    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1)
        * math.cos(lat2)
        * math.sin(dlon / 2) ** 2
    )

    return earth_radius_nm * (
        2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    )


# ============================================================
# PRE-COMPUTE AVAILABLE ROUTES
# ============================================================

def build_route_database():

    routes = {}

    for origin in AIRPORTS:

        routes[origin] = {
            aircraft: []
            for aircraft in AIRCRAFT_TYPES
        }

        for destination in AIRPORTS:

            if origin == destination:
                continue

            distance = calculate_distance_nm(
                origin, destination
            )

            for aircraft, (minimum, maximum) in AIRCRAFT_RANGE.items():

                if minimum <= distance <= maximum:

                    routes[origin][aircraft].append(
                        (destination, distance)
                    )

    return routes


ROUTE_DATABASE = build_route_database()


# ============================================================
# SELECT AIRCRAFT AND DESTINATION
# ============================================================

def select_departure(origin):

    available_aircraft = []

    for aircraft, destinations in ROUTE_DATABASE[origin].items():

        if destinations:
            available_aircraft.append(aircraft)

    if not available_aircraft:
        return None

    weights = [
        AIRCRAFT_WEIGHTS[aircraft]
        for aircraft in available_aircraft
    ]

    aircraft = random.choices(
        available_aircraft,
        weights=weights,
        k=1
    )[0]

    destination, distance = random.choice(
        ROUTE_DATABASE[origin][aircraft]
    )

    return aircraft, destination, distance


# ============================================================
# CRUISE ALTITUDE
# ============================================================

def altitude_band(aircraft, distance):

    # Commercial narrow-body jets
    if aircraft in {"A320", "B738"}:

        if distance < 150:
            return 8000, 16000
        elif distance < 500:
            return 18000, 28000
        elif distance < 1200:
            return 28000, 34000
        else:
            return 34000, 39000

    # Wide-body jet
    if aircraft == "B77W":

        if distance < 300:
            return 16000, 24000
        elif distance < 800:
            return 28000, 34000
        elif distance < 2500:
            return 34000, 39000
        else:
            return 36000, 43000

    # Concorde
    if aircraft == "CONC":

        if distance < 1000:
            return 35000, 45000
        elif distance < 2500:
            return 45000, 55000
        else:
            return 55000, 60000

    # Regional jet
    if aircraft == "E75L":

        if distance < 150:
            return 10000, 18000
        elif distance < 500:
            return 20000, 30000
        else:
            return 30000, 37000

    # Regional turboprop
    if aircraft == "DH8D":

        if distance < 150:
            return 6000, 12000
        elif distance < 500:
            return 12000, 20000
        else:
            return 18000, 25000

    # Business turboprop
    if aircraft == "B350":

        if distance < 150:
            return 8000, 14000
        elif distance < 500:
            return 14000, 24000
        else:
            return 20000, 30000

    # Cessna 172
    if aircraft == "C172":

        if distance < 150:
            return 3000, 6000
        elif distance < 350:
            return 6000, 10000
        else:
            return 8000, 12000

    # Cirrus SR22
    if aircraft == "SR22":

        if distance < 150:
            return 5000, 9000
        elif distance < 500:
            return 9000, 13000
        else:
            return 12000, 17000

    # Helicopters
    if aircraft == "EC35":

        if distance < 100:
            return 1000, 2500
        else:
            return 2000, 5000

    if aircraft == "R44":

        if distance < 100:
            return 1000, 2000
        else:
            return 2000, 4000

    # F16
    if aircraft == "F16":

        if distance < 150:
            return 10000, 18000
        elif distance < 500:
            return 20000, 30000
        else:
            return 30000, 45000

    # Glider
    if aircraft == "GLID":

        if distance < 150:
            return 2000, 5000
        else:
            return 4000, 8000

    return 10000, 20000


def generate_cruise_altitude(aircraft, distance):

    minimum, maximum = altitude_band(aircraft, distance)

    return random.randrange(
        minimum,
        maximum + 1,
        1000
    )


# ============================================================
# GENERATE FLIGHT
# ============================================================

def generate_flight(origin):

    selection = select_departure(origin)

    if selection is None:
        return None

    aircraft, destination, distance = selection

    altitude = generate_cruise_altitude(
        aircraft, distance
    )

    flight = {
        "origin_icao": origin,
        "destination_icao": destination,
        "aircraft_type": aircraft,
        "callsign": generate_callsign(aircraft),
        "cruise_alt_ft": altitude,
        "squawk": generate_squawk(),
        "icao_hex": generate_icao_hex(),
    }

    return flight, distance


# ============================================================
# SUBMIT FLIGHT
# ============================================================

def submit_flight(session, origin):

    flight_data = generate_flight(origin)

    if flight_data is None:
        print(f"No suitable routes available from {origin}")
        return False

    flight, estimated_distance = flight_data

    try:

        response = session.post(
            API_URL,
            json=flight,
            timeout=REQUEST_TIMEOUT
        )

        if response.status_code == 201:

            result = response.json()

            now = datetime.now().astimezone().strftime("%H:%M:%S")

            print(
                f"{now} | CREATED | "
                f"{flight['callsign']:8} | "
                f"{flight['aircraft_type']:4} | "
                f"{origin} -> {flight['destination_icao']} | "
                f"{estimated_distance:7.1f} NM | "
                f"FL{flight['cruise_alt_ft'] // 100:03}"
            )

            return True

        print(
            f"FAILED | {origin} | "
            f"HTTP {response.status_code} | "
            f"{response.text[:200]}"
        )

        return False

    except requests.RequestException as error:

        print(f"CONNECTION ERROR | {origin} | {error}")

        return False


# ============================================================
# DEPARTURE SCHEDULER
# ============================================================

def schedule_departures():

    session = requests.Session()

    departure_count = 0
    successful_count = 0
    failed_count = 0

    # Priority queue:
    # (next departure time, airport ICAO)

    schedule = []

    start_time = time.monotonic()

    print("\n==============================================")
    print("CONTINUOUS FLIGHT GENERATOR")
    print("==============================================")
    print(f"Airports: {len(AIRPORTS)}")
    print(f"Endpoint: {API_URL}")
    print("Mode: Continuous")
    print(f"Initial stagger: {STARTUP_STAGGER_SECONDS // 60} minutes")
    print("Press CTRL+C to stop.")
    print("==============================================\n")

    # Give each airport its own random initial departure time.
    for airport in AIRPORTS:

        initial_offset = random.uniform(
            0,
            STARTUP_STAGGER_SECONDS
        )

        next_departure = start_time + initial_offset

        heapq.heappush(
            schedule,
            (next_departure, airport)
        )

    try:

        while True:

            next_time, airport = heapq.heappop(schedule)

            # Wait until this airport's departure is due.
            wait_time = next_time - time.monotonic()

            if wait_time > 0:
                time.sleep(wait_time)

            departure_count += 1

            success = submit_flight(session, airport)

            if success:
                successful_count += 1
            else:
                failed_count += 1

            # Stop after the configured number of departures.
            if (
                MAX_DEPARTURES is not None
                and departure_count >= MAX_DEPARTURES
            ):
                break

            # Schedule the next departure from this airport.
            minimum_minutes, maximum_minutes = (
                AIRPORT_INTERVALS[airport]
            )

            interval_seconds = random.uniform(
                minimum_minutes * 60,
                maximum_minutes * 60
            )

            next_departure = (
                time.monotonic() + interval_seconds
            )

            heapq.heappush(
                schedule,
                (next_departure, airport)
            )

    except KeyboardInterrupt:

        print("\n\nGenerator stopped by user.")

    finally:

        session.close()

        print("\n==============================================")
        print("GENERATOR SUMMARY")
        print("==============================================")
        print(f"Departure attempts: {departure_count}")
        print(f"Successful:         {successful_count}")
        print(f"Failed:             {failed_count}")
        print("==============================================")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    try:
        schedule_departures()

    except Exception as error:
        print(f"\nFATAL ERROR: {error}")
