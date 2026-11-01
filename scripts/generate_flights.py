
import random
import time
import math
import requests


# ============================================================
# CONFIGURATION
# ============================================================

API_URL = "http://127.0.0.1:8000/api/v1/flights"

NUMBER_OF_FLIGHTS = 50       # Choose between 10 and 200
REQUEST_DELAY = 0.05         # Seconds between API requests


# ============================================================
# AVAILABLE AIRPORTS
# ICAO: (latitude, longitude)
# ============================================================

AIRPORTS = {
    "EBBR": (50.9010, 4.4844),     # Brussels
    "EDDF": (50.0379, 8.5622),     # Frankfurt
    "EDDM": (48.3537, 11.7861),    # Munich
    "EFHK": (60.3172, 24.9633),    # Helsinki
    "EGLL": (51.4700, -0.4543),    # London Heathrow
    "EHAM": (52.3105, 4.7683),     # Amsterdam
    "EIDW": (53.4213, -6.2701),    # Dublin
    "EKCH": (55.6180, 12.6508),    # Copenhagen
    "ENGM": (60.1975, 11.1004),    # Oslo
    "EPWA": (52.1672, 20.9679),    # Warsaw
    "ESSA": (59.6498, 17.9238),    # Stockholm

    "KATL": (33.6407, -84.4277),   # Atlanta
    "KJFK": (40.6413, -73.7781),   # New York JFK
    "KLAX": (33.9416, -118.4085),  # Los Angeles
    "KORD": (41.9742, -87.9073),   # Chicago O'Hare

    "LEBL": (41.2974, 2.0833),     # Barcelona
    "LEMD": (40.4839, -3.5680),    # Madrid
    "LFPG": (49.0097, 2.5479),     # Paris CDG
    "LGAV": (37.9364, 23.9445),    # Athens
    "LIRF": (41.8003, 12.2389),    # Rome
    "LOWW": (48.1103, 16.5697),    # Vienna
    "LPPT": (38.7742, -9.1342),   # Lisbon
    "LSZH": (47.4582, 8.5555),     # Zurich
    "LTFM": (41.2753, 28.7519),    # Istanbul

    "OMDB": (25.2532, 55.3657),    # Dubai
    "OTHH": (25.2731, 51.6081),    # Doha

    "RJAA": (35.7647, 140.3863),   # Tokyo Narita
    "RJTT": (35.5494, 139.7798),   # Tokyo Haneda

    "VHHH": (22.3080, 113.9185),   # Hong Kong
    "WSSS": (1.3644, 103.9915),    # Singapore
    "YSSY": (-33.9461, 151.1772),  # Sydney
}


# ============================================================
# AVAILABLE AIRCRAFT
# ============================================================

AIRCRAFT_TYPES = [
    "A320",
    "B350",
    "B738",
    "B77W",
    "C172",
    "CONC",
    "DH8D",
    "E75L",
    "EC35",
    "F16",
    "GLID",
    "R44",
    "SR22",
]


# Approximate maximum practical route distances in NM.
# These are simulation constraints, not certified aircraft ranges.

AIRCRAFT_MAX_RANGE_NM = {
    "A320": 6000,
    "B350": 1500,
    "B738": 6000,
    "B77W": 7500,
    "C172": 600,
    "CONC": 4000,
    "DH8D": 1200,
    "E75L": 2200,
    "EC35": 300,
    "F16": 1200,
    "GLID": 120,
    "R44": 250,
    "SR22": 1000,
}

# Concorde needs a reasonably long route to make sense.
AIRCRAFT_MIN_RANGE_NM = {
    "CONC": 700,
}


# ============================================================
# CALLSIGN GENERATION
# ============================================================

AIRLINE_PREFIXES = [
    "RYR", "EZY", "BAW", "DLH", "AFR",
    "KLM", "SAS", "AUA", "IBE", "TAP",
    "EIN", "WZZ", "LOT", "VLG", "FIN",
    "UAE", "QTR", "ANA", "JAL", "SIA",
    "CPA", "QFA", "DAL", "UAL", "AAL",
]

AIRLINE_AIRCRAFT = {
    "A320", "B738", "B77W", "DH8D", "E75L", "CONC"
}


def generate_callsign(aircraft_type):
    if aircraft_type in AIRLINE_AIRCRAFT:
        prefix = random.choice(AIRLINE_PREFIXES)
    else:
        prefix = aircraft_type

    return f"{prefix}{random.randint(100, 999)}"


# ============================================================
# RANDOM IDENTIFIERS
# ============================================================

def generate_squawk():
    """Generate a four-digit octal squawk, excluding emergency codes."""

    emergency_codes = {"7500", "7600", "7700", "0000"}

    while True:
        squawk = format(random.randint(0, 4095), "04o")

        if squawk not in emergency_codes:
            return squawk


def generate_icao_hex():
    return "".join(
        random.choices("0123456789ABCDEF", k=6)
    )


# ============================================================
# DISTANCE CALCULATION
# ============================================================

def calculate_distance_nm(origin, destination):
    """Calculate great-circle distance using the Haversine formula."""

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

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return earth_radius_nm * c


# ============================================================
# ROUTE SELECTION
# ============================================================

def select_route(aircraft_type):
    """
    Select a random airport pair within the approximate
    operating range of the selected aircraft.
    """

    max_range = AIRCRAFT_MAX_RANGE_NM[aircraft_type]
    min_range = AIRCRAFT_MIN_RANGE_NM.get(aircraft_type, 0)

    possible_routes = []

    airport_codes = list(AIRPORTS.keys())

    for origin in airport_codes:
        for destination in airport_codes:

            if origin == destination:
                continue

            distance = calculate_distance_nm(origin, destination)

            if min_range <= distance <= max_range:
                possible_routes.append(
                    (origin, destination, distance)
                )

    if not possible_routes:
        raise ValueError(
            f"No suitable airport pairs for {aircraft_type}"
        )

    return random.choice(possible_routes)


# ============================================================
# CRUISE ALTITUDE SELECTION
# ============================================================

# Altitude bands in feet, selected according to aircraft type
# and estimated flight distance.
#
# Each entry: short / medium / long route.
#
# Short:  under 150 NM
# Medium: 150–600 NM
# Long:   over 600 NM

ALTITUDE_PROFILES = {

    # Commercial narrow-body jets
    "A320": {
        "short": (18000, 26000),
        "medium": (28000, 34000),
        "long": (34000, 39000),
    },

    "B738": {
        "short": (18000, 26000),
        "medium": (28000, 34000),
        "long": (34000, 39000),
    },

    # Wide-body jet
    "B77W": {
        "short": (22000, 28000),
        "medium": (30000, 36000),
        "long": (35000, 41000),
    },

    # Concorde
    "CONC": {
        "short": (35000, 45000),
        "medium": (45000, 55000),
        "long": (55000, 60000),
    },

    # Regional jet
    "E75L": {
        "short": (16000, 24000),
        "medium": (28000, 34000),
        "long": (33000, 37000),
    },

    # Regional turboprop
    "DH8D": {
        "short": (8000, 14000),
        "medium": (15000, 21000),
        "long": (19000, 25000),
    },

    # Business turboprop
    "B350": {
        "short": (8000, 14000),
        "medium": (16000, 24000),
        "long": (24000, 30000),
    },

    # Light aircraft
    "C172": {
        "short": (3000, 6000),
        "medium": (6000, 10000),
        "long": (8000, 12000),
    },

    "SR22": {
        "short": (5000, 9000),
        "medium": (9000, 13000),
        "long": (12000, 16000),
    },

    # Helicopters
    "EC35": {
        "short": (1000, 2500),
        "medium": (2000, 4000),
        "long": (3000, 5000),
    },

    "R44": {
        "short": (1000, 2500),
        "medium": (2000, 4000),
        "long": (3000, 5000),
    },

    # Military fighter
    "F16": {
        "short": (10000, 18000),
        "medium": (20000, 30000),
        "long": (30000, 40000),
    },

    # Glider
    "GLID": {
        "short": (2000, 5000),
        "medium": (4000, 8000),
        "long": (6000, 10000),
    },
}


def generate_cruise_altitude(aircraft_type, distance_nm):
    """Select a plausible altitude band for aircraft and route length."""

    if distance_nm < 150:
        flight_length = "short"
    elif distance_nm <= 600:
        flight_length = "medium"
    else:
        flight_length = "long"

    min_alt, max_alt = ALTITUDE_PROFILES[aircraft_type][flight_length]

    # Select in 1,000 ft increments.
    return random.randrange(
        min_alt,
        max_alt + 1,
        1000
    )


# ============================================================
# FLIGHT GENERATION
# ============================================================

def generate_flight():
    aircraft_type = random.choice(AIRCRAFT_TYPES)

    origin, destination, distance_nm = select_route(aircraft_type)

    cruise_altitude = generate_cruise_altitude(
        aircraft_type,
        distance_nm
    )

    flight = {
        "origin_icao": origin,
        "destination_icao": destination,
        "aircraft_type": aircraft_type,
        "callsign": generate_callsign(aircraft_type),
        "cruise_alt_ft": cruise_altitude,
        "squawk": generate_squawk(),
        "icao_hex": generate_icao_hex(),
    }

    return flight, distance_nm


# ============================================================
# API SUBMISSION
# ============================================================

def submit_flights(number_of_flights):

    if not 10 <= number_of_flights <= 200:
        raise ValueError("Number of flights must be between 10 and 200.")

    session = requests.Session()

    successful = 0
    failed = 0

    print("\nStarting flight generation...")
    print(f"Target flights: {number_of_flights}")
    print(f"API endpoint: {API_URL}\n")

    for index in range(number_of_flights):

        flight, estimated_distance = generate_flight()

        try:
            response = session.post(
                API_URL,
                json=flight,
                timeout=15
            )

            if response.status_code == 201:

                successful += 1

                result = response.json()

                print(
                    f"[{index + 1:03}/{number_of_flights}] "
                    f"CREATED | "
                    f"{flight['callsign']:8} | "
                    f"{flight['aircraft_type']:4} | "
                    f"{flight['origin_icao']} -> "
                    f"{flight['destination_icao']} | "
                    f"{estimated_distance:7.1f} NM | "
                    f"FL{flight['cruise_alt_ft'] // 100:03}"
                )

            else:

                failed += 1

                print(
                    f"[{index + 1:03}/{number_of_flights}] "
                    f"FAILED | HTTP {response.status_code} | "
                    f"{response.text[:250]}"
                )

        except requests.exceptions.RequestException as error:

            failed += 1

            print(
                f"[{index + 1:03}/{number_of_flights}] "
                f"CONNECTION ERROR | {error}"
            )

        time.sleep(REQUEST_DELAY)

    session.close()

    print("\n======================================")
    print("FLIGHT GENERATION COMPLETE")
    print("======================================")
    print(f"Requested:  {number_of_flights}")
    print(f"Successful: {successful}")
    print(f"Failed:     {failed}")
    print("======================================")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    try:
        submit_flights(NUMBER_OF_FLIGHTS)

    except KeyboardInterrupt:
        print("\nGeneration interrupted by user.")

    except Exception as error:
        print(f"\nERROR: {error}")
