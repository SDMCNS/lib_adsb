"""REST API routes for airport metadata, aircraft profiles, and dynamic flight creation."""

from __future__ import annotations

import re
from typing import Any

from fastapi import APIRouter, HTTPException, Query, status
from pydantic import BaseModel, Field, model_validator

from adsb.api.manager import FlightCreateRequest, get_simulation_manager
from adsb.core.geodesy import haversine_distance
from adsb.synthesis.airports import (
    Airport,
    get_airport,
    has_airport,
    list_airports,
    search_airports,
)
from adsb.synthesis.performance import (
    get_aircraft_profile,
    has_aircraft_profile,
    list_aircraft_profiles,
)

router = APIRouter(prefix="/api/v1", tags=["flights"])


class FlightCreatePayload(BaseModel):
    """Incoming request payload for creating custom flight."""

    origin_icao: str = Field(..., description="Origin 4-letter ICAO airport code")
    destination_icao: str = Field(..., description="Destination 4-letter ICAO airport code")
    aircraft_type: str = Field(..., description="Aircraft type code (e.g. B738)")
    callsign: str | None = Field(default=None, description="Optional flight callsign")
    cruise_alt_ft: float | None = Field(default=None, gt=0.0, description="Optional cruise altitude in feet")
    squawk: str | None = Field(default="1200", description="Optional 4-digit octal squawk code")
    icao_hex: str | None = Field(default=None, description="Optional 24-bit ICAO address hex string")

    @model_validator(mode="before")
    @classmethod
    def _normalize_fields(cls, data: Any) -> Any:
        if isinstance(data, dict):
            if "origin" in data and "origin_icao" not in data:
                data["origin_icao"] = data["origin"]
            if "destination" in data and "destination_icao" not in data:
                data["destination_icao"] = data["destination"]
        return data


class FlightCreateResponse(BaseModel):
    """Response returned upon successful flight creation."""

    status: str = "CREATED"
    icao_hex: str
    icao24_hex: str
    callsign: str
    aircraft_type: str
    origin: str
    origin_icao: str
    destination: str
    destination_icao: str
    distance_nm: float
    cruise_alt_ft: float
    dep_rwy: str
    arr_rwy: str
    waypoints_count: int = 7


def _airport_to_dict(apt: Airport) -> dict[str, Any]:
    """Serialize Airport dataclass into JSON response dictionary."""
    return {
        "icao": apt.icao,
        "iata": apt.iata,
        "name": apt.name,
        "city": apt.city,
        "country": apt.country,
        "lat": apt.lat_deg,
        "lon": apt.lon_deg,
        "lat_deg": apt.lat_deg,
        "lon_deg": apt.lon_deg,
        "elevation_ft": apt.elevation_ft,
        "elevation_m": apt.elevation_m,
        "primary_runway": {
            "designation": apt.primary_runway.designation,
            "heading_deg": apt.primary_runway.true_heading_deg,
            "true_heading_deg": apt.primary_runway.true_heading_deg,
            "length_m": apt.primary_runway.length_m,
            "length_ft": apt.primary_runway.length_ft,
            "midpoint_lat": apt.primary_runway.midpoint_lat_deg,
            "midpoint_lon": apt.primary_runway.midpoint_lon_deg,
            "midpoint_lat_deg": apt.primary_runway.midpoint_lat_deg,
            "midpoint_lon_deg": apt.primary_runway.midpoint_lon_deg,
        },
    }


@router.get("/airports", response_model=list[dict[str, Any]])
def get_airports(
    query: str | None = Query(default=None, description="Optional search filter")
) -> list[dict[str, Any]]:
    """Retrieve catalog of embedded global airports."""
    airports = search_airports(query.strip()) if query and query.strip() else list_airports()
    return [_airport_to_dict(apt) for apt in airports]


@router.get("/aircraft-types", response_model=list[str])
def get_aircraft_types() -> list[str]:
    """Retrieve list of available aircraft performance profile keys."""
    return list_aircraft_profiles()


@router.post("/flights", response_model=FlightCreateResponse, status_code=status.HTTP_201_CREATED)
def create_flight(payload: FlightCreatePayload) -> FlightCreateResponse:
    """Create, route, and launch a custom flight in the active simulation."""
    origin_icao = payload.origin_icao.strip().upper()
    destination_icao = payload.destination_icao.strip().upper()
    aircraft_type = payload.aircraft_type.strip().upper()

    # 1. Validation: Airports
    if not has_airport(origin_icao):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Unknown origin airport ICAO code: '{origin_icao}'",
        )
    if not has_airport(destination_icao):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Unknown destination airport ICAO code: '{destination_icao}'",
        )
    if origin_icao == destination_icao:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Origin and destination airports must be different",
        )

    # 2. Validation: Aircraft Type
    if not has_aircraft_profile(aircraft_type):
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Unknown aircraft type: '{aircraft_type}'. Available: {list_aircraft_profiles()}",
        )

    # 3. Validation: Squawk code
    if payload.squawk and payload.squawk.strip():
        sq = payload.squawk.strip()
        if not re.match(r"^[0-7]{4}$", sq):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid squawk code '{sq}'. Must be 4-digit octal (0-7)",
            )

    # 4. Validation: ICAO hex
    if payload.icao_hex and payload.icao_hex.strip():
        hex_val = payload.icao_hex.strip().upper().replace("0X", "")
        if not re.match(r"^[0-9A-F]{6}$", hex_val):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Invalid 24-bit ICAO hex string '{payload.icao_hex}'",
            )

    origin_airport = get_airport(origin_icao)
    dest_airport = get_airport(destination_icao)
    profile = get_aircraft_profile(aircraft_type)

    dist_m = haversine_distance(
        origin_airport.lat_deg,
        origin_airport.lon_deg,
        dest_airport.lat_deg,
        dest_airport.lon_deg,
    )
    dist_nm = dist_m / 1852.0

    callsign = (
        payload.callsign.strip().upper()
        if payload.callsign and payload.callsign.strip()
        else f"SIM{dist_m:.0f}"[:7]
    )

    cruise_alt = (
        payload.cruise_alt_ft
        if payload.cruise_alt_ft is not None
        else profile.default_cruise_alt_ft
    )

    manager = get_simulation_manager()
    req = FlightCreateRequest(
        origin_icao=origin_icao,
        destination_icao=destination_icao,
        aircraft_type=aircraft_type,
        callsign=callsign,
        cruise_alt_ft=cruise_alt,
        squawk=payload.squawk or "1200",
        icao_hex=payload.icao_hex,
    )

    try:
        flight_result = manager.add_flight(req)
    except ValueError as err:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(err)) from err

    return FlightCreateResponse(
        status="CREATED",
        icao_hex=flight_result.icao_hex,
        icao24_hex=flight_result.icao_hex,
        callsign=callsign,
        aircraft_type=aircraft_type,
        origin=origin_airport.icao,
        origin_icao=origin_airport.icao,
        destination=dest_airport.icao,
        destination_icao=dest_airport.icao,
        distance_nm=round(dist_nm, 1),
        cruise_alt_ft=flight_result.effective_cruise_alt_ft,
        dep_rwy=origin_airport.primary_runway.designation,
        arr_rwy=dest_airport.primary_runway.designation,
        waypoints_count=7,
    )


__all__ = [
    "FlightCreatePayload",
    "FlightCreateResponse",
    "create_flight",
    "get_aircraft_types",
    "get_airports",
    "router",
]
