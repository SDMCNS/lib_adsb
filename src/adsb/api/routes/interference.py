"""REST API routes for placing, configuring, toggling, and deleting RF interference sources."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field, model_validator

from adsb.api.manager import get_simulation_manager

router = APIRouter(prefix="/api/v1/interference", tags=["interference"])


class InterferenceCreateRequest(BaseModel):
    """Payload for placing an RF interference / jamming source."""

    source_id: str | None = Field(default=None, description="Unique source ID (e.g. 'JAMMER_01')")
    name: str | None = Field(default=None, description="Descriptive name")
    lat_deg: float = Field(..., ge=-90.0, le=90.0, description="Latitude in decimal degrees")
    lon_deg: float = Field(..., ge=-180.0, le=180.0, description="Longitude in decimal degrees")
    alt_msl_m: float = Field(default=10.0, ge=-500.0, le=10000.0, description="Antenna altitude above MSL in meters")
    tx_power_dbm: float = Field(default=40.0, ge=0.0, le=80.0, description="Transmit power (EIRP) in dBm")
    frequency_hz: float = Field(default=1090.0e6, ge=900.0e6, le=1200.0e6, description="Center frequency in Hz")
    bandwidth_mhz: float = Field(default=2.0, ge=0.1, le=100.0, description="Jamming bandwidth in MHz")
    is_active: bool = Field(default=True, description="Whether the jammer is transmitting")

    @model_validator(mode="before")
    @classmethod
    def _normalize_fields(cls, data: Any) -> Any:
        if isinstance(data, dict):
            d = dict(data)
            if "lat" in d and "lat_deg" not in d:
                d["lat_deg"] = d["lat"]
            if "lon" in d and "lon_deg" not in d:
                d["lon_deg"] = d["lon"]
            if "alt_m" in d and "alt_msl_m" not in d:
                d["alt_msl_m"] = d["alt_m"]
            if "power_dbm" in d and "tx_power_dbm" not in d:
                d["tx_power_dbm"] = d["power_dbm"]
            return d
        return data


class InterferenceUpdateRequest(BaseModel):
    """Payload for updating or toggling an interference source."""

    is_active: bool | None = None
    tx_power_dbm: float | None = Field(default=None, ge=0.0, le=80.0)
    bandwidth_mhz: float | None = Field(default=None, ge=0.1, le=100.0)


class InterferenceResponse(BaseModel):
    """Summary response for an RF interference source."""

    source_id: str
    name: str
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    tx_power_dbm: float
    tx_power_w: float = Field(..., description="Transmit power in Watts")
    frequency_hz: float
    bandwidth_mhz: float
    is_active: bool
    effective_range_km: float = Field(..., description="Estimated 3 dB noise floor elevation radius in km")


@router.get("", response_model=list[InterferenceResponse])
def list_interference() -> list[dict[str, Any]]:
    """Retrieve all configured RF interference sources."""
    manager = get_simulation_manager()
    return manager.get_interference_sources()


@router.post("", response_model=InterferenceResponse, status_code=status.HTTP_201_CREATED)
def create_interference(payload: InterferenceCreateRequest) -> dict[str, Any]:
    """Place and deploy an RF interference source / jammer."""
    manager = get_simulation_manager()
    try:
        return manager.add_interference_source(payload)
    except ValueError as err:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(err)) from err


@router.get("/{source_id}", response_model=InterferenceResponse)
def get_interference(source_id: str) -> dict[str, Any]:
    """Retrieve a specific RF interference source by ID."""
    manager = get_simulation_manager()
    source = manager.get_interference_source(source_id)
    if source is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Interference source '{source_id}' not found",
        )
    return source


@router.patch("/{source_id}", response_model=InterferenceResponse)
def update_interference(source_id: str, payload: InterferenceUpdateRequest) -> dict[str, Any]:
    """Update active state or power level for an RF interference source."""
    manager = get_simulation_manager()
    source = manager.update_interference_source(source_id, payload)
    if source is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Interference source '{source_id}' not found",
        )
    return source


@router.post("/{source_id}/toggle", response_model=InterferenceResponse)
def toggle_interference(source_id: str) -> dict[str, Any]:
    """Toggle an RF interference source on or off."""
    manager = get_simulation_manager()
    source = manager.toggle_interference_source(source_id)
    if source is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Interference source '{source_id}' not found",
        )
    return source


@router.delete("/{source_id}", status_code=status.HTTP_200_OK)
def delete_interference(source_id: str) -> dict[str, Any]:
    """Remove an RF interference source."""
    manager = get_simulation_manager()
    removed = manager.remove_interference_source(source_id)
    if not removed:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Interference source '{source_id}' not found",
        )
    return {"status": "DELETED", "source_id": source_id}


__all__ = [
    "InterferenceCreateRequest",
    "InterferenceResponse",
    "InterferenceUpdateRequest",
    "router",
]
