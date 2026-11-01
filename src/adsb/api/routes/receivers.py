"""REST API routes for placing, inspecting, and deleting ground receiver stations."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field, model_validator

from adsb.api.manager import get_simulation_manager

router = APIRouter(prefix="/api/v1/receivers", tags=["receivers"])


class ReceiverCreateRequest(BaseModel):
    """Payload for placing a ground Mode S / ADS-B receiver."""

    rx_id: str | None = Field(default=None, description="Unique station ID (e.g. 'RX_EHAM_01')")
    name: str | None = Field(default=None, description="Station descriptive name")
    lat_deg: float = Field(..., ge=-90.0, le=90.0, description="WGS84 latitude in decimal degrees")
    lon_deg: float = Field(..., ge=-180.0, le=180.0, description="WGS84 longitude in decimal degrees")
    alt_msl_m: float = Field(default=10.0, ge=-500.0, le=10000.0, description="Antenna altitude above MSL in meters")
    antenna_gain_peak_dbi: float = Field(default=5.0, ge=-10.0, le=30.0, description="Peak antenna gain in dBi")
    sensitivity_dbm: float = Field(default=-84.0, ge=-120.0, le=-40.0, description="Receiver sensitivity threshold in dBm")
    noise_floor_dbm: float = Field(default=-90.0, ge=-130.0, le=-50.0, description="Thermal noise floor in dBm")

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
            if "gain_dbi" in d and "antenna_gain_peak_dbi" not in d:
                d["antenna_gain_peak_dbi"] = d["gain_dbi"]
            if "sensitivity" in d and "sensitivity_dbm" not in d:
                d["sensitivity_dbm"] = d["sensitivity"]
            return d
        return data


class ReceiverMetrics(BaseModel):
    """Real-time operational metrics for a ground receiver."""

    sinr_db: float = Field(..., description="Current Signal-to-Interference-plus-Noise Ratio in dB")
    noise_floor_eff_dbm: float = Field(..., description="Effective aggregate noise floor in dBm")
    instantaneous_ber: float = Field(..., description="Instantaneous 2-PPM Bit Error Rate [0.0, 0.5]")
    average_ber: float = Field(default=0.0, description="Rolling average Bit Error Rate")
    total_frames_detected: int = Field(default=0, ge=0, description="Cumulative Mode S frames detected")
    dropped_frames_count: int = Field(default=0, ge=0, description="Cumulative frames dropped due to SNR/BER")
    packet_loss_rate: float = Field(default=0.0, ge=0.0, le=1.0, description="Packet loss fraction [0.0, 1.0]")
    active_jammers_in_range: int = Field(default=0, ge=0, description="Count of active jammers elevating noise floor")


class ReceiverResponse(BaseModel):
    """Summary response for a placed ground receiver."""

    rx_id: str
    name: str
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    antenna_gain_peak_dbi: float
    sensitivity_dbm: float
    noise_floor_dbm: float
    status: str = Field(..., description="'NORMAL' | 'DEGRADED' | 'JAMMED'")
    metrics: ReceiverMetrics
    coverage_radius_km: float = Field(..., description="Effective 4/3-Earth radio horizon coverage radius in km")


class ReceiverStatusResponse(BaseModel):
    """Detailed diagnostic status for an individual receiver station."""

    rx_id: str
    name: str
    lat_deg: float
    lon_deg: float
    alt_msl_m: float
    status: str
    metrics: ReceiverMetrics
    dominant_jammer_id: str | None = None
    dominant_jammer_power_dbm: float | None = None
    active_interference_sources: list[str] = Field(default_factory=list)


@router.get("", response_model=list[ReceiverResponse])
def list_receivers() -> list[dict[str, Any]]:
    """Retrieve all placed ground receiver stations and their live metrics."""
    manager = get_simulation_manager()
    return manager.get_receivers()


@router.post("", response_model=ReceiverResponse, status_code=status.HTTP_201_CREATED)
def create_receiver(payload: ReceiverCreateRequest) -> dict[str, Any]:
    """Place and configure a new ground receiver station."""
    manager = get_simulation_manager()
    try:
        return manager.add_receiver(payload)
    except ValueError as err:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(err)) from err


@router.get("/{rx_id}", response_model=ReceiverResponse)
def get_receiver(rx_id: str) -> dict[str, Any]:
    """Retrieve a specific receiver station by ID."""
    manager = get_simulation_manager()
    rx = manager.get_receiver(rx_id)
    if rx is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Receiver station '{rx_id}' not found",
        )
    return rx


@router.get("/{rx_id}/status", response_model=ReceiverStatusResponse)
def get_receiver_status(rx_id: str) -> dict[str, Any]:
    """Retrieve live RF health diagnostics and degradation metrics for a receiver."""
    manager = get_simulation_manager()
    status_resp = manager.get_receiver_status(rx_id)
    if status_resp is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Receiver station '{rx_id}' not found",
        )
    return status_resp


@router.delete("/{rx_id}", status_code=status.HTTP_200_OK)
def delete_receiver(rx_id: str) -> dict[str, Any]:
    """Remove a placed ground receiver station."""
    manager = get_simulation_manager()
    removed = manager.remove_receiver(rx_id)
    if not removed:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Receiver station '{rx_id}' not found",
        )
    return {"status": "DELETED", "rx_id": rx_id}


__all__ = [
    "ReceiverCreateRequest",
    "ReceiverMetrics",
    "ReceiverResponse",
    "ReceiverStatusResponse",
    "router",
]
