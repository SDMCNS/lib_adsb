"""REST API routes for simulation lifecycle control."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field

from adsb.api.manager import SimulationConfig, get_simulation_manager

router = APIRouter(prefix="/api/v1/simulation", tags=["simulation"])


class StepRequest(BaseModel):
    """Payload for stepping simulation forward."""

    step_s: float = Field(default=1.0, gt=0.0)


class SpeedRequest(BaseModel):
    """Payload for setting simulation speed multiplier."""

    speed_multiplier: float = Field(default=1.0, gt=0.0)


@router.get("/state")
async def get_simulation_state() -> dict[str, Any]:
    """Retrieve current simulation state and progress."""
    manager = get_simulation_manager()
    return manager.get_state()


@router.post("/config", status_code=status.HTTP_200_OK)
async def configure_simulation(config: SimulationConfig) -> dict[str, Any]:
    """Apply new simulation scenario configuration."""
    manager = get_simulation_manager()
    try:
        return manager.configure(config)
    except ValueError as err:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(err)) from err


@router.post("/start")
async def start_simulation() -> dict[str, Any]:
    """Start or resume background simulation execution."""
    manager = get_simulation_manager()
    return manager.start()


@router.post("/pause")
async def pause_simulation() -> dict[str, Any]:
    """Pause running simulation."""
    manager = get_simulation_manager()
    return manager.pause()


@router.post("/reset")
async def reset_simulation() -> dict[str, Any]:
    """Reset simulation to initial state."""
    manager = get_simulation_manager()
    return manager.reset()


@router.post("/step")
async def step_simulation(payload: StepRequest | None = None) -> dict[str, Any]:
    """Step simulation forward by step_s seconds."""
    manager = get_simulation_manager()
    step_s = payload.step_s if payload is not None else 1.0
    return await manager.step(step_s)


@router.post("/speed")
async def set_simulation_speed(payload: SpeedRequest) -> dict[str, Any]:
    """Adjust simulation speed multiplier."""
    manager = get_simulation_manager()
    try:
        return manager.set_speed(payload.speed_multiplier)
    except ValueError as err:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(err)) from err
