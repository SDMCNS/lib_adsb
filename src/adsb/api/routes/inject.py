"""REST API routes for injecting raw wire frames into the tracking pipeline."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, Field

from adsb.api.manager import get_simulation_manager

router = APIRouter(prefix="/api/v1/inject", tags=["inject"])


class InjectFrameRequest(BaseModel):
    """Payload for live wire-format frame injection."""

    format: str = Field(description="Wire format: 'AVR', 'BEAST', or 'SBS'")
    raw_data: str = Field(description="Raw wire ASCII string or hex-encoded payload")


@router.post("/frame", status_code=status.HTTP_200_OK)
async def inject_frame(payload: InjectFrameRequest) -> dict[str, Any]:
    """Inject a single wire-format frame for live decoding, tracking update, and broadcast."""
    manager = get_simulation_manager()
    try:
        return await manager.inject_frame(payload.format, payload.raw_data)
    except ValueError as err:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(err),
        ) from err
