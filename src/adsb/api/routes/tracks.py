"""REST API routes for querying active aircraft tracks."""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, HTTPException, status

from adsb.api.manager import get_simulation_manager
from adsb.tracker.lifecycle import TrackState

router = APIRouter(prefix="/api/v1/tracks", tags=["tracks"])


@router.get("")
def list_tracks() -> list[dict[str, Any]]:
    """List all currently active tracked aircraft."""
    manager = get_simulation_manager()
    if manager.engine is None:
        return []
    return [
        manager.track_to_dict(t)
        for t in manager.engine.all_tracks(include_tentative=True)
        if t.state != TrackState.DELETED
    ]


@router.get("/{icao_hex}")
def get_track(icao_hex: str) -> dict[str, Any]:
    """Retrieve detailed tracking state for a specific aircraft by 24-bit ICAO address."""
    manager = get_simulation_manager()
    if manager.engine is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Track {icao_hex} not found (no active simulation engine)",
        )

    clean_hex = icao_hex.strip().upper()
    if clean_hex.startswith("0X"):
        clean_hex = clean_hex[2:]

    try:
        icao24 = int(clean_hex, 16)
    except ValueError as err:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Invalid ICAO hex string '{icao_hex}'",
        ) from err

    track = manager.engine.get_track(icao24) or manager.engine._tracks.get(icao24)
    if track is None or track.state == TrackState.DELETED:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Track {clean_hex} not found",
        )

    return manager.track_to_dict(track)
