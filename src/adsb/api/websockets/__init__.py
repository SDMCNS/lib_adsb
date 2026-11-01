"""WebSocket streaming route modules."""

from __future__ import annotations

from adsb.api.websockets.telemetry import router as telemetry_router
from adsb.api.websockets.wire import router as wire_router

__all__ = [
    "telemetry_router",
    "wire_router",
]
