"""REST API route modules."""

from __future__ import annotations

from adsb.api.routes.flights import router as flights_router
from adsb.api.routes.inject import router as inject_router
from adsb.api.routes.interference import router as interference_router
from adsb.api.routes.receivers import router as receivers_router
from adsb.api.routes.simulation import router as simulation_router
from adsb.api.routes.tracks import router as tracks_router

__all__ = [
    "flights_router",
    "inject_router",
    "interference_router",
    "receivers_router",
    "simulation_router",
    "tracks_router",
]
