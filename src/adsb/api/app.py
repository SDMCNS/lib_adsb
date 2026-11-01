"""FastAPI application factory, lifespan management, and static file mounting."""

from __future__ import annotations

import mimetypes
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from starlette.staticfiles import StaticFiles

from adsb.api.manager import get_simulation_manager
from adsb.api.routes.flights import router as flights_router
from adsb.api.routes.inject import router as inject_router
from adsb.api.routes.interference import router as interference_router
from adsb.api.routes.receivers import router as receivers_router
from adsb.api.routes.simulation import router as simulation_router
from adsb.api.routes.tracks import router as tracks_router
from adsb.api.websockets.telemetry import router as telemetry_router
from adsb.api.websockets.wire import router as wire_router

# Ensure standard MIME types on all OS platforms
mimetypes.add_type("application/javascript", ".js")
mimetypes.add_type("text/css", ".css")
mimetypes.add_type("text/html", ".html")


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Application lifespan context manager handling simulation startup and shutdown."""
    manager = get_simulation_manager()
    yield
    await manager.shutdown()


def create_app() -> FastAPI:
    """Create and configure the FastAPI application instance."""
    app = FastAPI(
        title="ADS-B / Mode S / UAT Simulation & Tracking API",
        description="REST and WebSocket streaming service for real-time air traffic simulation and tactical tracking.",
        version="0.1.0",
        lifespan=lifespan,
    )

    # CORS configuration
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Register API routers
    app.include_router(simulation_router)
    app.include_router(tracks_router)
    app.include_router(inject_router)
    app.include_router(flights_router)
    app.include_router(receivers_router)
    app.include_router(interference_router)

    # Register WebSocket routers
    app.include_router(telemetry_router)
    app.include_router(wire_router)

    # Mount static frontend assets
    static_dir = Path(__file__).parent / "static"
    static_dir.mkdir(parents=True, exist_ok=True)
    app.mount("/", StaticFiles(directory=str(static_dir), html=True), name="static")

    return app


# Module-level instance for test clients and ASGI servers
app = create_app()

__all__ = [
    "app",
    "create_app",
]
