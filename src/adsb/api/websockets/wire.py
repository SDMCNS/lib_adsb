"""WebSocket endpoint streaming raw wire-format frames."""

from __future__ import annotations

from fastapi import APIRouter, Query, WebSocket, WebSocketDisconnect

from adsb.api.manager import get_simulation_manager

router = APIRouter(tags=["websockets"])


@router.websocket("/ws/wire")
async def websocket_wire(
    websocket: WebSocket,
    wire_format: str = Query(default="AVR", alias="format"),
) -> None:
    """Stream raw wire-encoded frames (AVR, BEAST, or SBS)."""
    await websocket.accept()
    manager = get_simulation_manager()
    manager.register_wire_client(websocket, wire_format)
    try:
        while True:
            await websocket.receive_text()
    except (WebSocketDisconnect, ConnectionResetError):
        pass
    finally:
        manager.unregister_wire_client(websocket)
