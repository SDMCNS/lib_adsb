"""WebSocket endpoint streaming decoded telemetry and track state vectors."""

from __future__ import annotations

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from adsb.api.manager import get_simulation_manager

router = APIRouter(tags=["websockets"])


@router.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket) -> None:
    """Stream live aircraft track vectors in JSON format."""
    await websocket.accept()
    manager = get_simulation_manager()
    manager.register_telemetry_client(websocket)
    try:
        while True:
            msg = await websocket.receive_text()
            if msg == "snapshot":
                await websocket.send_json(manager.get_telemetry_snapshot())
    except (WebSocketDisconnect, ConnectionResetError):
        pass
    finally:
        manager.unregister_telemetry_client(websocket)
