"""WebSocket endpoints (transport layer).

``/ws``        - server -> console fan-out (telemetry, exec progress, logs,
                 I2C trace, vision obstacle reports). The console only
                 listens; incoming text is ignored keep-alive.
``/ws/vision`` - console-independent input channel for the camera / vision
                 process: it pushes the obstacles it currently sees. The
                 message protocol lives here; the *reaction* to store changes
                 (broadcast, mid-run re-plan) is the mission watcher's job.

File:    software/console/api/ws.py
Author:  Mouadh Jeridi <jeridimouadh@gmail.com>
Version: 3.0.0
Date:    2026-09-28
"""

from __future__ import annotations

import json
import time
from typing import Dict, Set

from fastapi import WebSocket, WebSocketDisconnect

from software.core.mission import watcher
from ..runtime.state import state


class WsHub:
    """Fan-out of JSON events to the browser sessions."""

    def __init__(self) -> None:
        """Create the hub (no clients yet)."""
        self._clients: Set[WebSocket] = set()

    async def broadcast(self, event: Dict) -> None:
        """Fan one event out to every connected console."""
        if not self._clients:
            return
        payload = json.dumps(event, separators=(",", ":"))
        for websocket in list(self._clients):
            try:
                await websocket.send_text(payload)
            except Exception:
                self._clients.discard(websocket)

    async def endpoint(self, websocket: WebSocket) -> None:
        """Serve /ws: register the client, forward events, keep the socket open."""
        await websocket.accept()
        self._clients.add(websocket)
        try:
            while True:
                await websocket.receive_text()  # client keep-alive; content ignored
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
        finally:
            self._clients.discard(websocket)


async def _send(websocket: WebSocket, message: Dict) -> None:
    """Send one JSON message on the socket, ignoring a dead client."""
    try:
        await websocket.send_text(json.dumps(message, separators=(",", ":")))
    except Exception:
        pass  # the client went away; nothing to do


async def _handle_vision_message(websocket: WebSocket, peer: str, raw: str) -> None:
    """One message from a vision client (protocol + acks)."""
    try:
        message = json.loads(raw)
        if not isinstance(message, dict):
            raise ValueError("message must be a JSON object")
    except (json.JSONDecodeError, ValueError) as exc:
        await _send(websocket, {"type": "error", "detail": f"bad message: {exc}"})
        return
    kind = str(message.get("type", ""))
    if kind in ("obstacles", "update", "report"):
        changed = state.vision.update(
            message.get("obstacles") or [],
            message.get("ttl_ms"),
            source=peer,
            t_ms=message.get("t"),
            client=str(message.get("client", "")),
        )
        await _send(websocket, {
            "type": "ack",
            "count": len(state.vision.active()),
            "changed": changed,
            "ttl_ms": state.vision.ttl_ms,
            # the camera's own clock (echoed) and the service receive time, so
            # the sender can measure latency and order reports - one ack per
            # message, in order, so a synthetic correlation id adds nothing
            "t": state.vision.reported_t,
            "t_server": round(time.time() * 1000.0, 1),
        })
        if changed:
            await watcher.notify_change(state, "camera update")
    elif kind == "poses":
        us = message.get("us")
        opponent = message.get("opponent")
        previous_opponent = state.session.poses.opponent()
        changed = await state.session.on_camera_poses(
            us if isinstance(us, dict) else None,
            opponent if isinstance(opponent, dict) else None,
            t_ms=message.get("t"),
            ttl_ms=message.get("ttl_ms"),
            source=peer,
            client=str(message.get("client", "")),
            t_server_ms=time.time() * 1000.0,
        )
        await _send(websocket, {
            "type": "ack",
            "seen": {
                "us": isinstance(us, dict),
                "opponent": isinstance(opponent, dict),
            },
            "changed": changed,
            "ttl_ms": state.session.poses.ttl_ms,
            # the camera's own clock (echoed) and the service receive time, so
            # the sender can measure the pipeline latency of its fixes
            "t": state.session.poses.reported_t,
            "t_server": round(time.time() * 1000.0, 1),
        })
        if changed:
            watcher.notify_opponent_change(
                state, previous_opponent, state.session.poses.opponent()
            )
    elif kind == "clear":
        changed = state.vision.clear()
        await _send(websocket, {"type": "ack", "count": 0, "changed": changed})
        if changed:
            await watcher.notify_change(state, "cleared by the camera")
    elif kind == "ping":
        await _send(websocket, {"type": "pong"})
    else:
        await _send(websocket, {
            "type": "error",
            "detail": f"unknown type '{kind}' (use 'obstacles', 'poses', 'clear' or 'ping')",
        })


async def _vision_source_gone(peer: str) -> None:
    """A vision client disconnected: its reports must not linger.

    Releasing the snapshots also releases the *ownership*: the console,
    which stands down while a real camera streams, sees ``source`` go empty
    and takes its mock camera role back.
    """
    previous_opponent = state.session.poses.opponent()
    if state.session.release_camera_poses(peer):
        await state.log("info", "camera disconnected - poses released")
        watcher.notify_opponent_change(state, previous_opponent, None)
    if state.vision.source != peer:
        return
    if state.vision.release(peer):
        await watcher.notify_change(state, "camera disconnected")


async def vision_endpoint(websocket: WebSocket) -> None:
    """Camera / detector input: pushes the obstacle snapshot it sees."""
    await websocket.accept()
    peer = "vision"
    if websocket.client is not None:
        peer = f"vision:{websocket.client.host}:{websocket.client.port}"
    try:
        await websocket.send_text(json.dumps({
            "type": "hello",
            "protocol": 1,
            "peer": peer,
            "ttl_ms": state.vision.ttl_ms,
            "hint": "send {\"type\":\"obstacles\",\"obstacles\":[...],\"ttl_ms\":2000}",
        }, separators=(",", ":")))
        while True:
            raw = await websocket.receive_text()
            await _handle_vision_message(websocket, peer, raw)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        await _vision_source_gone(peer)


hub = WsHub()
