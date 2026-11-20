"""Runtime state for the live Quest trajectory web stream."""

from __future__ import annotations

import queue
import threading
import time
from typing import Any

from .receiver import iso_now

REMOTE_FIELDS = [
    "recv_unix",
    "recv_iso",
    "seq",
    "channel",
    "port",
    "kind",
    "pos_x",
    "pos_y",
    "pos_z",
    "quat_x",
    "quat_y",
    "quat_z",
    "quat_w",
    "flag",
    "num_points",
    "point0_x",
    "point0_y",
    "point0_z",
    "point1_x",
    "point1_y",
    "point1_z",
    "point2_x",
    "point2_y",
    "point2_z",
    "raw_text",
]
EVENT_FIELDS = ["recv_unix", "recv_iso", "seq", "channel", "port", "text", "bytes"]


class LiveState:
    def __init__(self, max_points: int) -> None:
        self.max_points = max_points
        self.lock = threading.Lock()
        self.points: list[dict[str, Any]] = []
        self.clients: list[queue.Queue[dict[str, Any]]] = []
        self.gate_open = False
        self.pause_state: str | None = None
        self.resolution_state: str | None = None
        self.gripper_state: str | None = None
        self.gripper_count = 0
        self.gate_prereq_seen = False
        self.seq = 0
        self.total_received = 0
        self.total_written = 0
        self.started_at = time.time()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "type": "snapshot",
                "recv_iso": iso_now(),
                "points": list(self.points),
                "gate_open": self.gate_open,
                "pause_state": self.pause_state,
                "resolution_state": self.resolution_state,
                "gripper_state": self.gripper_state,
                "gripper_count": self.gripper_count,
                "total_received": self.total_received,
                "total_written": self.total_written,
                "uptime_sec": time.time() - self.started_at,
            }

    def add_client(self) -> queue.Queue[dict[str, Any]]:
        client: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=256)
        with self.lock:
            self.clients.append(client)
        client.put(self.snapshot())
        return client

    def remove_client(self, client: queue.Queue[dict[str, Any]]) -> None:
        with self.lock:
            if client in self.clients:
                self.clients.remove(client)

    def broadcast(self, event: dict[str, Any]) -> None:
        with self.lock:
            clients = list(self.clients)
        for client in clients:
            try:
                client.put_nowait(event)
            except queue.Full:
                try:
                    client.get_nowait()
                    client.put_nowait(event)
                except queue.Empty:
                    pass

    def set_status(
        self,
        *,
        pause_state: str | None = None,
        resolution_state: str | None = None,
        gripper_state: str | None = None,
    ) -> None:
        with self.lock:
            if pause_state is not None:
                self.pause_state = pause_state
            if resolution_state is not None:
                self.resolution_state = resolution_state
            if gripper_state is not None:
                self.gripper_state = gripper_state
                self.gripper_count += 1
            event = {
                "type": "status",
                "recv_iso": iso_now(),
                "gate_open": self.gate_open,
                "pause_state": self.pause_state,
                "resolution_state": self.resolution_state,
                "gripper_state": self.gripper_state,
                "gripper_count": self.gripper_count,
            }
        self.broadcast(event)

    def reset_points(self) -> None:
        with self.lock:
            self.points = []
        self.broadcast({"type": "reset", "recv_iso": iso_now()})

    def add_pose(self, point: dict[str, Any]) -> None:
        with self.lock:
            self.points.append(point)
            if len(self.points) > self.max_points:
                self.points = self.points[-self.max_points :]
            self.total_written += 1
        self.broadcast({"type": "pose", "recv_iso": point["recv_iso"], "point": point})


def is_origin(position: list[float]) -> bool:
    return all(abs(value) < 1e-8 for value in position)
