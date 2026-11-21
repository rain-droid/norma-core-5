"""Perception as files, for the agent workspace (`workspace/`, after RoboHarness V2).

A coding agent is started inside the workspace directory. Its tools are Python files that run
in the agent's own shell and reach the arm only through joint commands (`POST /api/joints`).
What the daemon does for the workspace is publish: the newest camera frames and the measured
robot state are kept under `perception/`, each folder playing the part of a topic that a
reader opens when it wants to look.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .daemon import Hub

log = logging.getLogger("elrobot.workspace")

PUBLISH_HZ = 2.0


def write_json(path: Path, obj: Any) -> None:
    """Atomic, so a reader never sees half a file."""
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(json.dumps(obj, indent=1) + "\n")
    os.replace(tmp, path)


def write_bytes(path: Path, data: bytes) -> None:
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


class Workspace:
    def __init__(self, hub: "Hub", root: Path):
        self.hub = hub
        self.root = root.resolve()
        self.perception_dir = self.root / "perception"
        self._sequence = 0

    def state(self) -> dict:
        """Measured robot state, in the units joint commands take (radians, 0..1 gripper)."""
        snap = self.hub.robot.snapshot()
        out = {
            "t": snap["t"],
            "mode": snap["mode"],
            "connected": snap["connected"],
            "calibrated": snap["calibrated"],
            "torque": snap["torque"],
            "busy": snap["busy"],
            "activity": snap["activity"],
            "joints": [
                {"id": j["id"], "rad": j["q"], "deg": round(math.degrees(j["q"]), 1), "goal_rad": j["q_goal"],
                 "lower_rad": j["lower"], "upper_rad": j["upper"], "load": j["load"], "temp_c": j["temp"], "error": j["error"]}
                for j in snap["joints"] if j["id"] <= 7
            ],
            "gripper": snap["gripper"],
            "tcp": None,
        }  # fmt: skip
        if snap["ee"]:
            a = snap["ee"]["approach"]
            out["tcp"] = {
                "xyz_m": snap["ee"]["pos"],
                "approach": a,
                "pitch_deg": round(math.degrees(math.asin(max(-1.0, min(1.0, -a[2])))), 1),
            }
        return out

    def publish(self) -> None:
        """Write the newest camera frames and robot state under perception/."""
        link = self.hub.link
        self._sequence += 1
        now = time.monotonic()
        cameras = {}
        for cam, fr in sorted(link.frames.items()):
            d = self.perception_dir / "exteroception" / cam
            d.mkdir(parents=True, exist_ok=True)
            write_bytes(d / "rgb.jpg", fr.jpeg)
            cameras[cam] = {"image": f"perception/exteroception/{cam}/rgb.jpg", "width": fr.width, "height": fr.height,
                            "age_ms": int((now - fr.stamp) * 1000), "frame_seq": fr.seq}  # fmt: skip
        d = self.perception_dir / "proprioception"
        d.mkdir(parents=True, exist_ok=True)
        state = self.state()
        write_json(d / "state.json", state)
        write_json(
            self.perception_dir / "latest.json",
            {
                "sequence": self._sequence,
                "t": time.time(),
                "connected": state["connected"],
                "busy": state["busy"],
                "cameras": cameras,
                "state": "perception/proprioception/state.json",
            },
        )

    async def publish_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0 / PUBLISH_HZ)
            try:
                self.publish()
            except Exception:
                log.exception("perception publish failed")
