"""The second arm on the table, for the digital twin.

The SO-101 runs on its own daemon (scripts/second_arm_up.sh). This one only watches it: ten times
a second it reads that daemon's state and adds it to its own state message as `second_arm`, with
where the arm stands in this arm's base frame, so the web UI can draw both arms in one scene. It
also passes that daemon's URDF and meshes through, because a browser page may only fetch from the
daemon that served it. Nothing here commands the second arm: the workspace does that
(workspace/tools/util/second_arm.py).

Where it stands comes from workspace/robot_config/second_arm.json, the fit made by
tools/calibrate_second_arm.py. That fit is of points measured in the air and comes out tilted by a
few degrees; an arm standing on the table is not, so the twin uses the fit's position on the
table and its turn about the vertical, and stands the arm upright on the table.
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import mimetypes
from pathlib import Path

import httpx

log = logging.getLogger("elrobot.second_arm")
logging.getLogger("httpx").setLevel(logging.WARNING)  # it logs every request, and this asks ten times a second


class SecondArm:
    def __init__(self, url: str, workspace_dir: Path):
        self.url = url.rstrip("/")
        self.fit_path = workspace_dir / "robot_config" / "second_arm.json"
        self._state: dict | None = None
        self._meta: dict | None = None
        self._fit: tuple[float, list | None] = (0.0, None)  # the file's mtime and the placement made from it
        self._assets: dict[str, tuple[bytes, str]] = {}

    async def poll_loop(self) -> None:
        async with httpx.AsyncClient(timeout=1.5) as client:
            while True:
                try:
                    if self._meta is None:
                        self._meta = (await client.get(self.url + "/api/meta")).json()
                        log.info("second arm found at %s (%s)", self.url, self._meta.get("robot"))
                    s = (await client.get(self.url + "/api/state")).json()
                    self._state = {
                        "connected": bool(s.get("connected")),
                        "torque": bool(s.get("torque")),
                        "activity": s.get("activity"),
                        "joints": [{k: j.get(k) for k in ("id", "name", "q", "q_goal", "error")} for j in s.get("joints") or []],
                        "gripper": s.get("gripper"),
                        "ee": s.get("ee"),
                    }
                    await asyncio.sleep(0.1)
                except (httpx.HTTPError, ValueError):
                    if self._state is not None:
                        log.info("second arm's daemon at %s stopped answering", self.url)
                    self._state, self._meta = None, None
                    await asyncio.sleep(2.0)

    def placement(self) -> list | None:
        """The second arm's base in this arm's base frame as a 4x4, upright on the table; None until it has been fitted."""
        try:
            mtime = self.fit_path.stat().st_mtime
        except OSError:
            return None
        if mtime != self._fit[0]:
            try:
                T = json.loads(self.fit_path.read_text())["T_base_second"]
                yaw = math.atan2(T[1][0], T[0][0])
                c, s = math.cos(yaw), math.sin(yaw)
                self._fit = (mtime, [[c, -s, 0.0, T[0][3]], [s, c, 0.0, T[1][3]], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
            except (ValueError, KeyError, IndexError, TypeError):
                self._fit = (mtime, None)
        return self._fit[1]

    def snapshot(self) -> dict | None:
        if self._state is None or self._meta is None:
            return None
        return {"robot": self._meta.get("robot"), "urdf": "/api/second_arm" + str(self._meta.get("urdf") or ""), "T": self.placement(), **self._state}

    async def asset(self, path: str) -> tuple[bytes, str] | None:
        """A file of the second arm's model, fetched from its daemon once and kept."""
        if path not in self._assets:
            try:
                async with httpx.AsyncClient(timeout=20.0) as client:
                    r = await client.get(f"{self.url}/assets/{path}")
            except httpx.HTTPError:
                return None
            if r.status_code != 200:
                return None
            self._assets[path] = (r.content, mimetypes.guess_type(path)[0] or "application/octet-stream")
        return self._assets[path]
