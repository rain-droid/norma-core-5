"""Push-T in the agent workspace, against the simulated robot. The simulated table gets a T
that the closed fingertips can shove about: seen by the simulated top camera, moved by nothing
but contact with the arm."""

import math
import os
import shutil
import socket
import sys
import threading
import time

import cv2
import numpy as np
import pytest
import uvicorn

from elrobot.config import REPO_ROOT, Config
from elrobot.daemon import create_app

TABLE_Z = 0.010
FINGER_RADIUS = 0.011  # the closed fingertips, as a disc
C2 = 0.0006  # how the simulated T trades sliding for turning: not the value the planner assumes


@pytest.fixture(scope="module")
def robot(tmp_path_factory):
    root = tmp_path_factory.mktemp("agent") / "workspace"
    shutil.copytree(REPO_ROOT / "workspace", root, ignore=shutil.ignore_patterns("perception", "tool_call_history", "state", "__pycache__", "assets", "*.sim.json", ".*"))
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    app = create_app(Config(sim=True, voice=False, workspace_dir=root, table_z=TABLE_Z))
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    while not server.started:
        time.sleep(0.05)
    os.environ["ELROBOT_DAEMON"] = f"http://127.0.0.1:{port}"  # read by util/arm.py when first imported
    sys.path.insert(0, str(root / "tools"))
    yield root, app.state.hub.link.frame_provider.__self__
    server.should_exit = True
    thread.join(5)


def run(tool: str, **arguments) -> dict:
    from util.cli import call

    return call(getattr(__import__(tool), tool), arguments)


class SimT:
    """A flat T lying on the simulated table. A fingertip that comes against its side pushes it:
    it gives way just far enough, sliding and turning about its centroid according to where it
    is pressed. A fingertip that comes down on top of it is recorded as `crushed`."""

    def __init__(self, world, polygon: np.ndarray, pose, height: float = 0.020, bgr=(200, 200, 40)):
        self.world, self.polygon, self.pose, self.height, self.bgr = world, polygon.astype(float), np.array(pose, dtype=float), height, bgr
        self.crushed, self.touched, self._tip = False, 0, None
        self._render, self._step = world._render, world._step
        world._render, world._step = self.render, self.step

    def remove(self) -> None:
        self.world._render, self.world._step = self._render, self._step

    def outline(self) -> np.ndarray:
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        return self.polygon @ np.array([[c, -s], [s, c]]).T + self.pose[:2]

    def press(self, tip: np.ndarray) -> None:
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        R = np.array([[c, -s], [s, c]])
        p = R.T @ (tip - self.pose[:2])
        nearest, gap = None, 1e9
        for a, b in zip(self.polygon, np.roll(self.polygon, -1, axis=0)):
            q = a + np.clip((p - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0) * (b - a)
            if np.linalg.norm(p - q) < gap:
                nearest, gap = q, float(np.linalg.norm(p - q))
        inside = cv2.pointPolygonTest(self.polygon.astype(np.float32), (float(p[0]), float(p[1])), False) > 0
        depth = FINGER_RADIUS + gap if inside else FINGER_RADIUS - gap
        if depth <= 0 or gap < 1e-9:
            return
        u = (nearest - p) / gap * (-1.0 if inside else 1.0)  # from the fingertip into the T
        lever = float(nearest[0] * u[1] - nearest[1] * u[0])
        slide = depth / (1 + lever * lever / C2)
        self.pose[:2] += R @ u * slide
        self.pose[2] += slide * lever / C2
        self.touched += 1

    def step(self) -> None:
        self._step()
        T = self.world.robot.tcp()
        tip = T[:3, 3] + T[:3, 2] * 0.012
        if tip[2] - self.world.table_z >= self.height:
            self._tip = None
            return
        if self._tip is None:  # just came down: beside the T, or onto it?
            p = tip[:2] - self.pose[:2]
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            local = np.array([[c, s], [-s, c]]) @ p
            if cv2.pointPolygonTest(self.polygon.astype(np.float32), (float(local[0]), float(local[1])), True) > -0.5 * FINGER_RADIUS:
                self.crushed = True
            self._tip = tip[:2].copy()
        for k in np.linspace(0.0, 1.0, max(2, int(np.linalg.norm(tip[:2] - self._tip) / 0.0005) + 1))[1:]:
            self.press(self._tip + k * (tip[:2] - self._tip))
        self._tip = tip[:2].copy()

    def render(self, cam, T_base_tool):
        img = self._render(cam, T_base_tool)
        z0 = self.world.table_z
        base = [cam.project([x, y, z0], T_base_tool) for x, y in self.outline()]
        top = [cam.project([x, y, z0 + self.height], T_base_tool) for x, y in self.outline()]
        if any(p is None for p in base + top):
            return img
        base, top = np.array(base), np.array(top)
        if np.abs(np.concatenate([base, top])).max() > 20 * max(cam.width, cam.height):
            return img
        side = tuple(int(c * 0.6) for c in self.bgr)
        for i in range(len(base)):
            j = (i + 1) % len(base)
            cv2.fillConvexPoly(img, np.round([base[i], base[j], top[j], top[i]]).astype(np.int32), side, cv2.LINE_AA)
        cv2.fillPoly(img, [np.round(top).astype(np.int32)], self.bgr, cv2.LINE_AA)
        return img


def clear_table(world) -> None:
    for b, xy in zip(world.blocks, ((0.21, 0.36), (-0.21, 0.36), (0.21, 0.17))):
        b.xy, b.z = np.array(xy), 0.0
    world.held = None


@pytest.mark.parametrize("start, target", [
    ((0.04, 0.27, 25.0), (-0.02, 0.24, 0.0)),
    ((-0.05, 0.22, -40.0), (0.03, 0.26, 20.0)),
])  # fmt: skip
def test_the_t_is_pushed_onto_its_target(robot, start, target):
    from util import pushing

    _, world = robot
    clear_table(world)
    polygon = pushing.t_polygon()
    t = SimT(world, polygon, (start[0], start[1], math.radians(start[2])))
    try:
        assert run("home")["pose"] == "ready"
        seen = run("push_t", x=target[0], y=target[1], yaw_deg=target[2], max_pushes=0)
        assert np.linalg.norm([seen["pose"]["x"] - start[0], seen["pose"]["y"] - start[1]]) < 0.004, seen
        assert abs(seen["pose"]["yaw_deg"] - start[2]) < 4, seen

        done = run("push_t", x=target[0], y=target[1], yaw_deg=target[2], max_pushes=25)
        print({k: v for k, v in done.items() if k != "pushes"}, len(done.get("pushes", [])), "true", t.pose[:2].round(4), round(math.degrees(t.pose[2]), 1))
        for p in done.get("pushes", []):
            print("  ", p)
        assert done.get("reached"), done
        want = np.array([target[0], target[1], math.radians(target[2])])
        off, heading = pushing.errors(t.pose, want)
        assert off < 0.008 and math.degrees(heading) < 8, (t.pose, "where the T really is")
        assert pushing.overlap(polygon, t.pose, want) > 0.75
        assert t.touched and not t.crushed, "moved by pushing its sides, never by coming down on it"
    finally:
        t.remove()
