"""The one contract with the robot: joint commands out, measured state and camera frames back.

The robot side is the daemon that owns the servos (`pixi run up` for the real arm, `pixi run
sim` for a simulated one). It takes joint angles and nothing else. There are no poses, grasps
or named skills over there: all of that is Python in this workspace, and every tool here ends
in `Arm.joints`.

    arm = Arm()
    arm.q()                                   # seven measured joint angles, radians
    arm.joints(q, gripper=None, speed=0.8)    # one target, or a list of waypoints
    arm.frame("top")                          # newest camera frame, BGR

What the robot side does with a command. It refuses it outside the calibrated joint limits,
outside the clearance envelope (fingertips into the table, arm into itself, tool behind the
base), or while a servo reports a fault. Otherwise it travels through the waypoints in straight
joint-space segments under one trapezoidal speed profile, trims out servo droop at the end,
and stops with an error if a joint is blocked on the way (`Arm.joints` then tries once more,
slower, before it gives up: robot_config/motion.json says how much slower). `check=False` skips
the envelope;
`allow_low=True` lets the fingertips come down to the table surface. A gripper opening of 0 is
a grasp: the jaws close until they stall and then hold just past that point.

The call returns when the motion has finished. Killing the calling process does not stop a
motion already sent: `Arm.stop()` (tools/stop.py) does. `Arm.pace()` slows one down for a moment.
"""

import json
import os
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

DAEMON = os.environ.get("ELROBOT_DAEMON", "http://127.0.0.1:8765")
MOTION_CONFIG = Path(__file__).resolve().parents[2] / "robot_config" / "motion.json"
BLOCKED_RETRY_SPEED = 0.5  # what applies when motion.json or its key is missing
SLOWEST = 0.05  # the robot side raises any speed below this to it
BLOCKED_PAUSE_S = 0.5  # let the arm settle where it stopped before the slower second try


class RobotError(Exception):
    """The robot side said no, or cannot be reached: something to act on, not a bug."""


def blocked_retry_speed() -> float:
    """The fraction of its speed at which a blocked move is sent again, from robot_config/motion.json.

    `blocked_retry_speed`, 0 to 1: 0.5 is half as fast, 1 the same speed again, 0 no second try.
    Read at the moment it is needed, so an edit applies to the next blocked move. Raises
    ValueError for a file that cannot be read as that.
    """
    try:
        text = MOTION_CONFIG.read_text()
    except FileNotFoundError:
        return BLOCKED_RETRY_SPEED
    try:
        value = json.loads(text).get("blocked_retry_speed", BLOCKED_RETRY_SPEED)
    except (json.JSONDecodeError, AttributeError) as e:
        raise ValueError(f"robot_config/motion.json is not a JSON object ({e})") from None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 <= value <= 1:
        raise ValueError(f"robot_config/motion.json: blocked_retry_speed must be a number from 0 to 1, not {value!r}")
    return float(value)


def _request(path: str, body: dict | None = None, timeout: float = 10.0, raw: bool = False):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(DAEMON + path, data=data, headers={"Content-Type": "application/json"} if data else {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            payload = r.read()
    except urllib.error.HTTPError as e:
        raise RobotError(f"the robot daemon answered HTTP {e.code} for {path}: {e.read().decode(errors='replace')[:200]}") from None
    except OSError as e:
        raise RobotError(
            f"the robot daemon is not reachable at {DAEMON} ({e}). Start it from the repo root: "
            "`pixi run up` (real arm) or `pixi run sim` (simulated)."
        ) from None
    return payload if raw else json.loads(payload)


class Arm:
    def __init__(self, source: str = "agent", call_id: str | None = None, log=None):
        self.source = source  # who is commanding, for the web UI
        self.call_id = call_id  # set when used from inside a tool, so its commands group under it
        self.log = log  # a file that gets one JSON line per command sent
        self.sent = 0
        self._meta: dict | None = None

    # ---- reading

    def state(self) -> dict:
        """The measured state now: joints (q in rad, limits, load, temp, error), gripper, tool pose, cameras."""
        return _request("/api/state")

    def meta(self) -> dict:
        """What does not change while the daemon runs: mode (real/sim), table height, clearance envelope."""
        if self._meta is None:
            self._meta = _request("/api/meta")
        return self._meta

    def why_not_ready(self, state: dict | None = None) -> str | None:
        s = state or self.state()
        if not s["connected"]:
            return "the arm is not connected (no fresh servo state): is it powered and plugged in?"
        if not s["calibrated"]:
            return "the arm's joints are not calibrated"
        faulted = [j["id"] for j in s["joints"] if j["error"]]
        if faulted:
            return (f"servo fault on joint(s) {faulted} (status 32 = overload). Do not move: a person must hold the arm while "
                    "torque is switched off and on again to clear it.")  # fmt: skip
        return None

    def q(self) -> np.ndarray:
        """Seven measured arm joint angles, radians."""
        s = self.state()
        why = self.why_not_ready(s)
        if why:
            raise RobotError(why)
        return np.array([j["q"] for j in s["joints"][:7]])

    def limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Calibrated joint limits (lower, upper), radians. Narrower than the URDF's where the servo arcs are."""
        joints = self.state()["joints"][:7]
        return np.array([j["lower"] for j in joints]), np.array([j["upper"] for j in joints])

    def frame(self, camera: str = "top", settle: float = 0.35) -> np.ndarray:
        """Newest BGR frame from a camera, taken `settle` seconds from now so the arm is still
        (the cameras lag by about 100 ms)."""
        time.sleep(settle)
        live = {c["name"]: c for c in self.state()["cameras"]}
        if camera not in live:
            raise RobotError(f"no camera '{camera}' (live: {sorted(live) or 'none'})")
        if live[camera]["age_ms"] > 2000:
            raise RobotError(f"camera '{camera}' is stale ({live[camera]['age_ms'] / 1000:.1f} s since its last frame)")
        img = cv2.imdecode(np.frombuffer(_request(f"/api/camera/{camera}/frame.jpg", raw=True), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise RobotError(f"could not decode the frame from camera '{camera}'")
        return img

    # ---- commanding

    def joints(self, path=None, *, gripper: float | None = None, speed: float = 0.8, check: bool = True,
               allow_low: bool = False, label: str = "", retry_blocked: bool = True) -> dict:  # fmt: skip
        """Send one joint command and wait for it to finish.

        path:    seven joint angles (rad), or a list of such waypoints travelled in one move.
        gripper: jaw opening 0..1, applied after the arm has arrived; 0 grasps.
        Returns the measured outcome: {"joints": [7 rad], "gripper": {open, width_mm, holding}, "tcp": {pos, quat, approach}}.
        Raises RobotError with the robot side's reason if it refuses or is blocked.
        A move stopped as blocked is sent once more first, slower (`_again_slower`); the result
        then carries `retried_blocked`, the first refusal. `retry_blocked=False` for a move that
        is meant to stop on contact.
        """
        if path is not None:
            from util import safe  # imported here: safe mode plans with util/planning.py, which imports this file

            if safe.enabled():
                path = safe.linear(self, path, label, allow_low)  # straight lines, gripper straight down, or a refusal
        body: dict = {"source": self.source, "call_id": self.call_id, "label": label, "speed": speed, "check": check, "allow_low": allow_low}
        if path is not None:
            p = np.asarray(path, dtype=float)
            body["path"] = [[round(float(v), 5) for v in q] for q in (p[None] if p.ndim == 1 else p)]
        if gripper is not None:
            body["gripper"] = float(gripper)
        res = self._send(body)
        if not res["ok"] and retry_blocked and "path" in body and (res.get("kind") == "blocked" or str(res["error"]).startswith("blocked")):
            res = self._again_slower(body, res)
        if not res["ok"]:
            raise RobotError(res["error"])
        return res

    def planner(self):
        """IK and straight lines for this arm (util/planning.py), made once."""
        if getattr(self, "_planner", None) is None:
            from util.planning import Planner

            self._planner = Planner(self)
        return self._planner

    def _send(self, body: dict) -> dict:
        res = _request("/api/joints", body, timeout=300.0)
        self.sent += 1
        if self.log is not None:
            sent = {k: body[k] for k in ("label", "path", "gripper", "speed", "check", "allow_low") if k in body}
            with open(self.log, "a") as f:
                f.write(json.dumps({"t": round(time.time(), 3), **sent, "outcome": "done" if res["ok"] else res["error"]}) + "\n")
        return res

    def _again_slower(self, body: dict, refused: dict) -> dict:
        """A move stopped as blocked, sent once more from where the arm stopped, at
        `blocked_retry_speed()` times its speed (robot_config/motion.json; half, unless changed).

        On this arm most "blocked" refusals are a joint trailing a fast setpoint with nothing in
        the way, and the same move goes through slower. If something really is in the way the
        slower move is stopped as blocked too, and that second refusal is the one raised. Not
        tried when the arm cannot be used (a servo fault, not connected) or the config says 0:
        then the first refusal stands. With a config that cannot be read the first refusal
        stands too, and says why there was no second try.
        """
        try:
            factor = blocked_retry_speed()
        except ValueError as e:
            return {**refused, "error": f"{refused['error']} Not tried again slower: {e}."}
        if factor == 0:
            return refused
        try:
            state = self.state()
            if self.why_not_ready(state):
                return refused
            now = np.array([j["q"] for j in state["joints"][:7]])
        except (RobotError, KeyError, IndexError, TypeError):
            return refused
        path = np.asarray(body["path"], dtype=float)
        rest = path[int(np.argmin(np.linalg.norm(path - now, axis=1))):]  # the waypoints still ahead: not back to the start of the line
        time.sleep(BLOCKED_PAUSE_S)
        speed = round(max(SLOWEST, body["speed"] * factor), 4)
        res = self._send({**body, "path": rest.tolist(), "speed": speed, "label": (body["label"] + " (again, slower)").strip()})
        if res["ok"]:
            res["retried_blocked"] = refused["error"]
        return res

    def torque(self, enable: bool) -> None:
        """Switch every servo's torque. Off lets the arm go limp: it drops unless it is resting on something."""
        res = _request("/api/torque", {"enable": bool(enable)})
        if not res.get("ok"):
            raise RobotError(res.get("error") or "the torque command was refused")
        for _ in range(30):  # the servos report the change a moment later
            if self.state()["torque"] == bool(enable):
                return
            time.sleep(0.05)

    def stop(self) -> None:
        """Halt any motion now and hold position. Always allowed."""
        _request("/api/stop", {})

    def pace(self, scale: float, hold_s: float = 0.5) -> None:
        """Run motions at `scale` (0.05..1) of their planned speed for the next `hold_s` seconds.
        It wears off by itself: keep calling to keep the arm slow."""
        _request("/api/pace", {"scale": scale, "hold_s": hold_s}, timeout=2.0)

    def emit(self, kind: str, **fields) -> None:
        """Show something in the web UI (timeline entries, markers, detections). Display only."""
        try:
            _request("/api/event", {"kind": kind, **fields}, timeout=2.0)
        except RobotError:
            pass
