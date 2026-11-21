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
and stops with an error if a joint is blocked on the way. `check=False` skips the envelope;
`allow_low=True` lets the fingertips come down to the table surface. A gripper opening of 0 is
a grasp: the jaws close until they stall and then hold just past that point.

The call returns when the motion has finished. Killing the calling process does not stop a
motion already sent: `Arm.stop()` (tools/stop.py) does.

Control. The arm is shared (the web UI, other people's scripts, you), so the robot side moves
it only for whoever holds control. The first command an `Arm` sends asks for control and waits
its turn; the tool call that made it gives control back when it ends (`release_control`). To
keep control across several tool calls, take a session with `tools/control.py`: its ticket is
kept in `.control` and every tool uses it until it is released. Reading the state, camera
frames and stop never need control.
"""

import json
import os
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

DAEMON = os.environ.get("ELROBOT_DAEMON", "http://127.0.0.1:8765")
KEY = os.environ.get("ELROBOT_KEY")  # needed when the daemon runs on another machine
SESSION_FILE = Path(__file__).resolve().parents[2] / ".control"  # a session ticket from tools/control.py
WAIT_FOR_CONTROL_S = float(os.environ.get("ELROBOT_CONTROL_WAIT", "300"))


class RobotError(Exception):
    """The robot side said no, or cannot be reached: something to act on, not a bug."""


def _request(path: str, body: dict | None = None, timeout: float = 10.0, raw: bool = False, token: str | None = None):
    data = None if body is None else json.dumps(body).encode()
    headers = {"Content-Type": "application/json"} if data else {}
    if KEY:
        headers["X-Elrobot-Key"] = KEY
    if token:
        headers["X-Control-Token"] = token
    req = urllib.request.Request(DAEMON + path, data=data, headers=headers)
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
        self._ticket: str | None = None  # control held for this Arm alone (not a session)
        self._beat: threading.Event | None = None
        self.looked = False  # a frame was taken: waiting for control afterwards makes it stale

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
        self.looked = True
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
               allow_low: bool = False, label: str = "") -> dict:  # fmt: skip
        """Send one joint command and wait for it to finish.

        path:    seven joint angles (rad), or a list of such waypoints travelled in one move.
        gripper: jaw opening 0..1, applied after the arm has arrived; 0 grasps.
        Returns the measured outcome: {"joints": [7 rad], "gripper": {open, width_mm, holding}, "tcp": {pos, quat, approach}}.
        Raises RobotError with the robot side's reason if it refuses or is blocked.
        """
        body: dict = {"source": self.source, "call_id": self.call_id, "label": label, "speed": speed, "check": check, "allow_low": allow_low}
        if path is not None:
            p = np.asarray(path, dtype=float)
            body["path"] = [[round(float(v), 5) for v in q] for q in (p[None] if p.ndim == 1 else p)]
        if gripper is not None:
            body["gripper"] = float(gripper)
        res = _request("/api/joints", body, timeout=300.0, token=self.control())
        self.sent += 1
        if self.log is not None:
            sent = {k: body[k] for k in ("label", "path", "gripper", "speed", "check", "allow_low") if k in body}
            with open(self.log, "a") as f:
                f.write(json.dumps({"t": round(time.time(), 3), **sent, "outcome": "done" if res["ok"] else res["error"]}) + "\n")
        if not res["ok"]:
            if res.get("kind") == "control" and read_session():
                raise RobotError(res["error"] + ". Your session ticket in .control no longer holds: run tools/control.py "
                                 "'{\"action\": \"acquire\"}' again, and look before acting: someone else may have moved things.")  # fmt: skip
            raise RobotError(res["error"])
        return res

    def torque(self, enable: bool) -> None:
        """Switch every servo's torque. Off lets the arm go limp: it drops unless it is resting on something."""
        res = _request("/api/torque", {"enable": bool(enable)}, token=self.control())
        if not res.get("ok"):
            raise RobotError(res.get("error") or "the torque command was refused")
        for _ in range(30):  # the servos report the change a moment later
            if self.state()["torque"] == bool(enable):
                return
            time.sleep(0.05)

    # ---- control

    def control(self) -> str:
        """The control ticket to send with a command: the session's if there is one, else this
        Arm's own, asked for (and waited for) on first use."""
        session = read_session()
        if session:
            return session
        if self._ticket:
            return self._ticket
        t0 = time.monotonic()
        res = _request("/api/control/request", {"name": self.source, "kind": "agent", "wait": 0})
        token, holder = res["token"], None
        while res.get("you", {}).get("state") != "granted":
            if res.get("you", {}).get("state") == "none":
                raise RobotError("lost my place in the queue for control of the arm; try again")
            holder = (res.get("holder") or {}).get("name") or holder
            if time.monotonic() - t0 > WAIT_FOR_CONTROL_S:
                _request("/api/control/release", {}, token=token)
                raise RobotError(f"waited {WAIT_FOR_CONTROL_S:.0f} s for control of the arm, {holder or 'someone'} still has it")
            wait = max(0.5, min(20.0, WAIT_FOR_CONTROL_S - (time.monotonic() - t0)))
            res = _request("/api/control/request", {"name": self.source, "kind": "agent", "wait": wait}, timeout=wait + 10, token=token)
        self._ticket = token
        self._start_heartbeat(token)
        waited = time.monotonic() - t0
        if waited > 3.0 and self.looked:
            self.release_control()
            raise RobotError(f"waited {waited:.0f} s for control of the arm ({holder or 'someone'} had it). What this tool saw "
                             "before may be out of date: look again, then retry. tools/control.py keeps control across calls.")  # fmt: skip
        return token

    def _start_heartbeat(self, token: str) -> None:
        stop = self._beat = threading.Event()

        def beat():
            while not stop.wait(5.0):
                try:
                    _request("/api/control/request", {"wait": 0}, timeout=5.0, token=token)
                except RobotError:
                    pass

        threading.Thread(target=beat, daemon=True).start()

    def release_control(self) -> None:
        """Give back control this Arm asked for (a session's ticket stays)."""
        if self._beat is not None:
            self._beat.set()
            self._beat = None
        if self._ticket:
            token, self._ticket = self._ticket, None
            try:
                _request("/api/control/release", {}, token=token)
            except RobotError:
                pass

    def stop(self) -> None:
        """Halt any motion now and hold position. Always allowed."""
        _request("/api/stop", {})

    def emit(self, kind: str, **fields) -> None:
        """Show something in the web UI (timeline entries, markers, detections). Display only."""
        try:
            _request("/api/event", {"kind": kind, **fields}, timeout=2.0)
        except RobotError:
            pass


def read_session() -> str | None:
    try:
        return SESSION_FILE.read_text().strip() or None
    except OSError:
        return None
