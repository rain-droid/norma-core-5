"""The robot daemon: one process that owns the arm and serves docs/API.md.

Clients (the agent workspace's joint commands, the MCP shim, the web UI, the voice agent) all
go through here, so there is a single place that decides what the arm does and a single stream
of events.
"""

from __future__ import annotations

import asyncio
import base64
import itertools
import json
import logging
import time
import uuid
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
import numpy as np

from . import tools as toolreg
from .config import Config
from .kinematics import Kinematics
from .link import Link, SimLink, StationLink
from .motion import interpolate_joint
from .robot import Robot, RobotError, Stopped
from .skills import GESTURES, Skills
from .workspace import Workspace, write_json

log = logging.getLogger("elrobot.daemon")


class Hub:
    """Shared state of the daemon: the robot, the event log, and connected sockets."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.link: Link = self._make_link()
        self.robot = Robot(self.link, cfg)
        self.skills = Skills(self.robot, cfg, self.emit)
        self.robot.on_plan = self._on_plan
        self.events: deque[dict] = deque(maxlen=300)
        self._ids = itertools.count(1)
        self.sockets: set[WebSocket] = set()
        self._task_lock = asyncio.Lock()  # one motion tool at a time
        self._current_call: str | None = None
        self.voice = None
        self.workspace = Workspace(self, cfg.workspace_dir) if cfg.workspace_dir.is_dir() else None
        if isinstance(self.link, SimLink):
            from .simworld import SimWorld

            world = SimWorld(self.robot, self.skills.cam)
            world.attach(self.link)
            if self.workspace is not None:  # the simulated overhead camera's calibration, in calibrate_top's format
                (self.workspace.root / "robot_config").mkdir(exist_ok=True)
                write_json(self.workspace.root / "robot_config" / "camera_top.sim.json", world.calibration())

    def _make_link(self) -> Link:
        cfg = self.cfg
        if cfg.sim:
            kin = Kinematics(cfg.urdf)
            limits = {j.id: (j.lower, j.upper) for j in kin.joints}
            limits[8] = (kin.gripper_lower, kin.gripper_upper)
            return SimLink(limits)
        return StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras)

    # ---- events

    def emit(self, kind: str, **fields) -> dict:
        ev = {"type": "event", "id": next(self._ids), "t": time.time(), "kind": kind, **fields}
        self.events.append(ev)
        self._broadcast(ev)
        return ev

    def emit_dict(self, ev: dict) -> None:
        """For the voice service, which hands over {"kind": ..., ...}."""
        ev = dict(ev)
        self.emit(ev.pop("kind"), **ev)

    def _on_plan(self, path: list, target: dict | None) -> None:
        self.emit("plan", call_id=self._current_call, path=path, target=target)

    def _broadcast(self, msg: dict) -> None:
        if not self.sockets:
            return
        text = json.dumps(msg, separators=(",", ":"))
        for ws in list(self.sockets):
            asyncio.create_task(self._send(ws, text))

    async def _send(self, ws: WebSocket, text: str) -> None:
        try:
            await asyncio.wait_for(ws.send_text(text), timeout=1.0)
        except Exception:
            self.sockets.discard(ws)

    def meta(self) -> dict:
        kin = self.robot.kin
        joints = [{"id": j.id, "name": j.name, "lower": round(float(kin.lower[j.id - 1]), 4), "upper": round(float(kin.upper[j.id - 1]), 4)} for j in kin.joints]
        joints.append({"id": 8, "name": "rev_motor_08", "lower": 0.0, "upper": 1.0})
        return {
            "type": "hello",
            "version": 1,
            "mode": self.link.mode,
            "urdf": "/assets/elrobot/elrobot_follower.urdf",
            "joints": joints,
            "tools": toolreg.describe(),
            "gestures": list(GESTURES),
            "cameras": sorted(self.link.cameras),
            "table_z": self.cfg.table_z,
            # the envelope every joint command is held to, for whoever plans them
            "clearance": {"floor": self.cfg.floor_clearance, "body": self.cfg.body_clearance, "min_y": self.cfg.min_y, "limit_margin": self.cfg.limit_margin},
        }

    # ---- tools

    async def call_tool(self, name: str, args: dict, source: str = "api") -> dict:
        t = toolreg.TOOLS.get(name)
        if t is None:
            return {"ok": False, "error": f"unknown tool '{name}'", "summary": f"unknown tool '{name}'"}
        call_id = uuid.uuid4().hex[:10]
        t0 = time.monotonic()

        def finish(ok: bool, summary: str, error: str | None = None, data: dict | None = None, images: list[bytes] | None = None) -> dict:
            self.emit("tool_end", call_id=call_id, tool=name, ok=ok, summary=summary, error=error,
                      duration_ms=int((time.monotonic() - t0) * 1000), source=source)  # fmt: skip
            out = {"ok": ok, "summary": summary}
            if error:
                out["error"] = error
            if data:
                out["data"] = data
            if images:
                out["images"] = [{"mime": "image/jpeg", "b64": base64.b64encode(j).decode()} for j in images]
            return out

        try:
            kwargs = toolreg.coerce(t, args)
        except RobotError as e:
            self.emit("tool_start", call_id=call_id, tool=name, args=args, source=source)
            return finish(False, str(e), str(e))
        self.emit("tool_start", call_id=call_id, tool=name, args=kwargs, source=source)

        if t.motion and self._task_lock.locked():
            return finish(False, f"busy: the arm is in the middle of '{self.robot.activity}'. Wait, or call stop() first.", "busy")
        try:
            if t.motion:
                async with self._task_lock:
                    self._current_call = call_id
                    self.robot.source = source
                    try:
                        res = await t.fn(self.skills, **kwargs)
                    finally:
                        self._current_call = None
                        self.robot.source = None
                        self.robot.activity = "idle"
                        self.emit("plan", call_id=call_id, path=[], target=None)
            else:
                res = await t.fn(self.skills, **kwargs)
            return finish(True, res.summary, data=res.data, images=res.images)
        except Stopped:
            return finish(False, "stopped before it finished; the arm is holding position", "stopped")
        except RobotError as e:
            return finish(False, str(e), str(e))
        except Exception as e:  # a bug, not a robot condition: surface it without hiding the type
            log.exception("tool %s crashed", name)
            return finish(False, f"internal error in {name}: {type(e).__name__}: {e}", "internal")

    # ---- joint commands: all the agent workspace can ask of the arm

    async def joint_command(self, body: dict) -> dict:
        """Carry out one joint command: waypoints of seven joint angles (rad) and/or a jaw opening.

        No poses, no inverse kinematics and no named skills cross this line. What does is held
        to the joint limits and the clearance envelope, refused while a servo reports a fault,
        timed with the speed profile and followed under the stall guard.
        """
        robot = self.robot
        source = str(body.get("source") or "agent")
        label = str(body.get("label") or "joint command")[:48]
        own = body.get("call_id") is None  # from the agent's own code, not from inside a workspace tool
        call_id = str(body.get("call_id") or uuid.uuid4().hex[:10])
        t0 = time.monotonic()
        if own:
            self.emit("tool_start", call_id=call_id, tool="joint_command", source=source,
                      args={k: body[k] for k in ("path", "gripper", "speed") if body.get(k) is not None})  # fmt: skip

        def done(error: str | None = None, kind: str = "robot") -> dict:
            if own:
                self.emit("tool_end", call_id=call_id, tool="joint_command", ok=error is None, summary=error or label,
                          error=kind if error else None, duration_ms=int((time.monotonic() - t0) * 1000), source=source)  # fmt: skip
            if error:
                return {"ok": False, "error": error, "kind": kind}
            snap = robot.snapshot()
            return {"ok": True, "joints": [j["q"] for j in snap["joints"][:7]], "gripper": snap["gripper"], "tcp": snap["ee"]}

        try:
            path, gripper, speed = self._parse_joint_command(body)
        except (TypeError, ValueError) as e:
            return done(str(e), "invalid")
        if self._task_lock.locked():
            return done(f"busy: the arm is in the middle of '{robot.activity}'. Wait, or stop it first.", "busy")
        async with self._task_lock:
            self._current_call, robot.source = call_id, source
            try:
                faulted = [i for i, m in sorted(self.link.motors.items()) if m.status]
                if faulted:
                    # A servo in overload protection gives 20 % torque: the joint sags and the arm no
                    # longer goes where it is sent. Moving on from there is how things get hit.
                    raise RobotError(f"servo fault on joint(s) {faulted} (status 32 = overload). Do not move: a person must hold "
                                     "the arm while torque is switched off and on again (POST /api/torque) to clear it.")  # fmt: skip
                if path is not None:
                    await self._follow(path, speed, bool(body.get("check", True)), bool(body.get("allow_low", False)), label)
                if gripper is not None:
                    # fully closed is a grasp: shut until the jaws stall, then hold just past that point
                    await (robot.grasp() if gripper <= 0.0 else robot.set_gripper(gripper, label=label))
            except Stopped:
                return done("stopped before it finished; the arm is holding position", "stopped")
            except RobotError as e:
                return done(str(e))
            finally:
                self._current_call, robot.source, robot.activity = None, None, "idle"
                self.emit("plan", call_id=call_id, path=[], target=None)
                if self.workspace is not None:
                    self.workspace.publish()  # perception/ shows the outcome at once
        return done()

    def _parse_joint_command(self, body: dict) -> tuple[np.ndarray | None, float | None, float]:
        robot = self.robot
        why = robot.why_not_ready()
        if why:
            raise ValueError(why)
        path = body.get("path")
        if path is not None:
            path = np.asarray(path, dtype=float)
            if path.ndim != 2 or path.shape[1] != 7 or not np.all(np.isfinite(path)):
                raise ValueError("path must be a list of waypoints, each seven finite joint angles in radians (servos 1..7)")
            lo, hi = robot.kin.lower, robot.kin.upper  # the calibrated limits (why_not_ready refreshed them)
            out = np.argwhere((path < lo - 1e-6) | (path > hi + 1e-6))
            if len(out):
                k, j = (int(v) for v in out[0])
                raise ValueError(f"waypoint {k}: joint {j + 1} at {path[k, j]:+.3f} rad is outside its limits [{lo[j]:+.3f}, {hi[j]:+.3f}]")
        gripper = body.get("gripper")
        if gripper is not None:
            gripper = float(gripper)
            if not 0.0 <= gripper <= 1.0:
                raise ValueError("gripper is an opening fraction between 0 (closed) and 1 (open)")
        if path is None and gripper is None:
            raise ValueError("a joint command needs a path, a gripper opening, or both")
        speed = float(body.get("speed", 0.8))
        if not speed > 0:
            raise ValueError("speed must be positive")
        return path, gripper, speed

    async def _follow(self, path: np.ndarray, speed: float, check: bool, allow_low: bool, label: str) -> None:
        robot = self.robot
        if len(path) == 1:
            await robot.move_joints(path[0], speed=speed, check=check, allow_low=allow_low, label=label)
            return
        lead = interpolate_joint(robot._plan_start(), path[0])  # from where the arm is to the first waypoint
        full = np.vstack([lead[:-1], path])
        if check:
            # as for a single target: the pose the arm starts in may be one it could not be sent to
            why = robot.check_path(full[max(1, (len(lead) - 1) // 3) :], allow_low=allow_low)
            if why:
                raise RobotError(f"move rejected: {why}")
        await robot.move_path(full, speed=speed, label=label)

    async def stop(self) -> None:
        await self.robot.stop()
        self.emit("log", level="warn", text="STOP: motion halted, holding position")


def create_app(cfg: Config | None = None) -> FastAPI:
    cfg = cfg or Config.load()
    hub = Hub(cfg)

    async def state_loop() -> None:
        while True:
            await asyncio.sleep(1 / 30)
            if hub.sockets:
                try:
                    hub._broadcast(hub.robot.snapshot())
                except Exception:
                    log.exception("state broadcast failed")

    async def health_loop() -> None:
        was = None
        while True:
            await asyncio.sleep(0.5)
            now = hub.link.connected
            if now != was:
                hub.emit("log", level="info" if now else "warn", text="arm connected" if now else "arm disconnected: no fresh servo state")
                was = now

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        await hub.link.start()
        tasks = [asyncio.create_task(state_loop()), asyncio.create_task(health_loop())]
        if hub.workspace is not None:
            tasks.append(asyncio.create_task(hub.workspace.publish_loop()))
        if hub.voice is not None:
            tasks.append(asyncio.create_task(_start_voice(hub)))
        yield
        for t in tasks:
            t.cancel()
        if hub.voice is not None:
            try:
                await hub.voice.stop()
            except Exception:
                log.exception("voice stop failed")
        # Leave torque as it is: dropping it here would let a raised arm fall.
        await hub.link.stop()

    app = FastAPI(title="ElRobot daemon", lifespan=lifespan)
    app.state.hub = hub

    # The voice agent is optional: without its package the daemon still serves everything else.
    # Its routes must be registered before the catch-all static mount at "/" below.
    if cfg.voice:
        try:
            from .voice.service import VoiceService, create_router

            hub.voice = VoiceService(hub.emit_dict, stop_robot=hub.stop)
            app.include_router(create_router(hub.voice))
        except ImportError as e:
            log.warning("voice agent not available: %s", e)

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        await sock.send_text(json.dumps({**hub.meta(), "events": list(hub.events)}, separators=(",", ":")))
        hub.sockets.add(sock)
        try:
            while True:
                await sock.receive_text()  # clients have nothing to say; this notices disconnects
        except WebSocketDisconnect:
            pass
        finally:
            hub.sockets.discard(sock)

    @app.get("/api/state")
    async def state():
        return hub.robot.snapshot()

    @app.get("/api/meta")
    async def meta():
        return hub.meta()

    @app.get("/api/camera/{name}/frame.jpg")
    async def frame(name: str):
        fr = hub.link.frames.get(name)
        if fr is None:
            raise HTTPException(404, f"no frame from camera '{name}'")
        return Response(fr.jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/api/camera/{name}/stream.mjpg")
    async def stream(name: str):
        async def gen():
            last = -1
            idle = 0.0
            while True:
                fr = hub.link.frames.get(name)
                if fr is not None and fr.seq != last:
                    last = fr.seq
                    idle = 0.0
                    yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(fr.jpeg)).encode() + b"\r\n\r\n" + fr.jpeg + b"\r\n"
                else:
                    idle += 0.02
                    if idle > 30:
                        return
                await asyncio.sleep(0.02)

        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame", headers={"Cache-Control": "no-store"})

    @app.post("/api/tools/{name}")
    async def run_tool(name: str, request: Request):
        try:
            body = await request.json()
        except Exception:
            body = {}
        return JSONResponse(await hub.call_tool(name, body.get("args") or {}, body.get("source") or "api"))

    @app.post("/api/joints")
    async def joints(request: Request):
        try:
            body = await request.json()
        except Exception:
            body = None
        if not isinstance(body, dict):
            raise HTTPException(400, "body must be a JSON object")
        return JSONResponse(await hub.joint_command(body))

    @app.post("/api/event")
    async def event(request: Request):
        """Lets the workspace's tools show on the timeline and the 3D view. Display only."""
        body = await request.json()
        kind = body.pop("kind", None) if isinstance(body, dict) else None
        if kind not in ("tool_start", "tool_end", "markers", "detections", "log"):
            raise HTTPException(400, "kind must be one of tool_start, tool_end, markers, detections, log")
        hub.emit(kind, **{k: v for k, v in body.items() if k not in ("type", "id", "t")})
        return {"ok": True}

    @app.post("/api/stop")
    async def stop():
        await hub.stop()
        return {"ok": True}

    @app.post("/api/torque")
    async def torque(request: Request):
        body = await request.json()
        try:
            if body.get("enable"):
                await hub.robot.enable()
            else:
                await hub.robot.release()
        except RobotError as e:
            return JSONResponse({"ok": False, "error": str(e)})
        hub.emit("log", level="info", text="torque on" if body.get("enable") else "torque off: the arm is limp")
        return {"ok": True}

    app.mount("/assets", StaticFiles(directory=cfg.assets_dir), name="assets")

    dist: Path = cfg.web_dist
    if dist.exists():
        app.mount("/", StaticFiles(directory=dist, html=True), name="web")
    else:
        @app.get("/")
        async def root():
            return {"ok": True, "note": "web UI not built yet (run `npm run build` in web/)", "api": "/api/meta"}

    return app


async def _start_voice(hub: Hub) -> None:
    try:
        await hub.voice.start()
        hub.emit("log", level="info", text="voice agent ready")
    except Exception as e:
        log.exception("voice agent failed to start")
        hub.emit("log", level="error", text=f"voice agent failed to start: {e}")

