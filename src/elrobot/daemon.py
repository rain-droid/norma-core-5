"""The robot daemon: one process that owns the arm and serves docs/API.md.

Clients (the MCP shim for Claude Code, the web UI, the voice agent) all go through here, so
there is a single place that decides what the arm does and a single stream of events.
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

from . import tools as toolreg
from .config import Config
from .kinematics import Kinematics
from .link import Link, SimLink, StationLink
from .robot import Robot, RobotError, Stopped
from .profile import ACTIVE as PROFILE
from .servo import GRIPPER_ID
from .autopilot import Autopilot
from .skills import GESTURES, Skills

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
        self.autopilot = Autopilot(self, cfg)

    def _make_link(self) -> Link:
        cfg = self.cfg
        if cfg.sim:
            kin = Kinematics(cfg.urdf)
            limits = {j.id: (j.lower, j.upper) for j in kin.joints}
            limits[GRIPPER_ID] = (kin.gripper_lower, kin.gripper_upper)
            link = SimLink(limits)
            cal = PROFILE.load_calibration()
            if cal:  # lay the simulated servos out the way the real calibration does
                for i, c in cal.items():
                    m = link.motors[i]
                    m.range_min, m.range_max = int(c["range_min"]), int(c["range_max"])
                    start = 2048 if i != GRIPPER_ID else int(c["range_max"])
                    m.tick = m.goal_tick = start
                    link._pos[i] = float(start)
            return link
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
        joints.append({"id": GRIPPER_ID, "name": "rev_motor_08", "lower": 0.0, "upper": 1.0})
        return {
            "type": "hello",
            "version": 1,
            "mode": self.link.mode,
            "urdf": "/assets/" + PROFILE.urdf,
            "robot": PROFILE.name,
            "reach_m": PROFILE.reach_m,
            "joints": joints,
            "tools": toolreg.describe(),
            "gestures": list(GESTURES),
            "cameras": sorted(self.link.cameras),
            "table_z": self.cfg.table_z,
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

    async def stop(self) -> None:
        self.autopilot._pending = None
        await self.autopilot.cancel()
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
        tasks = [asyncio.create_task(state_loop()), asyncio.create_task(health_loop()), asyncio.create_task(hub.autopilot.perception_loop()),
                 asyncio.create_task(hub.autopilot.warm_speech())]
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

    @app.post("/api/autopilot")
    async def autopilot_start(request: Request):
        body = await request.json()
        return JSONResponse(await hub.autopilot.start(body.get("prompt") or ""))

    @app.post("/api/autopilot/stop")
    async def autopilot_stop():
        await hub.stop()
        return {"ok": True}

    @app.get("/api/autopilot")
    async def autopilot_status():
        return hub.autopilot.status()

    @app.get("/api/autopilot/view.jpg")
    async def autopilot_view():
        fr = hub.link.frames.get("wrist")
        if fr is None:
            raise HTTPException(404, "no wrist frame")
        from . import vision

        return Response(vision.encode(hub.autopilot._grid_view(vision.decode(fr.jpeg))), media_type="image/jpeg")

    @app.post("/api/autopilot/hush")
    async def autopilot_hush():
        hub.autopilot.hush()
        return {"ok": True}

    @app.post("/api/autopilot/voice")
    async def autopilot_voice(request: Request):
        return JSONResponse(await hub.autopilot.hear(await request.body(), request.headers.get("content-type"),
                                                     need_name=request.headers.get("x-need-name") == "1"))

    @app.get("/api/autopilot/audio/{clip_id}.wav")
    async def autopilot_audio(clip_id: str):
        tts = hub.autopilot.tts
        path = tts.audio_path(clip_id) if tts is not None else None
        if path is None:
            raise HTTPException(404, "no such clip")
        return Response(path.read_bytes(), media_type="audio/wav")

    @app.get("/auto")
    async def autopilot_page():
        return Response((Path(__file__).parent / "autopilot.html").read_text(), media_type="text/html")

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

