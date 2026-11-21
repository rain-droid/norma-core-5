"""The daemon side of the agent workspace (`workspace/`, after RoboHarness V2).

The workspace is a directory a coding agent is started in. Its `tools/NAME.py` files hold the
algorithms; running one from a shell sends the call here, and this process, the only one that
owns the arm, imports that same file and runs it. Nothing about a tool lives in the daemon:
edit the file and the next call runs the new code.

This module is the plumbing around a call: loading the file, the one-motion-at-a-time lock,
the `tool_call_history/` folder, and publishing `perception/`.
"""

from __future__ import annotations

import asyncio
import importlib.util
import inspect
import json
import logging
import math
import os
import re
import sys
import time
import traceback
import uuid
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from . import vision
from .robot import RobotError, Stopped

if TYPE_CHECKING:
    from .daemon import Hub

log = logging.getLogger("elrobot.workspace")

_NAME = re.compile(r"[a-z][a-z0-9_]{0,63}")
PUBLISH_HZ = 2.0


def write_json(path: Path, obj: Any) -> None:
    """Atomic, so a reader never sees half a file."""
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(json.dumps(obj, indent=1, default=_jsonable) + "\n")
    os.replace(tmp, path)


def write_bytes(path: Path, data: bytes) -> None:
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def _jsonable(v: Any) -> Any:
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, np.generic):
        return v.item()
    if isinstance(v, Path):
        return str(v)
    raise TypeError(f"not JSON serialisable: {type(v).__name__}")


class ToolContext:
    """What a workspace tool is handed as `ctx`."""

    def __init__(self, ws: "Workspace", name: str, history: Path):
        self.ws = ws
        self.tool = name
        self.root = ws.root
        self.history = history  # this call's folder under tool_call_history/
        self.robot = ws.hub.robot  # elrobot.robot.Robot: the only thing that commands servos
        self.skills = ws.hub.skills  # elrobot.skills.Skills: the MCP stack's behaviours
        self.cfg = ws.hub.cfg
        self.emit = ws.hub.emit  # events for the web UI (markers, detections, log)
        self.images: list[dict] = []
        self.calls: list[str] = []  # nested tool calls, in order

    async def frame(self, camera: str = "wrist", settle: float = 0.35) -> np.ndarray:
        """Newest BGR frame from a camera, taken `settle` seconds from now so the arm is still."""
        return await self.skills._fresh_frame(camera, settle)

    def save_image(self, filename: str, img: np.ndarray, caption: str = "") -> str:
        """Write an image into this call's history folder; returns its workspace-relative path."""
        path = self.history / filename
        write_bytes(path, vision.encode(img, 90))
        rel = str(path.relative_to(self.root))
        self.images.append({"path": rel, "caption": caption})
        return rel

    def save_json(self, filename: str, obj: Any) -> str:
        path = self.history / filename
        write_json(path, obj)
        return str(path.relative_to(self.root))

    async def call(self, tool: str, **arguments) -> dict:
        """Run another workspace tool inside this call (same history folder, same lock)."""
        self.calls.append(tool)
        return await self.ws.function(tool)(self, **arguments)


class Workspace:
    def __init__(self, hub: "Hub", root: Path):
        self.hub = hub
        self.root = root.resolve()
        self.tools_dir = self.root / "tools"
        self.history_dir = self.root / "tool_call_history"
        self.perception_dir = self.root / "perception"
        self._stamp: float | None = None
        self._modules: dict[str, Any] = {}
        self._sequence = 0
        self._last_call: dict | None = None
        self.history_dir.mkdir(exist_ok=True)
        self._counter = max([int(p.name[:6]) for p in self.history_dir.iterdir() if p.name[:6].isdigit()], default=0)
        # Tools import their shared code as `util.*`, the same way they do when run from a shell.
        sys.path.insert(0, str(self.tools_dir))

    # ---- loading

    def names(self) -> list[str]:
        return sorted(p.stem for p in self.tools_dir.glob("*.py") if _NAME.fullmatch(p.stem))

    def function(self, name: str):
        """The coroutine function `name` from tools/name.py, re-imported if any tool file changed."""
        if not _NAME.fullmatch(name) or not (self.tools_dir / f"{name}.py").is_file():
            raise RobotError(f"no tool '{name}' in tools/ (have: {', '.join(self.names())})")
        stamp = max(p.stat().st_mtime for p in self.tools_dir.rglob("*.py"))
        if stamp != self._stamp:
            self._modules.clear()
            for mod in [m for m in sys.modules if m == "util" or m.startswith("util.")]:
                del sys.modules[mod]
            self._stamp = stamp
        mod = self._modules.get(name)
        if mod is None:
            spec = importlib.util.spec_from_file_location(f"workspace_tool_{name}", self.tools_dir / f"{name}.py")
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._modules[name] = mod
        fn = getattr(mod, name, None)
        if not inspect.iscoroutinefunction(fn):
            raise RobotError(f"tools/{name}.py must define `async def {name}(ctx, ...)`")
        return fn

    # ---- calling

    async def call(self, name: str, args: dict, source: str = "agent") -> dict:
        """Run one tool call end to end. The result has no generic success flag: it is the
        tool's own measurements, or an `error`."""
        hub = self.hub
        call_id = uuid.uuid4().hex[:10]
        t0 = time.monotonic()
        self._counter += 1
        history = self.history_dir / f"{self._counter:06d}_{name if _NAME.fullmatch(name) else 'unknown'}"
        history.mkdir()
        write_json(history / "input.json", {"tool": name, "arguments": args, "source": source, "t": time.time()})
        ctx = ToolContext(self, name, history)
        commands: list[dict] = []
        hub.emit("tool_start", call_id=call_id, tool=name, args=args, source=source)

        def record(cmd: dict) -> None:
            commands.append(cmd)
            with (history / "joint_commands.jsonl").open("a") as f:
                f.write(json.dumps({"t": round(time.time(), 3), **cmd}) + "\n")

        result: dict
        locked = False
        try:
            fn = self.function(name)
            try:
                inspect.signature(fn).bind(ctx, **args)
            except TypeError as e:
                raise RobotError(f"bad arguments for {name}{inspect.signature(fn)}: {e}") from None
            motion = getattr(self._modules[name], "MOTION", True)
            if motion:
                if hub._task_lock.locked():
                    raise RobotError(f"busy: the arm is in the middle of '{hub.robot.activity}'. Wait, or run tools/stop.py first.")
                await hub._task_lock.acquire()
                locked = True
                hub._current_call = call_id
                hub.robot.source = source
                hub.robot.on_command = record
            out = await fn(ctx, **args)
            result = dict(out) if isinstance(out, dict) else {"result": out}
        except Stopped:
            result = {"error": "stopped before it finished; the arm is holding position", "error_kind": "stopped"}
        except RobotError as e:
            result = {"error": str(e), "error_kind": "robot"}
        except Exception as e:  # a bug in a tool file: the agent needs the traceback to fix it
            log.exception("workspace tool %s crashed", name)
            result = {"error": f"{type(e).__name__}: {e}", "error_kind": "tool_bug", "traceback": traceback.format_exc(limit=8)}
        finally:
            if locked:
                hub.robot.on_command = None
                hub._current_call = None
                hub.robot.source = None
                hub.robot.activity = "idle"
                hub.emit("plan", call_id=call_id, path=[], target=None)
                hub._task_lock.release()

        result["history"] = str(history.relative_to(self.root))
        if ctx.images:
            result["images"] = ctx.images
        if ctx.calls:
            result["nested_calls"] = ctx.calls
        result["joint_command_count"] = len(commands)
        result["duration_s"] = round(time.monotonic() - t0, 2)
        try:
            write_json(history / "result.json", result)
            payload = json.loads((history / "result.json").read_text())
        except TypeError as e:
            payload = {"error": f"{name} returned something that is not JSON: {e}", "error_kind": "tool_bug", "history": result["history"]}
            write_json(history / "result.json", payload)
        self._last_call = {"history": payload["history"], "tool": name, "error": payload.get("error")}
        self.publish()
        ok = "error" not in payload
        summary = payload.get("summary") or (f"{name} done" if ok else payload["error"])
        hub.emit("tool_end", call_id=call_id, tool=name, ok=ok, summary=summary, error=payload.get("error_kind"),
                 duration_ms=int((time.monotonic() - t0) * 1000), source=source)  # fmt: skip
        return payload

    # ---- perception as files

    def state(self) -> dict:
        """Measured robot state, in the units the tools take (radians, metres, 0..1 gripper)."""
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
                "last_call": self._last_call,
            },
        )

    async def publish_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0 / PUBLISH_HZ)
            try:
                self.publish()
            except Exception:
                log.exception("perception publish failed")
