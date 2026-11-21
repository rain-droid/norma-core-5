"""What runs a tool: `pixi run python tools/NAME.py '{"x": 0.1}'`, or `call(function, {...})`.

A tool is an ordinary function, `def NAME(ctx, ...) -> dict`, in tools/NAME.py. It runs right
here, in the shell that started it. Nothing is handed to a server: the only thing a tool can do
to the robot is send joint commands through `ctx.arm` (util/arm.py).

Around each call this file keeps the record: a numbered folder in tool_call_history/ with
`input.json`, `result.json`, `joint_commands.jsonl` (every command sent) and the images made.
"""

import argparse
import inspect
import json
import os
import sys
import time
import traceback
import uuid
from pathlib import Path

import cv2
import numpy as np

from util import tracker
from util.arm import Arm, RobotError

ROOT = Path(__file__).resolve().parents[2]


class Context:
    """What a tool is handed as `ctx`."""

    def __init__(self, tool: str, history: Path, arm: Arm):
        self.tool = tool
        self.root = ROOT
        self.history = history  # this call's folder under tool_call_history/
        self.arm = arm  # the robot: state and frames in, joint commands out
        self.images: list[dict] = []
        self.calls: list[str] = []  # other tools this one ran, in order
        self._plan = None

    @property
    def plan(self):
        """IK and straight-line planning (util/planning.py), built on first use."""
        if self._plan is None:
            from util.planning import Planner

            self._plan = Planner(self.arm)
        return self._plan

    def frame(self, camera: str = "top", settle: float = 0.35) -> np.ndarray:
        return self.arm.frame(camera, settle)

    def save_image(self, filename: str, img: np.ndarray, caption: str = "") -> str:
        """Write an image into this call's history folder; returns its workspace-relative path."""
        path = self.history / filename
        cv2.imwrite(str(path), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        rel = str(path.relative_to(self.root))
        self.images.append({"path": rel, "caption": caption})
        return rel

    def save_json(self, filename: str, obj) -> str:
        path = self.history / filename
        path.write_text(json.dumps(obj, indent=1, default=_jsonable) + "\n")
        return str(path.relative_to(self.root))

    def emit(self, kind: str, **fields) -> None:
        """Markers and detections for the web UI's 3D view and camera panel."""
        self.arm.emit(kind, **fields)

    def call(self, function, **arguments) -> dict:
        """Run another tool's function as part of this call (same history folder)."""
        self.calls.append(function.__name__)
        return function(self, **arguments)


def _jsonable(v):
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, np.generic):
        return v.item()
    if isinstance(v, Path):
        return str(v)
    raise TypeError(f"not JSON serialisable: {type(v).__name__}")


def _new_history(name: str) -> Path:
    folder = ROOT / "tool_call_history"
    folder.mkdir(exist_ok=True)
    n = max([int(p.name[:6]) for p in folder.iterdir() if p.name[:6].isdigit()], default=0)
    while True:
        n += 1
        try:
            (folder / f"{n:06d}_{name}").mkdir()
            return folder / f"{n:06d}_{name}"
        except FileExistsError:
            pass


def _signature(function) -> str:
    return f"{function.__name__}({', '.join(str(p) for p in list(inspect.signature(function).parameters.values())[1:])})"


def call(function, arguments: dict | None = None) -> dict:
    """Run one tool call end to end and return its result. The result has no generic success
    flag: it is the tool's own measurements, or an `error` with an `error_kind`."""
    name = function.__name__
    arguments = dict(arguments or {})
    source = os.environ.get("ELROBOT_SOURCE", "agent")
    history = _new_history(name)
    call_id = uuid.uuid4().hex[:10]
    (history / "input.json").write_text(json.dumps({"tool": name, "arguments": arguments, "source": source, "t": time.time()}, indent=1) + "\n")
    arm = Arm(source=source, call_id=call_id, log=history / "joint_commands.jsonl")
    ctx = Context(name, history, arm)
    arm.emit("tool_start", call_id=call_id, tool=name, args=arguments, source=source)
    t0 = time.monotonic()
    try:
        try:
            inspect.signature(function).bind(ctx, **arguments)
        except TypeError as e:
            raise RobotError(f"bad arguments for {_signature(function)}: {e}") from None
        out = function(ctx, **arguments)
        result = dict(out) if isinstance(out, dict) else {"result": out}
    except RobotError as e:
        result = {"error": str(e), "error_kind": "robot"}
        if tracker.stopped_by_tracker(e):
            result["error"] += " The tracker stopped it, because something moved in the work area: see tools/track_objects.py."
    except Exception as e:  # a bug in a tool file: whoever edits it needs the traceback
        result = {"error": f"{type(e).__name__}: {e}", "error_kind": "tool_bug", "traceback": traceback.format_exc(limit=8)}
    finally:
        arm.release_control()  # control asked for by this call ends with it (a session's stays)
    result["history"] = str(history.relative_to(ROOT))
    if ctx.images:
        result["images"] = ctx.images
    if ctx.calls:
        result["nested_calls"] = ctx.calls
    result["joint_command_count"] = arm.sent
    result["duration_s"] = round(time.monotonic() - t0, 2)
    try:
        text = json.dumps(result, indent=1, default=_jsonable)
    except TypeError as e:
        text = json.dumps({"error": f"{name} returned something that is not JSON: {e}", "error_kind": "tool_bug", "history": result["history"]}, indent=1)
    (history / "result.json").write_text(text + "\n")
    payload = json.loads(text)
    ok = "error" not in payload
    arm.emit("tool_end", call_id=call_id, tool=name, ok=ok, summary=payload.get("summary") or (f"{name} done" if ok else payload["error"]),
             error=payload.get("error_kind"), duration_ms=int((time.monotonic() - t0) * 1000), source=source)  # fmt: skip
    return payload


def run_cli(function) -> None:
    name = function.__name__
    module = sys.modules[function.__module__]
    parser = argparse.ArgumentParser(prog=f"tools/{name}.py", description=inspect.getdoc(module), formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("arguments", nargs="?", default="{}", help="JSON object of arguments, or - to read it from stdin")
    parser.add_argument("--schema", action="store_true", help="print this tool's signature and description")
    args = parser.parse_args()
    if args.schema:
        print(json.dumps({"tool": name, "signature": _signature(function), "description": inspect.getdoc(module)}, indent=1))
        return
    try:
        arguments = json.loads(sys.stdin.read() if args.arguments == "-" else args.arguments)
    except json.JSONDecodeError as e:
        parser.error(f"arguments are not valid JSON: {e}")
    if not isinstance(arguments, dict):
        parser.error("arguments must be a JSON object")
    result = call(function, arguments)
    print(json.dumps(result, indent=1, ensure_ascii=False), flush=True)
    if "error" in result:
        raise SystemExit(1)
