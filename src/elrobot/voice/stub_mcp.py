"""Fake `elrobot` MCP server for developing the voice layer without a robot.

Run as a stdio MCP server:  python -m elrobot.voice.stub_mcp

Every tool sleeps for a second or two like a real motion and answers in the
daemon's result shape (`{"ok": ..., "summary": ..., "error": ...}`) so the
voice agent sees what it will see from the real server. `pick` fails the first
time it is called for a given target and succeeds the second, which is what
exercises the mess-up-and-retry behaviour.

Env:
  ELROBOT_STUB_DELAY   multiply every sleep (default 1.0; 0 for instant tools)
  ELROBOT_STUB_LOG     append one line per tool call to this file
"""

from __future__ import annotations

import asyncio
import io
import json
import os
import struct
import time
import zlib

try:  # mcp >= 2: FastMCP was renamed MCPServer
    from mcp.server.mcpserver import Image
    from mcp.server.mcpserver import MCPServer as FastMCP
except ModuleNotFoundError:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP, Image  # type: ignore[no-redef]

mcp = FastMCP(
    "elrobot",
    instructions=(
        "Tools for a small 7-joint robot arm with a gripper and a wrist camera, sitting on a table."
    ),
)

_DELAY = float(os.environ.get("ELROBOT_STUB_DELAY", "1.0"))
_LOG = os.environ.get("ELROBOT_STUB_LOG")
_pick_attempts: dict[str, int] = {}
_holding: str | None = None

# What the fake camera can see: name -> (x, y) on the table in metres, colour.
_SCENE: dict[str, tuple[tuple[float, float], tuple[int, int, int]]] = {
    "phone": ((0.08, 0.24), (40, 40, 48)),
    "red cup": ((-0.10, 0.28), (200, 40, 40)),
    "banana": ((0.02, 0.33), (235, 205, 60)),
    "marker": ((-0.04, 0.20), (40, 90, 200)),
}
GESTURES = ["dap", "wave", "nod", "shake_head", "dance", "point"]


def _log(line: str) -> None:
    if _LOG:
        with open(_LOG, "a") as fh:
            fh.write(f"{time.time():.3f} {line}\n")


async def _move(seconds: float) -> None:
    await asyncio.sleep(seconds * _DELAY)


def _ok(summary: str, **data: object) -> str:
    return json.dumps({"ok": True, "summary": summary, "data": data})


def _fail(error: str, summary: str) -> str:
    return json.dumps({"ok": False, "error": error, "summary": summary})


def _match(name: str) -> str | None:
    wanted = name.lower().strip()
    for known in _SCENE:
        if known in wanted or wanted in known:
            return known
    return None


def _png(width: int, height: int, pixel) -> bytes:
    """Encode an RGB image with nothing but zlib (no Pillow needed here)."""
    raw = io.BytesIO()
    for y in range(height):
        raw.write(b"\x00")
        for x in range(width):
            raw.write(bytes(pixel(x, y)))

    def chunk(tag: bytes, body: bytes) -> bytes:
        return (
            struct.pack(">I", len(body))
            + tag
            + body
            + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF)
        )

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(raw.getvalue(), 6))
        + chunk(b"IEND", b"")
    )


def _camera_frame() -> bytes:
    """A 160x120 top-down cartoon of the table: wood background, one blob per object."""
    width, height = 160, 120
    blobs = []
    for name, ((x, y), colour) in _SCENE.items():
        if name == _holding:
            continue
        u = int(width / 2 + x * 400)
        v = int(height - (y - 0.12) * 400)
        blobs.append((u, v, colour))

    def pixel(px: int, py: int) -> tuple[int, int, int]:
        for u, v, colour in blobs:
            if abs(px - u) <= 9 and abs(py - v) <= 7:
                return colour
        return (190, 160, 120) if (px // 20 + py // 20) % 2 else (180, 150, 112)

    return _png(width, height, pixel)


@mcp.tool()
async def look() -> list:
    """Take a picture with the wrist camera and return it with a short description."""
    _log("look")
    await _move(0.6)
    visible = [name for name in _SCENE if name != _holding]
    text = _ok(
        "Camera frame captured. Roughly visible on the table: " + ", ".join(visible) + ".",
        objects=visible,
    )
    return [text, Image(data=_camera_frame(), format="png")]


@mcp.tool()
async def find(prompt: str) -> str:
    """Locate an object on the table by description. Returns its position in metres."""
    _log(f"find {prompt!r}")
    await _move(1.0)
    name = _match(prompt)
    if name is None or name == _holding:
        return _fail("not_found", f"Could not see anything matching '{prompt}' on the table.")
    (x, y), _ = _SCENE[name]
    return _ok(f"Found the {name}.", label=name, pos=[x, y, 0.02], score=0.91)


@mcp.tool()
async def pick(target: str) -> str:
    """Pick up the named object and hold it in the gripper."""
    global _holding
    _log(f"pick {target!r}")
    name = _match(target)
    if name is None:
        await _move(0.8)
        return _fail("not_found", f"Could not see '{target}' to pick it up.")
    await _move(2.0)
    _pick_attempts[name] = _pick_attempts.get(name, 0) + 1
    if _pick_attempts[name] == 1:
        return _fail(
            "grasp_failed",
            f"Closed the gripper on the {name} but it slipped out; nothing is held.",
        )
    _holding = name
    return _ok(f"Picked up the {name}; grasp verified.", holding=name)


@mcp.tool()
async def place(where: str = "table") -> str:
    """Put down whatever is being held, at a named spot or into the user's hand ('hand')."""
    global _holding
    _log(f"place {where!r}")
    await _move(1.5)
    if _holding is None:
        return _fail("not_holding", "The gripper is empty, there is nothing to place.")
    name, _holding = _holding, None
    return _ok(f"Released the {name} at {where}.")


@mcp.tool()
async def touch(target: str) -> str:
    """Move the fingertip to gently touch the named object."""
    _log(f"touch {target!r}")
    await _move(1.5)
    name = _match(target)
    if name is None:
        return _fail("not_found", f"Could not see '{target}' to touch it.")
    return _ok(f"Touched the {name}.")


@mcp.tool()
async def gesture(name: str) -> str:
    """Perform a canned gesture. Names: dap (fist-bump), wave, nod, shake_head, dance, point."""
    _log(f"gesture {name!r}")
    if name not in GESTURES:
        await _move(0.2)
        return _fail("unknown_gesture", f"No gesture called '{name}'. Known: {', '.join(GESTURES)}.")
    await _move(2.0)
    return _ok(f"Did the {name} gesture.")


@mcp.tool()
async def home() -> str:
    """Move the arm back to its neutral home pose."""
    _log("home")
    await _move(1.2)
    return _ok("Arm is at home.")


@mcp.tool()
async def stop() -> str:
    """Immediately halt all motion and hold position."""
    _log("stop")
    return _ok("Stopped; holding position.")


def main() -> None:
    mcp.run("stdio")


if __name__ == "__main__":
    main()
