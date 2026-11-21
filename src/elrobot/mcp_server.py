"""MCP server (stdio) that exposes the robot's tools to Claude Code and other MCP clients.

It is a thin shim: tool definitions come from `elrobot.tools`, and every call is forwarded to
the daemon over HTTP, so the web UI shows what the agent is doing and only one process ever
commands the arm.

    claude mcp add elrobot -- uv run --project /path/to/elrobot-agent elrobot-mcp
"""

from __future__ import annotations

import asyncio
import os

import httpx
import mcp.types as types
from mcp.server import Server
from mcp.server.stdio import stdio_server

from . import tools as toolreg

DAEMON = os.environ.get("ELROBOT_DAEMON", "http://127.0.0.1:8765")
SOURCE = os.environ.get("ELROBOT_SOURCE", "claude-code")

INSTRUCTIONS = """\
You are controlling a real robot arm (NormaCore ElRobot: 7 joints and a parallel gripper) sitting on a table, \
with a camera on its wrist. Coordinates are metres in the robot's base frame: origin on the table under the \
base, x to the robot's right, y forward away from the base, z up.

How to work:
- Start a task with observe(): it raises the camera, and returns an image with every object on the table \
outlined and numbered, plus each one's position. Identify the thing you want from the image, then pass its \
number to pick(), touch() or point_at().
- Tools tell you what actually happened. pick() verifies the grasp; if it reports that the jaws closed on \
nothing or the object slipped, believe it, look again with observe() or scan(), and retry.
- After picking, place(x, y) puts the object down. For stacking, set z to the height of the object underneath.
- The arm reaches about 38 cm from its base across the table; straight-down grasps work within about 27 cm.
- stop() halts motion at once. rest() folds the arm and turns the motors off when you are finished.
- Positions come from one camera and are approximate until the camera is calibrated; pick() corrects for this \
by looking again from close up, so prefer it over hand-built move_to sequences.
"""

async def on_list_tools(ctx, params) -> types.ListToolsResult:
    return types.ListToolsResult(
        tools=[types.Tool(name=t.name, description=t.description, input_schema=t.input_schema) for t in toolreg.TOOLS.values()]
    )


async def on_call_tool(ctx, params: types.CallToolRequestParams) -> types.CallToolResult:
    def text(msg: str, error: bool = False) -> types.CallToolResult:
        return types.CallToolResult(content=[types.TextContent(type="text", text=msg)], is_error=error)

    try:
        async with httpx.AsyncClient(timeout=httpx.Timeout(180.0, connect=3.0)) as client:
            r = await client.post(f"{DAEMON}/api/tools/{params.name}", json={"args": params.arguments or {}, "source": SOURCE})
            r.raise_for_status()
            res = r.json()
    except httpx.ConnectError:
        return text(f"The robot daemon is not running at {DAEMON}. Start it with `uv run elrobot up` in the elrobot-agent repo, then try again.", True)
    except httpx.HTTPError as e:
        return text(f"Could not reach the robot daemon: {type(e).__name__}: {e}", True)

    ok = bool(res.get("ok"))
    msg = res.get("summary") or res.get("error") or ""
    content: list = [types.TextContent(type="text", text=msg if ok else f"FAILED: {msg}")]
    for img in res.get("images") or []:
        content.append(types.ImageContent(type="image", data=img["b64"], mime_type=img.get("mime", "image/jpeg")))
    # A failed robot action is a normal result the agent should read and react to, so it is
    # not flagged as a protocol-level error.
    return types.CallToolResult(content=content, is_error=False)


server = Server("elrobot", instructions=INSTRUCTIONS, on_list_tools=on_list_tools, on_call_tool=on_call_tool)


async def _run() -> None:
    async with stdio_server() as (read, write):
        await server.run(read, write, server.create_initialization_options())


def main() -> None:
    asyncio.run(_run())


if __name__ == "__main__":
    main()
