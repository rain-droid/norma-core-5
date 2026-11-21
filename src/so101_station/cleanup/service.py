"""Loopback tool service shared by CLI and MCP clients."""
import asyncio
import fcntl
import hashlib
import inspect
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from pydantic import BaseModel, Field

from .geometry import CleanupError

TOOLS = {
    "status": "Read robot state and cleanup count.",
    "get_observation": "Get a fresh tabletop image and joint state without moving. Use your vision to find paper/wrappers and a table-footprint grasp pixel.",
    "plan_pick": "Plan one crumpled_paper/snack_wrapper pickup from observation_id, confidence >=0.85, grasp_pixel [u,v], scene_clear=true. No movement. Rejects ungrounded/unreachable targets.",
    "execute_plan": "Execute a single-use plan_id slowly. Returns before/after evidence, not an automatic success claim.",
    "verify": "After viewing the after-image, provide run_id, observation_id, source_clear, trash_in_drop_zone, and evidence. Only both visual checks count as cleaned.",
    "stop": "Interrupt all stages and hold measured position. Does not release torque or the object.",
}


class ToolCall(BaseModel):
    args: dict = Field(default_factory=dict)


def create_app(harness):
    registry = getattr(harness, "tools", TOOLS)
    @asynccontextmanager
    async def lifespan(app):
        lease = None
        try:
            if harness.backend.mode == "real":
                identity = f"{harness.backend.host}/{harness.backend.bus}".encode()
                name = hashlib.sha256(identity).hexdigest()[:16]
                lease = open(Path("/tmp") / f"so101-cleanup-{name}.lock", "a")
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            await harness.backend.start()
            yield
        finally:
            if harness.busy:
                try:
                    await harness.stop()
                except Exception:
                    pass  # Preserve the original shutdown error; never torque-off.
            await harness.backend.close()
            if lease:
                lease.close()

    app = FastAPI(title="SO-101 Cleanup Tools", lifespan=lifespan)

    @app.get("/tools")
    async def tools():
        return registry

    @app.post("/tools/{name}")
    async def call(name: str, body: ToolCall):
        if name not in registry:
            return {"ok": False, "error": "unknown tool"}
        try:
            method = getattr(harness, name)
            result = method(**body.args)
            if inspect.isawaitable(result):
                result = await result
            return {"ok": result.get("execution") not in {"rejected", "fault", "stopped"}, "result": result}
        except (CleanupError, TypeError, ValueError, TimeoutError, OSError) as exc:
            return {"ok": False, "error": str(exc) or "timed out"}

    return app
