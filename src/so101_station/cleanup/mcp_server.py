"""MCP image/tools shim for the local SO-101 cleanup service (mcp 2.x)."""
import asyncio
import json
import os

import httpx
import mcp.types as types
from mcp.server import Server
from mcp.server.stdio import stdio_server

from .service import TOOLS

URL = os.environ.get("SO101_CLEANUP_URL", "http://127.0.0.1:8771")
INSTRUCTIONS = """You operate the SO-101 tabletop cleanup robot. Start with status and get_observation.
Use the returned images to identify ONLY discarded crumpled paper or empty snack wrappers. Leave useful items,
uncertain objects, food, liquids and hazardous objects alone. You are the visual recognizer; the tool service
does not run a separate detector. The whole image/path must be clear of people and obstacles before planning.
Select the grasp pixel at the object's table-contact footprint, not a guessed 3D coordinate. The planar map
does not give depth. Flat wrappers without a pinchable edge may be ungraspable: do not force them.
Call plan_pick for one object and inspect the returned path/result. If rejected, do not bypass limits.
execute_plan is single-use and reports motion completion only. Inspect its after-image and, if needed, take
another observation; call verify only with explicit evidence that the source is clear AND the item is in the
drop tray. Never infer success from an HTTP response or gripper contact alone. Reobserve before each new item.
At most two attempts per object; stop on unexpected motion, changed scene, stale state, or repeated failures.
stop interrupts and holds position. Do not issue concurrent motion tools. Synthetic mode tests plumbing and
kinematics, not grasp physics. Real mode requires measured camera/table calibration and bench validation.
"""
STR = {"type": "string"}
SCHEMAS = {
    "status": ({}, []), "get_observation": ({}, []), "stop": ({}, []),
    "plan_pick": ({"observation_id": STR, "label": {"type": "string", "enum": ["crumpled_paper", "snack_wrapper"]},
                   "confidence": {"type": "number", "minimum": .85, "maximum": 1},
                   "grasp_pixel": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2},
                   "scene_clear": {"type": "boolean"}}, ["observation_id", "label", "confidence", "grasp_pixel", "scene_clear"]),
    "execute_plan": ({"plan_id": STR}, ["plan_id"]),
    "verify": ({"run_id": STR, "observation_id": STR, "source_clear": {"type": "boolean"},
                "trash_in_drop_zone": {"type": "boolean"}, "evidence": STR},
               ["run_id", "observation_id", "source_clear", "trash_in_drop_zone", "evidence"]),
}

if os.environ.get("SO101_CLEANUP_MODE") == "session":
    from .session import TOOLS
    INSTRUCTIONS = """Use status and observe first. This is a checkpointed SO-101 session, not a calibrated whole-table picker.
Images are named overhead and wrist; coordinates for nudge are robot-base millimetres. Synthetic images do not prove grasp physics.
Begin a paper target, explicitly enable hold if needed, open and approach the saved hover, then use bounded visual nudges.
Every mutation needs a unique action_id and the current observation_id; retry a network request with the SAME ID and arguments.
Inspect fresh images and set scene_clear only if the motion corridor is clear. Do not guess geometry or bypass rejected paths.
After grasp, lift; assess held/empty/uncertain from both views. Contact is not success. Carry is gated on held and source_clear.
Release, retreat for a clear view, then assess delivered only if source_clear and on_cover are visible. Two grasp attempts maximum.
Stop holds position and latches cancellation. Torque limp is separate and requires physical support. No motion on startup.
Only after an actual operator instruction to continue a held-object task, resume_holding may check fresh stable state
with params operator_confirmed=true and evidence; after restart also supply target_id. It sends no motion and requires
a new held assessment before carry. Never fabricate operator confirmation or automatically clear a fault.
select_target remembers an inspected paper/wrapper patch in one view. locate_target tracks that patch in a new supplied observation.
These tools send no motion or camera request. Positions are IMAGE PIXELS, not robot XYZ or grasp coordinates.
A shifted wrist image can result from camera/arm movement. A lost/expired/changed-view target requires explicit reselection.
For local visual alignment, select a distinctive paper-only wrist bbox and call probe_alignment at clear travel height.
Alternatively select/locate a wrist target in the same observation and pass target_id instead of bbox to probe_alignment.
A probe bound to a target is rejected after that target is reselected, lost or expired; never silently substitute another object.
Start tests with prepare_alignment: it returns to configured folded rest if needed, then goes to ready hover.
Finish every successful test with park using fresh images, empty_gripper_confirmed and scene_clear. Faults hold; never auto-park through a fault.
Inspect the entire folded-rest and hover path, including the gripper. park keeps torque on.
If stopped, operator_confirmed must reflect an actual user instruction, never an assumed permission.
Use axis="x" first for +5mm/return/−5mm/return repeatability; it returns a diagnostic axis_test_id, NEVER a usable alignment map.
Only axis="xy" makes two-axis probes and an independent validation, returning a short-lived probe_id and reference_pixel.
align_target controls that same tracked feature locally to a supplied goal_pixel, without model calls per correction.
A successful return to reference_pixel proves image alignment, NOT that the jaws are over a valid grasp point.
Never invent a grasp reference, infer table contact from no motion, or use a probe after changing camera/height/region.
"""
    common = {"action_id": STR, "observation_id": STR}
    SCHEMAS = {
        "status": ({}, []), "observe": ({}, []), "stop": ({}, []),
        "act": ({**common, "action": {"type": "string", "enum": ["begin", "open", "nudge", "approach", "grasp", "lift", "carry", "release", "retreat", "resume_holding"]},
                 "params": {"type": "object", "properties": {"target_id": STR, "operator_confirmed": {"type": "boolean"}, "evidence": STR, "delta_mm": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}}, "additionalProperties": False},
                 "scene_clear": {"type": "boolean"}}, ["action_id", "observation_id", "action"]),
        "assess": ({**common, "outcome": {"type": "string", "enum": ["held", "empty", "uncertain", "delivered"]}, "evidence": STR,
                    "source_clear": {"type": "boolean"}, "on_cover": {"type": "boolean"}}, ["action_id", "observation_id", "outcome", "evidence"]),
        "set_torque": ({**common, "mode": {"type": "string", "enum": ["hold", "limp"]}, "arm_supported": {"type": "boolean"}}, ["action_id", "observation_id", "mode"]),
        "probe_alignment": ({**common, "axis": {"type": "string", "enum": ["x", "y", "xy"]}, "target_id": STR, "bbox": {"type": "array", "items": {"type": "integer"}, "minItems": 4, "maxItems": 4}, "scene_clear": {"type": "boolean"}, "tracker": {"type": "string", "enum": ["patch", "features"]}}, ["action_id", "observation_id", "scene_clear"]),
        "align_target": ({**common, "probe_id": STR, "goal_pixel": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2}, "scene_clear": {"type": "boolean"}}, ["action_id", "observation_id", "probe_id", "goal_pixel", "scene_clear"]),
        "select_target": ({**common, "target_id": STR, "label": {"type": "string", "enum": ["crumpled_paper", "snack_wrapper"]},
                           "view": {"type": "string", "enum": ["overhead", "wrist"]},
                           "bbox": {"type": "array", "items": {"type": "integer"}, "minItems": 4, "maxItems": 4},
                           "tracker": {"type": "string", "enum": ["patch", "features"]}},
                          ["action_id", "observation_id", "target_id", "label", "view", "bbox"]),
        "locate_target": ({**common, "target_id": STR}, ["action_id", "observation_id", "target_id"]),
        "prepare_alignment": ({**common, "empty_gripper_confirmed": {"type": "boolean"}, "scene_clear": {"type": "boolean"}, "operator_confirmed": {"type": "boolean"}}, ["action_id", "observation_id", "empty_gripper_confirmed", "scene_clear"]),
        "park": ({**common, "empty_gripper_confirmed": {"type": "boolean"}, "scene_clear": {"type": "boolean"}, "operator_confirmed": {"type": "boolean"}}, ["action_id", "observation_id", "empty_gripper_confirmed", "scene_clear"]),
    }


async def on_list_tools(ctx, params):
    return types.ListToolsResult(tools=[types.Tool(name=name, description=TOOLS[name], input_schema={
        "type": "object", "properties": props, "required": required, "additionalProperties": False})
        for name, (props, required) in SCHEMAS.items()])


def content_from_result(result):
    images = []
    def clean(value):
        if isinstance(value, dict):
            images.extend(value.get("images", []))
            return {k: clean(v) for k, v in value.items() if k != "images"}
        if isinstance(value, list):
            return [clean(v) for v in value]
        return value
    cleaned = clean(result)
    return [types.TextContent(type="text", text=json.dumps(cleaned)),
            *[types.ImageContent(type="image", data=i["b64"], mime_type=i["mime"]) for i in images]]


async def on_call_tool(ctx, params):
    if params.name not in SCHEMAS:
        return types.CallToolResult(content=[types.TextContent(type="text", text="Unknown cleanup tool")], is_error=True)
    try:
        async with httpx.AsyncClient(timeout=240) as client:
            response = await client.post(f"{URL}/tools/{params.name}", json={"args": params.arguments or {}})
            response.raise_for_status()
            result = response.json()
        return types.CallToolResult(content=content_from_result(result), is_error=not result.get("ok", False))
    except httpx.HTTPError as exc:
        return types.CallToolResult(content=[types.TextContent(type="text", text=f"Cleanup service unavailable: {exc}")], is_error=True)


server = Server("so101-cleanup", instructions=INSTRUCTIONS, on_list_tools=on_list_tools, on_call_tool=on_call_tool)


async def run():
    async with stdio_server() as (read, write):
        await server.run(read, write, server.create_initialization_options())


def main():
    asyncio.run(run())


if __name__ == "__main__":
    main()
