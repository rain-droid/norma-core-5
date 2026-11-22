"""The tools agents call. One registry, used by the daemon (HTTP), the MCP server and the UI.

A tool is a name, a description written for a language model, a JSON schema, and a coroutine
that takes the Skills object plus keyword arguments and returns a ToolResult.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable

from .robot import RobotError
from .skills import GESTURES, Skills


@dataclass
class ToolResult:
    summary: str
    data: dict = field(default_factory=dict)
    images: list[bytes] = field(default_factory=list)  # JPEG


@dataclass
class Tool:
    name: str
    description: str
    properties: dict[str, dict]
    fn: Callable[..., Awaitable[ToolResult]]
    required: tuple[str, ...] = ()
    motion: bool = True  # motion tools run one at a time; the rest are always allowed

    @property
    def input_schema(self) -> dict:
        return {"type": "object", "properties": self.properties, "required": list(self.required), "additionalProperties": False}


TOOLS: dict[str, Tool] = {}


def tool(name: str, description: str, properties: dict | None = None, required: tuple[str, ...] = (), motion: bool = True):
    def deco(fn):
        TOOLS[name] = Tool(name=name, description=" ".join(description.split()), properties=properties or {}, fn=fn, required=required, motion=motion)
        return fn

    return deco


NUM = {"type": "number"}
FRAME = (
    "Coordinates are metres in the robot's base frame: origin on the table under the base, "
    "x to the robot's right, y forward away from the base, z up from the table."
)
TARGET_PROPS = {
    "object": {"type": "integer", "description": "Object number from the last scan()."},
    "x": {**NUM, "description": "Table x in metres (use instead of object)."},
    "y": {**NUM, "description": "Table y in metres (use instead of object)."},
}


# ----------------------------------------------------------------------------- looking


@tool(
    "status",
    "Report where the arm is and what state it is in: tool position, gripper, whether it is holding something, "
    "torque, connection, and which cameras are live. Cheap; call it whenever you are unsure.",
    motion=False,
)
async def _status(sk: Skills) -> ToolResult:
    snap = sk.robot.snapshot()
    cams = ", ".join(f"{c['name']} ({c['width']}x{c['height']}, {c['fps']} fps)" for c in snap["cameras"]) or "none"
    text = f"{'connected' if snap['connected'] else 'NOT CONNECTED'} ({snap['mode']}), {sk.describe_pose()}. Cameras: {cams}."
    hot = [j for j in snap["joints"] if j["temp"] >= 55]
    if hot:
        text += " Warm servos: " + ", ".join(f"joint {j['id']} {j['temp']}C" for j in hot) + "."
    return ToolResult(text, {"ee": snap["ee"], "gripper": snap["gripper"], "torque": snap["torque"], "busy": snap["busy"]})


@tool(
    "look",
    "Get the current image from a camera. The wrist camera is on the gripper and sees whatever the gripper points at; "
    "your own jaws are the black shapes at the bottom of the frame. The image has gridlines labelled with pixel "
    "coordinates so you can refer to a spot precisely. Does not move the arm: use observe() first for an overview "
    "of the table.",
    {
        "camera": {"type": "string", "description": "Camera name, default 'wrist'. status() lists the live ones."},
        "grid": {"type": "boolean", "description": "Draw the pixel grid (default true)."},
    },
    motion=False,
)
async def _look(sk: Skills, camera: str = "wrist", grid: bool = True) -> ToolResult:
    text, jpeg = await sk.look(camera, grid)
    return ToolResult(text, images=[jpeg])


@tool(
    "observe",
    "Raise the wrist camera over the workspace and return what it sees, with every object that stands out on the "
    "table outlined, numbered and located. This is the normal first step of any task: the numbers can be passed "
    "straight to pick(), touch() and point_at(). " + FRAME,
)
async def _observe(sk: Skills) -> ToolResult:
    await sk.observe()
    text, jpeg, seen = await sk.scan()
    return ToolResult(text, {"objects": [_seen(s) for s in seen]}, [jpeg])


@tool(
    "scan",
    "Find and number the objects on the table in the wrist camera's current view, without moving the arm. "
    "Use it after you have moved the camera yourself, or to re-check where things are after acting. "
    "Returns the annotated image and each object's table position and size. " + FRAME,
    motion=False,
)
async def _scan(sk: Skills) -> ToolResult:
    text, jpeg, seen = await sk.scan()
    return ToolResult(text, {"objects": [_seen(s) for s in seen]}, [jpeg])


@tool(
    "fetch",
    "Do a whole pick in one call: look for the object of the given colour, on the table first and then held up in the "
    "air (in someone's hand, on top of something), grasp it and lift it. Use this whenever the person asks to pick up / grab something you can "
    "name by colour (e.g. 'the blue cube'). Fails with a clear reason if nothing of that colour is in reach or the "
    "grasp closes on nothing; calling it again is a fine retry.",
    {
        "color": {"type": "string", "description": "blue, green, yellow, orange, red, pink or purple."},
        "hand_over": {"type": "boolean", "description": "After lifting, hold it out towards the person (default false)."},
    },
    required=("color",),
)
async def _fetch(sk: Skills, color: str, hand_over: bool = False) -> ToolResult:
    out = await sk.fetch(color, hand_over)
    where = "took it out of the air (from a hand or off something)" if out.get("from") == "the air" else "lifted clear of the table"
    return ToolResult(f"got the {color} object: holding it ({out['width_mm']} mm between the jaws), {where}.", out)


def _seen(s) -> dict:
    return {"id": s.id, "x": s.xyz[0], "y": s.xyz[1], "size_mm": list(s.size_mm), "color": s.color, "pixel": list(s.pixel), "yaw": s.yaw_deg}


@tool(
    "pixel_to_table",
    "Convert a pixel in the wrist camera's current image into a table position, for pointing at something "
    "scan() did not outline. Give `height` in metres if the spot is on top of an object rather than on the table. " + FRAME,
    {"u": NUM, "v": NUM, "height": {**NUM, "description": "Height of the surface above the table, metres (default 0)."}},
    required=("u", "v"),
    motion=False,
)
async def _pixel_to_table(sk: Skills, u: float, v: float, height: float = 0.0) -> ToolResult:
    img, _ = sk._frame("wrist")
    p = sk.pixel_to_table(u, v, img.shape, height)
    sk.emit("markers", markers=[{"id": "pixel", "label": f"({u:.0f},{v:.0f})", "pos": [float(p[0]), float(p[1]), float(p[2])], "color": "#ff6ad5", "radius": 0.01}])
    return ToolResult(f"pixel ({u:.0f}, {v:.0f}) is at x={p[0]:+.3f} y={p[1]:+.3f} on the table", {"x": round(float(p[0]), 3), "y": round(float(p[1]), 3)})


# ----------------------------------------------------------------------------- manipulation


@tool(
    "pick",
    "Pick something up off the table: opens the gripper, moves above the target, looks again up close to line up, "
    "descends, closes, lifts, and checks that it is really holding the object. Give an object number from "
    "observe()/scan(), or x and y. Fails with an explanation if the jaws close on nothing or the object slips, "
    "so you can scan again and retry. The gripper opens 53 mm; it grasps across an object's narrow side.",
    {
        **TARGET_PROPS,
        "yaw": {**NUM, "description": "Rotate the jaws about the vertical, degrees (0 = jaws close left-right). Default: chosen from the object's shape."},
    },
)
async def _pick(sk: Skills, object: int | None = None, x: float | None = None, y: float | None = None, yaw: float | None = None) -> ToolResult:
    r = await sk.pick(object, x, y, yaw)
    return ToolResult(f"picked it up (gripping {r['width_mm']:.0f} mm), holding it above x={r['picked_from'][0]:+.3f} y={r['picked_from'][1]:+.3f}", r)


@tool(
    "place",
    "Put down what the gripper is holding at a table position, then back away. Set `z` to the height of the "
    "surface if placing on top of something (for stacking, the height of the thing underneath). " + FRAME,
    {"x": NUM, "y": NUM, "z": {**NUM, "description": "Surface height above the table in metres (default 0)."},
     "yaw": {**NUM, "description": "Jaw rotation in degrees (default 0)."},
     "pitch": {**NUM, "description": "Gripper tilt in degrees below horizontal to put it down with (default: the tilt it was picked up with, so it lands flat)."}},
    required=("x", "y"),
)  # fmt: skip
async def _place(sk: Skills, x: float, y: float, z: float = 0.0, yaw: float | None = None, pitch: float | None = None) -> ToolResult:
    r = await sk.place(x, y, z, yaw, pitch)
    what = "put it down" if r["released_something"] else "opened the gripper (it was not holding anything)"
    return ToolResult(f"{what} at x={x:+.3f} y={y:+.3f}", r)


@tool(
    "touch",
    "Reach out and gently touch something with a closed fingertip, then back off. Descends slowly and stops when "
    "it feels contact. Give an object number or x and y; set `height` (metres) to the object's height if you know "
    "it, so the approach is aimed at its top.",
    {**TARGET_PROPS, "height": {**NUM, "description": "Height of the object in metres (default 0 = the table surface)."}},
)
async def _touch(sk: Skills, object: int | None = None, x: float | None = None, y: float | None = None, height: float = 0.0) -> ToolResult:
    r = await sk.touch(object, x, y, height)
    how = "felt contact" if r["felt_contact"] else "reached the spot without feeling resistance"
    return ToolResult(f"touched it ({how}, fingertip {r['fingertip_height_mm']} mm above the table)", r)


@tool(
    "point_at",
    "Hover the gripper a few centimetres above a spot, pointing at it, and stay there. Good for answering "
    "'which one?' or showing where something is.",
    TARGET_PROPS,
)
async def _point_at(sk: Skills, object: int | None = None, x: float | None = None, y: float | None = None) -> ToolResult:
    r = await sk.point_at(object, x, y)
    return ToolResult(f"pointing at x={r['pointing_at'][0]:+.3f} y={r['pointing_at'][1]:+.3f}", r)


@tool(
    "gripper",
    "Open or close the gripper to a fraction: 1 = fully open (53 mm), 0 = closed. To grab something use grasp() "
    "or pick() instead, which check whether an object is actually held.",
    {"open": {**NUM, "minimum": 0, "maximum": 1}},
    required=("open",),
)
async def _gripper(sk: Skills, open: float) -> ToolResult:
    reached = await sk.robot.set_gripper(open)
    return ToolResult(f"gripper at {reached * 100:.0f}% open", {"open": round(reached, 3)})


@tool(
    "grasp",
    "Close the gripper on whatever is between the jaws right now and report whether something is held. "
    "For use after you have positioned the gripper yourself with move_to.",
)
async def _grasp(sk: Skills) -> ToolResult:
    g = await sk.robot.grasp()
    return ToolResult(f"holding something {g['width_mm']:.0f} mm wide" if g["holding"] else "closed on nothing", g)


# ----------------------------------------------------------------------------- direct motion


@tool(
    "move_to",
    "Move the tool point (the spot between the fingertips) to a position. " + FRAME + " The reachable area on the "
    "table is roughly a half-disc of radius 0.38 m in front of the base; straight-down grasps only work within "
    "about 0.27 m. `pitch` is how far below horizontal the gripper points: 90 = straight down, 0 = level. Leave "
    "it out to get the most vertical approach that reaches. Refuses moves that would hit the table or the arm itself.",
    {
        "x": NUM, "y": NUM, "z": NUM,
        "pitch": {**NUM, "description": "Degrees below horizontal, 0..90."},
        "yaw": {**NUM, "description": "Rotation of the jaws about the pointing direction, degrees."},
        "straight": {"type": "boolean", "description": "Move the tool in a straight line (default false: fastest joint path)."},
        "speed": {**NUM, "description": "0.1 (slow) to 1.5 (fast), default 0.8."},
    },
    required=("x", "y", "z"),
)  # fmt: skip
async def _move_to(sk: Skills, x: float, y: float, z: float, pitch: float | None = None, yaw: float | None = None,
                   straight: bool = False, speed: float = 0.8) -> ToolResult:  # fmt: skip
    await sk.robot.move_to([x, y, z], pitch=pitch, yaw=yaw, linear=straight, speed=speed)
    return ToolResult(sk.describe_pose())


@tool(
    "move_relative",
    "Shift the tool point by a small offset in a straight line, keeping the gripper's orientation. " + FRAME +
    " For example dz=0.05 lifts 5 cm, dy=0.03 pushes 3 cm away from the base.",
    {"dx": NUM, "dy": NUM, "dz": NUM, "speed": {**NUM, "description": "Default 0.6."}},
)  # fmt: skip
async def _move_relative(sk: Skills, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, speed: float = 0.6) -> ToolResult:
    if max(abs(dx), abs(dy), abs(dz)) > 0.25:
        raise RobotError("that offset is larger than 25 cm; use move_to for big moves")
    await sk.robot.move_relative([dx, dy, dz], speed=speed)
    return ToolResult(sk.describe_pose())


@tool(
    "move_joints",
    "Set individual joint angles in degrees. Joints: 1 base rotation, 2 shoulder, 3 lower elbow, 4 upper elbow, "
    "5 forearm roll, 6 wrist bend, 7 wrist roll. With relative=true the values are added to the current angles. "
    "Prefer move_to for getting somewhere; use this for expressive motion or fine adjustments.",
    {
        "joints": {"type": "object", "description": "Map of joint number to degrees, e.g. {\"1\": 20, \"7\": -30}.", "additionalProperties": NUM},
        "relative": {"type": "boolean", "description": "Add to current angles (default false)."},
        "speed": {**NUM, "description": "Default 0.8."},
    },
    required=("joints",),
)
async def _move_joints(sk: Skills, joints: dict, relative: bool = False, speed: float = 0.8) -> ToolResult:
    q = sk.robot._plan_start()
    n = len(q)
    for k, v in joints.items():
        i = int(k)
        if not 1 <= i <= n:
            raise RobotError(f"joint {k} does not exist (arm joints are 1..{n}; use gripper() for the jaws)")
        q[i - 1] = q[i - 1] + math.radians(float(v)) if relative else math.radians(float(v))
    lo, hi = sk.robot.kin.lower, sk.robot.kin.upper
    over = [i + 1 for i in range(n) if q[i] < lo[i] - 1e-6 or q[i] > hi[i] + 1e-6]
    if over:
        lim = ", ".join(f"joint {i}: {math.degrees(lo[i - 1]):.0f}..{math.degrees(hi[i - 1]):.0f}" for i in over)
        raise RobotError(f"outside joint limits ({lim} degrees)")
    await sk.robot.move_joints(q, speed=speed)
    deg = " ".join(f"{math.degrees(v):.0f}" for v in sk.robot.q())
    return ToolResult(f"joints now [{deg}] degrees. " + sk.describe_pose())


@tool("home", "Return to the ready pose: gripper over the table in front of the base, pointing down and forward. A safe place to start or finish.")
async def _home(sk: Skills) -> ToolResult:
    return ToolResult(await sk.home())


@tool(
    "rest",
    "Fold the arm into its sitting position and switch the motors off. Use when done for a while. "
    "The next motion command switches them back on.",
)
async def _rest(sk: Skills) -> ToolResult:
    return ToolResult(await sk.rest())


@tool(
    "gesture",
    "Perform an expressive motion. dap: hold out a closed fist for a fist bump and wait up to 6 s to feel it land. "
    "handshake: hold the open hand out, and shake when the person takes it. wave: wave hello. nod / shake: yes / no "
    "(shake is the head shake, not a handshake). bow, shrug, dance, peek, happy, sad, curious, celebrate, oops. "
    "`side` is where the person is for dap and handshake.",
    {"name": {"type": "string", "enum": list(GESTURES)}, "side": {"type": "string", "enum": ["left", "center", "right"]}},
    required=("name",),
)
async def _gesture(sk: Skills, name: str, side: str = "center") -> ToolResult:
    return ToolResult(await sk.gesture(name, side))


@tool(
    "throw",
    "Throw what is in the gripper in an arc: winds up overhead, whips forward and lets go. Give `color` to pick that "
    "object up first. `pan` aims it in degrees, left (-) or right (+) of straight ahead, up to 85 either way; leave it "
    "out with `here` true to throw where the arm is facing now. `style`: high lob, normal, or flat (a flatter, longer "
    "pass). `dry` is a practice swing that lets go of nothing.",
    {"color": {"type": "string", "description": "pick up the object of this colour first"},
     "pan": {"type": "number"}, "here": {"type": "boolean"}, "style": {"type": "string", "enum": ["high", "normal", "flat"]},
     "dry": {"type": "boolean", "description": "practice swing without letting go"}},
)
async def _throw(sk: Skills, color: str | None = None, pan: float = 0.0, here: bool = False, style: str = "normal", dry: bool = False) -> ToolResult:
    out = await sk.throw(color, None if here else pan, 50.0, dry, style)
    aim = "straight ahead" if abs(out["pan"]) < 8 else f"{abs(out['pan'])} degrees to my {'right' if out['pan'] > 0 else 'left'}"
    if dry:
        return ToolResult(f"practice swing done towards {aim}, nothing let go")
    if not out.get("released"):
        return ToolResult(f"swung towards {aim}, but the jaws did not open in time; it may have dropped late")
    return ToolResult(f"thrown towards {aim} ({out['style']}): let go at about {out['speed']} m/s, {out['elevation']:.0f} deg above level. "
                      "It is gone, nothing is held now.")


@tool("lineup", "Big, slow, wavy sweeping motion through the air for a while ('line me up', 'make it straight'). Keeps hold of whatever "
      "is in or attached to the gripper and does not need the camera.", {"seconds": {"type": "number"}})
async def _lineup(sk: Skills, seconds: float = 25.0) -> ToolResult:
    return ToolResult(await sk.lineup(seconds))


@tool("take", "Take something from the person's hand: hold the open gripper out, and close when they put it between the jaws and tap.",
      {"side": {"type": "string", "enum": ["left", "center", "right"]}})
async def _take(sk: Skills, side: str = "center") -> ToolResult:
    return ToolResult(await sk.take(side))


@tool("catch", "Catch something rolled along the table: open jaws on the table facing the person, snap shut when it rolls in.",
      {"side": {"type": "string", "enum": ["left", "center", "right"]}})
async def _catch(sk: Skills, side: str = "center") -> ToolResult:
    return ToolResult(await sk.catch(side))


@tool("give", "Hand over what is in the gripper: hold it out towards the person's hand, wait a moment, then let go. "
      "`side` is where their hand is from the robot's own point of view.",
      {"side": {"type": "string", "enum": ["left", "center", "right"]}})
async def _give(sk: Skills, side: str = "center") -> ToolResult:
    out = await sk.give(side)
    if out.startswith("FAILED"):
        raise RobotError(out[7:])
    return ToolResult(out)


@tool("turn", "Turn the head without going anywhere: left / right swings the whole arm about its base, up / down tilts the "
      "wrist camera. Never lets go of what is held.",
      {"direction": {"type": "string", "enum": ["left", "right", "up", "down"]}, "degrees": {"type": "number"}},
      required=("direction",))
async def _turn(sk: Skills, direction: str, degrees: float | None = None) -> ToolResult:
    return ToolResult(await sk.turn(direction, degrees))


@tool("search", "Turn the raised camera to look around (left, then right, then ahead on repeated calls). Never lets go of what is held.")
async def _search(sk: Skills) -> ToolResult:
    return ToolResult(await sk.search())


@tool("stop", "Stop all motion immediately and hold position. Always allowed, even while another tool is running.", motion=False)
async def _stop(sk: Skills) -> ToolResult:
    await sk.robot.stop()
    return ToolResult("stopped; holding position")


def describe() -> list[dict[str, Any]]:
    return [{"name": t.name, "description": t.description, "input_schema": t.input_schema} for t in TOOLS.values()]


def coerce(tool_: Tool, args: dict) -> dict:
    """Drop unknown keys and nulls so a slightly-off call still works."""
    out = {}
    for k, v in (args or {}).items():
        if v is None:
            continue
        if k not in tool_.properties:
            raise RobotError(f"{tool_.name}() has no argument '{k}' (it takes: {', '.join(tool_.properties) or 'nothing'})")
        out[k] = v
    missing = [k for k in tool_.required if k not in out]
    if missing:
        raise RobotError(f"{tool_.name}() needs {', '.join(missing)}")
    return out

