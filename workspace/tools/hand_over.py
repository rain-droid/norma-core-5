"""Hand what this arm holds to the second arm (the SO-101), in the air.

Before: this arm holds the thing from above by its top only, so that its lower part hangs free
below the fingertips for the other gripper: `pick_at` with `grasp_height` at the thing's full
height (0.03 for a 3 cm cube). This arm's fingers are wide wedges: gripped lower, they cover a
small thing completely and leave the other gripper nothing to take.

Three ways, the first that exists is used (`how` picks one: "poses", "pose", "computed"):

* "poses": a person showed the whole hand-over with both arms limp (record_poses, kept in
  state/two_arm/hand_over.json). This arm carries the thing to where it was when the second
  arm's jaws closed in the recording, by the way the person took it there; the second arm goes
  to where it was, by its own way; it closes. If it closes on nothing, it opens, this arm offers
  the thing 1 cm further along the way it points, and it closes again, up to 3 cm: the thing
  never sits in the fingers quite as it did in the person's hand. Then this arm lets go, draws
  back 5 cm and goes home, and the second arm goes to its ready pose with the thing.
* "pose": a person adjusted one hand-over by pushing this arm (teach_hand_over).
* "computed": nothing shown. Described below.

Computed and "pose":
1. This arm holds the thing out. If a person has shown the hand-over (teach_hand_over), it goes
   to the pose shown. Otherwise it tries spots in front of the second arm until its IK reaches
   one, pointing down as steeply as it can there.
2. The depth camera finds the thing by its colour (`colour`): it must be in the gripper.
3. Where the second arm's jaws go. Shown: where they were when shown, moved by however much this
   arm's tool point differs now from then. Not shown: where the depth camera saw the thing,
   through the fit of calibrate_second_arm. The second arm opens, goes to 5 cm under that place,
   rises with its jaws around the thing, is moved until its own joints say it is at the height
   meant (rising, it stops 8 to 18 mm short), and closes. Jaws that close on nothing open and
   try again 1 cm higher, then lower.
4. Both hold for a moment; this arm lets go; the second arm takes the thing 5 cm down, out of
   its fingers, and this arm goes home. The depth camera then looks for the thing at the second
   arm's tool point: `second_arm_has_it`. `second_arm_holding` is what its own jaws report.

If the second arm cannot reach, or closes on nothing three times, it draws back, this arm keeps
hold, and the error says which step failed: call it again. Afterwards the second arm is still
holding the thing: use tools/second_arm.py to put it down (move_relative down, gripper open).

`turn`, for a long thing held by one end, first turns the end that sticks out towards the second
arm by rotating the wrist and measuring again; that has not worked yet on the real arm.
"""

import json
import math
import time

import numpy as np

from home import home
from util import depth as depth_camera
from util import second_arm
from util.arm import RobotError
from util.motion import require_ready

# Degrees left of the second arm's straight ahead, and metres from its base. The thing ends up
# about 2 cm further out than the spot (it hangs in front of this arm's tool point), and the
# further out the second arm reaches the more it sags: nearer spots are tried first.
SPOTS = [(pan, r) for r in (0.275, 0.29, 0.305) for pan in (-45, -35, -55)]
HOLDS = [(0.13, 90), (0.13, 70), (0.15, 60), (0.16, 50)]  # height above the table (m) and how steeply this arm points down (deg), best first
TAUGHT = "hand_over_taught.json"  # in robot_config/: the pose pair a person showed


def taught(ctx) -> dict | None:
    path = ctx.root / "robot_config" / TAUGHT
    return json.loads(path.read_text()) if path.exists() else None


def see(ctx, colour: str, near) -> np.ndarray:
    found = depth_camera.find_in_air(ctx, colour, min_height=0.06, near=near, radius=0.09)
    if found is None:
        raise RobotError(f"the depth camera sees nothing {colour} in this arm's gripper: is it holding the thing, and is that its colour?")
    return found["pos"]


def hold_out(ctx, T: np.ndarray, shown: dict | None, speed: float) -> dict:
    """Carry the thing to where it is handed over. Returns the spot, this arm's pitch and jaw yaw there."""
    plan = ctx.plan
    if shown:
        ctx.arm.joints(np.array(shown["this_arm_q"]), speed=speed, label="holding it out, as shown")
        return {"spot": np.array(shown["this_arm_tcp"]), "pitch": None, "yaw": None}
    second_base = T[:3, 3]
    for height, pitch in HOLDS:
        for pan, r in SPOTS:
            p = T[:3, :3] @ np.array([r * math.sin(math.radians(pan)), r * math.cos(math.radians(pan)), 0.0]) + second_base
            p[2] = plan.table_z + height
            # jaws square to the second arm's line of sight, so that it meets the thing face on: the nearest quarter turn
            sight = math.degrees(math.atan2(p[1] - second_base[1], p[0] - second_base[0]))
            yaw = (sight + 45.0) % 90.0 - 45.0
            solution = plan.solve(p, pitch=pitch, yaw=yaw, q_seed=ctx.arm.q())
            if solution.ok:
                ctx.arm.joints(solution.q, speed=speed, label="holding it out")
                return {"spot": p, "pitch": pitch, "yaw": yaw}
    raise RobotError("no spot in front of the second arm that this arm reaches pointing down: are the two arms within 35 cm of each other?")


def jaws_place(ctx, T: np.ndarray, shown: dict | None, colour: str, deeper: float, higher: float) -> tuple[np.ndarray, np.ndarray]:
    """Where the second arm's tool point goes to take the thing, in its own frame, and where the depth camera sees the thing."""
    tcp = np.array(ctx.arm.state()["ee"]["pos"])
    seen = see(ctx, colour, tcp)
    T_inv = np.linalg.inv(T)
    if shown:
        # as shown, shifted by how far this arm's tool point is now from where it was then: only the turn of the fit is used
        return np.array(shown["second_arm_tool"]) + T_inv[:3, :3] @ (tcp - np.array(shown["this_arm_tcp"])), seen
    target = T_inv[:3, :3] @ seen + T_inv[:3, 3]
    along = np.array([target[0], target[1], 0.0]) / np.linalg.norm(target[:2])
    # `deeper`: the fit was made with a block that stuck out in front of its jaws, so the fit alone stops short.
    # `higher`: the middle the camera reports is of the faces it sees, a little low for the jaws.
    return target + deeper * along + [0, 0, higher], seen


def go(p, jaws: str = "level", **kw) -> dict:
    """Send the second arm's tool point to p (its own frame), level if it can, else a little nose-down: it has five joints."""
    for tilt in (0, 15, 30):
        try:
            return second_arm.call("move_to", x=round(float(p[0]), 3), y=round(float(p[1]), 3), z=round(float(p[2]), 3), pitch=tilt, yaw=0 if jaws == "level" else -90, **kw)
        except RobotError as e:
            if "reach" not in str(e) or tilt == 30:
                raise


def bring_jaws(grip: np.ndarray, jaws: str, speed: float) -> list[int]:
    """Open, go to 5 cm under `grip`, rise to it, and settle at its height. Returns how far short it was each time it was read, mm."""
    second_arm.call("gripper", open=1.0)
    go(grip - [0, 0, 0.05], jaws, speed=speed)  # from under the thing: a run-in from its own side would start where it has no level pose
    go(grip, jaws, speed=0.3, straight=True)
    # Rising from below it stops 8 to 18 mm short of the height asked for, and jaws that low close under a
    # small thing; coming down it stops a little high. So: read where its own joints say it settled (a
    # second after the move, sooner overshoots), ask for that much more, and again until it is there.
    ask, short_by = grip.copy(), []
    for _ in range(4):
        time.sleep(1.2)
        short = float(grip[2] - second_arm.tool_point()[2])
        short_by.append(round(short * 1000))
        if abs(short) < 0.005:
            break
        ask[2] += max(-0.02, min(0.02, short))
        go(ask, jaws, speed=0.3, straight=True)
    return short_by


def take(ctx, T: np.ndarray, colour: str) -> dict:
    """The second arm closes (trying again a little higher, then lower, if it closes on nothing), this arm lets go and goes home, and the depth camera checks."""
    ctx.save_image("jaws_front.jpg", ctx.frame("across", settle=0.0), "front view just before the second arm closes: its jaws should be on both sides of the thing")
    ctx.save_image("jaws_wrist.jpg", ctx.frame("wrist", settle=0.0), "this arm's wrist camera at the same moment, looking down past its fingers at the thing")
    tries, holding = [], False
    for nudge in (0.0, 0.010, -0.018):  # height is the tight direction
        if nudge:
            second_arm.call("gripper", open=1.0)
            second_arm.call("move_relative", dz=nudge, speed=0.3)
            time.sleep(0.6)
        held = second_arm.call("grasp").get("data") or {}
        tries.append({"height_m": round(float(second_arm.tool_point()[2]), 3), **held})
        holding = bool(held.get("holding"))
        if holding:
            break
    if not holding:
        second_arm.call("gripper", open=1.0)
        second_arm.call("move_relative", dz=-0.05, speed=0.3)
        second_arm.call("home")
        raise RobotError(f"the second arm closed on nothing three times ({tries}); it went home and this arm still holds the thing")
    time.sleep(0.8)  # both hold it for a moment
    ctx.arm.joints(gripper=1.0, label="letting go")
    time.sleep(0.4)
    second_arm.call("move_relative", dz=-0.05, speed=0.3)  # down and out from between this arm's open fingers
    ctx.call(home)  # out of the way, and out of the depth camera's line of sight to the second arm's jaws
    time.sleep(0.5)
    tool = T[:3, :3] @ second_arm.tool_point() + T[:3, 3]
    after = depth_camera.find_in_air(ctx, colour, min_height=0.03, near=tool, radius=0.09)
    return {
        "grasp_tries": tries,
        "second_arm_holding": bool(second_arm.state()["gripper"]["holding"]),
        "second_arm_has_it": after is not None,
        "this_arm_gripper": ctx.arm.state()["gripper"],
    }


def shown_poses(ctx) -> dict | None:
    path = ctx.root / "state" / "two_arm" / "hand_over.json"
    return json.loads(path.read_text()) if path.exists() else None


def _thinned(samples: list, coarse: float, end) -> list:
    out, last = [], None
    for q in samples:
        q = np.array(q)
        if last is None or np.abs(q - last).max() > coarse:
            out.append(q)
            last = q
    if out and np.abs(out[-1] - np.array(end)).max() < coarse / 2:
        out.pop()
    return [*out, np.array(end)]


def by_shown_poses(ctx, rec: dict, T: np.ndarray, colour: str, speed: float) -> dict:
    """The hand-over as a person showed it with both arms (see the top of this file)."""
    steps, track = rec["steps"], rec["track"]
    t0 = track[0]["t"]
    closes = [i for i in range(1, len(steps)) if steps[i]["second"]["gripper"] < steps[i - 1]["second"]["gripper"] - 0.15]
    if not closes:
        raise RobotError("in the poses shown the second arm's jaws never close: show the hand-over again and close them by hand on the thing")
    g = closes[0]

    def way(arm: str, coarse: float, above: float) -> list:
        k = max(i for i in range(1, g + 1) if np.abs(np.array(steps[i][arm]["q"]) - np.array(steps[i - 1][arm]["q"])).max() > 0.03)  # the step that brought it there
        between = [r[arm]["q"] for r in track if t0 + steps[k - 1]["at_s"] < r["t"] <= t0 + steps[k]["at_s"] and r[arm]["tool"][2] > above]
        return _thinned(between, coarse, steps[g][arm]["q"])

    # this arm, with the thing: the way it was carried, leaving out the low part near where it was picked up
    ctx.arm.joints(np.array(way("this", 0.12, 0.10)), speed=speed, label="holding it out, as shown")
    # Its soft servos do not always settle on the pose asked for (17 mm to the side, once, and the second arm's
    # jaws then came in beside the thing): read where the tool point is, ask for that much more, twice at most.
    want = np.array(steps[g]["this"]["tool"])
    ask, off_by = want.copy(), []
    for _ in range(3):
        time.sleep(0.7)
        off = want - np.array(ctx.arm.state()["ee"]["pos"])
        off_by.append(round(float(np.linalg.norm(off)) * 1000))
        if off_by[-1] < 4 or len(off_by) == 3:
            break
        ask += off
        try:
            ctx.arm.joints(ctx.plan.line(ctx.arm.q(), ask), speed=0.3, label="settling on the pose shown")
        except RobotError:
            break
    second_arm.call("gripper", open=1.0)
    # the second arm's way there, walked in the shared space against this arm first: only the two grippers may meet
    second_arm.move_checked(ctx.root, way("second", 0.3, 0.0), ctx.arm.q(), speed, grippers_may_meet=True)
    time.sleep(0.6)
    ctx.save_image("jaws_wrist.jpg", ctx.frame("wrist", settle=0.0), "this arm's wrist camera just before the second arm closes: the thing should be between its jaws")
    here = ctx.arm.state()["ee"]
    start, points = np.array(here["pos"]), np.array(here["approach"])
    tries, holding = [], False
    for offered in (0.0, 0.01, 0.02, 0.03):
        if offered:
            second_arm.call("gripper", open=1.0)
            ctx.arm.joints(ctx.plan.line(ctx.arm.q(), start + offered * points), speed=0.3, label="offering it further", near_other=True)
            time.sleep(0.5)
        held = second_arm.call("grasp").get("data") or {}
        tries.append({"offered_mm": round(offered * 1000), **held})
        holding = bool(held.get("holding"))
        if holding:
            break
    if not holding:
        second_arm.call("gripper", open=1.0)
        second_arm.home_checked(ctx.root, ctx.arm.q(), speed)
        raise RobotError(f"the second arm closed on nothing four times ({tries}; this arm was {off_by} mm off the pose shown); it went home and this arm still holds the thing")
    time.sleep(0.8)  # both hold it for a moment
    ctx.arm.joints(gripper=1.0, label="letting go")
    time.sleep(0.5)
    try:
        ctx.arm.joints(ctx.plan.line(ctx.arm.q(), np.array(ctx.arm.state()["ee"]["pos"]) - 0.05 * points), speed=0.3, label="drawing back", near_other=True)
    except RobotError:
        pass  # where it cannot draw back in a line it goes straight home: its fingers are open
    second_arm.home_checked(ctx.root, ctx.arm.q(), speed)  # the second arm leaves first, with the thing: to its ready pose, in view of the cameras
    ctx.call(home)
    time.sleep(0.6)
    tool = T[:3, :3] @ second_arm.tool_point() + T[:3, 3]
    after = depth_camera.find_in_air(ctx, colour, min_height=0.03, near=tool, radius=0.10)
    return {
        "how": "poses",
        "this_arm_off_mm": off_by,
        "grasp_tries": tries,
        "second_arm_holding": bool(second_arm.state()["gripper"]["holding"]),
        "second_arm_has_it": after is not None,
        "this_arm_gripper": ctx.arm.state()["gripper"],
    }


def hand_over(ctx, colour: str = "blue", jaws: str = "level", how: str = "auto", as_shown: bool = True, deeper: float = 0.025, higher: float = 0.009,
              turn: bool = False, speed: float = 0.5) -> dict:  # fmt: skip
    if jaws not in ("level", "upright"):
        raise RobotError("jaws is 'level' (the second arm closes on the thing's sides) or 'upright' (on its top and bottom)")
    state = require_ready(ctx, awake=True)
    if not state["gripper"]["holding"]:
        raise RobotError("this arm is not holding anything: pick the thing up first")
    T = second_arm.load(ctx.root)
    if how not in ("auto", "poses", "pose", "computed"):
        raise RobotError('how is "auto", "poses" (as shown with both arms), "pose" (as adjusted by teach_hand_over) or "computed"')
    rec = shown_poses(ctx) if how in ("auto", "poses") and as_shown else None
    if how == "poses" and rec is None:
        raise RobotError("no hand-over has been shown with both arms (state/two_arm/hand_over.json is missing): record_poses first")
    if rec is not None:
        return by_shown_poses(ctx, rec, T, colour, speed)
    shown = taught(ctx) if as_shown and how in ("auto", "pose") else None
    held = hold_out(ctx, T, shown, speed)

    turned = []
    for _ in range(3 if turn and not shown else 0):
        time.sleep(0.5)
        tcp = np.array(ctx.arm.state()["ee"]["pos"])
        out = (see(ctx, colour, tcp) - tcp)[:2]  # from the fingers to the part that sticks out
        if np.linalg.norm(out) < 0.015:
            break  # held by its middle: no free end to turn
        want = (T[:3, 3] - tcp)[:2]
        off = math.degrees(math.atan2(out[0] * want[1] - out[1] * want[0], out @ want))  # how far the free end must turn, counter-clockwise from above
        turned.append(round(off))
        if abs(off) < 20:
            break
        for sign in (1, -1):  # which way the wrist's yaw turns the thing is found by trying
            solution = ctx.plan.solve(held["spot"], pitch=held["pitch"], yaw=held["yaw"] + sign * off, q_seed=ctx.arm.q())
            if solution.ok:
                break
        else:
            break  # the wrist cannot turn that far: take it as it is
        held["yaw"] += sign * off
        ctx.arm.joints(solution.q, speed=speed, label="turning it towards the other arm")

    time.sleep(0.5)
    grip, seen = jaws_place(ctx, T, shown, colour, deeper, higher)
    try:
        short_by = bring_jaws(grip, jaws, speed)
    except RobotError as e:
        second_arm.call("home")
        raise RobotError(f"the second arm could not get to the thing ({e}); this arm still holds it") from None
    return {
        "handed_over_at": [round(float(v), 3) for v in seen],
        "how": "pose" if shown else "computed",
        "this_arm_pitch_deg": held["pitch"],
        "turned_deg": turned,
        "second_arm_short_mm": short_by,
        **take(ctx, T, colour),
    }


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(hand_over)
