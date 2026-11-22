"""Keeps watch on the table through the top camera, so object positions stay current.

Every other tool here runs and exits. This is the one process that stays. Ten times a second
it looks at the top camera with the same eyes as tools/find_objects.py and rewrites
perception/exteroception/tracks.json (and tracks.jpg, the view with each track outlined):

    {"t": ..., "clear_s": 12.3, "intrusion": null, "holding": null, "stopped_arm_t": null,
     "tracks": [{"id": "red-1", "state": "visible", "x": 0.071, "y": 0.199, "yaw_deg": -90,
                 "width_mm": 25, "height_mm": 25, "still_s": 31.0, "seen_s": 0.0, "on": null}]}

An object keeps its id for as long as it is on the table. Its `state` is one of
  visible  measured in this frame; `still_s` says for how long it has not moved
  hidden   behind the arm or a hand; x, y are where it was last measured
  held     in the gripper; x, y follow the tool point
  missing  its place is in view and empty; forgotten after two minutes
An object put down on another one gets `on` (the id underneath) and travels with it. One that
turns up somewhere else after being out of sight is recognised by its colour.

`intrusion` is set while something is moving on the table that the arm did not move: a thing
coming in over the edge of the searched area, something new appearing, a tracked object
sliding. That is what a hand looks like from here. It stays set until the table has been
quiet for a second and a half. While it is set the tracker stops any arm motion
(`stopped_arm_t`), unless it was started with --no-stop. `clear_s` is how long it has been
quiet. A thing that lies still is not an intrusion, whatever it is: one lying across the edge
is listed under `at_edge`, where it cannot be measured or picked by name.

The image and report that last raised an intrusion are kept as intrusion.jpg and
intrusion.json beside tracks.json, so a stopped tool can show what triggered it.
This is a motion heuristic, not a person detector or safety interlock: stationary hands,
occluded regions and table-coloured objects can be missed.

Started by tools/track_objects.py, or by the first tool that needs it. It ends by itself when
the robot side goes away. To watch it work: `cd tools && pixi run python -m util.tracker`.

The second half of this file is what tools use to read it: fresh(), track(), wait_for().
"""

import fcntl
import json
import os
import signal
import subprocess
import sys
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

from util import capture, vision
from util.arm import Arm, RobotError

ROOT = Path(__file__).resolve().parents[2]
FILE = ROOT / "perception" / "exteroception" / "tracks.json"
RUNTIME = ROOT / ".runtime"
NO_STOP = RUNTIME / "tracker_does_not_stop_the_arm"  # exists while someone has switched the stopping off

PERIOD = 0.1  # seconds between looks
SAME_SPOT = 12  # pixels: an outline centred this close to a track's is that object, whatever its colour reads as
SAME_OBJECT = 45  # pixels (about 6 cm on the table): one this close and of the same colour is that object, moved
MOVED = 4  # pixels: centred further than this from where it has been means it moved; a still outline wanders by one
CONFIRM = 3  # looks in a row before something new becomes a track
GONE_AFTER = 5  # looks in a row with its place in view and empty before a track is "missing"
FORGET_S = 120.0  # seconds a missing track is kept
NEWS_LOOKS = 2  # looks in a row something unexplained must show before it counts (one look can be a glitch)
LARGE_PX = 1500  # image area: something this large appearing at the edge counts at its first look
QUIET_S = 1.5  # seconds without anything moving before the table counts as clear again
LAG_LOOKS = 3  # the camera runs behind the joint angles: the arm is masked where it has been for this many looks
RENAME_LOOKS = 10  # looks in a row a track must read as another colour than its name before it is renamed
GRIP_REACH = 0.04  # m: when the jaws close on something, it is the track this close to the tool point
STACK_REACH = 0.03  # m: let go this close above another track means put down on it


def _apart(a, b) -> float:
    """Metres between two things on the table."""
    return float(np.hypot(a["x"] - b["x"], a["y"] - b["y"]))


def _px(a, b) -> float:
    """Pixels between the centres of two outlines in the image."""
    return float(np.hypot(a["u"] - b["u"], a["v"] - b["v"]))


class Watcher:
    def __init__(self, arm: Arm):
        import find_objects  # tools/find_objects.py: what stands out, and where it stands
        from util.planning import Planner

        self.find = find_objects
        self.arm = arm
        meta = arm.meta()
        self.mode, self.table_z = meta["mode"], meta["table_z"]
        self.ctx = SimpleNamespace(root=ROOT, plan=Planner(arm))  # what find_objects asks of a tool context
        self.cam = capture.TopCamera(ROOT, self.mode, self.table_z)
        self.tracks: list[dict] = []
        self.maybe: list[dict] = []  # seen, not yet often enough to be a track
        self.numbers: dict[str, int] = {}
        self.arm_masks: deque = deque(maxlen=LAG_LOOKS)
        self.edge: list[dict] = []  # things across the edge of the searched area, and for how many looks each has lain still
        self.news: list[tuple[str, bool, list[int]]] = []  # reason, whether one look is proof, and image pixel
        self.news_looks = 0
        self.news_t = 0.0
        self.said_t = 0.0
        self.intrusion: dict | None = None
        self.clear_since = time.time()
        self.stopped_arm_t: float | None = None
        self.holding: str | None = None  # id of the track in the gripper; "" for something that was never a track
        self.shown = ""

    # ---- one look

    def look(self, stop_on_intrusion: bool = True) -> tuple[dict, np.ndarray]:
        now = time.time()
        state = self.arm.state()
        if state["mode"] != self.mode:
            raise SystemExit("the robot side changed between real and simulated")
        img = self.arm.frame("top", settle=0.0)
        tcp = np.array(state["ee"]["pos"]) if state.get("ee") else None
        gripping = bool(state["gripper"]["holding"]) and tcp is not None
        q = [j["q"] for j in state["joints"][:7]]
        self.arm_masks.append(self.find.arm_mask(self.ctx, self.cam, img.shape, q))
        hide = np.logical_or.reduce(list(self.arm_masks))
        if gripping:  # what the jaws hold hangs in view beside the arm: it is not standing on the table
            uv = self.cam.to_pixel(tcp)
            disc = np.zeros(img.shape[:2], np.uint8)
            cv2.circle(disc, (int(uv[0]), int(uv[1])), int(np.linalg.norm(self.cam.to_pixel(tcp + [0.045, 0, 0]) - uv)) + 6, 255, -1)
            hide |= disc > 0
        blobs, area, covered = self.find.table_blobs(self.ctx, self.cam, img, {"table_z": self.table_z, "joints_rad": q}, hide)
        near = cv2.dilate(covered.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0  # the arm and a little around it
        # Wider than that, around the arm and its hand, nothing counts as news: there the arm's
        # own parts come out from behind the mask (the wrist camera's plate sticks out sideways
        # when the wrist turns), the object in the jaws shows, and outlines change as the arm passes.
        by_arm = cv2.dilate(covered.astype(np.uint8), np.ones((61, 61), np.uint8))
        if tcp is not None:
            uv = self.cam.to_pixel(tcp)
            cv2.circle(by_arm, (int(uv[0]), int(uv[1])), int(np.linalg.norm(self.cam.to_pixel(tcp + [0.12, 0, 0]) - uv)), 1, -1)
        self.by_arm = by_arm > 0
        # The edge of the searched area that a hand can come in over: the left, right and camera
        # sides. Not the robot's side: everything tall leans that way in this view, so the top of
        # an object standing near that edge crosses it.
        rim = area & ~(cv2.erode(area.astype(np.uint8), np.ones((9, 9), np.uint8)) > 0)
        ends = [tuple(int(round(c)) for c in self.cam.to_pixel([x, self.find.SEARCH_Y[0], self.table_z])) for x in self.find.SEARCH_X]
        rim &= cv2.line(np.zeros(img.shape[:2], np.uint8), *ends, 255, 15) == 0

        self.news = []
        seen, across = [], []
        for blob in blobs:
            shape = np.zeros(img.shape[:2], np.uint8)
            cv2.fillPoly(shape, [np.array(blob.polygon, np.int32)], 255)
            shape = shape > 0
            cut = bool((shape & near).any())  # part of it is behind the arm: it is there, but this is no measurement of it
            if (shape & rim & ~near).any():  # it lies across the edge, or is coming in over it: either way it cannot be measured
                if not cut:
                    across.append(blob)
                continue
            one = self.find.describe(self.cam, blob, img, "")
            if one:
                seen.append({**one[0], "cut": cut, "u": blob.centroid[0], "v": blob.centroid[1], "area_px": round(blob.area), "polygon": blob.polygon, "box": list(blob.box)})

        self._edge(across)
        self._match(seen, near, [list(b.box) for b in across], now)
        self._gripper(gripping, tcp, now)
        self._intrusion(state, now, stop_on_intrusion)
        self.tracks = [t for t in self.tracks if not (t["state"] == "missing" and now - t["seen_t"] > FORGET_S)]
        return self._report(now), self._picture(img)

    def _edge(self, across) -> None:
        """Things across the edge of the searched area. One that looks as it did at some look in
        the last few seconds is lying there (two things side by side merge into one outline and
        part again from look to look). One never seen like this is something coming in."""
        for e in self.edge:
            e["unseen"] += 1
        for blob in across:
            cx, cy = blob.centroid
            same = next((e for e in self.edge if abs(e["cx"] - cx) < 8 and abs(e["cy"] - cy) < 8 and 0.75 < blob.area / e["area"] < 1.33), None)
            if same is None:
                self._tell("something is moving at the edge of the work area", blob.area >= LARGE_PX, cx, cy)
                self.edge.append({"cx": cx, "cy": cy, "area": blob.area, "box": list(blob.box), "looks": 1, "unseen": 0})
            else:
                same.update(looks=same["looks"] + 1, unseen=0)
        # what has lain there for a second is remembered while the arm passes in front of it; what only passed through, for three seconds
        self.edge = [e for e in self.edge if e["unseen"] < (600 if e["looks"] >= 10 else 30)]

    def _tell(self, what: str, proof: bool, u: float, v: float) -> None:
        u, v = int(round(u)), int(round(v))
        if not self.by_arm[min(max(v, 0), self.by_arm.shape[0] - 1), min(max(u, 0), self.by_arm.shape[1] - 1)]:
            self.news.append((what, proof, [u, v]))

    def _intrusion(self, state, now, stop_on_intrusion) -> None:
        self.news_looks = self.news_looks + 1 if self.news else 0
        if self.news and (self.news_looks >= NEWS_LOOKS or any(proof for _, proof, _ in self.news)):
            self.news_t = now
            what, _, pixel = next((news for news in self.news if news[1]), self.news[0])
            if self.intrusion is None:
                self.intrusion = {"since": round(now, 2)}
                if now - self.said_t > 5.0:  # the timeline is for people: not a line for every twitch of a hand
                    self.said_t = now
                    self.arm.emit("log", level="warn", text=f"work area: {what}")
            self.intrusion.update(what=what, pixel=pixel)
        elif self.intrusion and now - self.news_t > QUIET_S:
            self.intrusion, self.clear_since = None, now
        if self.intrusion and stop_on_intrusion and state["busy"]:
            self.arm.stop()
            self.stopped_arm_t = round(now, 2)

    def _hidden(self, t: dict, near: np.ndarray, boxes) -> bool:
        """Is the place where this track was last measured out of the camera's sight: behind the
        arm, or behind something else that is not an object on the table (a hand)?"""
        u, v = int(round(t["u"])), int(round(t["v"]))
        if not (0 <= v < near.shape[0] and 0 <= u < near.shape[1]) or near[v, u]:
            return True
        return any(b[0] - 12 <= u <= b[2] + 12 and b[1] - 12 <= v <= b[3] + 12 for b in boxes)

    def _sighting(self, t: dict, d: dict, now: float, elsewhere: bool = False) -> None:
        t["misses"] = 0
        if d["cut"]:
            t["state"] = "hidden"
            return
        # Named for the colour it first showed. Two things touching are one outline with one
        # (mixed) colour, and one of them keeps the name when they part: so a name that has not
        # fitted for a second is replaced by one that does. What carries others reads as a mixture.
        fits = d["color"] == t["color"] or any(o["on"] == t["id"] for o in self.tracks)
        t["misnamed"] = 0 if fits else t["misnamed"] + 1
        if t["misnamed"] >= RENAME_LOOKS:
            self._name(t, d["color"])
        if elsewhere or (t["recent"] and _px(d, t) > MOVED):
            if t.get("state") == "visible" and now - t["seen_t"] < 0.35:  # it was in plain view a look ago: nothing hid it, something moved it
                self._tell(f"{t['id']} is being moved", True, d["u"], d["v"])
            t["recent"].clear()
            t["moved_t"] = now
            t["moves"] += 1
        t["recent"].append(d)
        # Where the outline is centred in the image is steady to a pixel while the object lies
        # still. Where it stands on the table is worked out from the outline's shape and wanders
        # by centimetres for anything that is not a plain box, so that is taken as the median
        # of every measurement since the object last moved.
        x, y, u, v = np.median([[r["x"], r["y"], r["u"], r["v"]] for r in t["recent"]], axis=0)
        mid = min(t["recent"], key=lambda r: (r["x"] - x) ** 2 + (r["y"] - y) ** 2)
        t.update({k: mid[k] for k in ("yaw_deg", "width_mm", "footprint_mm", "area_px")}, x=round(float(x), 3), y=round(float(y), 3), u=float(u), v=float(v),
                 pixel=[round(float(c)) for c in self.cam.to_pixel([x, y, self.table_z])], state="visible", seen_t=now, polygon=d["polygon"], box=d["box"])  # fmt: skip
        heights = [r["height_mm"] for r in t["recent"] if r["height_mm"] is not None]
        if len(heights) * 2 > len(t["recent"]):  # only objects with a flat, lit top have a height to measure
            t["height_mm"] = round(float(np.median(heights)))

    def _name(self, t: dict, colour: str) -> None:
        word = colour or "object"
        self.numbers[word] = self.numbers.get(word, 0) + 1
        t.update(id=f"{word}-{self.numbers[word]}", color=colour, misnamed=0)

    def _match(self, seen: list[dict], near, boxes: list, now: float) -> None:
        riders = {t["on"] for t in self.tracks if t["on"]}
        loose = [t for t in self.tracks if t["state"] != "held" and not t["on"]]
        taken_t, taken_d = set(), set()
        for gap, i, j in sorted((_px(t, d), i, j) for i, t in enumerate(loose) for j, d in enumerate(seen)):
            t, d = loose[i], seen[j]
            if i in taken_t or j in taken_d:
                continue
            # By place alone when close; further off, when the colour agrees. A pile reads as some
            # mixture of its colours, and what has been out of sight has no fresh colour to compare.
            if gap < SAME_SPOT or (gap < SAME_OBJECT and (d["color"] == t["color"] or t["id"] in riders or t["state"] != "visible")):
                taken_t.add(i), taken_d.add(j)
                self._sighting(t, d, now)
        # one that is no longer where it was and is seen somewhere else, or fell off what it was put on: told by its colour
        for j, d in enumerate(seen):
            if j in taken_d or d["cut"]:
                continue
            lost = [t for i, t in enumerate(loose) if i not in taken_t and t["color"] == d["color"]]
            lost += [t for t in self.tracks if t["on"] and t["color"] == d["color"]]
            if lost:
                t = min(lost, key=lambda t: _px(t, d))
                taken_d.add(j)
                taken_t.update(i for i, other in enumerate(loose) if other is t)
                t["on"], t["z_mm"] = None, 0
                self._sighting(t, d, now, elsewhere=True)
        for i, t in enumerate(loose):
            if i in taken_t:
                continue
            if self._hidden(t, near, boxes + [d["box"] for j, d in enumerate(seen) if j not in taken_d]):
                t["state"], t["misses"] = "hidden", 0
            else:
                t["misses"] += 1
                if t["misses"] >= GONE_AFTER:
                    t["state"] = "missing"
        # what is left is new: a track once it has been seen in the same place a few looks running
        again = []
        for j, d in enumerate(seen):
            if j in taken_d or d["cut"]:
                continue
            self._tell("something new is on the table", False, d["u"], d["v"])
            before = next((m for m in self.maybe if _px(m, d) < SAME_SPOT), None)
            d["looks"] = (before["looks"] if before else 0) + 1
            if d["looks"] < CONFIRM:
                again.append(d)
                continue
            t = {"height_mm": None, "on": None, "z_mm": 0, "moved_t": now, "moves": 0, "recent": deque(maxlen=25)}
            self._name(t, d["color"])
            self._sighting(t, d, now)
            self.tracks.append(t)
        self.maybe = again
        for t in self.tracks:  # what was put on something goes where that goes
            base = next((b for b in self.tracks if b["id"] == t["on"]), None) if t["on"] else None
            if base:
                t.update(x=base["x"], y=base["y"], u=base["u"], v=base["v"], pixel=base["pixel"], state=base["state"], seen_t=base["seen_t"], moved_t=max(t["moved_t"], base["moved_t"]))
            elif t["on"]:
                t["on"], t["state"] = None, "missing"

    def _gripper(self, gripping: bool, tcp, now: float) -> None:
        held = next((t for t in self.tracks if t["id"] == self.holding), None)
        if gripping and self.holding is None:
            # the jaws just closed on something: the track at the tool point, the top one of a pile
            at = {"x": tcp[0], "y": tcp[1]}
            carried = {t["on"] for t in self.tracks if t["on"]}
            reach = [t for t in self.tracks if _apart(t, at) < GRIP_REACH and t["id"] not in carried]
            held = min(reach, key=lambda t: _apart(t, at), default=None)
            self.holding = held["id"] if held else ""
            if held:
                held.update(state="held", on=None, z_mm=0)
        if held and gripping:
            u, v = (float(c) for c in self.cam.to_pixel(tcp))
            held.update(x=round(float(tcp[0]), 3), y=round(float(tcp[1]), 3), u=u, v=v, pixel=[round(u), round(v)], seen_t=now, moved_t=now)
        if not gripping and self.holding is not None:
            self.holding = None
            if held:  # let go: on whatever is right underneath, else on the table, and out of sight under the gripper for now
                held["recent"].clear()
                base = min((t for t in self.tracks if t is not held and not t["on"] and _apart(t, held) < STACK_REACH), key=lambda t: _apart(t, held), default=None)
                # how high the surface it was put on is: place_at opens the jaws with the tool point 22 mm above it
                surface = max(0, round((float(tcp[2]) - self.table_z) * 1000 - 22)) if base is not None and tcp is not None else 0
                u, v = (float(c) for c in self.cam.to_pixel([held["x"], held["y"], self.table_z + surface / 1000 + 0.012]))  # about where its outline will be centred
                held.update(state="hidden", misses=0, moved_t=now, moves=held["moves"] + 1, u=u, v=v, on=base["id"] if base else None, z_mm=surface)

    # ---- what goes out

    def _report(self, now: float) -> dict:
        keys = ("id", "color", "state", "x", "y", "yaw_deg", "width_mm", "footprint_mm", "height_mm", "on", "z_mm", "moves", "pixel", "box", "area_px")
        tracks = [{**{k: t[k] for k in keys}, "still_s": round(now - t["moved_t"], 1), "seen_s": round(now - t["seen_t"], 1)} for t in self.tracks]
        lying = [e for e in self.edge if e["looks"] >= 10 and e["unseen"] < 3]
        return {
            "t": round(now, 3),
            "mode": self.mode,
            "clear_s": 0.0 if self.intrusion else round(now - self.clear_since, 1),
            "intrusion": self.intrusion,
            "holding": self.holding,
            "stopped_arm_t": self.stopped_arm_t,
            "tracks": tracks,
            "at_edge": [dict(zip("xy", (round(float(c), 3) for c in self.cam.to_base(e["cx"], e["cy"])[:2])), area_px=round(e["area"])) for e in lying],
        }

    def _picture(self, img: np.ndarray) -> np.ndarray:
        colours = {"visible": (120, 255, 120), "hidden": (160, 160, 160), "held": (80, 220, 255), "missing": (90, 90, 255)}
        marks = [{"label": f"{t['id']} {t['state']}" + (f" on {t['on']}" if t["on"] else ""), "point": t["pixel"], "bgr": colours[t["state"]],
                  "polygon": t["polygon"] if t["state"] == "visible" and not t["on"] else None} for t in self.tracks]  # fmt: skip
        marks += [{"label": "at the edge: not tracked" if e["looks"] >= 10 else "", "box": e["box"], "bgr": (0, 160, 255) if e["looks"] >= 10 else (0, 0, 255)} for e in self.edge if not e["unseen"]]
        if self.intrusion:
            img = img.copy()
            vision._label(img, f"INTRUSION: {self.intrusion['what']}", (8, 20), scale=0.55, color=(0, 0, 255))
            cv2.circle(img, tuple(self.intrusion["pixel"]), 22, (0, 0, 255), 2, cv2.LINE_AA)
        return vision.draw_marks(img, marks)

    def show(self, report: dict) -> None:
        """Tell the web UI, when something changed: markers in the 3D view, outlines on the camera."""
        key = json.dumps([(t["id"], t["state"], round(t["x"], 2), round(t["y"], 2)) for t in report["tracks"]])
        if key == self.shown:
            return
        self.shown = key
        on_table = [t for t in self.tracks if t["state"] in ("visible", "hidden")]
        self.arm.emit("markers", markers=[{"id": t["id"], "label": t["id"], "pos": [t["x"], t["y"], self.table_z + 0.01], "color": "#ffd166", "radius": 0.012} for t in on_table])
        self.arm.emit("detections", camera="top", width=640, height=480,
                      items=[{"label": t["id"], "score": 1.0, "box": t["box"], "point": t["pixel"], "polygon": t["polygon"]} for t in on_table if t["state"] == "visible"])  # fmt: skip


def _write(path: Path, data: bytes) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)  # whoever reads sees the old file or the new one, never half


def main() -> None:
    RUNTIME.mkdir(exist_ok=True)
    lock = open(RUNTIME / "tracker.pid", "a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        raise SystemExit("a tracker is already running here") from None
    lock.truncate(0)
    lock.write(str(os.getpid()))
    lock.flush()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    stop_on_intrusion = "--no-stop" not in sys.argv
    FILE.parent.mkdir(parents=True, exist_ok=True)
    arm = Arm(source="tracker")
    watcher, failing_since, n, intrusion_since = None, None, 0, None
    try:
        while True:
            t0 = time.time()
            try:
                watcher = watcher or Watcher(arm)
                report, picture = watcher.look(stop_on_intrusion)
                failing_since = None
            except RobotError as e:  # no frame, no arm, no calibration: say so, and keep trying while the robot side is there
                failing_since = failing_since or t0
                if "not reachable" in str(e) and t0 - failing_since > 5.0:
                    raise SystemExit(f"the robot side is gone: {e}") from None
                report, picture = {"t": round(t0, 3), "error": str(e), "clear_s": 0.0, "intrusion": None, "holding": None, "stopped_arm_t": None, "tracks": [], "at_edge": []}, None
            report["stops_arm_on_intrusion"] = stop_on_intrusion
            _write(FILE, (json.dumps(report, indent=1) + "\n").encode())
            if picture is not None and report["intrusion"] and report["intrusion"]["since"] != intrusion_since:
                intrusion_since = report["intrusion"]["since"]
                _write(FILE.with_name("intrusion.jpg"), cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes())
                _write(FILE.with_name("intrusion.json"), (json.dumps(report, indent=1) + "\n").encode())
            n += 1
            if picture is not None and n % 5 == 0:
                _write(FILE.with_suffix(".jpg"), cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes())
                watcher.show(report)
            time.sleep(max(0.0, PERIOD - (time.time() - t0)))
    finally:
        FILE.unlink(missing_ok=True)  # no file means no tracker: a stale one would be read as the truth


# --------------------------------------------------------------------------- for tools: reading it


def read() -> dict | None:
    """The newest tracks, or None if no tracker is running."""
    try:
        data = json.loads(FILE.read_text())
    except (OSError, ValueError):
        return None
    return data if time.time() - data["t"] < 2.0 else None


def start() -> dict:
    """Make sure the tracker is running and return its first report."""
    data = read()
    if data is None:
        RUNTIME.mkdir(exist_ok=True)
        with open(RUNTIME / "tracker.log", "ab") as log:
            subprocess.Popen([sys.executable, "-m", "util.tracker", *(["--no-stop"] if NO_STOP.exists() else [])], cwd=ROOT / "tools",
                             stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)  # fmt: skip
        deadline = time.time() + 15.0
        while data is None and time.time() < deadline:
            time.sleep(0.1)
            data = read()
        if data is None:
            tail = (RUNTIME / "tracker.log").read_text(errors="replace").strip().splitlines()[-3:]
            raise RobotError("the tracker did not start; .runtime/tracker.log ends: " + " | ".join(tail))
        time.sleep(CONFIRM * PERIOD + 0.2)  # long enough for what is on the table to become tracks
        data = read() or data
    if data.get("error"):
        raise RobotError(f"the tracker cannot see: {data['error']}")
    return data


def stop() -> bool:
    """End the tracker. True if one was running."""
    try:
        with open(RUNTIME / "tracker.pid") as f:
            try:
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                return False  # nobody holds it: the number in it is from a tracker long gone
            except OSError:
                os.kill(int(f.read()), signal.SIGTERM)
    except (OSError, ValueError):
        return False
    for _ in range(30):
        if not FILE.exists():
            break
        time.sleep(0.1)
    return True


def fresh(after: float | None = None, timeout: float = 3.0) -> dict:
    """A report from a camera frame taken after `after` (default: now). The camera lags by a
    tenth of a second, so this waits two looks rather than trust the next one."""
    after = (time.time() if after is None else after) + 0.2
    deadline = time.time() + timeout
    while True:
        data = start()
        if data["t"] > after:
            return data
        if time.time() > deadline:
            raise RobotError("the tracker has stopped reporting")
        time.sleep(0.05)


def track(data: dict, name: str) -> dict:
    for t in data["tracks"]:
        if t["id"] == name:
            return t
    raise RobotError(f"no tracked object named '{name}'. There are: {', '.join(t['id'] + ' (' + t['state'] + ')' for t in data['tracks']) or 'none'}")


def wait_for(ctx, name: str | None = None, patience: float = 20.0, still: float = 0.6) -> dict | None:
    """Wait until the table is quiet (nothing moving that the arm did not move) and, if `name`
    is given, that object is in view and lying still; returns its track. If it stays out of
    sight, the arm goes home once, in case the arm is what hides it. Gives up with the reason
    after `patience` seconds. A tracker told not to stop the arm is not waited on for quiet."""
    from home import home

    deadline, unseen_since, went_home, why = time.time() + patience, None, False, ""
    while time.time() < deadline:
        data = fresh()
        if data["stops_arm_on_intrusion"] and (data["intrusion"] or data["clear_s"] < 0.3):
            why, unseen_since = f"the work area is not quiet ({(data['intrusion'] or {}).get('what', 'something just moved')}): a hand?", None
            continue
        if name is None:
            return None
        t = track(data, name)
        if t["state"] == "held":
            raise RobotError(f"{name} is in the gripper")
        if t["state"] != "visible":
            why = f"{name} is out of sight ({t['state']}), last measured at x={t['x']:+.3f} y={t['y']:+.3f}"
            unseen_since = unseen_since or time.time()
            if time.time() - unseen_since > 1.0 and not went_home:
                try:
                    ctx.call(home)
                    went_home = True
                except RobotError as e:
                    if not stopped_by_tracker(e):
                        raise
            continue
        unseen_since = None
        if t["still_s"] < still:
            why = f"{name} is being moved"
            continue
        return t
    raise RobotError(f"gave up after {patience:.0f} s: {why}")


def stopped_by_tracker(error: RobotError) -> bool:
    """Was this motion cut short by the tracker, because something entered the work area? Then
    it is something to wait out (wait_for) and not a failure. A stop from a person is not."""
    data = read() or {}
    return "stopped" in str(error) and time.time() - (data.get("stopped_arm_t") or 0.0) < 3.0


if __name__ == "__main__":
    main()
