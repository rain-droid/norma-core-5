"""Autopilot: a perception loop feeding a fast decision model that drives the arm by itself.

Three loops run inside the daemon, none of them needs Claude Code:

    perception  ~8 Hz   camera frame -> objects with table coordinates -> `percept` message
    decision    ~1-3 Hz  last ~2 s of percepts + the instruction -> OpenAI model -> action chunk
    execution   continuous: plays the newest chunk, drops what is left of the old one

An action chunk is either up to 40 tool-point waypoints (x, y, z, pitch, grip) or one built-in
skill (fetch, throw, gesture ...) for the things a few-hertz planner cannot do, like the
timing of a throw. This is the usual receding-horizon "action chunking" arrangement: the model
always plans further ahead than it will get to execute before the next plan lands.
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field

import httpx
import numpy as np

from . import vision
from .config import REPO_ROOT, Config
from .profile import ACTIVE as PROFILE
from .robot import RobotError, Stopped
from .skills import COLOR_HSV, GESTURES

log = logging.getLogger("elrobot.autopilot")

SKILLS = ("none", "fetch", "grab", "take", "catch", "lineup", "give", "search", "turn", "throw", "gesture", "place", "home", "open", "close", "look")

SYSTEM = """You are the brain of a small 5-joint robot arm with a parallel gripper and a camera on its wrist, and you are
who the person is talking to. You work in single steps: each reply is ONE action. It is carried out, and then you
are asked again with the result and a fresh view. You never act twice in one reply.

Coordinates: metres in the robot base frame. x is the robot's right, y is forward (away from the base, towards the
person), z is up, the table is z=0. The gripper reaches about 0.10 to {reach:.2f} m from the base. `pitch` is how
far the gripper points below horizontal in degrees: 90 straight down, 0 level forward. `grip` is 1 open, 0 closed.

Reply format: `physical` is true when the instruction asks you to move or handle something, false for talk; `skill` is the routine name or "none"; `arg` is its colour or gesture name, else ""; `xyz` is
[x, y, z] for place, else []; `path` is a waypoint list, each waypoint [x, y, z, pitch, grip], else [].

The one action is either a skill or a path:
- fetch(color): find, grasp and lift the object of that colour, wherever it is: lying on the table or held up in
  the air in someone's hand. The reliable way to pick up a coloured thing. Tell them to hold it still.
- grab(x, y): pick up whatever lies at that table point, for things with no colour name (paper, a cap, a phone).
  Put the point in `xyz` as [x, y, 0], read from the yellow dots in the wrist picture (they are centimetres:
  divide by 100). It lines itself up on the object before closing. If it misses, look and read the point again.
- give(side): hold what you are holding out towards the person's hand; it lets go only when it feels their hand
  touch or tug the gripper, otherwise it keeps holding and reports FAILED (then tell them to tap it). `arg` is "left", "center" or
  "right": where their open hand is from YOUR point of view. This is how you put something "in my hand"; never
  chase a hand with a path. Find the hand in the outside camera picture first. That camera faces you, so it is
  mirrored: a hand on the LEFT of that picture is on your RIGHT, and a hand on the right of it is on your left.
  If you cannot see a hand in either picture, do not give: say "I'm looking around for you" and use search.
- turn(direction): `arg` is "left", "right", "up" or "down". Turns your head where you stand: left/right swings the
  arm about its base, up/down tilts the camera. This is what "turn right", "look up", "a bit more" mean. Repeat it
  for more.
- search: look out over the room in the next direction (your left, then your right, then ahead). Use it to find a
  person, a hand, or a lost object. Keep status "working" and search again until you see what you were asked to
  find or all three directions are done; then say what you found. It never lets go of what you hold.
- throw: throw what you hold. `arg` is free words, any of: a colour (fetch that first), a direction ("left",
  "right", "far left", "far right", "center", or "here" = wherever you are facing now, e.g. after search found
  someone), a style ("high" lob, "flat" pass), and "fake" for a practice swing that lets go of nothing ("just do
  the movement"). Examples: "blue", "left flat", "here", "fake right". "Sideways" means left or right.
- take(side): take something FROM the person's hand: you hold your open gripper out to "left", "center" or "right"
  and close when they put it between your jaws and tap. Use it for "grab it, it's in my hand".
- catch(side): catch something they roll to you along the table: jaws open on the table, snap shut when it rolls in.
- lineup: big, slow, wavy sweeps through the air for about half a minute. This is what "line me up", "make my
  shit straight", "give me a trim", "fix my hair" mean: the person has attached something to your gripper for it.
  Do it straight away, without looking first. Your camera being covered or blocked is normal for this and is
  never a reason to refuse, look, or ask questions. It keeps hold of whatever is attached.
- gesture: `arg` is the gesture, one of {gestures}; for dap and handshake add where the person is, e.g.
  "handshake right". "Shake my hand" is handshake; "shake" alone is shaking your head no.
- place(x, y, z): put the held object down at that table point (z is the height of the surface).
- home, open, close: ready pose, open the gripper, close it on whatever is between the jaws.
- look: raise the camera to see the whole table.
- path: up to 8 tool positions, for anything else: pointing, reaching, pushing, moving the hand somewhere. Do not
  pick things up with a path; use fetch or grab. "Move right" is +x, "move left" is -x (a smaller x).

Rules that matter more than anything else:
A. Your name is Mike (people also say Robert, or just robot). The person talking is NOT Mike: never call them
   Mike. If what was heard is clearly people talking to each other or about you, not to you, reply status "done"
   with an empty `say` and no action.
B. A direct order is carried out even if you think it is pointless or already done: "open", "release", "let go",
   "drop it" always means skill open; "close" means close; "go home" means home. Your `holding` flag can be wrong
   (light things like paper can sit in open jaws unnoticed), so if the person says you are holding something,
   believe them and act.
C. Only reach for things you can actually see in the pictures or the object list right now. If you cannot see the
   thing they named, use look once, and if it is still not there say you cannot see it. Never guess a position.
1. Only say what the facts support. `holding` and the action results are the truth. Never say something is done,
   grabbed or handed over unless the latest result or `holding` shows it. If you cannot tell, say so.
2. If the person asked for a movement, the reply must contain an action. Never answer a request to move with words only.
0. Never let go of what you are holding unless the person asked for it to be given, placed, thrown or dropped.
   Only give, place, throw and open release it. While you look around or move, the gripper stays shut.
3. A miss is not the end. When a pick-up FAILED or closed on nothing and you have actions left, try again in the
   next step with something changed: look first, aim from the fresh picture, or use a different height. Only stop
   after a second failed attempt. When an action FAILED, do not repeat it unchanged. If the retry fails too, stop
   with status "stuck" and ask the person what they want.
4. Chain actions. An instruction often has several parts ("find it, pick it up, then throw it to him", "find my
   hand and dap me"): do them in order, one action per reply, with status "working" until the last part, which is
   "done". If a single action is the whole job, mark it "done" straight away. "stuck" if you cannot do it. You get
   at most {steps} actions per instruction, so do not waste them repeating a step that already worked.
5. `say` is spoken aloud with every action, and you are a talker. Say two or three full sentences each time,
   up to about forty words: what you are about to do, a bit of commentary or trash talk, a joke, a question back
   at them. When they are just chatting, give a real answer of a few sentences and keep the conversation going,
   never a two-word reply. No emoji, and never the same line twice. Do not shout "Kobe" yourself on a throw; that
   is added for you.

Personality, in what you say: laid-back, funny, a bit cocky, talks like a friend ("alright, bet", "say less",
"oh shit, my bad"). "Dap me up" means gesture dap with "alright, bet". If they are only chatting, answer in `say`
with status "done" and no action.

The pictures: the wrist camera has yellow dots on the table labelled x,y in centimetres; read positions of
anything lying on the table from them. The other camera looks at you from the person's side and has no
coordinates. The object list only knows coloured things; `age` is seconds since one was last seen."""

# Replies are kept as short as possible: almost all of the response time is output tokens.
SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": ["physical", "status", "say", "skill", "arg", "xyz", "path"],
    "properties": {
        "physical": {"type": "boolean"},
        "status": {"type": "string", "enum": ["working", "done", "stuck"]},
        "say": {"type": "string"},
        "skill": {"type": "string", "enum": list(SKILLS)},
        "arg": {"type": "string"},
        "xyz": {"type": "array", "items": {"type": "number"}},
        "path": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}},
    },
}


def parse_reply(d: dict) -> dict:
    """The compact reply -> {"status", "say", "skill": {...}, "waypoints": [...]}."""
    arg = (d.get("arg") or "").strip().lower()
    xyz = [float(v) for v in (d.get("xyz") or [])][:3]
    xyz += [0.0] * (3 - len(xyz))
    wps = []
    for w in (d.get("path") or [])[:40]:
        if len(w) >= 3:
            wps.append({"x": float(w[0]), "y": float(w[1]), "z": float(w[2]), "pitch": float(w[3]) if len(w) > 3 else 60.0,
                        "grip": float(w[4]) if len(w) > 4 else 1.0})  # fmt: skip
    return {"status": d["status"], "say": d.get("say") or "", "waypoints": wps, "physical": bool(d.get("physical")),
            "skill": {"name": d.get("skill") or "none", "color": arg, "gesture": arg, "x": xyz[0], "y": xyz[1], "z": xyz[2]}}  # fmt: skip


# Hands-free listening only reacts to speech that says one of these (as Whisper tends to spell them).
WAKE_WORDS = ("robot", "robo", "robots", "roboto", "arm", "bot", "robert", "roberts", "roberto", "rob", "robb", "robbie",
              "robby", "bob", "bobby", "bert", "rupert", "rabbit", "robber", "rober", "robin", "hobart", "herbert", "albert",
              "mike", "mikes", "mikey", "mic", "mick", "micky", "mickey", "michael", "mikael", "mike's")


def _short(line: str, words: int = 60) -> str:
    """What gets spoken: no emoji, and cut at a sentence end if it runs past `words`."""
    line = "".join(c for c in line if ord(c) < 0x2190 or 0x2010 <= ord(c) <= 0x201F).strip()
    w = line.split()
    if len(w) <= words:
        return line
    cut = " ".join(w[:words])
    last = max(cut.rfind("."), cut.rfind("!"), cut.rfind("?"))
    return cut[: last + 1] if last > 20 else cut


def api_key() -> str | None:
    key = os.environ.get("OPENAI_API_KEY")
    env = REPO_ROOT / ".env"
    if not key and env.exists():
        for line in env.read_text().splitlines():
            if line.strip().startswith("OPENAI_API_KEY="):
                key = line.split("=", 1)[1].strip().strip("'\"")
    return key or None


@dataclass
class Tracked:
    label: str
    pos: np.ndarray
    seen: float
    px: tuple[int, int]
    area: float


@dataclass
class Run:
    prompt: str
    started: float = field(default_factory=time.time)
    status: str = "working"
    say: str = ""
    decisions: int = 0
    history: list = field(default_factory=list)  # what earlier actions did
    said: list = field(default_factory=list)
    look: int = 0  # how many of the next decisions get camera pictures


class Autopilot:
    def __init__(self, hub, cfg: Config):
        self.hub, self.cfg = hub, cfg
        self.robot, self.skills = hub.robot, hub.skills
        self.percepts: deque[dict] = deque(maxlen=int(cfg.autopilot_window_s * cfg.autopilot_percept_hz) + 1)
        self.objects: dict[str, Tracked] = {}
        self.run: Run | None = None
        self._task: asyncio.Task | None = None
        self._plan: list[dict] = []
        self._plan_id = 0
        self._q_prev: np.ndarray | None = None
        self._http = httpx.AsyncClient(timeout=20)
        self.dialogue: deque[tuple[str, str]] = deque(maxlen=12)  # ("person" | "you", line)
        self.stt = self.tts = None  # local speech models, loaded in the background
        self.speech_ready = False
        self._spoken: deque[tuple[float, str]] = deque(maxlen=8)
        self._hush_until = 0.0
        self._acting = False  # an action is physically running right now
        self._pending: str | None = None  # instruction heard mid-action, to run next
        self.latency_ms = 0

    # ================================================================== perception

    def _detect(self, img: np.ndarray, T: np.ndarray) -> list[Tracked]:
        """Colour blobs in a wrist frame, as points on the table. Runs in a worker thread."""
        cam = self.skills._cam(img)
        now, out = time.time(), []
        for color in COLOR_HSV:
            if color in ("orange", "red", "pink") and PROFILE.name == "so101":
                continue  # the gripper itself is orange
            for b in self.skills._color_blobs(img, color, min_area=220.0)[:2]:
                p = cam.pixel_to_plane(*b.centroid, T, self.skills.table_z + 0.012)
                if p is None or p[1] < 0.0 or math.hypot(p[0], p[1] - PROFILE.base_y) > PROFILE.reach_m + 0.10:
                    continue
                out.append(Tracked(color, np.array([p[0], p[1], self.skills.table_z]), now, (int(b.centroid[0]), int(b.centroid[1])), float(b.area)))
        return out

    async def perception_loop(self) -> None:
        period = 1.0 / self.cfg.autopilot_percept_hz
        while True:
            t0 = time.monotonic()
            try:
                await self._perceive_once()
            except Exception:
                log.exception("perception tick failed")
            await asyncio.sleep(max(0.01, period - (time.monotonic() - t0)))

    async def _perceive_once(self) -> None:
        r = self.robot
        if not r.link.connected or not r.ready:
            return
        q = r.q()
        moving = self._q_prev is not None and float(np.max(np.abs(q - self._q_prev))) > 0.02
        self._q_prev = q
        fr = r.link.frames.get("wrist")
        now = time.time()
        fresh = False
        # A frame is ~100 ms old: while the arm moves, the pose it was taken at is not the pose now,
        # so positions are only trusted from frames taken standing still.
        if fr is not None and time.monotonic() - fr.stamp < 0.5 and not moving:
            T = r.tcp()
            found = await asyncio.to_thread(lambda: self._detect(vision.decode(fr.jpeg), T))
            fresh = True
            for t in found:
                old = self.objects.get(t.label)
                if old is not None and now - old.seen < 1.0:
                    t.pos = 0.5 * old.pos + 0.5 * t.pos
                self.objects[t.label] = t
        for k in [k for k, t in self.objects.items() if now - t.seen > 60]:
            del self.objects[k]
        ee = r.tcp()
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -float(ee[2, 2])))))
        p = {
            "type": "percept",
            "t": round(now, 2),
            "ee": [round(float(v), 3) for v in ee[:3, 3]],
            "pitch": round(pitch),
            "grip": round(float(r.gripper_open()), 2),
            "holding": bool(r.holding),
            "moving": moving,
            "activity": r.activity,
            "camera_fresh": fresh,
            "objects": [
                {"label": t.label, "x": round(float(t.pos[0]), 3), "y": round(float(t.pos[1]), 3), "z": round(float(t.pos[2]), 3),
                 "age": round(now - t.seen, 1), "px": list(t.px)}
                for t in sorted(self.objects.values(), key=lambda t: t.label)
            ],  # fmt: skip
        }
        self.percepts.append(p)
        self.hub._broadcast(p)

    # ================================================================== decision

    def _messages(self, run: Run) -> list[dict]:
        P = list(self.percepts)
        now = time.time()
        lines = []
        step = max(1, len(P) // self.cfg.autopilot_history)
        for p in P[::-1][::step][: self.cfg.autopilot_history][::-1]:
            objs = " ".join(f"{o['label']}@({o['x']:+.2f},{o['y']:+.2f}) age {o['age']}s" for o in p["objects"]) or "nothing"
            lines.append(f"t{p['t'] - now:+.1f}s ee=({p['ee'][0]:+.2f},{p['ee'][1]:+.2f},{p['ee'][2]:.2f}) pitch={p['pitch']} grip={p['grip']} "
                         f"holding={str(p['holding']).lower()} {'moving' if p['moving'] else 'still'} | sees: {objs}")  # fmt: skip
        queued = " ".join(f"({w['x']:+.2f},{w['y']:+.2f},{w['z']:.2f})" for w in self._plan[:40]) or "none"
        talk = "\n".join(f"{who}: {line}" for who, line in self.dialogue) or "nothing yet"
        text = (
            f"Conversation so far:\n{talk}\n\nWhat you already said out loud on this instruction: {' | '.join(run.said) or 'nothing'}\n\n"
            f"Instruction: {run.prompt}\n\nRunning for {now - run.started:.0f}s.\n\n"
            "Your earlier actions, oldest first:\n" + ("\n".join(run.history) or "none yet") + "\n\n"
            f"The arm is standing still. Actions used: {sum(1 for h in run.history if not h.startswith('(reminder)'))} of {self.cfg.autopilot_max_steps}.\n\nLast two seconds of perception, oldest first:\n" + ("\n".join(lines) or "no perception yet")
        )
        content: list[dict] = [{"type": "text", "text": text}]
        if self.cfg.autopilot_images or run.look > 0:
            run.look -= 1
            import cv2

            fr = self.robot.link.frames.get("wrist")
            if fr is not None and time.monotonic() - fr.stamp < 1.0:
                b64 = base64.b64encode(vision.encode(self._grid_view(vision.decode(fr.jpeg)), 75)).decode()
                content.append({"type": "text", "text": "Wrist camera now. The yellow dots are points on the table labelled with their x,y in "
                                "centimetres in the robot frame: read off where anything is from the dots around it."})  # fmt: skip
                content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}", "detail": "high"}})
            fr = self.robot.link.frames.get("top")
            if fr is not None and time.monotonic() - fr.stamp < 1.0:
                small = cv2.resize(vision.decode(fr.jpeg), (512, 288))
                b64 = base64.b64encode(vision.encode(small, 70)).decode()
                content.append({"type": "text", "text": "Outside camera now, looking at the robot from the person's side (no coordinates):"})
                content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}", "detail": "low"}})
        system = SYSTEM.format(reach=PROFILE.reach_m, gestures=", ".join(GESTURES), steps=self.cfg.autopilot_max_steps)
        return [{"role": "system", "content": system}, {"role": "user", "content": content}]

    def _grid_view(self, img: np.ndarray) -> np.ndarray:
        """The wrist frame with table points drawn on it, so a vision model can read coordinates."""
        import cv2

        img = cv2.resize(img, (768, 432))
        cam, T = self.skills._cam(img), self.robot.tcp()
        for xc in range(-30, 31, 5):
            for yc in range(5, 46, 5):
                uv = cam.project([xc / 100, yc / 100, self.skills.table_z], T)
                if uv is None or not (12 < uv[0] < 756 and 12 < uv[1] < 420):
                    continue
                u, v = int(uv[0]), int(uv[1])
                cv2.circle(img, (u, v), 3, (0, 0, 0), -1)
                cv2.circle(img, (u, v), 2, (0, 255, 255), -1)
                cv2.putText(img, f"{xc},{yc}", (u + 4, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, f"{xc},{yc}", (u + 4, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1, cv2.LINE_AA)
        return img

    async def _decide(self, run: Run) -> dict:
        key = api_key()
        if not key:
            raise RobotError("no OpenAI key: put OPENAI_API_KEY=... in .env at the repo root")
        body = {
            "model": self.cfg.autopilot_model,
            "messages": self._messages(run),
            "response_format": {"type": "json_schema", "json_schema": {"name": "action_chunk", "strict": True, "schema": SCHEMA}},
        }
        if self.cfg.autopilot_reasoning:
            body["reasoning_effort"] = self.cfg.autopilot_reasoning
        if self.cfg.autopilot_tier:
            body["service_tier"] = self.cfg.autopilot_tier
        t0 = time.monotonic()
        r = await self._http.post(f"{self.cfg.autopilot_base_url}/chat/completions", json=body, headers={"Authorization": f"Bearer {key}"})
        self.latency_ms = int((time.monotonic() - t0) * 1000)
        if r.status_code != 200:
            raise RobotError(f"decision model error {r.status_code}: {r.text[:300]}")
        return parse_reply(json.loads(r.json()["choices"][0]["message"]["content"]))

    # ================================================================== execution

    async def _run_skill(self, run: Run, s: dict) -> None:
        name = s["name"]
        if name == "look":
            res = await self.hub.call_tool("observe", {}, source="autopilot")
            run.history.append(f"look -> {'ok: camera raised over the table, see the pictures' if res.get('ok') else 'FAILED: ' + res.get('summary', '')[:120]}")
            return
        color = (s.get("color") or "").strip().lower()
        args: dict = {}
        tool = name
        if name == "fetch":
            args = {"color": color}
        elif name == "throw":
            words = color.replace(",", " ").split()
            args = {}
            hue = next((w for w in words if w in COLOR_HSV), None)
            if hue:
                args["color"] = hue
            far = "far" in words or "sideways" in words
            if "left" in words:
                args["pan"] = -75.0 if far else -40.0
            elif "right" in words:
                args["pan"] = 75.0 if far else 40.0
            elif "here" in words:
                args["here"] = True
            if "flat" in words or "low" in words:
                args["style"] = "flat"
            elif "high" in words or "lob" in words:
                args["style"] = "high"
            if any(w in words for w in ("fake", "dry", "practice", "mime")):
                args["dry"] = True
        elif name in ("take", "catch"):
            args = {"side": next((w for w in color.split() if w in ("left", "center", "right")), "center")}
        elif name == "grab":
            tool, args = "pick", {"x": s["x"], "y": s["y"]}
        elif name == "turn":
            args = {"direction": color}
        elif name == "give":
            args = {"side": color if color in ("left", "center", "right") else "center"}
        elif name == "gesture":
            words = (s.get("gesture") or "nod").split()
            g = next((w for w in words if w in GESTURES), words[0])
            args = {"name": {"shakehand": "handshake", "fistbump": "dap"}.get(g, g)}
            side = next((w for w in words if w in ("left", "center", "right")), None)
            if side:
                args["side"] = side
        elif name == "place":
            args = {"x": s["x"], "y": s["y"], "z": s["z"]}
        elif name == "open":
            tool, args = "gripper", {"open": 1.0}
        elif name == "close":
            tool = "grasp"
        shout = asyncio.create_task(self._callout()) if name == "throw" and not args.get("dry") else None
        try:
            res = await self.hub.call_tool(tool, args, source="autopilot")
        finally:
            if shout:
                shout.cancel()
        run.history.append(f"{name}({json.dumps(args)}) -> {'ok' if res.get('ok') else 'FAILED'}: {res.get('summary', '')[:160]}")

    async def _play(self, run: Run) -> None:
        """Play queued waypoints until the queue is empty or replaced."""
        r = self.robot
        while self._plan and self.run is run:
            pid, wp = self._plan_id, self._plan.pop(0)
            pos = [float(wp["x"]), float(wp["y"]), max(self.cfg.floor_clearance, float(wp["z"]))]
            try:
                async with self.hub._task_lock:
                    r.source = "autopilot"
                    try:
                        await r.move_to(pos, pitch=float(wp["pitch"]), speed=self.cfg.autopilot_speed, label="autopilot")
                    except RobotError as e:
                        if isinstance(e, Stopped):
                            raise
                        await r.move_to(pos, speed=self.cfg.autopilot_speed, label="autopilot")  # any pitch that reaches
                    grip = float(wp["grip"])
                    if grip < 0.3 and r.gripper_open() > 0.35 and not r.holding:
                        g = await r.grasp()
                        run.history.append(f"closed gripper at ({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:.2f}) -> {'ok: holding something' if g['holding'] else 'FAILED: closed on nothing, the gripper is empty'}")
                    elif grip > 0.7 and r.gripper_open() < 0.6:
                        await r.set_gripper(1.0)
                        run.history.append(f"opened gripper at ({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:.2f})")
            except Stopped:
                raise
            except RobotError as e:
                run.history.append(f"waypoint ({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:.2f}) FAILED: {str(e)[:140]}")
                if pid == self._plan_id:
                    self._plan.clear()  # the rest of this chunk assumed we got there
                return
            finally:
                r.source = None
        if self.run is run and not self._plan:
            ee = r.tcp()[:3, 3]
            run.history.append(f"path finished ok, tool now at ({ee[0]:+.2f},{ee[1]:+.2f},{ee[2]:.2f}), holding={str(bool(r.holding)).lower()}")

    def _scene(self) -> dict:
        return {"holding": bool(self.robot.holding), **{k: t.pos.copy() for k, t in self.objects.items()}}

    def _scene_changed(self, then: dict) -> bool:
        """Has anything happened that a plan made `then` could not have known about?"""
        now = self._scene()
        if now["holding"] != then["holding"] or set(now) != set(then):
            return True
        return any(float(np.linalg.norm(now[k] - then[k])) > 0.025 for k in now if k != "holding")

    async def _loop(self, run: Run) -> None:
        """One step at a time: decide, do that one thing, look at what happened, decide again.

        Nothing is decided while the arm is moving, so every decision is made from a settled
        scene and the result of the last action. After `autopilot_max_steps` actions the run
        stops and hands back to the person instead of pressing on with a plan that is not working.
        """
        cfg = self.cfg
        steps = 0
        last_failed = None
        pushes = 0
        try:
            while self.run is run:
                if time.time() - run.started > cfg.autopilot_max_s:
                    run.status, run.say = "stuck", "That took too long, I stopped."
                    break
                d = await self._decide(run)
                if self.run is not run:
                    break
                run.decisions += 1
                skill, wps = d["skill"], (d.get("waypoints") or [])[: cfg.autopilot_max_waypoints]
                acting = skill["name"] != "none" or bool(wps)
                say = _short(d.get("say") or "")
                self.hub.emit("autopilot", phase="decision", n=run.decisions, latency_ms=self.latency_ms, status=d["status"],
                              say=say, skill=skill["name"], waypoints=wps, prompt=run.prompt)  # fmt: skip
                if not acting:
                    unfinished = bool(run.history) and "FAILED" in run.history[-1] and d["status"] != "stuck"
                    idle = not run.history and d.get("physical")
                    if (unfinished or idle) and pushes < 2 and steps < cfg.autopilot_max_steps:
                        pushes += 1
                        run.history.append("(reminder) that last step FAILED, so the job is not done: try again now with something changed"
                                           if unfinished else "(reminder) the person asked for something physical: reply with an action, not just words")
                        continue
                    if d["status"] == "working":
                        d["status"] = "stuck"  # still to do, but nothing proposed: hand back rather than spin
                    run.status, run.say = d["status"], say
                    break
                self._speak(run, say)  # a line with every step; repeats are dropped
                steps += 1
                mark = len(run.history)
                self._acting = True
                try:
                    if skill["name"] != "none":
                        await self._run_skill(run, skill)
                    else:
                        self._plan_id += 1
                        self._plan[:] = wps
                        run.history.append(f"started {len(wps)} waypoints: " + " ".join(f"({w['x']:+.2f},{w['y']:+.2f},{w['z']:.2f})" for w in wps))
                        await self._play(run)
                finally:
                    self._acting = False
                if self._pending:
                    run.status, run.say = "stopped", ""  # the person said something new meanwhile: that comes first
                    break
                failed = any("FAILED" in h for h in list(run.history)[min(mark, len(run.history) - 1):])
                if failed and "FAILED" not in run.history[-1]:
                    run.history.append("that step FAILED (see above): the job is not done yet")
                if d["status"] == "done" and not failed:
                    run.status, run.say = "done", ""  # its line was written before the action ran: stale now
                    break
                key = (skill["name"], skill["color"]) if skill["name"] != "none" else ("path", round(wps[-1]["x"], 2), round(wps[-1]["y"], 2), round(wps[-1]["z"], 2))
                if failed and key is not None and key == last_failed:
                    run.status, run.say = "stuck", "That failed twice. What do you want me to do?"
                    break
                last_failed = key if failed else None
                if steps >= cfg.autopilot_max_steps:
                    run.status = "stuck"
                    run.say = "" if "FAILED" not in run.history[-1] else "That is not working. What do you want me to do?"
                    break
                await asyncio.sleep(0.35)  # let the arm settle and the camera catch up before judging the result
            self._speak(run, run.say)
        except Stopped:
            run.status, run.say = "stopped", "stopped"
        except asyncio.CancelledError:
            run.status = "stopped"
            raise
        except Exception as e:
            log.exception("autopilot failed")
            run.status, run.say = "error", str(e)[:200]
        finally:
            self._plan.clear()
            self.hub.emit("autopilot", phase="end", status=run.status, say=run.say, decisions=run.decisions, prompt=run.prompt,
                          seconds=round(time.time() - run.started, 1))  # fmt: skip
            if self.run is run:
                self.run = None
                nxt, self._pending = self._pending, None
                if nxt:
                    asyncio.get_running_loop().call_soon(lambda: asyncio.create_task(self.start(nxt)))

    # ================================================================== voice

    async def warm_speech(self) -> None:
        """Load Whisper (speech to text) and Kokoro (text to speech), both local and open source."""
        try:
            from .voice.stt import STT
            from .voice.tts import TTS

            self.stt, self.tts = STT(), TTS()
            await asyncio.gather(self.stt.awarm(), self.tts.awarm())
            self.speech_ready = True
            self.hub.emit("log", level="info", text=f"autopilot voice ready (speech out: {self.tts.active or 'local'})")
        except Exception as e:
            log.exception("speech models failed to load")
            self.hub.emit("log", level="error", text=f"autopilot voice unavailable: {e}")

    CALLOUTS = ("Kobe!", "Kobe!", "Kobe!", "Buckets!", "From downtown!", "Splash!", "Yeet!")

    async def _callout(self) -> None:
        """Shout as the throw leaves the hand. The clip is made during the wind-up so it lands on the swing."""
        line = self.CALLOUTS[int(time.time() * 7) % len(self.CALLOUTS)]
        clip = None
        if self.tts is not None:
            try:
                clip = await self.tts.asynthesize(line)
            except Exception:
                log.exception("callout synthesis failed")
        for _ in range(1500):  # up to 30 s: a throw with a colour picks the thing up first
            if self.robot.activity == "throwing":
                break
            await asyncio.sleep(0.02)
        else:
            return
        self._spoken.append((time.monotonic(), line))
        self.dialogue.append(("you", line))
        self.hub.emit("autopilot", phase="speech", text=line, url=f"/api/autopilot/audio/{clip.id}.wav" if clip else None)

    def _speak(self, run: Run, line: str) -> None:
        line = (line or "").strip()
        if not line or line.lower() in (x.lower() for x in run.said):
            return
        run.said.append(line)
        self.dialogue.append(("you", line))
        asyncio.create_task(self._synthesize(line))

    async def _synthesize(self, line: str) -> None:
        if time.monotonic() < self._hush_until:
            return
        self._spoken.append((time.monotonic(), line))
        url = None
        if self.tts is not None:
            try:
                clip = await self.tts.asynthesize(line)
                url = f"/api/autopilot/audio/{clip.id}.wav"
            except Exception:
                log.exception("speech synthesis failed")
        self.hub.emit("autopilot", phase="speech", text=line, url=url)

    def _is_echo(self, text: str) -> bool:
        """Is this the microphone picking our own voice back up? Most of its words were just spoken by us."""
        words = [w for w in "".join(c if c.isalnum() else " " for c in text.lower()).split() if len(w) > 2]
        if len(words) < 2:
            return False
        now = time.monotonic()
        for t, line in self._spoken:
            if now - t < 8.0:
                mine = set("".join(c if c.isalnum() else " " for c in line.lower()).split())
                if sum(w in mine for w in words) >= 0.7 * len(words):
                    return True
        return False

    def hush(self) -> None:
        """The person talked over us: whatever is still to be said for this instruction is dropped."""
        self._hush_until = time.monotonic() + 1.5

    async def hear(self, data: bytes, mime: str | None, need_name: bool = False) -> dict:
        """One recorded utterance: transcribe it and act on it."""
        from .voice.service import is_stop_command

        if self.stt is None:
            return {"ok": False, "error": "speech models are still loading"}
        t0 = time.monotonic()
        try:
            tr = await self.stt.atranscribe(data, mime)
        except Exception as e:
            return {"ok": False, "error": f"could not understand the audio: {e}"}
        text = (tr.text or "").strip()
        ms = int((time.monotonic() - t0) * 1000)
        if not text:
            return {"ok": True, "text": "", "stt_ms": ms}
        if self._is_echo(text):
            return {"ok": True, "text": "", "stt_ms": ms, "echo": True}
        if need_name and not is_stop_command(text):
            # Hands-free in a room full of people: only speech that names the robot is for the robot.
            low = "".join(c if c.isalpha() else " " for c in text.lower())
            hit = next((w for w in low.split() if w in WAKE_WORDS
                        or (w.startswith(("robo", "robe")) and not w.startswith(("robotic", "robbe")))), None)
            if hit is None:
                self.hub.emit("autopilot", phase="ignored", text=text)
                return {"ok": True, "text": "", "stt_ms": ms, "ignored": text}
        self.hub.emit("autopilot", phase="heard", text=text, stt_ms=ms)
        if is_stop_command(text):
            await self.hub.stop()
            return {"ok": True, "text": text, "stt_ms": ms, "stopped": True}
        if self.run is not None and self._acting:
            # Mid-action (a pick, a throw): cutting it off halfway leaves the arm in a bad place, and
            # half the time the words were not a new order anyway. It runs as soon as this action ends.
            self._pending = text
            self.hub.emit("autopilot", phase="queued", text=text)
            return {"ok": True, "text": text, "stt_ms": ms, "queued": True}
        return {**await self.start(text), "text": text, "stt_ms": ms}

    # ================================================================== control

    async def start(self, prompt: str) -> dict:
        prompt = (prompt or "").strip()
        if not prompt:
            return {"ok": False, "error": "empty instruction"}
        if not api_key():
            return {"ok": False, "error": "no OpenAI key: put OPENAI_API_KEY=... in .env at the repo root"}
        await self.cancel()
        self.dialogue.append(("person", prompt))
        self.run = Run(prompt)
        self.hub.emit("autopilot", phase="start", prompt=prompt, model=self.cfg.autopilot_model)
        self._task = asyncio.create_task(self._loop(self.run))
        return {"ok": True, "prompt": prompt, "model": self.cfg.autopilot_model}

    async def cancel(self) -> None:
        self.run = None
        self._acting = False
        self._plan.clear()
        t, self._task = self._task, None
        if t and not t.done():
            t.cancel()
            try:
                await t
            except (asyncio.CancelledError, Exception):
                pass

    def status(self) -> dict:
        run = self.run
        return {
            "running": run is not None,
            "prompt": run.prompt if run else None,
            "status": run.status if run else "idle",
            "decisions": run.decisions if run else 0,
            "latency_ms": self.latency_ms,
            "model": self.cfg.autopilot_model,
            "has_key": api_key() is not None,
            "percept": self.percepts[-1] if self.percepts else None,
        }
