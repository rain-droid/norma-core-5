"""Tools for Codex: observe, propose, execute once, inspect, verify."""
from __future__ import annotations

import asyncio
import base64
import json
import math
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

from .executor import Executor
from .geometry import CleanupError, TableMap

TRASH_CLASSES = {"crumpled_paper", "snack_wrapper"}


class Harness:
    def __init__(self, backend, model, table: TableMap, output_dir):
        self.backend, self.model, self.table = backend, model, table
        self.out = Path(output_dir).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.executor = Executor(backend, model, table.floor)
        self.observations, self.plans, self.runs = {}, {}, {}
        self.epoch = 0
        self.busy = False
        self.planning = False
        self.verified = 0
        self.not_before_ns = 0

    def log(self, event, **data):
        with (self.out / "events.jsonl").open("a") as f:
            f.write(json.dumps({"time": time.time(), "event": event, **data}, allow_nan=False) + "\n")

    def status(self):
        state = self.backend.state()
        return {"mode": self.backend.mode, "motion_enabled": self.backend.enabled, "busy": self.busy,
                "planning": self.planning, "verified_cleanups": self.verified, "state": state,
                "tool_xyz_m": self.model.fk(state["q"])[:3, 3].tolist(),
                "accepted_trash": sorted(TRASH_CLASSES), "drop_xy_m": self.table.drop.tolist()}

    def checked_frame(self, frame):
        if (frame.width, frame.height) != self.table.size or frame.camera_id != self.table.config["camera_id"]:
            raise CleanupError("camera identity/resolution differs from table calibration")
        if not -.1 <= (time.time_ns() - frame.captured_ns) / 1e9 <= .75:
            raise CleanupError("camera frame is stale")

    async def get_observation(self):
        return await self._capture()

    async def _capture(self, completed=False):
        state = self.backend.state()
        async with asyncio.timeout(3):
            while True:
                frame = await self.backend.look()
                if frame.captured_ns >= self.not_before_ns:
                    break
                await asyncio.sleep(.05)
        self.checked_frame(frame)
        ident = uuid.uuid4().hex
        path = self.out / f"{ident}.jpg"
        path.write_bytes(frame.jpeg)
        result = {"observation_id": ident, "camera_id": frame.camera_id, "image_size": [frame.width, frame.height],
                  "captured_ns": frame.captured_ns, "image_path": str(path), "mode": self.backend.mode,
                  "state": state, "busy": self.busy and not completed, "epoch": self.epoch}
        self.observations[ident] = {**result, "received": time.monotonic(), "jpeg": frame.jpeg}
        # Retain only recent observations in memory; files/event records remain inspectable.
        for key in list(self.observations)[:-20]:
            del self.observations[key]
        self.log("observation", **result)
        return {**result, "images": [{"mime": "image/jpeg", "b64": base64.b64encode(frame.jpeg).decode()}]}

    def observation(self, ident):
        obs = self.observations.get(ident)
        if obs is None or time.monotonic() - obs["received"] > 60 or obs["epoch"] != self.epoch:
            raise CleanupError("observation is missing, expired, or predates motion; look again")
        return obs

    def build_plan(self, start, xy):
        paths, q = {}, np.array(start)
        self.model.validate(q, self.table.floor)
        initial = self.model.fk(q)[:3, 3]
        self.table.check_xy(initial[:2])
        stages = [
            ("raise", [*initial[:2], self.table.travel_z]),
            ("approach", [*xy, self.table.travel_z]),
            ("descend", [*xy, self.table.grasp_z]),
            ("lift", [*xy, self.table.travel_z]),
            ("carry", [*self.table.drop, self.table.travel_z]),
            ("drop", [*self.table.drop, self.table.drop_z]),
            ("retreat", [*self.table.drop, self.table.travel_z]),
            ("return", [*initial[:2], self.table.travel_z]),
        ]
        for name, xyz in stages:
            self.table.check_xy(xyz[:2])
            paths[name] = self.model.line(q, xyz, self.table.floor)
            q = np.array(paths[name][-1])
        return paths

    async def plan_pick(self, observation_id, label, confidence, grasp_pixel, scene_clear=False):
        if self.busy or self.planning:
            raise CleanupError("another action or plan is active")
        if label not in TRASH_CLASSES:
            raise CleanupError("only crumpled_paper and snack_wrapper may be picked")
        if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not math.isfinite(confidence) or not .85 <= confidence <= 1:
            raise CleanupError("need confidence >=0.85; uncertain objects must be left alone")
        if scene_clear is not True:
            raise CleanupError("inspect the full arm path for hands/obstacles before planning")
        obs = self.observation(observation_id)
        if obs["busy"]:
            raise CleanupError("take an observation while the arm is idle")
        xy = self.table.project(grasp_pixel)
        epoch = self.epoch
        start = self.backend.state()["q"]
        self.planning = True
        try:
            paths = await asyncio.to_thread(self.build_plan, start, xy)
        finally:
            self.planning = False
        if epoch != self.epoch:
            raise CleanupError("stop/motion occurred during planning; discard this plan")
        ident = uuid.uuid4().hex
        plan = {"plan_id": ident, "observation_id": observation_id, "label": label, "confidence": confidence,
                "grasp_pixel": list(grasp_pixel), "target_xy_m": xy.tolist(), "drop_xy_m": self.table.drop.tolist(),
                "start_q": start, "paths": paths, "created": time.monotonic(), "epoch": epoch, "used": False}
        self.plans[ident] = plan
        self.log("plan", **plan)
        return {k: v for k, v in plan.items() if k not in ("created", "epoch", "used")}

    async def execute_plan(self, plan_id):
        if self.busy or self.planning:
            raise CleanupError("arm is busy")
        plan = self.plans.get(plan_id)
        if plan is None or plan["used"] or plan["epoch"] != self.epoch or time.monotonic() - plan["created"] > 60:
            raise CleanupError("plan is missing, used, expired, or invalidated")
        if not self.backend.enabled:
            raise CleanupError("motion is disabled")
        obs = self.observation(plan["observation_id"])
        if np.max(np.abs(np.array(self.backend.state()["q"]) - plan["start_q"])) > .02:
            raise CleanupError("arm moved since planning")
        # Claim the writer before the first await. stop() can still interrupt.
        self.busy, plan["used"] = True, True
        self.executor.stopped.clear()
        run_id = uuid.uuid4().hex
        stage = "scene_check"
        try:
            frame = await self.backend.look()
            self.executor.check()
            self.checked_frame(frame)
            before = cv2.imdecode(np.frombuffer(obs["jpeg"], np.uint8), cv2.IMREAD_GRAYSCALE)
            current = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
            delta = cv2.absdiff(cv2.GaussianBlur(before, (7, 7), 0), cv2.GaussianBlur(current, (7, 7), 0))
            u, v = (int(round(n)) for n in plan["grasp_pixel"])
            radius = max(24, int(min(self.table.size) * .08))
            crop = delta[max(0, v-radius):v+radius+1, max(0, u-radius):u+radius+1]
            if (float(delta.mean()) > 8 or float((delta > 30).mean()) > .01
                    or float(crop.mean()) > 5 or float((crop > 25).mean()) > .04):
                raise CleanupError("scene changed since observation; look and plan again")
            self.epoch += 1
            await self.backend.enable(self.executor.stopped)
            async with asyncio.timeout(180):
                await self.executor.gripper(1)
                for stage in ("raise", "approach", "descend"):
                    await self.executor.path(plan["paths"][stage])
                stage = "grasp"
                contact = await self.executor.gripper(0, grasp=True)
                for stage in ("lift", "carry", "drop"):
                    await self.executor.path(plan["paths"][stage])
                stage = "release"
                await self.executor.gripper(1)
                for stage in ("retreat", "return"):
                    await self.executor.path(plan["paths"][stage])
            self.executor.check()
            self.not_before_ns = time.time_ns()
            after = await self._capture(completed=True)
            self.executor.check()
            run = {"run_id": run_id, "plan_id": plan_id, "label": plan["label"], "status": "awaiting_visual_verification",
                   "before_observation_id": plan["observation_id"], "after_observation_id": after["observation_id"],
                   "contact": contact, "verified": False}
            self.runs[run_id] = run
            self.log("execution", **run)
            return {**run, "after": after}
        except BaseException as exc:
            self.executor.stopped.set()
            hold_error = None
            try:
                await self.backend.hold()
            except Exception as hold_exc:
                hold_error = str(hold_exc)
            self.epoch += 1
            self.log("execution_failed", run_id=run_id, stage=stage, error=str(exc), hold_error=hold_error)
            if isinstance(exc, asyncio.CancelledError):
                raise
            raise CleanupError(f"{stage}: {exc or 'timed out'}; hold_error={hold_error}") from exc
        finally:
            self.busy = False

    def verify(self, run_id, observation_id, source_clear, trash_in_drop_zone, evidence):
        run = self.runs.get(run_id)
        if run is None or run["status"] != "awaiting_visual_verification":
            raise CleanupError("no unverified completed action with this run_id")
        obs = self.observation(observation_id)
        after = self.observations.get(run["after_observation_id"])
        if after is None or obs["received"] < after["received"] or obs["busy"]:
            raise CleanupError("verification must use an idle observation after this action")
        if type(source_clear) is not bool or type(trash_in_drop_zone) is not bool or not isinstance(evidence, str) or len(evidence.strip()) < 8:
            raise CleanupError("supply explicit booleans and a visual evidence description")
        verified = source_clear and trash_in_drop_zone
        run.update(status="verified_cleaned" if verified else "needs_inspection", verified=verified, evidence=evidence,
                   verification_observation_id=observation_id)
        self.verified += int(verified)
        self.log("verification", **run)
        return run

    async def stop(self):
        self.epoch += 1
        await self.executor.stop()
        self.log("stop")
        return {"stopped": True, "holding_position": True}
