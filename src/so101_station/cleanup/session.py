"""Checkpointed live actions. Codex assesses images; this process owns motion."""
from __future__ import annotations

import asyncio
import base64
import hashlib
import json
import math
import time
import uuid
from pathlib import Path

import numpy as np

from .executor import Executor, EmptyGrasp
from .geometry import ArmModel, CleanupError, vector
from .visual import VisualAlignment, SamplingError
from .targets import TargetMemory

TOOLS = {
    "status": "Read session stage, action progress, motor state and permitted actions.",
    "observe": "Fresh overhead and wrist images plus measured state. Does not move.",
    "act": "One checked action using a fresh observation: begin, open, nudge, approach, grasp, lift, carry, release, retreat. Motion requires scene_clear=true. Never infer task success from completion.",
    "assess": "Record image evidence: held/empty/uncertain after lift, delivered/uncertain after release. Carry requires held; delivery needs source_clear and on_cover.",
    "set_torque": "Explicit hold or limp; never automatic on startup. Limp requires arm_supported=true and invalidates the task.",
    "stop": "Latch cancellation, invalidate assessments, hold measured pose. Restart requires fresh observations; never auto-resumes.",
    "probe_alignment": "At clear travel-height hover with open gripper, select textured wrist bbox [x,y,w,h]. Probe +/-5mm XY, validate independently, return to start. Fits a local image response; NOT contact or grasp calibration.",
    "align_target": "Use a fresh validated probe to move its tracked feature to goal_pixel. Local XY only, bounded corrections and two fresh convergence frames. Never descends or grasps.",
    "prepare_alignment": "Fresh-start routine: explicit empty-gripper confirmation, stable telemetry, checked move to saved neutral ready hover, fresh images. After a fault requires actual operator confirmation; never closes gripper or picks trash.",
    "park": "Return an inspected empty/open gripper to the configured folded rest pose along a checked path. No torque-off. Fault recovery requires actual operator direction.",
    "select_target": "Remember a Codex-selected paper/wrapper patch from an inspected observation. Returns image position and identity; sends no motion. This is selection, not automatic object detection or robot coordinates.",
    "locate_target": "Locate a remembered target in a supplied fresh observation. Reports tracked/shifted/lost; image shift does not prove world motion. Loss requires explicit reselection. No motion or camera request.",
}
ALLOWED = {
    "idle": ["begin", "resume_holding"], "aligning": ["open", "nudge", "approach", "grasp"],
    "grasped": ["lift"], "awaiting_lift": [], "holding": ["carry"],
    "at_drop": ["release"], "awaiting_placement": ["retreat"],
    "retry_required": ["open", "retreat"], "complete": ["begin", "retreat"],
    "blocked": [], "stopped": ["resume_holding"],
}


def mock_session(model):
    return {"sim_only": True, "calibration_version": "synthetic-v1",
            "cameras": {"overhead": "mock-overhead", "wrist": "mock-wrist"},
            "floor_z_m": 0, "min_tip_z_m": .008, "travel_z_m": .075,
            "poses": {"pickup_hover": model.solve([.025, -.19, .075], np.zeros(5)).tolist(),
                      "drop": model.solve([.10, -.14, .04], np.zeros(5)).tolist()}}


class SessionHarness:
    tools = TOOLS

    def __init__(self, backend, model, config, output_dir):
        self.backend, self.model, self.config = backend, model, config
        if backend.mode == "real" and config.get("sim_only") is not False:
            raise CleanupError("real session must explicitly set sim_only=false")
        self.cameras = config["cameras"]
        if set(self.cameras) != {"overhead", "wrist"} or len(set(self.cameras.values())) != 2:
            raise CleanupError("session requires distinct overhead and wrist camera identities")
        self.floor = float(config["floor_z_m"])
        self.min_tip = float(config["min_tip_z_m"])
        self.travel = float(config["travel_z_m"])
        if not all(math.isfinite(v) for v in (self.floor, self.min_tip, self.travel)) or not self.floor + .003 <= self.min_tip < self.travel <= self.floor + .25:
            raise CleanupError("invalid session heights")
        self.poses = {k: vector(v, 5, k).tolist() for k, v in config["poses"].items()}
        for q in self.poses.values():
            model.validate(q, self.floor)
        self.calibration_version = str(config["calibration_version"])
        self.config_hash = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()[:16]
        self.executor = Executor(backend, model, self.floor, config.get("speed_rad_s", .15), config.get("command_hz", 20))
        self.planner = ArmModel(model.calibration)
        self.planner.lower += .04
        self.planner.upper -= .04
        self.out = Path(output_dir).resolve() / uuid.uuid4().hex
        self.out.mkdir(parents=True)
        self.run_id = self.out.name
        self.stage, self.busy, self.active = "idle", False, None
        self.revision, self.attempt, self.verified = 0, 0, 0
        self.actions, self.observations, self.queues = {}, {}, None
        self.target, self.used_targets, self.nudge_total = None, set(), 0.0
        self.action_count, self.started = 0, None
        self.last_error = None
        self.recoveries = 0
        self.active_progress = None
        self.visual = VisualAlignment(self)
        self.targets = TargetMemory()
        self.alignment_prepared = False
        self.return_to_rest_pending = False

    def log(self, event, **data):
        with (self.out / "events.jsonl").open("a") as f:
            f.write(json.dumps({"time_ns": time.time_ns(), "run_id": self.run_id,
                                "event": event, "stage": self.stage, **data}, allow_nan=False) + "\n")

    def status(self):
        state = self.backend.state()
        return {"run_id": self.run_id, "stage": self.stage, "busy": self.busy,
                "active_action": self.active, "attempt": self.attempt, "target": self.target,
                "at_rest": self.at_rest(state["q"]), "return_to_rest_pending": self.return_to_rest_pending,
                "active_progress": self.active_progress, "speed_rad_s": self.executor.speed, "command_hz": self.executor.command_hz,
                "permitted_actions": ALLOWED[self.stage], "verified_cleanups": self.verified,
                "stopped": self.executor.stopped.is_set(), "motion_enabled": self.backend.enabled,
                "last_error": self.last_error, "state": state,
                "tool_xyz_m": self.model.fk(state["q"])[:3, 3].tolist(), "log_directory": str(self.out)}

    async def observe(self):
        if self.busy:
            raise CleanupError("busy; wait for the action's post-observation")
        return await self._observe()

    async def _observe(self):
        revision = self.revision
        if self.queues is None:
            self.queues = await self.backend.discover_cameras(self.cameras)
        not_before = time.time_ns()
        deadline = time.monotonic() + 4
        while True:
            frames = await asyncio.gather(*(self.backend.look(self.queues[n]) for n in self.cameras))
            stamps = [f.captured_ns for f in frames]
            if min(stamps) >= not_before and max(stamps) - min(stamps) <= 250_000_000:
                break
            if time.monotonic() > deadline:
                raise CleanupError("stale or skewed camera frames")
            await asyncio.sleep(.05)
        state = self.backend.state()
        if revision != self.revision:
            raise CleanupError("state changed during observation")
        ident = uuid.uuid4().hex
        views, images = {}, []
        for name, frame in zip(self.cameras, frames):
            if frame.camera_id != self.cameras[name]:
                self.queues = None
                raise CleanupError("camera identity changed; rediscovery required")
            path = self.out / f"{ident}-{name}.jpg"
            path.write_bytes(frame.jpeg)
            views[name] = {"camera_id": frame.camera_id, "captured_ns": frame.captured_ns,
                           "image_size": [frame.width, frame.height], "path": str(path),
                           "age_ms": (time.time_ns() - frame.captured_ns) / 1e6}
            images.append({"view": name, "mime": "image/jpeg", "b64": base64.b64encode(frame.jpeg).decode()})
        result = {"observation_id": ident, "run_id": self.run_id, "revision": revision,
                  "stage": self.stage, "attempt": self.attempt, "views": views, "state": state,
                  "state_read_ns": time.time_ns(), "calibration_version": self.calibration_version,
                  "config_hash": self.config_hash, "images": images}
        self.observations[ident] = {"created": time.monotonic(), "revision": revision, "state": state, "views": views}
        if len(self.observations) > 64:
            del self.observations[next(iter(self.observations))]
        self.log("observation", **{k: v for k, v in result.items() if k not in {"images", "run_id", "stage"}})
        return result

    def fresh(self, ident):
        obs = self.observations.get(ident)
        if not obs or obs["revision"] != self.revision or time.monotonic() - obs["created"] > 30:
            raise CleanupError("stale_observation: observe again")
        state = self.backend.state()
        if max(abs(state["ticks"][i] - obs["state"]["ticks"][i]) for i in range(1, 7)) > 10 or state["torque"] != obs["state"]["torque"]:
            raise CleanupError("state_changed: observe again")
        return state

    async def _request(self, action_id, request, observation_id, perform):
        if not isinstance(action_id, str) or not 1 <= len(action_id) <= 128:
            raise CleanupError("action_id must be a nonempty string up to 128 characters")
        signature = json.dumps(request, sort_keys=True, allow_nan=False)
        if action_id in self.actions:
            old = self.actions[action_id]
            if old["signature"] != signature:
                raise CleanupError("action_id_conflict")
            return old["result"]
        if self.busy:
            raise CleanupError("busy: another operation owns motion")
        state = self.fresh(observation_id)
        self.busy, self.active = True, action_id
        record = {"signature": signature, "result": {"action_id": action_id, "execution": "in_progress"}}
        self.actions[action_id] = record
        started = time.monotonic()
        self.log("request", action_id=action_id, request=request)
        result = {"action_id": action_id, "execution": "rejected"}
        try:
            async with asyncio.timeout(180):
                detail = await perform(state)
            result.update(execution="completed", detail=detail)
        except EmptyGrasp as exc:
            self.stage = "retry_required"
            result.update(execution="empty_grasp", reason=str(exc))
        except asyncio.CancelledError:
            await self.stop()
            record["result"] = {"action_id": action_id, "execution": "stopped"}
            raise
        except Exception as exc:
            result.update(execution="stopped" if self.executor.stopped.is_set() else "rejected", reason=str(exc))
            self.last_error = str(exc)
            if isinstance(exc, CleanupError) and isinstance(getattr(exc, "diagnostic", None), dict):
                result["diagnostic"] = exc.diagnostic
            # Any failure after a command started invalidates the physical task.
            if getattr(self, "command_started", False):
                result["execution"] = "fault"
                await self.stop()
        finally:
            self.revision += 1
            self.command_started = False
            try:
                result["observation"] = await self._observe()
            except Exception as exc:
                result["observation_error"] = str(exc)
                if result["execution"] == "completed":
                    result["execution"] = "completed_without_observation"
            result.update(stage=self.stage, attempt=self.attempt, elapsed_s=time.monotonic() - started)
            record["result"] = result
            self.log("result", **{k: v for k, v in result.items() if k not in {"observation", "stage"}})
            self.busy, self.active = False, None
            self.active_progress = None
        return result

    def _target_request(self, action_id, observation_id, tool, params):
        if not isinstance(action_id, str) or not 1 <= len(action_id) <= 128:
            raise CleanupError("action_id must be a nonempty string up to 128 characters")
        request = dict(tool=tool, observation_id=observation_id, **params)
        signature = json.dumps(request, sort_keys=True, allow_nan=False)
        if action_id in self.actions:
            record = self.actions[action_id]
            if record["signature"] != signature:
                raise CleanupError("action_id_conflict")
            return record["result"]
        if self.busy:
            raise CleanupError("busy: wait for the current action")
        self.fresh(observation_id)
        observation = {**self.observations[observation_id], "observation_id": observation_id}
        self.log("request", action_id=action_id, request=request)
        try:
            method = self.targets.select if tool == "select_target" else self.targets.locate
            detail = method(observation=observation, **params)
            result = {"action_id": action_id, "execution": "completed", "detail": detail, "stage": self.stage}
        except (CleanupError, TypeError, ValueError) as exc:
            result = {"action_id": action_id, "execution": "rejected", "reason": str(exc), "stage": self.stage}
        self.actions[action_id] = {"signature": signature, "result": result}
        self.log("result", **{k:v for k,v in result.items() if k != "stage"})
        return result

    def select_target(self, action_id, observation_id, target_id, label, view, bbox, tracker="patch"):
        return self._target_request(action_id, observation_id, "select_target",
            dict(target_id=target_id, label=label, view=view, bbox=bbox, tracker=tracker))

    def locate_target(self, action_id, observation_id, target_id):
        return self._target_request(action_id, observation_id, "locate_target", dict(target_id=target_id))

    async def probe_alignment(self, action_id, observation_id, bbox=None, scene_clear=False, tracker="patch", target_id=None, axis="xy"):
        async def perform(state):
            if "rest" in self.poses and not self.alignment_prepared:
                raise CleanupError("prepare_alignment must start this test from folded rest before selecting a probe target")
            if (bbox is None) == (target_id is None):
                raise CleanupError("provide exactly one of bbox or target_id")
            selected_bbox, selected_tracker, binding = bbox, tracker, None
            if target_id is not None:
                selected = self.targets.resolve(target_id, observation_id)
                selected_bbox, selected_tracker = selected["bbox"], selected["tracker"]
                binding = {"target_id": target_id, "generation": selected["generation"]}
            return await self.visual.probe(observation_id, selected_bbox, state, scene_clear, selected_tracker, binding, axis)
        return await self._request(action_id, dict(tool="probe_alignment", observation_id=observation_id,
            bbox=bbox, scene_clear=scene_clear, tracker=tracker, target_id=target_id, axis=axis), observation_id, perform)

    async def align_target(self, action_id, observation_id, probe_id, goal_pixel, scene_clear=False):
        async def perform(state):
            return await self.visual.align(probe_id, goal_pixel, state, scene_clear)
        return await self._request(action_id, dict(tool="align_target", observation_id=observation_id,
            probe_id=probe_id, goal_pixel=goal_pixel, scene_clear=scene_clear), observation_id, perform)

    def at_rest(self, q):
        return None if "rest" not in self.poses else bool(np.max(np.abs(np.asarray(q)-self.poses["rest"])) <= .015)

    async def prepare_alignment(self, action_id, observation_id, empty_gripper_confirmed=False,
                                scene_clear=False, operator_confirmed=False):
        return await self._position_alignment(action_id, observation_id, empty_gripper_confirmed,
                                               scene_clear, operator_confirmed, park=False)

    async def park(self, action_id, observation_id, empty_gripper_confirmed=False,
                   scene_clear=False, operator_confirmed=False):
        return await self._position_alignment(action_id, observation_id, empty_gripper_confirmed,
                                               scene_clear, operator_confirmed, park=True)

    async def _position_alignment(self, action_id, observation_id, empty_gripper_confirmed,
                                  scene_clear, operator_confirmed, *, park):
        async def perform(state):
            if self.stage not in {"idle", "aligning", "complete", "stopped"}:
                raise CleanupError("cannot prepare while an object may be held")
            if empty_gripper_confirmed is not True or scene_clear is not True or state["ticks"][6] < 3200:
                raise CleanupError("preparation requires inspected empty open gripper and clear neutral path")
            if not state["torque"] or not self.backend.enabled:
                raise CleanupError("prepare requires explicit torque hold first")
            if (self.executor.stopped.is_set() or self.attempt > 0) and operator_confirmed is not True:
                raise CleanupError("fault or attempted-pickup reset requires actual operator direction")
            epoch = self.revision
            for _ in range(5):
                await asyncio.sleep(.1)
                current = self.backend.state()
                if not current["torque"] or max(abs(current["ticks"][i]-state["ticks"][i]) for i in range(1,7)) > 8:
                    raise CleanupError("neutral preparation requires stable holding state")
            def route():
                if park and "rest" not in self.poses:
                    raise CleanupError("folded rest pose is not configured")
                start = np.array(current["q"])
                rest_path = []
                if "rest" in self.poses and not self.at_rest(start):
                    rest_path = self.joint_path(start, self.poses["rest"], step_rad=.20)
                    start = np.array(self.poses["rest"])
                ready_path = [] if park else self.joint_path(start, self.poses["pickup_hover"], step_rad=.20)
                return rest_path, ready_path
            rest_path, ready_path = await asyncio.to_thread(route)
            self.fresh(observation_id)
            if self.revision != epoch:
                raise CleanupError("new stop during neutral preparation")
            self.executor.stopped.clear()
            self.visual.probes.clear()
            self.executor.grip_goal = current["ticks"][6]
            self.alignment_prepared = False
            self.return_to_rest_pending = True
            self.command_started = bool(rest_path or ready_path)
            self.active_progress = {"phase": "folded_rest"}
            await self.executor.path(rest_path)
            self.executor.check()
            if "rest" in self.poses:
                if not self.at_rest(self.backend.state()["q"]):
                    raise CleanupError("arm did not reach folded rest")
                self.log("folded_rest_reached", moved=bool(rest_path))
            if park:
                self.stage, self.target = "idle", None
                self.return_to_rest_pending = False
                return {"parked": True, "pose": "rest", "at_rest": True, "torque_off": False}
            self.active_progress = {"phase": "neutral_ready_hover"}
            await self.executor.path(ready_path)
            self.executor.check()
            self.stage = "aligning"
            self.target = "probe-"+uuid.uuid4().hex[:8]
            self.used_targets.add(self.target)
            self.started = time.monotonic()
            self.action_count, self.attempt, self.nudge_total = 0, 0, 0.
            self.alignment_prepared = True
            return {"ready": True, "started_from_rest": "rest" in self.poses, "neutral_pose": "pickup_hover",
                    "return_to_rest_pending": True, "requires": "select target in fresh wrist image before probe; park after test"}
        return await self._request(action_id, dict(tool="park" if park else "prepare_alignment", observation_id=observation_id,
            empty_gripper_confirmed=empty_gripper_confirmed, scene_clear=scene_clear,
            operator_confirmed=operator_confirmed), observation_id, perform)

    def joint_path(self, start, target, *, step_rad=.025):
        path = np.linspace(start, target, max(2, math.ceil(float(np.max(np.abs(np.asarray(target)-start))) / .025)) + 1)
        for q in path:
            self.model.validate(q, self.floor)
        # Validate densely, execute long transfers without stopping at every validation sample.
        count = max(2, math.ceil(float(np.max(np.abs(np.asarray(target)-start))) / step_rad))
        return np.linspace(start, target, count+1)[1:].tolist()

    def plan(self, action, params, state):
        q = np.array(state["q"])
        xyz = self.model.fk(q)[:3, 3]
        if action == "nudge":
            delta = vector(params["delta_mm"], 3, "delta_mm") / 1000
            if np.linalg.norm(delta) > .010001 or self.nudge_total + np.linalg.norm(delta) > .12:
                raise CleanupError("nudge exceeds 10 mm step or 120 mm task budget")
            if (xyz + delta)[2] < self.min_tip:
                raise CleanupError("nudge below configured minimum fingertip height")
            target = self.planner.solve(xyz + delta, q, self.floor)
            return self.joint_path(q, target)
        if action in {"lift", "retreat"}:
            z = min(self.travel, xyz[2] + .04) if action == "lift" else self.travel
            if z <= xyz[2] + .005:
                raise CleanupError("no upward clearance available")
            return self.joint_path(q, self.planner.solve([*xyz[:2], z], q, self.floor))
        if action == "approach":
            return self.joint_path(q, self.poses["pickup_hover"])
        if action == "carry":
            drop = np.array(self.poses["drop"])
            dxyz = self.model.fk(drop)[:3, 3]
            lift = self.planner.solve([*xyz[:2], self.travel], q, self.floor)
            hover = self.planner.solve([*dxyz[:2], self.travel], drop, self.floor)
            return self.joint_path(q, lift) + self.joint_path(lift, hover) + self.joint_path(hover, drop)
        return []

    async def act(self, action_id, observation_id, action, params=None, scene_clear=False):
        params = params or {}
        async def perform(state):
            if action == "resume_holding":
                # Explicit operator recovery only: no motor command and no restored
                # visual assessment. A new held assessment is required before carry.
                expected = {"operator_confirmed", "evidence"} | ({"target_id"} if self.target is None else set())
                if (self.stage not in {"idle", "stopped"} or self.recoveries >= 1
                        or set(params) != expected
                        or params["operator_confirmed"] is not True
                        or not isinstance(params["evidence"], str) or not params["evidence"].strip()):
                    raise CleanupError("resume requires one explicit operator-confirmed held-object recovery")
                if self.target is None and (not isinstance(params["target_id"], str) or not params["target_id"].strip()):
                    raise CleanupError("recovery after restart requires target identity")
                if not state["torque"] or not self.backend.enabled or scene_clear is not True:
                    raise CleanupError("resume requires all motors holding and a clear inspected scene")
                epoch = self.revision
                for _ in range(3):
                    await asyncio.sleep(.1)
                    current = self.backend.state()
                    if not current["torque"] or max(abs(current["ticks"][i]-state["ticks"][i]) for i in range(1, 7)) > 8:
                        raise CleanupError("resume requires stable fresh telemetry")
                    self.model.validate(current["q"], self.floor)
                if self.revision != epoch:
                    raise CleanupError("new stop during recovery; stop remains latched")
                if self.target is None:
                    self.target = params["target_id"]
                    self.used_targets.add(self.target)
                    self.attempt = 2  # Unknown prior attempts: permit placement, never extra grasps.
                self.executor.stopped.clear()
                self.stage = "awaiting_lift"
                self.recoveries += 1
                self.started = time.monotonic()  # Explicit new recovery time window; attempt budget retained.
                return {"motion_sent": False, "requires": "fresh held assessment before carry"}
            self.executor.check()
            if action not in ALLOWED[self.stage]:
                raise CleanupError(f"invalid_stage: {action} unavailable in {self.stage}")
            allowed_keys = {"begin": {"target_id"}, "nudge": {"delta_mm"}}.get(action, set())
            if set(params) != allowed_keys:
                raise CleanupError(f"{action} requires exactly {sorted(allowed_keys)}")
            if action == "begin":
                target = params["target_id"]
                if not isinstance(target, str) or not target.strip() or target in self.used_targets:
                    raise CleanupError("target_id must be new and nonempty")
                self.target, self.attempt, self.nudge_total = target, 0, 0
                self.used_targets.add(target)
                self.started, self.action_count, self.stage = time.monotonic(), 0, "aligning"
                return {"target_id": target}
            if scene_clear is not True or not state["torque"] or not self.backend.enabled:
                raise CleanupError("motion requires enabled backend, all motors holding and scene_clear=true")
            if self.action_count >= 60 or time.monotonic() - self.started > 600:
                raise CleanupError("task action/time budget exhausted")
            if action == "grasp" and self.attempt >= 2:
                raise CleanupError("two grasp attempts exhausted")
            path = await asyncio.to_thread(self.plan, action, params, state)
            self.executor.check()
            self.fresh(observation_id)
            self.model.validate(state["q"], self.floor)
            self.command_started = True
            self.action_count += 1
            detail = {}
            if action in {"open", "release", "grasp"}:
                if action == "grasp":
                    self.attempt += 1
                detail = await self.executor.gripper(0 if action == "grasp" else .9, grasp=action == "grasp")
            else:
                await self.executor.path(path)
            if action == "nudge":
                self.nudge_total += float(np.linalg.norm(params["delta_mm"])) / 1000
            self.stage = {"open": "aligning", "grasp": "grasped", "lift": "awaiting_lift",
                          "carry": "at_drop", "release": "awaiting_placement"}.get(action, self.stage)
            return detail
        return await self._request(action_id, dict(tool="act", observation_id=observation_id, action=action,
                                                   params=params, scene_clear=scene_clear), observation_id, perform)

    async def assess(self, action_id, observation_id, outcome, evidence, source_clear=False, on_cover=False):
        async def perform(state):
            self.executor.check()
            if not isinstance(evidence, str) or not evidence.strip():
                raise CleanupError("visual evidence is required")
            if self.stage == "awaiting_lift" and outcome in {"held", "empty", "uncertain"}:
                if outcome == "held" and source_clear is not True:
                    raise CleanupError("held assessment requires a visibly cleared source")
                self.stage = {"held": "holding", "empty": "retry_required"}.get(outcome, self.stage)
            elif self.stage == "awaiting_placement" and outcome in {"delivered", "uncertain"}:
                if outcome == "delivered":
                    if source_clear is not True or on_cover is not True:
                        raise CleanupError("delivery requires source_clear and on_cover")
                    self.verified += 1
                    self.stage = "complete"
            else:
                raise CleanupError("assessment does not match current stage")
            return {"assessment": outcome, "evidence": evidence, "verified_cleanups": self.verified}
        return await self._request(action_id, dict(tool="assess", observation_id=observation_id, outcome=outcome,
            evidence=evidence, source_clear=source_clear, on_cover=on_cover), observation_id, perform)

    async def set_torque(self, action_id, observation_id, mode, arm_supported=False):
        async def perform(state):
            if mode not in {"hold", "limp"}:
                raise CleanupError("mode must be hold or limp")
            if not self.backend.enabled:
                raise CleanupError("motion disabled")
            if mode == "hold":
                self.executor.check()
                if self.stage not in {"idle", "aligning", "complete"}:
                    raise CleanupError("torque change unavailable during pickup")
                self.command_started = True
                await self.backend.enable(self.executor.stopped, self.executor.speed)
            else:
                if arm_supported is not True:
                    raise CleanupError("limp requires arm_supported=true")
                self.executor.stopped.set()
                self.stage = "stopped"
                self.command_started = True
                await self.backend.disable()
            self.executor.grip_goal = None
            return {"mode": mode, "torque": self.backend.state()["torque"]}
        return await self._request(action_id, dict(tool="set_torque", observation_id=observation_id,
            mode=mode, arm_supported=arm_supported), observation_id, perform)

    async def stop(self):
        self.executor.stopped.set()
        self.visual.probes.clear()
        self.alignment_prepared = False
        self.stage = "stopped"
        self.revision += 1
        error = None
        try:
            if self.backend.enabled:
                await self.backend.hold()
        except Exception as exc:
            error = str(exc)
        self.log("stop", hold_error=error)
        return {"stage": self.stage, "holding_position": self.backend.enabled and error is None, "hold_error": error}
