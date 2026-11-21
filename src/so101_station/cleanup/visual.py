"""Local wrist-image probe and bounded alignment. No contact/height inference."""
from __future__ import annotations

import asyncio
import json
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

from .geometry import CleanupError, vector
from .tracking import FeatureTracker
from .axis_probe import summarize_axis_probe
from .endpoint import next_trim


class PatchTracker:
    """Track one operator-selected textured patch, keeping the original template."""
    def __init__(self, image, bbox):
        box = vector(bbox, 4, "bbox [x,y,width,height]")
        if not np.equal(box, np.round(box)).all():
            raise CleanupError("bbox must use integer pixels")
        x, y, w, h = map(int, box)
        if w < 24 or h < 24 or w > 300 or h > 300 or x < 0 or y < 0 or x+w > image.shape[1] or y+h > image.shape[0]:
            raise CleanupError("bbox outside image or patch dimensions outside 24..300 pixels")
        self.template = self.gray(image)[y:y+h, x:x+w].copy()
        if self.template.std() < 8:
            raise CleanupError("target patch lacks texture; select a distinctive region")
        self.size = (w, h)
        self.center = np.array([x+w/2, y+h/2], float)
        self.shape = image.shape[:2]

    @staticmethod
    def gray(image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        return cv2.GaussianBlur(gray, (3, 3), 0)

    def locate(self, image, radius=65):
        if image.shape[:2] != self.shape:
            raise CleanupError("camera image dimensions changed")
        w, h = self.size
        x, y = self.center - [w/2, h/2]
        left, top = max(0, int(x-radius)), max(0, int(y-radius))
        right, bottom = min(self.shape[1], int(x+w+radius)), min(self.shape[0], int(y+h+radius))
        scores = cv2.matchTemplate(self.gray(image)[top:bottom, left:right], self.template, cv2.TM_CCOEFF_NORMED)
        _, confidence, _, point = cv2.minMaxLoc(scores)
        # A second distant peak indicates an ambiguous/repeating target.
        excluded = scores.copy()
        px, py = point
        excluded[max(0, py-12):py+13, max(0, px-12):px+13] = -1
        second = float(excluded.max())
        if not np.isfinite(confidence) or confidence < .72 or confidence-second < .025:
            raise CleanupError(f"target lost/ambiguous: correlation={confidence:.3f}, separation={confidence-second:.3f}")
        subpixel = np.zeros(2)
        for axis, values in enumerate((scores[py, px-1:px+2] if 0 < px < scores.shape[1]-1 else [],
                                       scores[py-1:py+2, px] if 0 < py < scores.shape[0]-1 else [])):
            if len(values) == 3:
                a, b, c = map(float, values)
                denominator = a-2*b+c
                if denominator < -1e-6:
                    subpixel[axis] = np.clip(.5*(a-c)/denominator, -.5, .5)
        center = np.array([left+px+w/2, top+py+h/2])+subpixel
        if np.linalg.norm(center-self.center) > radius-5:
            raise CleanupError("target displacement exceeds tracker search bound")
        self.center = center
        return center.copy(), float(confidence)


def fit_response(commands_mm, pixels, reference):
    commands, displacement = np.asarray(commands_mm, float), np.asarray(pixels, float)-reference
    if commands.shape != displacement.shape or commands.ndim != 2 or commands.shape[1] != 2:
        raise CleanupError("invalid response samples")
    if not np.isfinite(commands).all() or not np.isfinite(displacement).all() or np.linalg.matrix_rank(commands) < 2:
        raise CleanupError("probe does not excite both axes")
    matrix = np.linalg.lstsq(commands, displacement, rcond=None)[0].T
    singular = np.linalg.svd(matrix, compute_uv=False)
    residual = np.linalg.norm(displacement - commands @ matrix.T, axis=1)
    if singular[-1] < .5 or singular[0]/singular[-1] > 12 or singular[0] > 40:
        raise CleanupError("weak/ill-conditioned response; no reliable local image map")
    if residual.max() > 5:
        raise CleanupError(f"nonlinear/drifting probe: max residual {residual.max():.1f} px")
    return matrix, {"max_fit_error_px": float(residual.max()), "condition_number": float(singular[0]/singular[-1]),
                    "singular_values_px_per_commanded_mm": singular.tolist()}


def wrist_image(observation):
    path = observation["views"]["wrist"]["path"]
    image = cv2.imread(str(path))
    if image is None:
        raise CleanupError("wrist evidence image cannot be decoded")
    return image


class SamplingError(CleanupError):
    def __init__(self, code, phase, metrics, timing):
        self.diagnostic = {"code": code, "phase": phase, "metrics": metrics, "timing": timing}
        super().__init__(f"visual sample unstable: {code} during {phase}")


class VisualAlignment:
    settle_window_s = .15
    settle_timeout_s = 3.0
    image_timeout_s = 1.2
    burst_frames = 3
    max_frame_gap_s = .25

    def __init__(self, harness):
        self.h = harness
        self.probes = {}
        self.target_q = None
        self.precision_q = None
        self.endpoint_result = None

    def endpoint_errors(self, q, desired):
        actual, goal = self.h.model.fk(q), self.h.model.fk(desired)
        return {"position_error_mm": float(np.linalg.norm(actual[:3, 3]-goal[:3, 3])*1000),
                "rotation_error": float(np.linalg.norm(actual[:3, :3]-goal[:3, :3]))}

    def guard(self, state, scene_clear):
        h = self.h
        h.executor.check()
        if h.stage not in {"idle", "aligning", "complete"}:
            raise CleanupError("visual probing/alignment requires an empty-gripper task stage")
        if scene_clear is not True or not h.backend.enabled or not state["torque"] or state["ticks"][6] < 3200:
            raise CleanupError("requires inspected clear XY corridor, motion enabled and open holding gripper")
        h.model.validate(state["q"], h.floor)
        xyz = h.model.fk(state["q"])[:3, 3]
        if xyz[2] < h.travel-.004:
            raise CleanupError("probe/alignment must start at configured travel clearance")
        return xyz

    def sample_state(self):
        self.h.executor.check()
        state = self.h.backend.state()
        if not state["torque"] or state["ticks"][6] < 3200:
            raise CleanupError("visual sampling requires open holding gripper")
        return state

    def state_problem(self, states):
        ticks = np.array([[s["ticks"][i] for i in range(1, 7)] for s in states])
        spans = np.ptp(ticks, axis=0)
        ages = [j["age_ms"] for s in states for j in s.get("report", {}).get("joints", [])]
        errors = np.max(np.abs(np.array([s["q"] for s in states])-self.target_q), axis=0) if self.target_q is not None else np.zeros(5)
        metrics = {"motor_span_ticks": int(spans.max()), "span_by_motor": spans.tolist(),
                   "max_telemetry_age_ms": max(ages, default=0),
                   "max_target_error_rad": float(errors.max()), "joint_errors_rad": errors.tolist()}
        if any(not s["torque"] for s in states): return "torque_not_holding", metrics
        if any(s["ticks"][6] < 3200 for s in states): return "gripper_not_open", metrics
        if metrics["max_telemetry_age_ms"] > 100: return "telemetry_not_fresh", metrics
        if self.target_q is not None and self.precision_q is not None:
            precise_errors = [self.endpoint_errors(s["q"], self.precision_q) for s in states]
            metrics.update({key: max(e[key] for e in precise_errors) for key in precise_errors[0]})
            tick_error = max(np.max(np.abs(np.asarray(s["q"])-self.precision_q)) for s in states)
            if tick_error > .0047 or metrics["position_error_mm"] > .5 or metrics["rotation_error"] > .006:
                return "precise_endpoint_drift", metrics
        if metrics["max_target_error_rad"] > .0047: return "joint_target_error", metrics
        if metrics["motor_span_ticks"] > 2: return "encoder_motion", metrics
        return None, metrics

    def quiet(self, states):
        return self.state_problem(states)[0] is None

    async def capture(self, tracker):
        """Separate cumulative settling/image budgets; three quiet, recent frames."""
        h = self.h
        remaining = {"settle": self.settle_timeout_s, "image": self.image_timeout_s}
        used = {"settle": 0., "image": 0.}
        phase, frames, anchor = "settle", [], []
        code, metrics = "encoder_motion", {}
        prior_progress = dict(h.active_progress or {})
        def timing():
            return {"settle_s": used["settle"], "image_s": used["image"],
                    "settle_budget_s": self.settle_timeout_s, "image_budget_s": self.image_timeout_s}
        def sample():
            h.executor.check()
            try:
                state = h.backend.state()
            except CleanupError as exc:
                raise SamplingError("telemetry_unavailable", phase, {"reason": str(exc)}, timing()) from exc
            if not state["torque"] or state["ticks"][6] < 3200:
                reason, values = self.state_problem([state])
                raise SamplingError(reason, phase, values, timing())
            return state
        try:
            while True:
                stage = phase
                began = time.monotonic()
                timed_out = False
                h.active_progress = {**prior_progress, "sampling_phase": stage}
                try:
                    async with asyncio.timeout(max(0., remaining[stage])):
                        if stage == "settle":
                            window_start = time.monotonic()
                            states = [sample()]
                            last_logged = None
                            while True:
                                problem, metrics = self.state_problem(states)
                                if problem:
                                    code = problem
                                    if problem != last_logged:
                                        h.log("visual_sampling_wait", phase=stage, code=code, metrics=metrics)
                                        last_logged = problem
                                    window_start = time.monotonic()
                                    states = [states[-1]]
                                elif time.monotonic()-window_start >= self.settle_window_s:
                                    anchor = states
                                    phase = "image"
                                    frames = []
                                    code = "insufficient_fresh_frames"
                                    break
                                await asyncio.sleep(.03)
                                states.append(sample())
                        else:
                            states = [*anchor, sample()]
                            task = asyncio.create_task(h._observe())
                            try:
                                while not task.done():
                                    await asyncio.wait({task}, timeout=.03)
                                    states.append(sample())
                                try:
                                    observation = await task
                                except (CleanupError, TimeoutError) as exc:
                                    raise SamplingError("camera_capture_failed", stage, {"reason": str(exc)}, timing()) from exc
                            finally:
                                if not task.done(): task.cancel()
                                await asyncio.gather(task, return_exceptions=True)
                            states.append(observation["state"])
                            problem, metrics = self.state_problem(states)
                            if problem:
                                code = problem
                                h.log("visual_sampling_wait", phase=stage, code=code, metrics=metrics)
                                phase, frames, anchor = "settle", [], []
                                continue
                            anchor = states  # Preserve pose extrema across every frame in this burst.
                            try:
                                pixel, confidence = tracker.locate(wrist_image(observation))
                            except CleanupError as exc:
                                raise SamplingError("target_tracking_lost", stage, {"reason": str(exc)}, timing()) from exc
                            h.executor.check()
                            if time.monotonic()-began >= remaining[stage]:
                                code = "image_processing_timeout"
                                raise TimeoutError
                            stamp = observation["views"]["wrist"]["captured_ns"]
                            if frames and stamp <= frames[-1][0]:
                                raise SamplingError("camera_timestamp_not_increasing", stage, {"captured_ns": stamp}, timing())
                            if frames and (stamp-frames[-1][0])/1e9 > self.max_frame_gap_s:
                                code = "frame_gap"
                                metrics["frame_gap_s"] = (stamp-frames[-1][0])/1e9
                                frames = []
                            frames.append((stamp, pixel.copy(), observation["observation_id"]))
                            frames = frames[-self.burst_frames:]
                            points = np.array([f[1] for f in frames])
                            drift = float(np.linalg.norm(points[:,None]-points[None,:],axis=2).max())
                            metrics.update(frame_drift_px=drift, frame_count=len(frames))
                            if drift > 1.5:
                                code = "image_drift"
                            elif len(frames) >= self.burst_frames:
                                details = {"measured_xyz_m": h.model.fk(observation["state"]["q"])[:3,3].tolist(),
                                    "measured_q": observation["state"]["q"], "state_read_ns": observation["state_read_ns"],
                                    "wrist_captured_ns": stamp, "settle_elapsed_s": used["settle"],
                                    "image_elapsed_s": used["image"]+time.monotonic()-began,
                                    "stable_frames": len(frames), "frame_drift_px": drift,
                                    "motor_span_ticks": metrics["motor_span_ticks"],
                                    "endpoint": self.endpoint_result if self.target_q is not None else None,
                                    "frame_observation_ids": [f[2] for f in frames],
                                    "max_frame_gap_s": max(np.diff([f[0] for f in frames]))/1e9}
                                observation["visual_sample"] = details
                                h.log("visual_stable_sample", observation_id=observation["observation_id"], **details)
                                return observation, pixel, confidence
                            h.log("visual_sampling_frame", observation_id=observation["observation_id"],
                                  pixel=pixel.tolist(), confidence=confidence, code=code, metrics=metrics)
                            # Preserve the quiet-pose anchor across the entire burst.
                except TimeoutError:
                    timed_out = True
                finally:
                    elapsed = time.monotonic()-began
                    used[stage] += elapsed
                    remaining[stage] -= elapsed
                if timed_out:
                    raise SamplingError(code, stage, metrics, timing())
        except SamplingError as exc:
            exc.diagnostic["timing"] = timing()  # Include time spent in the failed current attempt.
            h.log("visual_sampling_failure", **exc.diagnostic)
            raise
        finally:
            h.active_progress = prior_progress or None

    def check_corridor(self, q, home, *, measured=False):
        self.h.model.validate(q, self.h.floor)
        xyz = self.h.model.fk(q)[:3, 3]
        height, radius = (.004, .011) if measured else (.003, .010)
        if abs(xyz[2]-home[2]) > height or np.linalg.norm(xyz[:2]-home[:2]) > radius:
            raise CleanupError("endpoint trim leaves horizontal probe corridor")
        return xyz

    async def refine_endpoint(self, q, home, gripper):
        """Correct only settled encoder bias, inside the already checked hover corridor.

        Nominal target and compensated servo command remain separate. This is
        model/encoder repeatability, not proof of image or grasp accuracy.
        """
        h = self.h
        desired = [h.model.ticks(q, gripper)[i] for i in range(1, 6)]
        nominal_q = np.array([j.ticks_to_radians(t) for j, t in zip(h.model.joints, desired)])
        command = desired.copy()
        started, rows = time.monotonic(), []
        best, non_improving = float("inf"), 0
        self.precision_q = None
        self.endpoint_result = None
        metrics = {}
        try:
            async with asyncio.timeout(4):
                for iteration in range(6):
                    # Wait for a fresh, quiet encoder window before adding bias.
                    window = []
                    while True:
                        state = self.sample_state()
                        if abs(state["ticks"][6]-gripper) > 2:
                            raise CleanupError("gripper changed during endpoint trim")
                        self.check_corridor(state["q"], home, measured=True)
                        stamp = time.monotonic()
                        window.append((stamp, state))
                        while len(window) > 1 and window[1][0] <= stamp-self.settle_window_s:
                            window.pop(0)
                        ticks = np.array([[s["ticks"][i] for i in range(1, 6)] for _, s in window])
                        ages = [j["age_ms"] for _, s in window for j in s.get("report", {}).get("joints", [])]
                        if stamp-window[0][0] >= self.settle_window_s and np.ptp(ticks, axis=0).max() <= 1 and max(ages, default=0) <= 100:
                            break
                        await asyncio.sleep(.025)
                    measured = [state["ticks"][i] for i in range(1, 6)]
                    errors = np.asarray(desired)-measured
                    metrics = {"iteration": iteration, "desired_ticks": desired, "measured_ticks": measured,
                               "command_ticks": command.copy(), "residual_ticks": errors.tolist(),
                               "requested_joint_error_rad": float(np.max(np.abs(np.asarray(state["q"])-q))),
                               **self.endpoint_errors(state["q"], nominal_q)}
                    rows.append(metrics)
                    h.log("visual_endpoint_sample", **metrics)
                    # Keep the existing three-tick joint bound. Small opposing
                    # joint errors can cancel at the tool; do not chase encoder
                    # ticks when both endpoint position and orientation agree.
                    if (np.abs(errors).max() <= 3 and metrics["requested_joint_error_rad"] <= .0047
                            and metrics["position_error_mm"] <= .5 and metrics["rotation_error"] <= .006):
                        self.precision_q = nominal_q
                        self.endpoint_result = {"converged": True, "trim_commands": iteration,
                                                "elapsed_s": time.monotonic()-started, "samples": rows}
                        return self.check_corridor(state["q"], home, measured=True)
                    residual = float(np.linalg.norm(errors))
                    if residual < best-.1:
                        best, non_improving = residual, 0
                    else:
                        non_improving += 1
                    if non_improving >= 3:
                        raise CleanupError("endpoint trim stalled or oscillated")
                    if iteration == 5:
                        raise CleanupError("endpoint trim did not converge within five corrections")
                    proposal = next_trim(desired, measured, command, tolerance=0 if np.abs(errors).max() <= 1 else 1)
                    proposed_q = np.array([j.ticks_to_radians(t) for j, t in zip(h.model.joints, proposal)])
                    path = h.joint_path(np.array(state["q"]), proposed_q)
                    for point in [state["q"], *path]:
                        self.check_corridor(point, home)
                    # Synchronous planning cannot hide a new stop or a moved arm.
                    h.executor.check()
                    fresh = self.sample_state()
                    if abs(fresh["ticks"][6]-gripper) > 2:
                        raise CleanupError("gripper changed before endpoint trim")
                    if max((j["age_ms"] for j in fresh.get("report", {}).get("joints", [])), default=0) > 100:
                        raise CleanupError("telemetry became stale before endpoint trim")
                    if any(abs(fresh["ticks"][i]-state["ticks"][i]) > 1 for i in range(1, 7)):
                        raise CleanupError("robot moved while checking endpoint trim")
                    for waypoint in path:
                        await h.executor.move(waypoint)
                        after = self.sample_state()
                        self.check_corridor(after["q"], home, measured=True)
                        if abs(after["ticks"][6]-gripper) > 2:
                            raise CleanupError("gripper changed during endpoint trim")
                        if max((j["age_ms"] for j in after.get("report", {}).get("joints", [])), default=0) > 100:
                            raise CleanupError("telemetry became stale during endpoint trim")
                    command = proposal
        except (CleanupError, TimeoutError) as exc:
            error = CleanupError(f"endpoint refinement failed: {str(exc) or 'four-second deadline exceeded'}")
            error.diagnostic = {"code": "endpoint_refinement_failed", "phase": "endpoint",
                                "metrics": metrics, "samples": rows, "elapsed_s": time.monotonic()-started}
            h.log("visual_endpoint_failure", **error.diagnostic)
            raise error from exc

    async def move(self, home, home_q, offset_mm):
        h = self.h
        h.executor.check()
        current = h.backend.state()
        actual = h.model.fk(current["q"])[:3, 3]
        target = home + np.r_[offset_mm, 0]/1000
        if np.linalg.norm(target-actual) > .011:
            raise CleanupError("visual correction exceeds 11 mm measured transition bound")
        # A zero-offset return restores the exact anchor joints, including wrist orientation.
        q = np.array(home_q).copy() if np.linalg.norm(offset_mm) < 1e-9 else await asyncio.to_thread(h.planner.solve, target, home_q, h.floor)
        if np.linalg.norm(h.model.fk(q)[:3,3]-target) > .0015:
            raise CleanupError("visual IK endpoint error exceeds 1.5 mm")
        path = h.joint_path(np.array(current["q"]), q)
        for point in [current["q"], *path]:
            xyz = h.model.fk(point)[:3,3]
            if abs(xyz[2]-home[2]) > .003 or np.linalg.norm(xyz[:2]-home[:2]) > .010:
                raise CleanupError("visual path leaves the checked horizontal corridor")
        # No concurrent writer is admitted, but manual drift still invalidates the plan.
        checked = h.backend.state()
        if not checked["torque"] or checked["ticks"][6] < 3200:
            raise CleanupError("visual motion requires open holding gripper throughout")
        if np.max(np.abs(np.asarray(checked["q"])-current["q"])) > .015:
            raise CleanupError("robot moved during visual planning")
        h.executor.check()
        h.executor.grip_goal = checked["ticks"][6]  # Never revive a stale closed goal during XY probing.
        h.command_started = True
        for waypoint in path:
            await h.executor.move(waypoint)
            h.executor.check()
            measured = h.model.fk(h.backend.state()["q"])[:3, 3]
            if abs(measured[2]-home[2]) > .004 or np.linalg.norm(measured[:2]-home[:2]) > .011:
                raise CleanupError("measured pose left horizontal probe corridor")
        measured = await self.refine_endpoint(q, home, checked["ticks"][6])
        self.target_q = q.copy()
        return measured.tolist()

    async def probe(self, observation_id, bbox, state, scene_clear, tracker_kind="patch", target_binding=None, axis="xy"):
        h = self.h
        if axis not in {"x", "y", "xy"}:
            raise CleanupError("probe axis must be x, y or xy")
        home = self.guard(state, scene_clear)
        initial = h.observations[observation_id]
        if "views" not in initial:
            raise CleanupError("observation has no image evidence")
        if tracker_kind not in {"patch", "features"}:
            raise CleanupError("tracker must be patch or features")
        tracker = (FeatureTracker if tracker_kind == "features" else PatchTracker)(wrist_image(initial), bbox)
        self.target_q = None
        selected = tracker.center.copy()
        obs, reference, score = await self.capture(tracker)
        if np.linalg.norm(reference-selected) > 5:
            raise CleanupError("target moved since selection; inspect and select again")
        h.fresh(observation_id)
        state = obs["state"]
        home = self.guard(state, scene_clear)
        home_q = np.array(state["q"])
        # Every target is anchored to this one initial pose, including reversals.
        offsets = [[5,0],[0,0],[-5,0],[0,0],[0,5],[0,0],[0,-5],[0,0]]
        if axis == "x": offsets = offsets[:4]
        if axis == "y": offsets = offsets[4:]
        # Check reachability before the first command; execution rechecks actual transitions.
        for offset in offsets + ([[3,3]] if axis == "xy" else []):
            target_q = home_q if offset == [0,0] else await asyncio.to_thread(h.planner.solve, home+np.r_[offset,0]/1000, home_q, h.floor)
            h.joint_path(home_q, target_q)
        h.fresh(observation_id)
        rows, pixels = [], []
        started = time.monotonic()
        async with asyncio.timeout(60):
            for offset in offsets:
                h.active_progress = {"phase": "probe", "sample": len(rows)+1, "total": len(offsets)+(2 if axis == "xy" else 0), "axis": axis}
                t = time.monotonic()
                measured = await self.move(home, home_q, offset)
                obs, pixel, confidence = await self.capture(tracker)
                row = {"offset_mm": offset, "pixel": pixel.tolist(), "confidence": confidence,
                       "measured_xyz_m": obs["visual_sample"]["measured_xyz_m"], "sampling": obs["visual_sample"],
                       "observation_id": obs["observation_id"], "elapsed_s": time.monotonic()-t}
                rows.append(row); pixels.append(pixel)
                h.log("visual_probe_sample", **row)
            if axis != "xy":
                summary = summarize_axis_probe(axis, offsets, pixels, reference)
                ident = uuid.uuid4().hex
                data = {**summary, "axis_test_id": ident, "tracker": tracker_kind,
                        "target_binding": target_binding, "reference_pixel": reference.tolist(),
                        "final_pixel": pixel.tolist(), "samples": rows, "home_q": home_q.tolist(),
                        "home_xyz_m": home.tolist(), "elapsed_s": time.monotonic()-started,
                        "camera_id": obs["views"]["wrist"]["camera_id"],
                        "image_size": obs["views"]["wrist"]["image_size"], "config_hash": h.config_hash,
                        "alignment_available": False, "requires": "park after test; a two-axis map and grasp reference remain unvalidated"}
                (h.out/f"axis-test-{ident}.json").write_text(json.dumps(data, indent=2))
                h.log("visual_axis_result", **data)
                return data
            matrix, quality = fit_response(offsets, pixels, reference)
            if max(np.linalg.norm(np.asarray(row["pixel"])-reference) for row in rows if row["offset_mm"] == [0,0]) > 5:
                raise CleanupError("return drift exceeds 5 pixels; inspect backlash/scene stability")
            # Independent diagonal validation, excluded from fitting.
            measured = await self.move(home, home_q, [3,3])
            obs, pixel, confidence = await self.capture(tracker)
            validation_error = float(np.linalg.norm(pixel-reference-matrix @ [3,3]))
            h.log("visual_probe_validation", observation_id=obs["observation_id"], error_px=validation_error, measured_xyz_m=obs["visual_sample"]["measured_xyz_m"], sampling=obs["visual_sample"])
            if validation_error > 5:
                raise CleanupError(f"held-out probe error {validation_error:.1f} px exceeds 5 px")
            await self.move(home, home_q, [0,0])
            obs, pixel, confidence = await self.capture(tracker)
            return_error = float(np.linalg.norm(pixel-reference))
            if return_error > 5:
                raise CleanupError("final return drift exceeds 5 pixels")
        ident = uuid.uuid4().hex
        data = {"probe_id": ident, "tracker": tracker_kind, "target_binding": target_binding,
                "matrix_px_per_commanded_mm": matrix.tolist(), "reference_pixel": reference.tolist(),
                "final_pixel": pixel.tolist(), "bbox": list(bbox), "home_xyz_m": home.tolist(), "home_q": home_q.tolist(),
                "camera_id": obs["views"]["wrist"]["camera_id"], "image_size": obs["views"]["wrist"]["image_size"],
                "config_hash": h.config_hash, "quality": {**quality, "held_out_error_px": validation_error, "return_error_px": return_error},
                "samples": rows, "elapsed_s": time.monotonic()-started,
                "validity": "Same target, camera, clearance height and local pose only; expires in 300 seconds. Not grasp/contact calibration.",
                "max_radius_mm": 10, "grasp_calibrated": False}
        self.probes[ident] = {"data": data, "tracker": tracker, "created": time.monotonic(), "rotation": h.model.fk(home_q)[:3,:3]}
        (h.out/f"probe-{ident}.json").write_text(json.dumps(data, indent=2))
        h.active_progress = None
        return data

    def compatible(self, probe, state):
        h = self.h
        data = probe["data"]
        if data.get("target_binding"):
            h.targets.validate_binding(**data["target_binding"])
        pose = h.model.fk(state["q"])
        home = np.array(data["home_xyz_m"])
        if (time.monotonic()-probe["created"] > 300 or data["config_hash"] != h.config_hash
                or data["camera_id"] != h.cameras["wrist"]
                or np.linalg.norm(pose[:2,3]-home[:2]) > .010
                or abs(pose[2,3]-home[2]) > .003
                or np.linalg.norm(pose[:3,:3]-probe["rotation"]) > .10):
            raise CleanupError("probe expired or pose/camera/height changed; reprobe")
        return home, np.array(data["home_q"])

    async def align(self, probe_id, goal_pixel, state, scene_clear):
        h = self.h
        self.guard(state, scene_clear)
        if probe_id not in self.probes:
            raise CleanupError("unknown/invalidated probe; run probe_alignment in this session")
        probe = self.probes[probe_id]
        home, home_q = self.compatible(probe, state)
        goal = vector(goal_pixel, 2, "goal_pixel")
        size = probe["data"]["image_size"]
        if np.any(goal < 0) or np.any(goal >= size):
            raise CleanupError("goal pixel outside image")
        tracker, matrix = probe["tracker"], np.array(probe["data"]["matrix_px_per_commanded_mm"])
        self.target_q = None
        obs, pixel, confidence = await self.capture(tracker)
        start_error = float(np.linalg.norm(goal-pixel))
        if start_error > 60:
            raise CleanupError("goal exceeds 60 pixel local capture range")
        offset = (h.model.fk(h.backend.state()["q"])[:2,3]-home[:2])*1000
        total, stable, previous, rows = 0., 0, None, []
        async with asyncio.timeout(45):
            for iteration in range(10):
                h.active_progress = {"phase": "align", "iteration": iteration, "error_px": float(np.linalg.norm(goal-pixel))}
                self.compatible(probe, h.backend.state())
                error = float(np.linalg.norm(goal-pixel))
                if error <= 4:
                    stable += 1
                    if stable >= 2:
                        h.active_progress = None
                        return {"aligned": True, "grasp_ready": False, "probe_id": probe_id,
                                "goal_pixel": goal.tolist(), "final_pixel": pixel.tolist(), "initial_error_px": start_error,
                                "final_error_px": error, "moves": rows, "stable_fresh_frames": stable,
                                "note": "Image feature alignment only; no descent, grasp or contact inference."}
                    obs, pixel, confidence = await self.capture(tracker)
                    continue
                stable = 0
                if previous is not None and error > previous+3:
                    raise CleanupError("alignment error increased; stop and reobserve")
                correction = .8 * np.linalg.solve(matrix.T @ matrix + .04*np.eye(2), matrix.T @ (goal-pixel))
                correction *= min(1., 5./max(np.linalg.norm(correction), 1e-9))
                if np.linalg.norm(offset+correction) > 9.5 or total+np.linalg.norm(correction) > 25:
                    raise CleanupError("local alignment movement budget exceeded")
                if len(rows) >= 8:
                    raise CleanupError("alignment did not converge within eight moves")
                before = pixel.copy()
                offset += correction; total += float(np.linalg.norm(correction))
                measured = await self.move(home, home_q, offset)
                obs, pixel, confidence = await self.capture(tracker)
                prediction_error = float(np.linalg.norm((pixel-before)-matrix @ correction))
                row = {"step_mm": correction.tolist(), "pixel": pixel.tolist(), "error_px": float(np.linalg.norm(goal-pixel)),
                       "prediction_error_px": prediction_error, "confidence": confidence,
                       "measured_xyz_m": obs["visual_sample"]["measured_xyz_m"], "sampling": obs["visual_sample"],
                       "observation_id": obs["observation_id"]}
                rows.append(row); h.log("visual_alignment_step", **row)
                if prediction_error > 6:
                    raise CleanupError("visual response disagrees with probe; target/camera may have moved")
                previous = error
        raise CleanupError("alignment failed to obtain two stable fresh frames")
