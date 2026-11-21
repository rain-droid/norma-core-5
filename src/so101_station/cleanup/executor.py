"""Slow, measured, cancellable joint execution; all motion stays in this process."""
import asyncio
import math
import time

import numpy as np

from .geometry import CleanupError


class EmptyGrasp(CleanupError):
    """Motion completed, but aperture gives no evidence of an object."""


class Executor:
    step_wait_s = 1.0
    def __init__(self, backend, model, floor_z, speed_rad_s=.15, command_hz=20):
        if not 0 < speed_rad_s <= .45:
            raise CleanupError("speed must be in (0, 0.45] rad/s")
        self.speed = speed_rad_s
        if command_hz not in {10, 20}:
            raise CleanupError("command cadence must be 10 or 20 Hz")
        self.command_hz = command_hz
        self.backend, self.model, self.floor = backend, model, floor_z
        self.stopped = asyncio.Event()
        self.grip_goal = None

    def check(self):
        if self.stopped.is_set():
            raise CleanupError("stopped; no further stage will execute")

    async def stop(self):
        self.stopped.set()
        await self.backend.hold()

    async def move(self, target):
        self.check()
        self.model.validate(target, self.floor)
        start_state = self.backend.state()
        start = np.array(start_state["q"])
        target = np.asarray(target)
        distance = np.max(np.abs(target - start))
        if distance > 0.27:
            raise CleanupError("segment exceeds 0.27 rad; path must be interpolated")
        duration = max(.1, float(distance / self.speed))
        steps = max(1, math.ceil(duration * self.command_hz))
        gripper = self.grip_goal if self.grip_goal is not None else start_state["ticks"][6]
        previous = start.copy()
        for t in np.linspace(0, 1, steps + 1)[1:]:
            self.check()
            q = start + t * (target - start)
            deadline = time.monotonic() + self.step_wait_s
            while True:
                self.check()
                state = self.backend.state()  # Enforces fresh state and matching calibration.
                measured = np.array(state["q"])
                if np.max(np.abs(measured - previous)) > .08 + 1e-9:
                    raise CleanupError("arm is not following the commanded path")
                if np.max(np.abs(measured - q)) <= .08 + 1e-9:
                    break
                if time.monotonic() >= deadline:
                    raise CleanupError("arm did not catch up before the next waypoint")
                # Hold the existing goal; never command farther ahead to force progress.
                await asyncio.sleep(.02)
            self.model.validate(q, self.floor)
            await self.backend.goals(self.model.ticks(q, gripper), self.stopped)
            previous = q
            await asyncio.sleep(.001 if self.backend.mode == "sim" else 1/self.command_hz)
        deadline = time.monotonic() + 3
        while True:
            self.check()
            error = np.max(np.abs(np.array(self.backend.state()["q"]) - target))
            if error < .012:
                break
            if time.monotonic() > deadline:
                raise CleanupError("arm did not settle at the target")
            await asyncio.sleep(.05)

    async def path(self, path):
        for q in path:
            await self.move(q)

    async def gripper(self, open_fraction, *, grasp=False):
        self.check()
        state = self.backend.state()
        start = state["ticks"][6]
        target = self.model.calibration.gripper_tick(open_fraction)
        q = state["q"]
        duration = max(.3, abs(target - start) / 200)
        steps = max(1, math.ceil(duration * 20))
        last_actual, stable = start, 0
        for value in np.linspace(start, target, steps + 1)[1:]:
            self.check()
            actual = self.backend.state()["ticks"][6]
            stable = stable + 1 if abs(actual - last_actual) <= 2 else 0
            last_actual = actual
            if grasp and stable >= 10 and actual > value + 30 and 2090 < actual < 3400:
                self.grip_goal = max(2048, actual - 8)
                await self.backend.goals(self.model.ticks(q, self.grip_goal), self.stopped)
                return {"contact_inferred": True, "tick": actual}
            await self.backend.goals(self.model.ticks(q, int(round(value))), self.stopped)
            self.grip_goal = int(round(value))
            await asyncio.sleep(.001 if self.backend.mode == "sim" else .05)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            self.check()
            actual = self.backend.state()["ticks"][6]
            if grasp and 2090 < actual < 3400:
                self.grip_goal = max(2048, actual - 8)
                await self.backend.goals(self.model.ticks(q, self.grip_goal), self.stopped)
                return {"contact_inferred": True, "tick": actual}
            if abs(actual - target) < 12:
                if grasp:
                    raise EmptyGrasp("gripper closed without detectable contact; pickup unconfirmed")
                return {"contact_inferred": False, "tick": actual}
            await asyncio.sleep(.05)
        raise CleanupError("gripper did not reach its target")
