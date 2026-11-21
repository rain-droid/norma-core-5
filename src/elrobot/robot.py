"""High-level arm control: state, safe motion, gripper, guarded moves.

`Robot` is the only thing that commands the servos. One motion runs at a time; `stop()` can
interrupt it at any tick. Every joint path is checked for table and self clearance before it
runs, and execution aborts if the arm is blocked or servo state goes stale.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from .config import Config
from .kinematics import (
    GRIPPER_MAX_WIDTH,
    IKResult,
    Kinematics,
    approach_from_pitch,
    jaw_from_yaw,
    quat_from_matrix,
    unit,
)
from .link import Link
from .motion import Trajectory, interpolate_joint
from .servo import ALL_IDS, ARM_IDS, GRIPPER_ID, REG_TORQUE_LIMIT, JointRange

log = logging.getLogger("elrobot.robot")


class RobotError(Exception):
    """Something the caller (a person or an agent) can act on: unreachable, blocked, not ready."""


class Stopped(RobotError):
    """The motion was interrupted by stop()."""


@dataclass
class MoveResult:
    outcome: str  # "done" | "contact"
    detail: str = ""
    residual: float = 0.0  # rad, worst joint error after settling


@dataclass
class ContactGuard:
    """Detects the arm touching something during a slow guarded move.

    Baselines are taken while the move is already under way, so steady gravity load and the
    normal tracking lag of a moving joint are not mistaken for contact.
    """

    load_rise: float = 110.0  # servo load units (0.1 % of stall torque) above baseline
    lag_rise: float = 0.045  # rad of extra tracking error above baseline
    warmup: float = 0.25  # s
    confirm: int = 3  # consecutive ticks

    def __post_init__(self) -> None:
        self._t0: float | None = None
        self._base_load: np.ndarray | None = None
        self._base_lag: np.ndarray | None = None
        self._acc: list[tuple[np.ndarray, np.ndarray]] = []
        self._hits = 0

    def update(self, load: np.ndarray, lag: np.ndarray) -> str | None:
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        if now - self._t0 < self.warmup:
            self._acc.append((np.abs(load), np.abs(lag)))
            return None
        if self._base_load is None:
            if self._acc:
                self._base_load = np.mean([a for a, _ in self._acc], axis=0)
                self._base_lag = np.mean([b for _, b in self._acc], axis=0)
            else:
                self._base_load, self._base_lag = np.abs(load), np.abs(lag)
        d_load = np.abs(load) - self._base_load
        d_lag = np.abs(lag) - self._base_lag
        if d_load.max() > self.load_rise or d_lag.max() > self.lag_rise:
            self._hits += 1
        else:
            self._hits = 0
        if self._hits >= self.confirm:
            j = int(np.argmax(d_load / self.load_rise + d_lag / self.lag_rise)) + 1
            return f"joint {j} (load +{d_load[j - 1]:.0f}, lag +{math.degrees(d_lag[j - 1]):.1f} deg)"
        return None


class Robot:
    def __init__(self, link: Link, cfg: Config):
        self.link = link
        self.cfg = cfg
        self.kin = Kinematics(cfg.urdf)
        self._urdf_lower = self.kin.lower.copy()
        self._urdf_upper = self.kin.upper.copy()
        self._ranges: dict[int, JointRange] = {}
        self._ranges_key: tuple | None = None

        self._motion = asyncio.Lock()
        self._stop_evt = asyncio.Event()
        self._cmd_q: np.ndarray | None = None  # last desired arm angles
        self._bias = np.zeros(7)  # added to the desired angles to cancel servo droop
        self._grip_goal: float | None = None
        self.holding = False
        self.activity = "idle"
        self.source: str | None = None
        # Called with (path_points, target) whenever a motion is planned; path [] clears.
        self.on_plan: Callable[[list, dict | None], None] | None = None
        # Called with each command handed to the servos: {"joints": [7 rad], ...} for an arm
        # trajectory's end point, {"gripper": 0..1} for the jaws.
        self.on_command: Callable[[dict], None] | None = None

    # ================================================================== state

    def ranges(self) -> dict[int, JointRange]:
        """Per-servo tick <-> angle mapping, rebuilt whenever the Station's calibration changes."""
        key = tuple((i, m.range_min, m.range_max) for i, m in sorted(self.link.motors.items()))
        if key != self._ranges_key:
            ranges = {}
            for i, m in self.link.motors.items():
                if i == GRIPPER_ID:
                    lo, hi = self.kin.gripper_lower, self.kin.gripper_upper
                else:
                    lo, hi = float(self._urdf_lower[i - 1]), float(self._urdf_upper[i - 1])
                ranges[i] = JointRange(lower=lo, upper=hi, range_min=m.range_min, range_max=m.range_max)
            self._ranges, self._ranges_key = ranges, key
            if all(i in ranges and ranges[i].valid for i in ARM_IDS):
                lims = [ranges[i].angle_limits() for i in ARM_IDS]
                self.kin.set_limits([a for a, _ in lims], [b for _, b in lims])
        return self._ranges

    @property
    def ready(self) -> bool:
        r = self.ranges()
        return self.link.connected and all(i in r and r[i].valid for i in ALL_IDS)

    def why_not_ready(self) -> str | None:
        if not self.link.connected:
            return "the arm is not connected (no fresh servo state)"
        r = self.ranges()
        bad = [i for i in ALL_IDS if i not in r or not r[i].valid]
        if bad:
            return f"joint(s) {bad} are not calibrated; run scripts/calibrate.py --manual"
        return None

    def q(self) -> np.ndarray:
        """Measured arm joint angles (rad)."""
        r = self.ranges()
        return np.array([r[i].angle(self.link.motors[i].tick) for i in ARM_IDS])

    def q_goal(self) -> np.ndarray:
        r = self.ranges()
        return np.array([r[i].angle(m.goal_tick) if (m := self.link.motors[i]).torque else r[i].angle(m.tick) for i in ARM_IDS])

    def gripper_open(self) -> float:
        """0 = closed, 1 = fully open."""
        n = self.ranges()[GRIPPER_ID].norm(self.link.motors[GRIPPER_ID].tick)
        return n if self.cfg.gripper_open_at_max else 1.0 - n

    def _gripper_tick(self, open_frac: float) -> int:
        n = open_frac if self.cfg.gripper_open_at_max else 1.0 - open_frac
        return self.ranges()[GRIPPER_ID].tick_from_norm(n)

    @property
    def torque_on(self) -> bool:
        return all(self.link.motors[i].torque for i in ARM_IDS if i in self.link.motors)

    def tcp(self) -> np.ndarray:
        return self.kin.fk(self.q())

    def _plan_start(self) -> np.ndarray:
        """Where the next motion starts from: the last command if the arm is holding it,
        otherwise the measured pose. Starting from the command avoids a small dip, because a
        loaded arm sags slightly below what it was told."""
        q = self.q()
        if self._cmd_q is not None and self.torque_on and np.max(np.abs(self._cmd_q - q)) < 0.15:
            return self._cmd_q.copy()
        return q

    def snapshot(self) -> dict:
        """The `state` message of docs/API.md."""
        link = self.link
        out: dict = {
            "type": "state",
            "t": time.time(),
            "mode": link.mode,
            "connected": link.connected,
            "calibrated": False,
            "torque": False,
            "busy": self._motion.locked(),
            "activity": self.activity,
            "source": self.source,
            "joints": [],
            "gripper": {"open": 0.0, "width_mm": 0.0, "holding": self.holding},
            "ee": None,
            "cameras": [],
        }
        now = time.monotonic()
        for name, info in sorted(link.cameras.items()):
            fr = link.frames.get(name)
            if fr is not None:
                out["cameras"].append({"name": name, "width": fr.width, "height": fr.height, "fps": round(info.fps, 1), "age_ms": int((now - fr.stamp) * 1000)})
        if not link.motors:
            return out
        r = self.ranges()
        calibrated = all(i in r and r[i].valid for i in ALL_IDS)
        out["calibrated"] = calibrated
        out["torque"] = any(m.torque for m in link.motors.values())
        names = [j.name for j in self.kin.joints] + ["rev_motor_08"]
        for i in ALL_IDS:
            m = link.motors.get(i)
            if m is None:
                continue
            rng = r.get(i)
            ok = rng is not None and rng.valid
            if i == GRIPPER_ID:
                lo, hi = 0.0, 1.0
                qv = self.gripper_open() if ok else 0.0
                qg = qv
                if ok and m.torque:
                    n = rng.norm(m.goal_tick)
                    qg = n if self.cfg.gripper_open_at_max else 1.0 - n
            else:
                lo, hi = (float(self.kin.lower[i - 1]), float(self.kin.upper[i - 1]))
                qv = rng.angle(m.tick) if ok else 0.0
                qg = rng.angle(m.goal_tick) if ok and m.torque else qv
            out["joints"].append(
                {
                    "id": i, "name": names[i - 1], "q": round(qv, 4), "q_goal": round(qg, 4),
                    "lower": round(lo, 4), "upper": round(hi, 4), "tick": m.tick, "current": m.current,
                    "load": m.load, "temp": m.temp, "volt": m.volt, "torque": m.torque,
                    "error": m.status or (1 if m.has_error else 0),
                }
            )  # fmt: skip
        if calibrated:
            g = self.gripper_open()
            out["gripper"] = {"open": round(g, 3), "width_mm": round(g * GRIPPER_MAX_WIDTH * 1000, 1), "holding": self.holding}
            T = self.kin.fk(self.q())
            out["ee"] = {
                "pos": [round(float(v), 4) for v in T[:3, 3]],
                "quat": [round(float(v), 5) for v in quat_from_matrix(T[:3, :3])],
                "approach": [round(float(v), 4) for v in T[:3, 2]],
            }
        return out

    # ================================================================== safety checks

    def check_config(self, q, *, allow_low: bool = False) -> str | None:
        """Why this arm configuration is not allowed, or None if it is fine."""
        c = self.cfg
        g = self.kin.geometry(q)
        floor = c.table_z + (0.0 if allow_low else c.floor_clearance)
        low = float(min(g.tips[:, 2].min(), g.tcp[2]))
        if low < floor - 1e-6:
            return f"fingertips would be {1000 * (floor - low):.0f} mm into the table"
        body = np.vstack([g.joints[2:], g.wrist[None], g.extras]) if len(g.extras) else np.vstack([g.joints[2:], g.wrist[None]])
        if float(body[:, 2].min()) < c.table_z + c.body_clearance:
            return "the arm body would hit the table"
        if float(g.tcp[1]) < c.min_y or float(g.wrist[1]) < c.min_y:
            return "that is behind the robot, off the table"
        return self.kin.self_collision(q)

    def check_path(self, path, *, allow_low: bool = False) -> str | None:
        for q in path:
            why = self.check_config(q, allow_low=allow_low)
            if why:
                return why
        return None

    # ================================================================== planning

    def solve(self, pos, *, approach=None, pitch: float | None = None, yaw: float | None = None,
              q_seed=None, allow_low: bool = False, **kw) -> IKResult:  # fmt: skip
        """IK for a tool position with optional orientation.

        pitch: degrees below horizontal that the gripper points (90 = straight down). None lets
               the solver pick any orientation unless `approach` is given.
        yaw:   degrees of rotation of the jaws about the approach direction (0 = jaws horizontal).
        """
        pos = np.asarray(pos, dtype=float)
        if approach is None and pitch is not None:
            approach = approach_from_pitch(pos[:2], pitch)
        jaw = jaw_from_yaw(approach, yaw) if (yaw is not None and approach is not None) else None
        seed = self._plan_start() if q_seed is None else q_seed
        return self.kin.ik(
            pos, approach=approach, jaw=jaw, q_seed=seed,
            limit_margin=self.cfg.limit_margin,
            accept=lambda q: self.check_config(q, allow_low=allow_low) is None,
            **kw,
        )  # fmt: skip

    def solve_reach(self, pos, *, pitches=(90, 75, 60, 45, 30, 15), yaw: float | None = None, **kw) -> tuple[IKResult, float | None]:
        """Try progressively flatter approaches until one reaches `pos`. Top-down grasps only
        work close in; further out the gripper has to lean forward."""
        best: IKResult | None = None
        for p in pitches:
            r = self.solve(pos, pitch=p, yaw=yaw, **kw)
            if r.ok:
                return r, float(p)
            if best is None or r.pos_err < best.pos_err:
                best = r
        return best, None

    def plan_linear(self, q_start, pos, *, approach=None, step: float = 0.008, allow_low: bool = False) -> np.ndarray:
        """Joint path that moves the TCP in a straight line to `pos`, holding `approach`
        (defaults to the current approach direction)."""
        T0 = self.kin.fk(q_start)
        p0 = T0[:3, 3]
        a = unit(approach) if approach is not None else T0[:3, 2]
        pos = np.asarray(pos, dtype=float)
        n = max(2, int(math.ceil(np.linalg.norm(pos - p0) / step)) + 1)
        path = [np.asarray(q_start, dtype=float)]
        for k in range(1, n):
            p = p0 + (pos - p0) * (k / (n - 1))
            r = self.kin.ik(p, approach=a, q_seed=path[-1], limit_margin=self.cfg.limit_margin,
                            random_seeds=0, ref_weight=0.05, pos_tol=0.002, ang_tol=math.radians(4))  # fmt: skip
            if not r.ok:
                raise RobotError(f"can't move in a straight line to there: stuck {100 * k / (n - 1):.0f}% of the way ({r.describe()})")
            if np.max(np.abs(r.q - path[-1])) > 0.35:
                raise RobotError("can't move in a straight line to there: the arm would have to flip configuration")
            path.append(r.q)
        why = self.check_path(path, allow_low=allow_low)
        if why:
            raise RobotError(f"straight-line move rejected: {why}")
        return np.array(path)

    def _emit_plan(self, path, target: dict | None = None) -> None:
        if self.on_plan is None:
            return
        try:
            pts = [[round(float(v), 4) for v in self.kin.fk(q)[:3, 3]] for q in path[:: max(1, len(path) // 40)]]
            if len(path):
                pts.append([round(float(v), 4) for v in self.kin.fk(path[-1])[:3, 3]])
            self.on_plan(pts, target)
        except Exception:
            log.exception("plan callback failed")

    # ================================================================== low-level execution

    def _arm_ticks(self, q) -> dict[int, int]:
        r = self.ranges()
        return {i: r[i].tick(float(q[i - 1]), margin_ticks=8) for i in ARM_IDS}

    async def enable(self) -> None:
        """Turn torque on without a jump: every servo is first told to hold where it is."""
        why = self.why_not_ready()
        if why:
            raise RobotError(why)
        if self.torque_on and self.link.motors[GRIPPER_ID].torque:
            return
        link = self.link
        self._bias[:] = 0.0
        await link.write_goals({i: link.motors[i].tick for i in ALL_IDS})
        await link.write_speed({**{i: self.cfg.servo_speed for i in ARM_IDS}, GRIPPER_ID: self.cfg.gripper_speed})
        await link.write_acc({**{i: self.cfg.servo_acc for i in ARM_IDS}, GRIPPER_ID: 0})
        limits = {i: int(self.cfg.arm_torque_limit).to_bytes(2, "little") for i in ARM_IDS}
        limits[GRIPPER_ID] = int(self.cfg.gripper_torque_limit).to_bytes(2, "little")
        await link.sync_write(REG_TORQUE_LIMIT, limits)
        await asyncio.sleep(0.05)
        await link.set_torque(ALL_IDS, True)
        self._cmd_q = self.q()
        t0 = time.monotonic()
        while not self.torque_on and time.monotonic() - t0 < 1.0:
            await asyncio.sleep(0.02)
        if not self.torque_on:
            raise RobotError("servos did not accept torque-on")

    async def release(self) -> None:
        """Torque off: the arm goes limp and will drop if it is not resting on something."""
        self._stop_evt.set()
        await self.link.set_torque(ALL_IDS, False)
        self._cmd_q = None
        self._bias[:] = 0.0
        self.holding = False

    async def stop(self) -> None:
        """Halt any motion now and hold position."""
        self._stop_evt.set()
        if self.link.connected and self.torque_on and self.ready:
            try:
                await self._hold_here()
            except Exception:
                log.exception("failed to write hold position on stop")

    async def _hold_here(self) -> None:
        """Freeze at the measured pose. The droop bias is kept: it is what is holding the arm
        up against gravity, and dropping it would let the arm sag."""
        q = self.q()
        await self.link.write_goals(self._arm_ticks(q + self._bias))
        self._cmd_q = q

    def _integrate_droop(self, lag: np.ndarray, gain: float, dt: float) -> None:
        cfg = self.cfg
        small = np.abs(lag) < cfg.droop_max_lag
        self._bias = np.clip(self._bias + np.where(small, lag, 0.0) * gain * dt, -cfg.droop_max_bias, cfg.droop_max_bias)

    async def _run(self, traj: Trajectory, *, guard: ContactGuard | None = None) -> MoveResult:
        cfg, link = self.cfg, self.link
        dt = 1.0 / cfg.control_hz
        t0 = time.monotonic()
        lag_since: float | None = None
        tick_n = 0
        while True:
            now = time.monotonic()
            t = now - t0
            if self._stop_evt.is_set():
                await self._hold_here()
                raise Stopped("stopped")
            if link.state_age > cfg.stale_state_s:
                raise RobotError(f"lost servo state for {link.state_age:.1f}s; holding position")
            q_cmd = traj.sample(t)
            await link.write_goals(self._arm_ticks(q_cmd + self._bias))
            self._cmd_q = q_cmd
            q_meas = self.q()
            lag = q_cmd - q_meas
            if guard is None:
                # a guarded move detects contact through lag, so it must not cancel it
                self._integrate_droop(lag, cfg.droop_gain_moving, dt)
            if guard is not None:
                load = np.array([link.motors[i].load for i in ARM_IDS], dtype=float)
                hit = guard.update(load, lag)
                if hit:
                    await self._hold_here()
                    return MoveResult("contact", hit)
            worst = float(np.max(np.abs(lag)))
            if worst > cfg.stall_error:
                lag_since = lag_since or now
                if now - lag_since > cfg.stall_time:
                    j = int(np.argmax(np.abs(lag))) + 1
                    await self._hold_here()
                    raise RobotError(f"blocked: joint {j} is {math.degrees(abs(lag[j - 1])):.0f} deg behind where it should be. Something is in the way or the load is too heavy.")
            else:
                lag_since = None
            if t >= traj.duration:
                break
            tick_n += 1
            await asyncio.sleep(max(0.0, t0 + tick_n * dt - time.monotonic()))

        # Settle on the final setpoint, trimming out droop until the joints are where they
        # were asked to be (and have stopped moving).
        end = traj.end
        t1 = time.monotonic()
        residual = float("inf")
        ok_since: float | None = None
        while time.monotonic() - t1 < cfg.settle_timeout:
            if self._stop_evt.is_set():
                await self._hold_here()
                raise Stopped("stopped")
            lag = end - self.q()
            residual = float(np.max(np.abs(lag)))
            if residual < cfg.settle_tol:
                ok_since = ok_since or time.monotonic()
                if time.monotonic() - ok_since > 0.15:
                    break
            else:
                ok_since = None
            if guard is None:
                self._integrate_droop(lag, cfg.droop_gain_settling, dt)
                await link.write_goals(self._arm_ticks(end + self._bias))
            await asyncio.sleep(dt)
        return MoveResult("done", residual=residual)

    async def _begin(self, label: str) -> None:
        why = self.why_not_ready()
        if why:
            raise RobotError(why)
        self._stop_evt.clear()
        self.activity = label
        await self.enable()

    # ================================================================== arm motion

    async def move_joints(self, q_target, *, speed: float = 1.0, label: str = "moving", check: bool = True,
                          allow_low: bool = False, guard: ContactGuard | None = None) -> MoveResult:  # fmt: skip
        """Move the arm joints to `q_target` (7 angles) along a straight joint-space path."""
        async with self._motion:
            await self._begin(label)
            try:
                q0 = self._plan_start()
                q1 = self.kin.clamp(np.asarray(q_target, dtype=float))
                path = interpolate_joint(q0, q1)
                if check:
                    # the start is allowed to be "bad" (e.g. resting on the table); the rest is not
                    why = self.check_path(path[len(path) // 3 :], allow_low=allow_low)
                    if why:
                        raise RobotError(f"move rejected: {why}")
                self._emit_plan(path)
                return await self._execute(path, speed, guard)
            finally:
                self.activity = "idle"

    async def move_path(self, path, *, speed: float = 1.0, label: str = "moving", guard: ContactGuard | None = None,
                        target: dict | None = None) -> MoveResult:  # fmt: skip
        """Follow a pre-planned, pre-checked joint path (e.g. from plan_linear)."""
        async with self._motion:
            await self._begin(label)
            try:
                self._emit_plan(path, target)
                return await self._execute(np.asarray(path), speed, guard)
            finally:
                self.activity = "idle"

    async def _execute(self, path: np.ndarray, speed: float, guard: ContactGuard | None) -> MoveResult:
        speed = min(2.0, max(0.05, speed))
        if self.on_command is not None:
            self.on_command({"joints": [round(float(v), 5) for v in path[-1]], "waypoints": len(path), "speed": speed, "guarded": guard is not None})
        traj = Trajectory.through(path, self.cfg.max_joint_speed * speed, self.cfg.max_joint_accel * min(1.5, max(0.3, speed)))
        return await self._run(traj, guard=guard)

    async def move_to(self, pos, *, pitch: float | None = None, yaw: float | None = None, approach=None,
                      speed: float = 1.0, linear: bool = False, label: str = "moving",
                      allow_low: bool = False, guard: ContactGuard | None = None) -> MoveResult:  # fmt: skip
        """Move the tool point to `pos` (metres, base frame).

        With `pitch` None the most vertical reachable approach is chosen automatically.
        `linear` moves the tool in a straight line, keeping its current orientation unless
        pitch/approach is given.
        """
        pos = np.asarray(pos, dtype=float)
        q0 = self._plan_start()
        if linear:
            a = approach
            if a is None and pitch is not None:
                a = approach_from_pitch(pos[:2], pitch)
            path = self.plan_linear(q0, pos, approach=a, allow_low=allow_low)
            target = {"pos": pos.tolist(), "approach": self.kin.fk(path[-1])[:3, 2].tolist()}
            return await self.move_path(path, speed=speed, label=label, guard=guard, target=target)
        if approach is None and pitch is None:
            r, used = self.solve_reach(pos, yaw=yaw, allow_low=allow_low)
        else:
            r, used = self.solve(pos, approach=approach, pitch=pitch, yaw=yaw, allow_low=allow_low), pitch
        if not r.ok:
            raise RobotError(self._unreachable(pos, r))
        return await self.move_joints(r.q, speed=speed, label=label, allow_low=allow_low, guard=guard)

    def _unreachable(self, pos, r: IKResult) -> str:
        d = float(np.linalg.norm(np.asarray(pos)[:2] - self.kin.joints[0].pre[:2, 3]))
        hint = ""
        if d > 0.40:
            hint = f" It is {d * 100:.0f} cm from the base and the arm reaches about 40 cm."
        elif d < 0.08:
            hint = " It is too close to the base."
        return f"can't reach ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}): {r.describe()}.{hint}"

    async def move_relative(self, delta, *, speed: float = 0.6, label: str = "moving", allow_low: bool = False,
                            guard: ContactGuard | None = None) -> MoveResult:  # fmt: skip
        """Shift the tool point by `delta` metres in the base frame, in a straight line."""
        q0 = self._plan_start()
        p = self.kin.fk(q0)[:3, 3] + np.asarray(delta, dtype=float)
        path = self.plan_linear(q0, p, allow_low=allow_low)
        return await self.move_path(path, speed=speed, label=label, guard=guard)

    # ================================================================== gripper

    async def _gripper_wait(self, timeout: float = 2.5) -> float:
        """Wait for the jaws to stop moving; returns the achieved opening (0..1)."""
        # StationLink replaces link.motors on every state update, so look the servo up each time
        tick_now = lambda: self.link.motors[GRIPPER_ID].tick  # noqa: E731
        t0 = time.monotonic()
        last_tick = tick_now()
        still_since = None
        # the real servo starts moving ~0.3 s after the command; don't mistake that for a stall
        while abs(tick_now() - last_tick) <= 2 and time.monotonic() - t0 < 0.6:
            await asyncio.sleep(0.02)
        while time.monotonic() - t0 < timeout:
            if self._stop_evt.is_set():
                raise Stopped("stopped")
            await asyncio.sleep(0.04)
            tick = tick_now()
            if abs(tick - last_tick) <= 2:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 0.2:
                    break
            else:
                still_since = None
            last_tick = tick
        return self.gripper_open()

    async def set_gripper(self, open_frac: float, *, label: str = "gripper") -> float:
        """Open the jaws to `open_frac` (0 closed .. 1 fully open). Returns what was reached."""
        async with self._motion:
            await self._begin(label)
            try:
                open_frac = min(1.0, max(0.0, float(open_frac)))
                await self.link.write_goals({GRIPPER_ID: self._gripper_tick(open_frac)})
                if self.on_command is not None:
                    self.on_command({"gripper": open_frac})
                self._grip_goal = open_frac
                reached = await self._gripper_wait()
                if open_frac > 0.5:
                    self.holding = False
                return reached
            finally:
                self.activity = "idle"

    async def grasp(self) -> dict:
        """Close on whatever is between the jaws and report whether something is held.

        The jaws are driven shut; if they stall before closing fully, an object is in the way.
        The goal is then backed off to just past the stall point so the servo keeps a firm grip
        without sitting at its current limit.
        """
        async with self._motion:
            await self._begin("grasping")
            try:
                closed_tick = self._gripper_tick(0.0)
                await self.link.write_goals({GRIPPER_ID: closed_tick})
                if self.on_command is not None:
                    self.on_command({"gripper": 0.0, "until_stall": True})
                reached = await self._gripper_wait(timeout=3.0)
                self.holding = reached > self.cfg.holding_min_open
                if self.holding:
                    tick = self.link.motors[GRIPPER_ID].tick
                    step = self.cfg.grasp_squeeze_ticks * (1 if closed_tick > tick else -1)
                    await self.link.write_goals({GRIPPER_ID: tick + step})
                self._grip_goal = reached
                return {"holding": self.holding, "width_mm": round(reached * GRIPPER_MAX_WIDTH * 1000, 1)}
            finally:
                self.activity = "idle"

    def still_holding(self) -> bool:
        """Re-check a grasp: if the jaws have closed to nothing, the object slipped out."""
        if self.holding and self.gripper_open() <= self.cfg.holding_min_open * 0.6:
            self.holding = False
        return self.holding
