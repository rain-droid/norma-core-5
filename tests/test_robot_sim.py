"""End-to-end checks of Robot against the simulated servo bus."""

import asyncio
import math
import time

import numpy as np
import pytest

from elrobot.config import Config
from elrobot.kinematics import Kinematics
from elrobot.link import SimLink
from elrobot.robot import Robot, RobotError, Stopped


def make() -> tuple[Robot, SimLink]:
    cfg = Config()
    kin = Kinematics(cfg.urdf)
    limits = {j.id: (j.lower, j.upper) for j in kin.joints}
    limits[8] = (kin.gripper_lower, kin.gripper_upper)
    link = SimLink(limits)
    return Robot(link, cfg), link


async def started():
    robot, link = make()
    await link.start()
    assert await link.wait_connected(2)
    return robot, link


def test_tick_angle_mapping_round_trips():
    robot, link = make()
    r = robot.ranges()
    for i in range(1, 8):
        for a in np.linspace(r[i].lower + 0.1, r[i].upper - 0.1, 7):
            assert abs(r[i].angle(r[i].tick(a)) - a) < 0.002
    # the simulated arm starts mid-range on every joint
    assert np.allclose(robot.q(), [(r[i].lower + r[i].upper) / 2 for i in range(1, 8)], atol=0.002)


def test_move_to_reaches_target_and_reports_state():
    async def run():
        robot, link = await started()
        try:
            target = np.array([0.05, 0.20, 0.05])
            res = await robot.move_to(target, pitch=90)
            assert res.outcome == "done"
            T = robot.tcp()
            assert np.linalg.norm(T[:3, 3] - target) < 0.006, T[:3, 3]
            assert T[:3, 2] @ np.array([0, 0, -1]) > math.cos(math.radians(6))
            s = robot.snapshot()
            assert s["connected"] and s["calibrated"] and s["torque"] and not s["busy"]
            assert len(s["joints"]) == 8 and s["ee"] is not None
        finally:
            await link.stop()

    asyncio.run(run())


def test_table_and_reach_are_enforced():
    async def run():
        robot, link = await started()
        try:
            with pytest.raises(RobotError, match="reach"):
                await robot.move_to([0.0, 0.60, 0.10])
            with pytest.raises(RobotError):
                await robot.move_to([0.0, 0.22, -0.03], pitch=90)
        finally:
            await link.stop()

    asyncio.run(run())


def test_pace_slows_a_motion_under_way_and_wears_off():
    async def run():
        robot, link = await started()
        try:
            a, b = robot.q(), robot.q()
            b[0] += 0.8

            async def timed(q, slow_for: float = 0.0) -> float:
                t0 = time.monotonic()
                move = asyncio.create_task(robot.move_joints(q))
                while slow_for and time.monotonic() - t0 < slow_for:
                    assert robot.set_pace(0.25, 0.2) == 0.25
                    await asyncio.sleep(0.05)
                assert (await move).outcome == "done"
                return time.monotonic() - t0

            full = await timed(b)
            slowed = await timed(a, slow_for=1.0)
            assert slowed > full + 0.5, (full, slowed)  # a second at a quarter speed costs three quarters of one
            assert np.max(np.abs(robot.q() - a)) < 0.02  # and it still arrives
            await asyncio.sleep(0.3)
            assert robot.pace == 1.0
            assert abs(await timed(b) - full) < 0.3  # back to the speed it had
        finally:
            await link.stop()

    asyncio.run(run())


def test_stop_interrupts_motion_and_next_move_works():
    async def run():
        robot, link = await started()
        try:
            await robot.move_to([0.0, 0.20, 0.06], pitch=90)
            task = asyncio.create_task(robot.move_to([0.12, 0.18, 0.04], pitch=90, speed=0.3))
            await asyncio.sleep(0.4)
            await robot.stop()
            with pytest.raises(Stopped):
                await task
            p_stop = robot.tcp()[:3, 3].copy()
            await asyncio.sleep(0.3)
            assert np.linalg.norm(robot.tcp()[:3, 3] - p_stop) < 0.004  # it actually stopped
            assert (await robot.move_to([0.0, 0.20, 0.06], pitch=90)).outcome == "done"
        finally:
            await link.stop()

    asyncio.run(run())


def test_linear_move_is_straight():
    async def run():
        robot, link = await started()
        try:
            await robot.move_to([0.0, 0.20, 0.09], pitch=90)
            start = robot.tcp()[:3, 3].copy()
            task = asyncio.create_task(robot.move_relative([0, 0, -0.06], speed=0.5))
            worst = 0.0
            while not task.done():
                await asyncio.sleep(0.03)
                p = robot.tcp()[:3, 3]
                worst = max(worst, float(np.linalg.norm(p[:2] - start[:2])))
            await task
            assert worst < 0.008, worst
            assert abs(robot.tcp()[2, 3] - (start[2] - 0.06)) < 0.006
        finally:
            await link.stop()

    asyncio.run(run())


def test_grasp_detects_an_object_and_an_empty_close():
    async def run():
        robot, link = await started()
        try:
            await robot.set_gripper(1.0)
            assert robot.gripper_open() > 0.95
            assert (await robot.grasp())["holding"] is False
            await robot.set_gripper(1.0)
            block_tick = robot._gripper_tick(0.45)  # a 24 mm object between the jaws
            link.gripper_block = lambda new: block_tick
            g = await robot.grasp()
            assert g["holding"] is True and 20 < g["width_mm"] < 28, g
        finally:
            await link.stop()

    asyncio.run(run())
