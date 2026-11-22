"""The raw interface, for your own Python: read the state, send joint commands.

    from robot_config.control import Robot

    robot = Robot()
    state = robot.observe()                 # measured joints, tool point, camera image paths
    q = [j["rad"] for j in state["joints"]]
    q[6] += 0.3                             # roll the wrist 17 degrees
    result = robot.step(q)                  # one absolute joint command; returns measurements
    robot.step([q1, q2, q3])                # a list of waypoints is travelled in one move
    robot.step(None, gripper=0.0)           # the jaws alone: 0 grasps, 1 opens

`step` is tools/joint_command.py, and joint commands are all the robot takes: seven absolute
joint angles in radians and a jaw opening 0..1. There is no IK or planning on the other side.
For that, use what the tools use: `from util.planning import Planner` (IK, straight lines) and
`from util.arm import Arm` (state, camera frames). Run scripts from the workspace root with
the project interpreter: `pixi run python my_script.py`.
"""

import importlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from util.cli import call  # noqa: E402


class Robot:
    def observe(self) -> dict:
        """The newest published state and camera image paths (perception/). Sends nothing."""
        latest = json.loads((ROOT / "perception/latest.json").read_text())
        state = json.loads((ROOT / "perception/proprioception/state.json").read_text())
        return {**state, "sequence": latest["sequence"], "cameras": latest["cameras"]}

    def step(self, joints, gripper=None, speed: float = 0.8, check: bool = True, allow_low: bool = False) -> dict:
        """One joint command; returns after the arm has settled. Recorded like any tool call."""
        return self.call("joint_command", joints=joints, gripper=gripper, speed=speed, check=check, allow_low=allow_low)

    def call(self, tool: str, **arguments) -> dict:
        """Any tool in tools/, by name, run in this process."""
        return call(getattr(importlib.import_module(tool), tool), arguments)


if __name__ == "__main__":
    print(json.dumps(Robot().observe(), indent=1))
