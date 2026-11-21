"""The raw interface, for your own Python: read the state, send one joint command.

    from robot_config.control import Robot

    robot = Robot()
    state = robot.observe()                       # measured joints, tool point, camera paths
    q = [j["rad"] for j in state["joints"]]
    q[6] += 0.3                                   # roll the wrist 17 degrees
    result = robot.step(q, gripper=None)          # one absolute command, returns measurements

`step` is tools/joint_command.py: seven absolute joint angles in radians plus an optional jaw
opening 0..1, no IK, no planning. Every other tool in tools/ ends in the same command. Run
scripts with the project interpreter from the workspace root: `uv run python my_script.py`.
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from util.cli import request  # noqa: E402


class Robot:
    def observe(self) -> dict:
        """Latest published state and camera image paths. Sends nothing to the robot."""
        latest = json.loads((ROOT / "perception/latest.json").read_text())
        state = json.loads((ROOT / "perception/proprioception/state.json").read_text())
        return {**state, "sequence": latest["sequence"], "cameras": latest["cameras"]}

    def step(self, joints, gripper=None, speed: float = 0.8, check: bool = True) -> dict:
        """One absolute joint command; returns after the arm has settled."""
        return request("joint_command", {"joints": [float(v) for v in joints], "gripper": gripper, "speed": speed, "check": check})

    def call(self, tool: str, **arguments) -> dict:
        """Any tool in tools/, by name."""
        return request(tool, arguments)


if __name__ == "__main__":
    print(json.dumps(Robot().observe(), indent=1))
