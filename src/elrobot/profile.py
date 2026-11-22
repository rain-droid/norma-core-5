"""Which arm this is. Everything that differs between the supported arms lives here.

Set `robot: so101` or `robot: elrobot` in config.yaml (or ELROBOT_ROBOT in the environment).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Profile:
    name: str
    urdf: str  # relative to assets/
    arm_joint_names: tuple[str, ...]
    tip_link: str
    arm_ids: tuple[int, ...]
    gripper_id: int
    ready_deg: tuple[float, ...]
    # (tool point, pitch) candidates for the camera-up pose, best first
    observe_poses: tuple
    # joint-space keyframes for gestures: name -> list of (absolute degrees, speed) or special steps
    ik_ang_tol_deg: float = 4.0
    ik_approach_weight: float = 0.15
    # file with per-joint {id, center_tick, direction, homing_offset, range_min, range_max};
    # None means the Station's own calibration is used
    calibration: str | None = None
    camera: dict | None = None  # wrist camera intrinsics at a reference size
    camera_link: str | None = None  # optical frame link in the URDF
    reach_m: float = 0.38
    base_y: float = 0.04  # where the shoulder axis is, for "distance from the base"
    pitches: tuple[int, ...] = (90, 80, 70, 60, 50, 40, 30)
    extras: dict = field(default_factory=dict)

    @property
    def all_ids(self) -> tuple[int, ...]:
        return self.arm_ids + (self.gripper_id,)

    def load_calibration(self) -> dict[int, dict] | None:
        if not self.calibration:
            return None
        data = yaml.safe_load((REPO_ROOT / self.calibration).read_text())["joints"]
        return {int(v["id"]): {"name": k, **v} for k, v in data.items()}


ELROBOT = Profile(
    name="elrobot",
    urdf="elrobot/elrobot_follower.urdf",
    arm_joint_names=tuple(f"rev_motor_{i:02d}" for i in range(1, 8)),
    tip_link="Gripper_Base_v1_1",
    arm_ids=(1, 2, 3, 4, 5, 6, 7),
    gripper_id=8,
    ready_deg=(0.0, -21.0, -29.0, 36.0, 0.0, 73.5, 0.0),
    observe_poses=(([0.0, 0.15, 0.16], 60), ([0.0, 0.18, 0.18], 60), ([0.0, 0.15, 0.14], 70)),
)

SO101 = Profile(
    name="so101",
    urdf="so101/so101.urdf",
    arm_joint_names=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"),
    tip_link="pgripper_base_link",
    arm_ids=(1, 2, 3, 4, 5),
    gripper_id=6,
    ready_deg=(0.0, -45.0, 45.0, 75.0, 0.0),
    # the camera is tilted 25 deg further down than the gripper, so a shallow tool pitch looks out over the table
    observe_poses=(([0.0, 0.17, 0.25], 25), ([0.0, 0.18, 0.22], 30), ([0.0, 0.16, 0.20], 40)),
    # 5 joints cannot hold position and a full orientation at once: position wins
    ik_ang_tol_deg=7.0,
    ik_approach_weight=0.06,
    calibration="data/so101_calibration.yaml",
    # Innomaker U20CAM on the gripper, calibrated at 1280x720 (so_arm101_pgripper_cameras)
    camera={
        "width": 1280, "height": 720,
        "fx": 1062.9552590981268, "fy": 1061.0780596993211, "cx": 586.96477675128847, "cy": 418.95591086714143,
        "k1": -0.48729951756808654, "k2": 0.22328964397337178, "p1": -0.0092010518372207441, "p2": 0.00086924935412355611,
    },  # fmt: skip
    camera_link="camera_optical_frame",
    reach_m=0.36,
    base_y=0.04,
)

PROFILES = {"elrobot": ELROBOT, "so101": SO101}


def _active() -> Profile:
    name = os.environ.get("ELROBOT_ROBOT")
    if not name:
        cfg = REPO_ROOT / "config.yaml"
        if cfg.exists():
            name = (yaml.safe_load(cfg.read_text()) or {}).get("robot")
    name = name or "elrobot"
    if name not in PROFILES:
        raise ValueError(f"unknown robot {name!r}; known: {sorted(PROFILES)}")
    return PROFILES[name]


ACTIVE = _active()
