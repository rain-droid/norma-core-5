"""Runtime configuration: defaults, overridden by `config.yaml` at the repo root and by env vars."""

from __future__ import annotations

import os
from dataclasses import dataclass, field, fields
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass
class Config:
    # --- processes
    station_host: str = "127.0.0.1:8888"
    host: str = "127.0.0.1"
    port: int = 8765
    sim: bool = False
    voice: bool = True  # start the conversational agent with the daemon
    bus_serial: str | None = None
    # camera unique-id substring -> name. The two KD cameras are the same model, so they can only be
    # told apart by the USB port they are on; `scripts/probe.py` prints the ids.
    cameras: dict[str, str] = field(default_factory=dict)

    # --- paths
    urdf: Path = REPO_ROOT / "assets/elrobot/elrobot_follower.urdf"
    assets_dir: Path = REPO_ROOT / "assets"
    web_dist: Path = REPO_ROOT / "web/dist"
    data_dir: Path = REPO_ROOT / "data"
    # RoboHarness-style agent workspace: tools, skills, live perception and call history as files.
    workspace_dir: Path = REPO_ROOT / "workspace"
    # The second arm's daemon (scripts/second_arm_up.sh), watched for the digital twin. Empty: no second arm.
    second_arm_url: str = "http://127.0.0.1:8770"

    # --- workspace
    table_z: float = 0.0  # table height in the base frame (base sits on the table)
    floor_clearance: float = 0.008  # fingertips never go lower than table_z + this
    body_clearance: float = 0.03  # elbow/wrist joints stay this far above the table
    min_y: float = -0.10  # the tool never goes further behind the base than this (off the table)

    # --- motion
    control_hz: float = 50.0
    # Cruise speed and ramp acceleration of the trapezoidal profile at speed=1.0. Chosen so moves
    # take as long as they did under the minimum-jerk profile these replaced (1.2 and 5.0 there).
    max_joint_speed: float = 0.96  # rad/s
    max_joint_accel: float = 3.9  # rad/s^2
    limit_margin: float = 0.06  # rad kept clear of each joint limit
    # Torque limit (0..1000) written to the arm servos when torque comes on. The Station's
    # calibration leaves it at 500, which on a 7.5 V supply is too weak to hold the arm out.
    arm_torque_limit: int = 800
    servo_speed: int = 0  # goal-speed register while streaming (0 = servo max)
    servo_acc: int = 150  # acceleration register while streaming (x100 ticks/s^2, 0 = max)
    settle_tol: float = 0.012  # rad: a move has arrived when every joint is this close
    settle_timeout: float = 2.0
    # The Station sets the servos to a soft proportional-only gain, so a loaded joint settles
    # several degrees short of its goal. An outer loop integrates that error into a per-joint
    # bias on the command. Gains are per second; the bias is capped so a blocked joint cannot
    # be driven hard into an obstacle.
    droop_gain_moving: float = 1.5
    droop_gain_settling: float = 5.0
    droop_max_bias: float = 0.16  # rad
    droop_max_lag: float = 0.12  # rad: larger errors are obstacles or fast motion, not droop
    # a joint that lags its command by this much for this long means the arm is blocked
    stall_error: float = 0.22
    stall_time: float = 0.45
    stale_state_s: float = 0.4

    # --- leading the arm by hand (POST /api/guide). The servos are soft (proportional only), so a
    # hand displaces a joint from where it is told to be; the command then follows the hand.
    guide_deadband: float = 0.045  # rad a joint must be pushed off its resting error before the command follows
    guide_gain: float = 6.0  # 1/s: how fast the command follows what is beyond the deadband
    guide_max_speed: float = 0.9  # rad/s
    guide_idle_s: float = 3.0  # left alone this long after having been moved, the guiding ends

    # --- gripper
    gripper_speed: int = 1500  # ticks/s
    gripper_torque_limit: int = 500  # 0..1000
    grasp_squeeze_ticks: int = 40  # how far past the stall point to hold an object
    holding_min_open: float = 0.06  # jaws stopped wider than this => holding something
    gripper_open_at_max: bool = True  # which end of the gripper servo's arc is "open"

    @classmethod
    def load(cls, path: Path | None = None) -> "Config":
        cfg = cls()
        path = path or REPO_ROOT / "config.yaml"
        if path.exists():
            data = yaml.safe_load(path.read_text()) or {}
            known = {f.name: f for f in fields(cls)}
            for k, v in data.items():
                if k not in known:
                    raise ValueError(f"unknown key {k!r} in {path}")
                if known[k].type in ("Path", Path):
                    v = Path(v)
                setattr(cfg, k, v)
        if os.environ.get("ELROBOT_SIM"):
            cfg.sim = os.environ["ELROBOT_SIM"] not in ("0", "false", "")
        if os.environ.get("ELROBOT_VOICE"):
            cfg.voice = os.environ["ELROBOT_VOICE"] not in ("0", "false", "")
        if os.environ.get("ELROBOT_PORT"):
            cfg.port = int(os.environ["ELROBOT_PORT"])
        if os.environ.get("ELROBOT_STATION"):
            cfg.station_host = os.environ["ELROBOT_STATION"]
        return cfg
