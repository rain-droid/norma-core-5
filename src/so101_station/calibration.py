"""Load aligned SO-101 calibration without touching servo EEPROM.

Position registers already include the servo's homing offset. Apply only the
exported center_tick/direction here; subtracting homing_offset again is wrong.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

RAD_PER_TICK = 2 * math.pi / 4096
DEFAULT_PROFILE = Path(__file__).parent / "profiles" / "pgripper"
JOINT_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}
EPS = 1e-9  # The upstream YAML rounds angles to 12 significant digits.


class CalibrationError(ValueError):
    pass


def _mapping(value, label: str) -> dict:
    if not isinstance(value, dict):
        raise CalibrationError(f"{label} must be a mapping")
    return value


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise CalibrationError(f"duplicate configuration key: {key}")
        result[key] = value
    return result


class _UniqueLoader(yaml.SafeLoader):
    pass


def _yaml_mapping(loader, node):
    return _unique_pairs(loader.construct_pairs(node))


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _yaml_mapping)


def _read(path: Path) -> dict:
    try:
        text = path.read_text()
        value = json.loads(text, object_pairs_hook=_unique_pairs) if path.suffix == ".json" else yaml.load(text, Loader=_UniqueLoader)
        return _mapping(value, str(path))
    except (OSError, json.JSONDecodeError, yaml.YAMLError) as exc:
        raise CalibrationError(f"cannot load {path}: {exc}") from exc


def _integer(value, label: str, low: int, high: int) -> int:
    if type(value) is not int or not low <= value <= high:
        raise CalibrationError(f"{label} must be an integer in {low}..{high}")
    return value


def _number(value, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise CalibrationError(f"{label} must be a finite number")
    return float(value)


def _joints(data: dict, label: str, key: str | None = None) -> dict:
    items = _mapping(data.get(key), label) if key else data
    if set(items) != set(JOINT_IDS):
        raise CalibrationError(f"{label} must contain exactly: {', '.join(JOINT_IDS)}")
    for name, item in items.items():
        _mapping(item, f"{label}.{name}")
    return items


@dataclass(frozen=True)
class JointCalibration:
    name: str
    motor_id: int
    homing_offset: int
    range_min: int
    range_max: int
    center_tick: int
    direction: int
    lower: float
    upper: float

    def ticks_to_radians(self, tick: int) -> float:
        tick = _integer(tick, f"{self.name} tick", 0, 4095)
        return self.direction * (tick - self.center_tick) * RAD_PER_TICK

    def radians_to_ticks(self, angle: float) -> int:
        angle = _number(angle, f"{self.name} angle")
        if not self.lower - EPS <= angle <= self.upper + EPS:
            raise CalibrationError(f"{self.name} angle {angle:.6f} outside {self.lower:.6f}..{self.upper:.6f} rad")
        tick = round(self.center_tick + self.direction * angle / RAD_PER_TICK)
        self.validate_tick(tick)
        return tick

    def validate_tick(self, tick: int) -> None:
        angle = self.ticks_to_radians(tick)
        if not self.range_min <= tick <= self.range_max or not self.lower - EPS <= angle <= self.upper + EPS:
            raise CalibrationError(f"{self.name} tick {tick} outside calibrated/canonical limits")

    def nudge(self, current_tick: int, delta_degrees: float) -> int:
        self.validate_tick(current_tick)
        delta = _number(delta_degrees, "delta_degrees")
        if abs(delta) > 3:
            raise CalibrationError("a preview nudge is limited to 3 degrees")
        return self.radians_to_ticks(self.ticks_to_radians(current_tick) + math.radians(delta))


@dataclass(frozen=True)
class Calibration:
    joints: tuple[JointCalibration, ...]
    source: dict

    def joint(self, name: str) -> JointCalibration:
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise CalibrationError(f"unknown joint {name!r}; choose from {', '.join(JOINT_IDS)}")

    def gripper_tick(self, open_fraction: float) -> int:
        fraction = _number(open_fraction, "gripper opening")
        if not 0 <= fraction <= 1:
            raise CalibrationError("gripper opening must be in 0..1 (0 closed, 1 open)")
        gripper = self.joint("gripper")
        tick = round(2048 + fraction * (gripper.center_tick - 2048))
        gripper.validate_tick(tick)
        return tick

    def summary(self) -> dict:
        return {
            "robot": self.source["robot_id"],
            "variant": "SO-101 with NormaCore pgripper (5 arm joints + gripper)",
            "source_revision": self.source["revision"],
            "hardware_verified": False,
            "joints": [
                {"name": j.name, "motor_id": j.motor_id, "center_tick": j.center_tick,
                 "direction": j.direction, "homing_offset": j.homing_offset,
                 "tick_limits": [j.range_min, j.range_max], "radian_limits": [j.lower, j.upper]}
                for j in self.joints
            ],
            "gripper": {"closed_tick": self.gripper_tick(0), "open_tick": self.gripper_tick(1)},
        }


def load_calibration(directory: str | Path = DEFAULT_PROFILE) -> Calibration:
    directory = Path(directory)
    measured = _joints(_read(directory / "calibration.json"), "calibration")
    state = _joints(_read(directory / "state.yaml"), "state", "joints")
    control = _joints(_read(directory / "control.yaml"), "control", "joints")
    limits = _joints(_read(directory / "joint_limits.yaml"), "limits", "joint_limits")
    canonical = _joints(_read(directory / "canonical_joint_limits.yaml"), "canonical", "joint_limits")
    source = _read(directory / "SOURCE.json")
    if not all(isinstance(source.get(k), str) and source[k] for k in ("robot_id", "revision")):
        raise CalibrationError("SOURCE.json needs robot_id and revision")
    joints = []
    for name, expected_id in JOINT_IDS.items():
        raw, mapping, config = measured[name], state[name], control[name]
        for record, label in ((raw, "calibration"), (mapping, "state"), (config, "control")):
            if _integer(record.get("id"), f"{label}.{name}.id", 1, 253) != expected_id:
                raise CalibrationError(f"{name} must use motor {expected_id}")
        low = _integer(raw.get("range_min"), f"{name}.range_min", 0, 4095)
        high = _integer(raw.get("range_max"), f"{name}.range_max", 0, 4095)
        offset = _integer(raw.get("homing_offset"), f"{name}.homing_offset", -2047, 2047)
        mode = _integer(raw.get("drive_mode"), f"{name}.drive_mode", 0, 1)
        if not low < high or not low <= 2048 <= high:
            raise CalibrationError(f"{name} needs a non-wrapping range containing tick 2048")
        center = _integer(mapping.get("center_tick"), f"{name}.center_tick", 0, 4095)
        direction = _integer(mapping.get("direction"), f"{name}.direction", -1, 1)
        if name == "gripper":
            expected_center, expected_direction = (high, -1) if mode == 0 else (low, 1)
            if (low if mode == 0 else high) != 2048:
                raise CalibrationError("gripper closed endpoint must be tick 2048")
        else:
            expected_center, expected_direction = 2048, 1
            if mode != 0:
                raise CalibrationError(f"{name}: unsupported arm drive_mode {mode}")
        if (center, direction) != (expected_center, expected_direction):
            raise CalibrationError(f"{name}: state mapping disagrees with aligned calibration")
        for key, expected in {"center_tick": center, "direction": direction, "homing_offset": offset,
                              "range_min": low, "range_max": high}.items():
            if type(config.get(key)) is not int or config[key] != expected:
                raise CalibrationError(f"{name}: control {key} disagrees with calibration/state")
        measured_lo, measured_hi = sorted(direction * (tick - center) * RAD_PER_TICK for tick in (low, high))
        exported_lo = _number(limits[name].get("lower"), f"{name}.lower")
        exported_hi = _number(limits[name].get("upper"), f"{name}.upper")
        padding = 8 * RAD_PER_TICK if name == "gripper" else 0
        if abs(exported_lo - (measured_lo - padding)) > EPS or abs(exported_hi - (measured_hi + padding)) > EPS:
            raise CalibrationError(f"{name}: joint_limits disagree with measured tick ranges")
        canonical_lo = _number(canonical[name].get("lower"), f"canonical.{name}.lower")
        canonical_hi = _number(canonical[name].get("upper"), f"canonical.{name}.upper")
        # The ROS gripper limits include tolerance beyond the physical endpoints.
        # Command conversion uses the intersection, never that extra travel.
        lower = max(measured_lo, exported_lo, canonical_lo)
        upper = min(measured_hi, exported_hi, canonical_hi)
        if lower >= upper:
            raise CalibrationError(f"{name}: joint limits have no usable intersection")
        joint = JointCalibration(name, expected_id, offset, low, high, center, direction, lower, upper)
        joint.radians_to_ticks(0.0)
        joints.append(joint)
    calibration = Calibration(tuple(joints), source)
    calibration.gripper_tick(0)
    calibration.gripper_tick(1)
    return calibration
