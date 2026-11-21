"""Hardware-free regression tests for the supplied SO-101 calibration."""

import contextlib
import io
import json
import math
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

from so101_station.calibration import DEFAULT_PROFILE, RAD_PER_TICK, CalibrationError, load_calibration
from so101_station.cli import main


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.cal = load_calibration()

    def test_measured_wrist_zero_is_not_the_range_midpoint(self):
        wrist = self.cal.joint("wrist_roll")
        self.assertEqual(wrist.motor_id, 5)
        self.assertEqual(wrist.radians_to_ticks(0), 2048)
        self.assertAlmostEqual(wrist.ticks_to_radians(2686), math.radians(56.07421875))
        self.assertLess(wrist.upper, 1.0)
        with self.assertRaises(CalibrationError):
            wrist.radians_to_ticks(math.radians(90))

    def test_arm_offsets_are_not_applied_twice(self):
        shoulder = self.cal.joint("shoulder_lift")
        self.assertEqual(shoulder.homing_offset, -573)
        self.assertEqual(shoulder.ticks_to_radians(2048), 0)
        self.assertEqual(shoulder.radians_to_ticks(0), 2048)
        self.assertAlmostEqual(shoulder.ticks_to_radians(2112), math.pi / 32)

    def test_gripper_open_closed_and_padding(self):
        grip = self.cal.joint("gripper")
        self.assertEqual(self.cal.gripper_tick(0), 2048)
        self.assertEqual(self.cal.gripper_tick(1), 3484)
        self.assertEqual(self.cal.gripper_tick(0.5), 2766)
        self.assertEqual(grip.ticks_to_radians(3484), 0)
        self.assertGreater(grip.ticks_to_radians(2048), 2)
        # Exported ROS limits allow eight extra ticks. Physical commands do not.
        for angle in (-RAD_PER_TICK, grip.upper + RAD_PER_TICK):
            with self.assertRaises(CalibrationError):
                grip.radians_to_ticks(angle)

    def test_integer_tick_round_trip_for_entire_measured_ranges(self):
        for joint in self.cal.joints:
            for tick in range(joint.range_min, joint.range_max + 1):
                self.assertEqual(joint.radians_to_ticks(joint.ticks_to_radians(tick)), tick)

    def test_preview_is_small_and_never_clamps_bad_input(self):
        joint = self.cal.joint("wrist_roll")
        self.assertEqual(joint.nudge(2048, 2), 2071)
        for current, delta in ((2686, 1), (2048, 3.1), (10, 0), (2048, float("nan"))):
            with self.assertRaises(CalibrationError):
                joint.nudge(current, delta)
        for value in (True, float("nan"), float("inf"), -0.1, 1.1):
            with self.assertRaises(CalibrationError):
                self.cal.gripper_tick(value)

    def changed_profile(self, filename, mutate):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        dest = Path(temp.name) / "profile"
        shutil.copytree(DEFAULT_PROFILE, dest)
        path = dest / filename
        data = json.loads(path.read_text()) if path.suffix == ".json" else yaml.safe_load(path.read_text())
        mutate(data)
        path.write_text(json.dumps(data) if path.suffix == ".json" else yaml.safe_dump(data))
        return dest

    def test_conflicting_export_rejected(self):
        mutations = [
            ("state.yaml", lambda d: d["joints"]["gripper"].update(id=8)),
            ("state.yaml", lambda d: d["joints"]["gripper"].update(direction=1)),
            ("control.yaml", lambda d: d["joints"]["wrist_roll"].update(homing_offset=0)),
            ("joint_limits.yaml", lambda d: d["joint_limits"]["wrist_roll"].update(upper=2.8)),
            ("calibration.json", lambda d: d["shoulder_pan"].update(range_min=3500)),
            ("calibration.json", lambda d: d["shoulder_pan"].update(drive_mode=1)),
            ("calibration.json", lambda d: d.pop("gripper")),
            ("canonical_joint_limits.yaml", lambda d: d["joint_limits"]["wrist_roll"].update(upper=float("nan"))),
        ]
        for filename, mutate in mutations:
            with self.subTest(filename=filename):
                with self.assertRaises(CalibrationError):
                    load_calibration(self.changed_profile(filename, mutate))

    def test_duplicate_and_wrong_type_fields_rejected(self):
        directory = self.changed_profile("state.yaml", lambda d: d["joints"]["shoulder_pan"].update(id=True))
        with self.assertRaises(CalibrationError):
            load_calibration(directory)
        (directory / "state.yaml").write_text("joints: {}\njoints: {}\n")
        with self.assertRaisesRegex(CalibrationError, "duplicate"):
            load_calibration(directory)

    def test_profile_and_previews_do_not_connect(self):
        for args in (["profile"], ["preview", "wrist_roll", "--from-tick", "2048", "--delta-deg", "2"],
                     ["gripper-preview", "--open", "1"]):
            with self.subTest(args=args), patch("so101_station.station._client", side_effect=AssertionError("connected")):
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    self.assertEqual(main(args), 0)
                result = json.loads(output.getvalue())
                self.assertFalse(result.get("motion_enabled", False))

    def test_invalid_cli_target_has_nonzero_exit_code(self):
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(["preview", "wrist_roll", "--from-tick", "2686", "--delta-deg", "2"]), 2)


if __name__ == "__main__":
    unittest.main()
