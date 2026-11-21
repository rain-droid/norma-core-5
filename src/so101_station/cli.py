"""Inspect SO-101 calibration, preview joint targets, and read Station state."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys

from .calibration import DEFAULT_PROFILE, JOINT_IDS, CalibrationError, load_calibration


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE), help="directory containing the imported calibration bundle")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("profile", help="validate and print calibration; no connection")
    preview = sub.add_parser("preview", help="calculate a bounded joint nudge; does not send commands")
    preview.add_argument("joint", choices=list(JOINT_IDS))
    preview.add_argument("--from-tick", type=int, required=True, help="current calibrated servo position register")
    preview.add_argument("--delta-deg", type=float, required=True, help="at most +/-3 degrees")
    gripper = sub.add_parser("gripper-preview", help="convert opening fraction to a tick target; no connection")
    gripper.add_argument("--open", type=float, required=True, help="0 closed, 1 open")
    for name in ("discover", "status"):
        command = sub.add_parser(name, help="read Station telemetry without commanding motors")
        command.add_argument("--host", default="127.0.0.1:8888")
        command.add_argument("--timeout", type=float, default=5.0)
        if name == "status":
            command.add_argument("--bus-serial", required=True)
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.WARNING)
    try:
        calibration = load_calibration(args.profile)
        code = 0
        if args.command == "profile":
            result = calibration.summary()
        elif args.command == "preview":
            joint = calibration.joint(args.joint)
            target = joint.nudge(args.from_tick, args.delta_deg)
            result = {"preview_only": True, "motion_enabled": False, "joint": joint.name,
                      "motor_id": joint.motor_id, "from_tick": args.from_tick, "target_tick": target,
                      "target_rad": joint.ticks_to_radians(target),
                      "calibration_verified_on_hardware": False}
        elif args.command == "gripper-preview":
            result = {"preview_only": True, "motion_enabled": False, "motor_id": 6,
                      "open": args.open, "target_tick": calibration.gripper_tick(args.open)}
        else:
            from .station import discover, inspect_state, read_state

            payload = asyncio.run(read_state(args.host, args.timeout))
            if args.command == "discover":
                result = discover(payload)
            else:
                result = inspect_state(calibration, payload, args.bus_serial)
                code = 0 if result["profile_matches_live_state"] else 1
        print(json.dumps(result, indent=2, allow_nan=False))
        return code
    except (CalibrationError, OSError, TimeoutError) as exc:
        print(json.dumps({"error": str(exc) or "timed out waiting for Station", "motion_enabled": False}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
