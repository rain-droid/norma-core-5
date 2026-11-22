#!/usr/bin/env bash
# Start the second arm's daemon: the SO-101 with the parallel gripper, next to the ElRobot.
#
#   scripts/second_arm_up.sh            real arm, on http://127.0.0.1:8770
#   scripts/second_arm_up.sh --sim      simulated, for trying its tools without hardware
#
# It is the `autopilot` branch of this repository (its SO-101 profile: URDF, servo calibration, IK),
# checked out once into ../norma-core-5-so101 and run with this repository's Python environment.
# It needs the Station already running (`pixi run up` here starts it) and finds its own servo bus by
# serial number (5B91029197 in that checkout's config.yaml). Port 8770: 8766 is the SAM 3 service.
# The workspace reaches it through tools/util/second_arm.py (ELROBOT_SECOND_ARM overrides the address).
set -euo pipefail
cd "$(dirname "$0")/.."
here=$PWD
other=$here/../norma-core-5-so101
[ -d "$other" ] || { git fetch -q origin autopilot && git worktree add -q --detach "$other" origin/autopilot; }
cd "$other"
PYTHONPATH=$other/src ELROBOT_ROBOT=so101 exec "$here/.pixi/envs/default/bin/python" -c \
  "import sys; sys.argv=['elrobot','daemon','--no-voice','--port','8770',*sys.argv[1:]]; from elrobot.cli import main; main()" "$@"
