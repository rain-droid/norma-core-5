"""The simulator tests run with safe mode off unless a test switches it on: the workspace copies
they use carry robot_config/motion.json, where it is on for the real arm."""

import os

os.environ.setdefault("ELROBOT_SAFE_MODE", "0")
