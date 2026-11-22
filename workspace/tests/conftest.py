"""Tests of the workspace's own code that need no robot and no daemon: the robot side is faked.

    uv run --with pytest pytest -q tests        # from this directory
"""

import os
import sys
from pathlib import Path

os.environ.setdefault("ELROBOT_SAFE_MODE", "0")  # these tests fake the robot side; safe mode has its own in the repository's tests

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
