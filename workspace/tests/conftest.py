"""Tests of the workspace's own code that need no robot and no daemon: the robot side is faked.

    uv run --with pytest pytest -q tests        # from this directory
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
