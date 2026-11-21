"""Import shim for the vendored NormaCore Station client and generated protobuf readers.

The upstream code imports itself with absolute paths rooted at the norma-core repo
(`target.gen_python...`, `shared.gremlin_py...`, `software.station...`), so the vendored copy
under `vendor/norma_core` has to be on `sys.path`.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
VENDOR = REPO_ROOT / "vendor" / "norma_core"

if str(VENDOR) not in sys.path:
    sys.path.insert(0, str(VENDOR))

from software.station.shared.station_py import new_station_client, send_commands  # noqa: E402
from target.gen_python.protobuf.drivers.st3215 import st3215  # noqa: E402
from target.gen_python.protobuf.drivers.usbvideo import usbvideo  # noqa: E402
from target.gen_python.protobuf.station import commands, drivers, inference  # noqa: E402

__all__ = [
    "REPO_ROOT",
    "new_station_client",
    "send_commands",
    "st3215",
    "usbvideo",
    "commands",
    "drivers",
    "inference",
]
