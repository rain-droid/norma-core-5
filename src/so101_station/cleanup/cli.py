"""Start the cleanup harness or call its bounded tools."""
import argparse
import json
import sys
from pathlib import Path

import httpx
import uvicorn

from ..calibration import load_calibration, DEFAULT_PROFILE
from .backend import MockBackend, StationBackend, mock_workspace
from .geometry import ArmModel, CleanupError, TableMap
from .harness import Harness
from .service import create_app
from .session import SessionHarness, mock_session


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="command", required=True)
    serve = sub.add_parser("serve")
    serve.add_argument("--sim", action="store_true")
    serve.add_argument("--workspace", type=Path)
    serve.add_argument("--session", type=Path, help="Checkpointed dual-camera session config (no table map required)")
    serve.add_argument("--session-sim", action="store_true", help="Run checkpointed tools with synthetic cameras/physics")
    serve.add_argument("--profile", default=str(DEFAULT_PROFILE))
    serve.add_argument("--station", default="127.0.0.1:8888")
    serve.add_argument("--bus-serial")
    serve.add_argument("--camera-queue")
    serve.add_argument("--enable-motion", action="store_true")
    serve.add_argument("--port", type=int, default=8771)
    serve.add_argument("--out", type=Path, default=Path("logs/cleanup"))
    call = sub.add_parser("call")
    call.add_argument("tool")
    call.add_argument("args", nargs="?", default="{}")
    call.add_argument("--url", default="http://127.0.0.1:8771")
    check = sub.add_parser("check-workspace")
    check.add_argument("path", type=Path)
    args = ap.parse_args(argv)
    try:
        if args.command == "call":
            response = httpx.post(f"{args.url}/tools/{args.tool}", json={"args": json.loads(args.args)}, timeout=240)
            response.raise_for_status()
            result = response.json()
            def compact(value):
                if isinstance(value, dict):
                    return {k: compact(v) for k, v in value.items() if k != "images"}
                if isinstance(value, list):
                    return [compact(v) for v in value]
                return value
            print(json.dumps(compact(result), indent=2))
            return 0 if result.get("ok") else 1
        if args.command == "check-workspace":
            table = TableMap.load(args.path)
            print(json.dumps({"valid": True, "max_reprojection_error_m": table.max_error_m, "hardware_verified": False}))
            return 0
        model = ArmModel(load_calibration(args.profile))
        if args.session or args.session_sim:
            if args.session_sim:
                config, backend = mock_session(model), MockBackend(model)
            else:
                config = json.loads(args.session.read_text())
                backend = StationBackend(config.get("station", args.station), config.get("bus_serial", args.bus_serial),
                                         None, model, args.enable_motion)
            harness = SessionHarness(backend, model, config, args.out)
            uvicorn.run(create_app(harness), host="127.0.0.1", port=args.port, log_level="warning")
            return 0
        if args.sim:
            config = mock_workspace()
            backend = MockBackend(model)
        else:
            if not args.workspace:
                raise CleanupError("real mode requires a measured --workspace JSON file")
            config = json.loads(args.workspace.read_text())
            if config.get("sim_only") is not False:
                raise CleanupError("real workspace must explicitly set sim_only=false; synthetic calibration is refused")
            backend = StationBackend(args.station, args.bus_serial, args.camera_queue, model, args.enable_motion)
        harness = Harness(backend, model, TableMap(config), args.out)
        uvicorn.run(create_app(harness), host="127.0.0.1", port=args.port, log_level="warning")
        return 0
    except (CleanupError, ValueError, OSError, KeyError, httpx.HTTPError) as exc:
        print(json.dumps({"error": str(exc)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
