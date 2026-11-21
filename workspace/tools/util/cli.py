"""Shell entry shared by every tool: `uv run tools/NAME.py '{"x": 0.1}'`.

A tool file is both the algorithm and its own command. Run from a shell, it does not execute
here: the arm is owned by one long-running process (the daemon, `uv run elrobot up`), so the
call is handed to it over local HTTP and the daemon imports this workspace's tools/NAME.py and
runs it. That is process coordination, not a tool server: there is no second implementation.
"""

import argparse
import inspect
import json
import os
import sys
import urllib.error
import urllib.request

DAEMON = os.environ.get("ELROBOT_DAEMON", "http://127.0.0.1:8765")


def request(tool: str, arguments: dict | None = None, timeout: float = 600.0) -> dict:
    """Run a workspace tool in the daemon and return its result dict."""
    body = json.dumps({"args": arguments or {}, "source": os.environ.get("ELROBOT_SOURCE", "agent")}).encode()
    req = urllib.request.Request(f"{DAEMON}/api/workspace/tools/{tool}", data=body, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return {"error": f"daemon answered HTTP {e.code}: {e.read().decode(errors='replace')[:300]}", "error_kind": "daemon"}
    except OSError as e:
        return {
            "error": f"robot daemon not reachable at {DAEMON} ({e}). Start it from the repo root: "
            "`uv run elrobot up --no-voice` (real arm) or `uv run elrobot up --sim --no-voice` (simulated).",
            "error_kind": "daemon",
        }


def run_cli(function) -> None:
    name = function.__name__
    parser = argparse.ArgumentParser(prog=f"tools/{name}.py", description=inspect.getdoc(sys.modules[function.__module__]))
    parser.add_argument("arguments", nargs="?", default="{}", help="JSON object of arguments, or - to read it from stdin")
    parser.add_argument("--schema", action="store_true", help="print this tool's signature and description")
    args = parser.parse_args()
    if args.schema:
        module = sys.modules[function.__module__]
        signature = inspect.signature(function)
        params = list(signature.parameters.values())[1:]  # drop ctx
        print(json.dumps({
            "tool": name,
            "signature": f"{name}({', '.join(str(p) for p in params)})",
            "moves_robot": bool(getattr(module, "MOTION", True)),
            "description": inspect.getdoc(module),
        }, indent=1))  # fmt: skip
        return
    try:
        arguments = json.loads(sys.stdin.read() if args.arguments == "-" else args.arguments)
    except json.JSONDecodeError as e:
        parser.error(f"arguments are not valid JSON: {e}")
    if not isinstance(arguments, dict):
        parser.error("arguments must be a JSON object")
    result = request(name, arguments)
    print(json.dumps(result, indent=1, ensure_ascii=False), flush=True)
    if "error" in result:
        raise SystemExit(1)
