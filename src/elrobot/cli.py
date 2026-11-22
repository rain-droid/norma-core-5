"""`elrobot` command: start and inspect the stack.

    elrobot up            Station + daemon (the normal way to run everything)
    elrobot up --sim      daemon only, with simulated servos (no hardware)
    elrobot daemon        just the daemon (expects a Station already running)
    elrobot station       just the NormaCore Station
    elrobot call <tool> [json-args]     call a tool on the running daemon, e.g. `elrobot call home`
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import socket
import subprocess
import sys
import time

from .config import REPO_ROOT, Config

STATION_BIN = REPO_ROOT / "bin" / "station"
STATION_DIR = REPO_ROOT / "station"


def _port_open(hostport: str) -> bool:
    host, port = hostport.rsplit(":", 1)
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex((host, int(port))) == 0


def start_station(cfg: Config) -> subprocess.Popen | None:
    """Launch the Station unless one is already listening. It must run from `station/`:
    its config and its calibration data are both relative to the working directory."""
    if _port_open(cfg.station_host):
        print(f"Station already running at {cfg.station_host}")
        return None
    if not STATION_BIN.exists():
        sys.exit(f"Station binary not found at {STATION_BIN}. See README: build it from norma-core and copy it there.")
    (REPO_ROOT / "logs").mkdir(exist_ok=True)
    logf = open(REPO_ROOT / "logs" / "station.log", "w")
    proc = subprocess.Popen(
        [str(STATION_BIN), "--tcp", cfg.station_host, "--web", "127.0.0.1:8889", "--max-queue-disk-size", "512M"],
        cwd=STATION_DIR, stdout=logf, stderr=subprocess.STDOUT,
    )  # fmt: skip
    for _ in range(50):
        if _port_open(cfg.station_host):
            print(f"Station started (pid {proc.pid}), log: logs/station.log, viewer: http://127.0.0.1:8889")
            return proc
        if proc.poll() is not None:
            sys.exit("Station exited at startup; see logs/station.log")
        time.sleep(0.1)
    sys.exit("Station did not start listening; see logs/station.log")


def run_daemon(cfg: Config) -> None:
    import uvicorn

    from .daemon import create_app

    print(f"daemon on http://{cfg.host}:{cfg.port}  ({'SIMULATED arm' if cfg.sim else 'real arm via Station ' + cfg.station_host})")
    uvicorn.run(create_app(cfg), host=cfg.host, port=cfg.port, log_level="warning")


def call(cfg: Config, tool: str, args_json: str | None) -> int:
    import base64

    import httpx

    args = json.loads(args_json) if args_json else {}
    try:
        r = httpx.post(f"http://{cfg.host}:{cfg.port}/api/tools/{tool}", json={"args": args, "source": "cli"}, timeout=180)
    except httpx.ConnectError:
        print("daemon is not running (start it with `elrobot up`)")
        return 2
    res = r.json()
    print(("ok: " if res.get("ok") else "FAILED: ") + (res.get("summary") or res.get("error") or ""))
    if res.get("data"):
        print(json.dumps(res["data"], indent=1))
    for i, img in enumerate(res.get("images") or []):
        out = REPO_ROOT / "logs" / f"call_{tool}_{i}.jpg"
        out.parent.mkdir(exist_ok=True)
        out.write_bytes(base64.b64decode(img["b64"]))
        print(f"image: {out}")
    return 0 if res.get("ok") else 1


def main() -> None:
    ap = argparse.ArgumentParser(prog="elrobot", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("up", "daemon"):
        p = sub.add_parser(name)
        p.add_argument("--sim", action="store_true", help="simulated servos, no hardware")
        p.add_argument("--no-voice", action="store_true", help="do not start the conversational agent")
        p.add_argument("--port", type=int)
        p.add_argument("-v", "--verbose", action="store_true")
    sub.add_parser("station")
    p = sub.add_parser("auto", help="give the autopilot an instruction, e.g. elrobot auto \"throw me the blue cube\"")
    p.add_argument("prompt", nargs="+")
    p = sub.add_parser("call")
    p.add_argument("tool")
    p.add_argument("args", nargs="?")
    a = ap.parse_args()

    cfg = Config.load()
    if a.cmd == "auto":
        import httpx

        base = f"http://{cfg.host}:{cfg.port}/api/autopilot"
        text = " ".join(a.prompt)
        print(httpx.post(base + "/stop" if text == "stop" else base, json={"prompt": text}, timeout=30).json())
        return
    if a.cmd == "call":
        sys.exit(call(cfg, a.tool, a.args))
    if a.cmd == "station":
        proc = start_station(cfg)
        if proc:
            try:
                proc.wait()
            except KeyboardInterrupt:
                proc.send_signal(signal.SIGINT)
        return

    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    logging.getLogger("elrobot.station").setLevel(logging.CRITICAL)  # upstream client logs every reconnect attempt
    if a.sim:
        cfg.sim = True
    if a.no_voice:
        cfg.voice = False
    if a.port:
        cfg.port = a.port
    station = start_station(cfg) if (a.cmd == "up" and not cfg.sim) else None
    try:
        run_daemon(cfg)
    finally:
        if station is not None:
            station.send_signal(signal.SIGINT)
            try:
                station.wait(timeout=5)
            except subprocess.TimeoutExpired:
                station.kill()


if __name__ == "__main__":
    main()
