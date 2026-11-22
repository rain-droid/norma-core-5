"""Camera calibration.

Lens (intrinsics), one camera at a time, with the chessboard:

    uv run python scripts/calibrate_camera.py detect --cam top
        # prints the board's inner-corner count, if you are not sure

    uv run python scripts/calibrate_camera.py intrinsics --cam top --cols 8 --rows 6 --square-mm 25
        # open http://127.0.0.1:8790 for the live view, then move the board around slowly

    uv run python scripts/calibrate_camera.py intrinsics --cam top --cols 8 --rows 6 --square-mm 25 \\
        --from-dir logs/calib/top/20261121-153000
        # redo the maths on the frames saved by an earlier run, without the camera

`--cols`/`--rows` count INNER corners (where four squares meet): a board with 9 x 7 squares has
8 x 6. Measure the square size with a ruler over several squares and divide, the result is
only as good as this number.

A frame is captured automatically when the board is fully visible, held still, and placed
differently enough from every frame captured so far. Cover the whole image: the corners, the
edges, near and far, and tilted 20-40 degrees left/right/up/down. The result goes into
`data/camera_<cam>.json` (the lens part only; where the camera sits is calibrated separately).

Where the camera sits (extrinsics). Needs the daemon running (`uv run elrobot up`):

    uv run python scripts/calibrate_camera.py extrinsics --cam top
        # fixed camera: AprilTags (tag25h9) on the table; you touch each tag centre with the
        # limp arm's fingertip. 6-12 points spread over the table.

    uv run python scripts/calibrate_camera.py extrinsics --cam wrist
        # wrist camera: the chessboard lies still on the table; you hold the limp arm in 15
        # clearly different poses that see the whole board (live view on :8790)

    uv run python scripts/calibrate_camera.py verify --cam top --touch
        # finds the tags with the fixed camera and taps each one
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

from elrobot import calib, vision
from elrobot.config import REPO_ROOT, Config

# ----------------------------------------------------------------------------- feedback


def say(text: str, voice: str | None = None) -> None:
    """Print, and speak on a Mac so you can watch the board instead of the terminal."""
    print(f"  {text}", flush=True)
    if shutil.which("say"):
        subprocess.Popen(["say", "-r", "200", *(["-v", voice] if voice else []), text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# Spoken with --guide during the wrist hand-eye run, one before each pose, in this order. The
# views that the first runs on this arm lacked come first: straight down, from the robot's left,
# close up. Lay the board about 19 cm in front of the base so the arm can get above and beside it.
GUIDE_VOICE = "Anna"
GUIDE = [
    "Senkrecht von oben aufs Brett, Greifer gerade.",
    "Senkrecht von oben, Greifer eine Vierteldrehung nach links gedreht.",
    "Senkrecht von oben, Greifer eine Vierteldrehung nach rechts gedreht.",
    "Steil von oben, etwas nach vorne gekippt.",
    "Steil von oben, etwas zurück gekippt, Greifer verdreht.",
    "Arm nach links schwenken. Von links aufs Brett schauen, von hoch oben.",
    "Von links, mittlere Höhe.",
    "Von links, tief und flach.",
    "Von links, mittlere Höhe, Greifer verdreht.",
    "Von links, näher am Roboter.",
    "Nah ran, von oben. Das Brett soll das Bild fast füllen.",
    "Nah ran, schräg von links.",
    "Nah ran, schräg von rechts.",
    "Arm nach rechts schwenken. Von rechts aufs Brett schauen, von hoch oben.",
    "Von rechts, tief und flach.",
    "Von rechts, mittlere Höhe, Greifer verdreht.",
    "Von der Roboterseite, flach nach vorne schauen.",
    "Von der Roboterseite, flach, Greifer nach links verdreht.",
    "Von der Roboterseite, flach, Greifer nach rechts verdreht.",
    "Zum Schluss: weit weg und hoch, das Brett klein im Bild.",
]


def guide(n: int, count: int, on: bool) -> None:
    """Say which pose comes next."""
    if on and n < count:
        say(f"Pose {n + 1}. {GUIDE[n % len(GUIDE)]}", GUIDE_VOICE)


class Preview:
    """Tiny MJPEG server so the live detection can be watched in a browser (OpenCV here is headless)."""

    def __init__(self, port: int):
        self.jpeg: bytes | None = None
        self.cond = threading.Condition()
        preview = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):  # keep the terminal clean
                pass

            def do_GET(self):
                if self.path == "/":
                    body = (b"<html><body style='margin:0;background:#111'>"
                            b"<img src='/stream' style='width:100%;height:auto'></body></html>")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path != "/stream":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with preview.cond:
                            preview.cond.wait(timeout=2.0)
                            jpeg = preview.jpeg
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def show(self, img: np.ndarray) -> None:
        with self.cond:
            self.jpeg = vision.encode(img, 80)
            self.cond.notify_all()

    def close(self) -> None:
        self.server.shutdown()


def banner(img: np.ndarray, lines: list[str]) -> np.ndarray:
    out = img.copy()
    for i, text in enumerate(lines):
        y = 24 + 22 * i
        cv2.putText(out, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return out


# ----------------------------------------------------------------------------- camera access


async def open_link(cfg: Config, cam: str):
    from elrobot.link import StationLink

    link = StationLink(cfg.station_host, cfg.bus_serial, cfg.cameras)
    await link.start()
    t0 = time.monotonic()
    while cam not in link.frames:
        if time.monotonic() - t0 > 10:
            await link.stop()
            raise SystemExit(f"no frames from camera {cam!r} after 10 s (seen: {sorted(link.frames) or 'none'}). "
                             "Is the Station running, and does config.yaml name this camera?")
        await asyncio.sleep(0.1)
    return link


async def next_frame(link, cam: str, last_seq: int) -> tuple[np.ndarray, int]:
    while True:
        fr = link.frames.get(cam)
        if fr is not None and fr.seq != last_seq:
            return vision.decode(fr.jpeg), fr.seq
        await asyncio.sleep(0.01)


# ----------------------------------------------------------------------------- detect


async def cmd_detect(cfg: Config, args) -> int:
    link = await open_link(cfg, args.cam)
    try:
        for attempt in range(10):
            img, _ = await next_frame(link, args.cam, -1)
            size = calib.detect_pattern_size(img)
            if size:
                print(f"chessboard found: --cols {size[0]} --rows {size[1]}  (inner corners; image {img.shape[1]}x{img.shape[0]})")
                return 0
            print("  no chessboard in view yet, hold it flat and fully visible ...")
            await asyncio.sleep(0.5)
        print("no chessboard found")
        return 1
    finally:
        await link.stop()


# ----------------------------------------------------------------------------- intrinsics


def report_and_save(cfg: Config, args, res: calib.IntrinsicsResult, session: Path | None) -> None:
    print()
    print(f"camera {args.cam}: {res.width}x{res.height}, {len(res.used)} views used")
    print(f"  fx={res.fx:.1f} fy={res.fy:.1f} cx={res.cx:.1f} cy={res.cy:.1f}  k1={res.dist[0]:+.4f} k2={res.dist[1]:+.4f}")
    print(f"  field of view {res.hfov_deg:.1f} x {res.vfov_deg:.1f} deg")
    print(f"  reprojection error {res.rms_px:.3f} px (good: < 0.5, usable: < 1.0)")
    worst = sorted(zip(res.per_view_px, res.used), reverse=True)[:3]
    print("  worst views: " + ", ".join(f"#{i} {e:.2f}px" for e, i in worst))
    if abs(res.fx - res.fy) / res.fx > 0.03:
        print("  warning: fx and fy differ by more than 3%; check the square size and that the board is flat")
    if abs(res.cx - res.width / 2) > 0.1 * res.width or abs(res.cy - res.height / 2) > 0.1 * res.height:
        print("  warning: the image centre is far off; capture more views near the image edges and corners")
    if res.rms_px > 1.0:
        print("  warning: error above 1 px, NOT saving. Usual causes: wrong --cols/--rows/--square-mm, a bent board, blur.")
        return

    path = cfg.data_dir / f"camera_{args.cam}.json"
    model = vision.CameraModel.load(path)
    model.width, model.height = res.width, res.height
    model.fx, model.fy, model.cx, model.cy = res.fx, res.fy, res.cx, res.cy
    model.k1, model.k2 = float(res.dist[0]), float(res.dist[1])
    model.rms_px = round(res.rms_px, 4)
    model.save(path)
    print(f"  saved lens model to {path.relative_to(REPO_ROOT)} (camera mounting not changed)")
    if session is not None:
        (session / "result.json").write_text(json.dumps({
            "camera": args.cam, "width": res.width, "height": res.height,
            "board": {"cols": args.cols, "rows": args.rows, "square_mm": args.square_mm},
            "K": res.K.tolist(), "dist": res.dist.tolist(), "rms_px": res.rms_px,
            "per_view_px": res.per_view_px, "used": res.used,
        }, indent=2))


def intrinsics_from_dir(cfg: Config, args, board: calib.Chessboard) -> int:
    folder = Path(args.from_dir)
    files = sorted(folder.glob("view_*.jpg"))
    if not files:
        print(f"no view_*.jpg in {folder}")
        return 1
    corners, size = [], None
    for f in files:
        img = cv2.imread(str(f))
        size = size or (img.shape[1], img.shape[0])
        c = calib.find_corners(img, board.cols, board.rows)
        print(f"  {f.name}: {'ok' if c is not None else 'board not found'}")
        if c is not None:
            corners.append(c)
    res = calib.calibrate_intrinsics(corners, board, *size)
    report_and_save(cfg, args, res, folder)
    return 0


async def intrinsics_live(cfg: Config, args, board: calib.Chessboard) -> int:
    session = REPO_ROOT / "logs/calib" / args.cam / datetime.now().strftime("%Y%m%d-%H%M%S")
    session.mkdir(parents=True, exist_ok=True)
    preview = Preview(args.port)
    link = await open_link(cfg, args.cam)
    print(f"live view: http://127.0.0.1:{args.port}   frames saved to {session.relative_to(REPO_ROOT)}")
    say(f"Calibrating the {args.cam} camera. Show the whole chessboard and hold it still.")

    corners_kept: list[np.ndarray] = []
    sigs: list[np.ndarray] = []
    prev: np.ndarray | None = None
    still_since: float | None = None
    seq = -1
    size = None
    last_hint = 0.0
    size_checked = False
    try:
        while len(corners_kept) < args.count:
            img, seq = await next_frame(link, args.cam, seq)
            h, w = img.shape[:2]
            if size is None:
                size = (w, h)
            elif size != (w, h):
                raise SystemExit(f"camera resolution changed from {size} to {(w, h)}; restart the run")
            # corner search can take ~100 ms; keep the event loop (camera queue) responsive
            c = await asyncio.to_thread(calib.find_corners, img, board.cols, board.rows)
            grid = calib.coverage(corners_kept, w, h)
            status = "board not found"
            if c is not None and not size_checked:
                # the corner detector can also accept a wrong size (e.g. one row too many, using the
                # board's margin), so compare once against the size the image actually shows
                size_checked = True
                guess = await asyncio.to_thread(calib.detect_pattern_size, img)
                if guess and guess != (board.cols, board.rows):
                    say(f"Warning: the board looks like {guess[0]} by {guess[1]} inner corners, "
                        f"not {board.cols} by {board.rows}. Check --cols and --rows.")
            if c is not None:
                moved = float(np.max(np.linalg.norm(c - prev, axis=2))) if prev is not None and prev.shape == c.shape else 1e9
                prev = c
                now = time.monotonic()
                if moved > args.still_px:
                    still_since = None
                    status = "hold still"
                else:
                    still_since = still_since or now
                    sig = calib.view_signature(c, board.cols, board.rows, w, h)
                    diff = min((float(np.linalg.norm(sig - s)) for s in sigs), default=1e9)
                    if diff < args.min_diff:
                        status = "already have this view: move or tilt the board"
                        if now - last_hint > 6:
                            say("Move the board somewhere new, or tilt it.")
                            last_hint = now
                    elif now - still_since >= args.hold_s:
                        n = len(corners_kept)
                        cv2.imwrite(str(session / f"view_{n:02d}.jpg"), img)
                        corners_kept.append(c)
                        sigs.append(sig)
                        still_since = None
                        status = "captured"
                        say(f"{n + 1}")
                        last_hint = now
                    else:
                        status = "holding ..."
            else:
                prev = None
                still_since = None
            shown = calib.draw_coverage(calib.draw_corners(img, c, board.cols, board.rows), grid)
            preview.show(banner(shown, [
                f"{args.cam}: {len(corners_kept)}/{args.count} views   cells covered {int(grid.sum())}/{grid.size}",
                status,
            ]))
        grid = calib.coverage(corners_kept, *size)
        if grid.sum() < grid.size - 2:
            print(f"  note: only {int(grid.sum())}/{grid.size} image cells were covered; the edges may be less accurate")
        say("Got all views. Computing.")
        res = calib.calibrate_intrinsics(corners_kept, board, *size)
        report_and_save(cfg, args, res, session)
        return 0
    finally:
        await link.stop()
        preview.close()


async def cmd_intrinsics(cfg: Config, args) -> int:
    board = calib.Chessboard(cols=args.cols, rows=args.rows, square_m=args.square_mm / 1000.0)
    if args.from_dir:
        return intrinsics_from_dir(cfg, args, board)
    return await intrinsics_live(cfg, args, board)


# ----------------------------------------------------------------------------- daemon access
#
# Everything that needs the arm goes through the running daemon (`uv run elrobot up`), which owns
# the robot, so this script never fights it for the servos.


class Daemon:
    def __init__(self, cfg: Config):
        import httpx

        self.http = httpx.Client(base_url=f"http://{cfg.host}:{cfg.port}", timeout=60.0)
        try:
            self.http.get("/api/meta").raise_for_status()
        except Exception as e:
            raise SystemExit(f"daemon not reachable at {self.http.base_url} ({e}). Start it with `uv run elrobot up`.")

    def state(self) -> dict:
        return self.http.get("/api/state").json()

    def frame(self, cam: str) -> np.ndarray:
        r = self.http.get(f"/api/camera/{cam}/frame.jpg")
        if r.status_code != 200:
            raise SystemExit(f"no frame from camera {cam!r} (HTTP {r.status_code})")
        return vision.decode(r.content)

    def tool(self, name: str, **args) -> dict:
        res = self.http.post(f"/api/tools/{name}", json={"args": args, "source": "calibration"}).json()
        if not res.get("ok"):
            raise SystemExit(f"{name} failed: {res.get('error') or res.get('summary')}")
        return res

    def torque(self, enable: bool) -> None:
        res = self.http.post("/api/torque", json={"enable": enable}).json()
        if not res.get("ok"):
            raise SystemExit(f"torque {'on' if enable else 'off'} failed: {res.get('error')}")


def confirm(prompt: str) -> bool:
    """Wait for Enter. False when nobody can press it (started without a terminal)."""
    try:
        input(prompt)
        return True
    except EOFError:
        print()
        return False


def tool_pose(state: dict) -> np.ndarray:
    """T_base_tool (TCP) from a daemon state message."""
    from scipy.spatial.transform import Rotation

    ee = state.get("ee")
    if not ee:
        raise SystemExit("daemon reports no tool pose (is the arm calibrated and connected?)")
    return calib.make_T(Rotation.from_quat(ee["quat"]).as_matrix(), ee["pos"])  # quat is [x, y, z, w]


def still_pose(daemon: Daemon, samples: int = 8, dt: float = 0.06) -> tuple[np.ndarray, float, float]:
    """Average tool pose over ~0.5 s, plus how much it moved meanwhile (mm, degrees)."""
    poses = []
    for _ in range(samples):
        poses.append(tool_pose(daemon.state()))
        time.sleep(dt)
    pos = np.array([T[:3, 3] for T in poses])
    move_mm = float(np.max(np.linalg.norm(pos - pos.mean(axis=0), axis=1)) * 1000)
    move_deg = max(calib.rotation_deg(poses[0][:3, :3], T[:3, :3]) for T in poses)
    T = poses[len(poses) // 2].copy()
    T[:3, 3] = pos.mean(axis=0)
    return T, move_mm, move_deg


def load_lens(cfg: Config, cam: str) -> vision.CameraModel:
    model = vision.CameraModel.load(cfg.data_dir / f"camera_{cam}.json")
    if model.rms_px is None:
        raise SystemExit(f"camera {cam!r} has no lens calibration yet; run `intrinsics --cam {cam}` first")
    return model


def tag_pixels(daemon: Daemon, cam: str, model: vision.CameraModel, detector, frames: int = 10):
    """Tag centres averaged over several frames: id -> (u, v), plus the lens model at this resolution."""
    seen: dict[int, list[np.ndarray]] = {}
    scaled = model
    for _ in range(frames):
        img = daemon.frame(cam)
        scaled = model.scaled(img.shape[1], img.shape[0])
        for tid, uv in calib.detect_tags(img, detector).items():
            seen.setdefault(tid, []).append(uv)
        time.sleep(0.08)
    out = {}
    for tid, uvs in seen.items():
        if len(uvs) < frames // 2:
            continue  # flickering detection, not trustworthy
        uvs = np.array(uvs)
        if np.max(np.linalg.norm(uvs - uvs.mean(axis=0), axis=1)) > 2.0:
            print(f"  tag {tid}: position jitters by more than 2 px, skipped")
            continue
        out[tid] = uvs.mean(axis=0)
    return out, scaled, img


# ----------------------------------------------------------------------------- extrinsics: fixed camera


def cmd_extrinsics_fixed(cfg: Config, args) -> int:
    """Fixed (room) camera: touch tag centres with the fingertip; the camera sees the same tags."""
    from elrobot.kinematics import TCP_BACKOFF

    daemon = Daemon(cfg)
    model = load_lens(cfg, args.cam)
    detector = calib.tag_detector(args.family)
    session = REPO_ROOT / "logs/calib" / f"{args.cam}-extrinsics" / datetime.now().strftime("%Y%m%d-%H%M%S")
    session.mkdir(parents=True, exist_ok=True)
    points, pixels, labels = [], [], []
    scaled = model
    print(__doc_touch__)
    round_no = 0
    torque_off = False
    try:
        while True:
            round_no += 1
            input(f"\nRound {round_no}: lay the tags flat on the table, spread over the work area, and move the arm "
                  "OUT of the camera's view. Press Enter ... ")
            tags, scaled, img = tag_pixels(daemon, args.cam, model, detector)
            cv2.imwrite(str(session / f"round{round_no}.jpg"), img)
            if not tags:
                print("  no tags seen. Check the camera view and --family.")
                continue
            print(f"  camera sees tags {sorted(tags)}")
            if not torque_off:
                input("  Next the arm goes LIMP. Hold it now, then press Enter ... ")
                daemon.torque(False)
                torque_off = True
            for tid in sorted(tags):
                while True:
                    ans = input(f"  Put the fingertips (jaws closed) on the CENTRE of tag {tid}, gripper pointing straight "
                                "down; hold still and press Enter (s = skip) ... ").strip().lower()
                    if ans == "s":
                        break
                    T, move_mm, _ = still_pose(daemon)
                    if move_mm > 1.5:
                        print(f"    the arm moved {move_mm:.1f} mm while measuring, try again")
                        continue
                    approach = T[:3, 2]
                    if approach[2] > -0.85:
                        print(f"    warning: the gripper is {np.degrees(np.arccos(-approach[2])):.0f} deg off vertical")
                    tip = T[:3, 3] + TCP_BACKOFF * approach
                    points.append(tip)
                    pixels.append(tags[tid])
                    labels.append(f"r{round_no}/tag{tid}")
                    print(f"    tag {tid}: fingertip at x={tip[0]:+.4f} y={tip[1]:+.4f} z={tip[2]:+.4f} m, pixel ({tags[tid][0]:.1f}, {tags[tid][1]:.1f})")
                    break
            more = input(f"\n  {len(points)} points so far (6-12 is good). Another round with the tags moved elsewhere? [y/N] ")
            if more.strip().lower() != "y":
                break
    finally:
        if torque_off:
            if confirm("\nHold the arm: torque comes back on where it is now. Press Enter ... "):
                daemon.torque(True)
            else:
                print("no terminal to confirm on: the arm stays limp")

    if len(points) < 4:
        print(f"only {len(points)} points; need at least 4")
        return 1
    res = calib.solve_fixed_camera(np.array(points), np.array(pixels), scaled.K, scaled.dist)
    print("\nper point: reprojection px / table error mm")
    for lab, e_px, e_mm in zip(labels, res.reproj_px, res.plane_err_mm):
        print(f"  {lab:14s} {e_px:5.2f} px  {e_mm:5.1f} mm" + ("   <- check this one" if e_mm > 3 * np.median(res.plane_err_mm) + 2 else ""))
    T = res.T_base_cam
    _, z0, tilt = calib.fit_plane(np.array(points))
    print(f"\ncamera at x={T[0, 3]:+.3f} y={T[1, 3]:+.3f} z={T[2, 3]:+.3f} m, looking {np.degrees(np.arcsin(-T[2, 2])):.0f} deg down")
    print(f"reprojection {res.rms_px:.2f} px rms, table error median {np.median(res.plane_err_mm):.1f} mm, max {max(res.plane_err_mm):.1f} mm")
    print(f"touched points: table plane at z={z0 * 1000:+.1f} mm, tilted {tilt:.1f} deg (config table_z = {cfg.table_z * 1000:+.1f} mm)")
    if abs(z0 - cfg.table_z) > 0.005:
        print("  note: the measured table height differs from table_z by more than 5 mm")
    (session / "points.json").write_text(json.dumps({"labels": labels, "points": np.array(points).tolist(),
                                                     "pixels": np.array(pixels).tolist(), "T_base_cam": T.tolist()}, indent=2))
    if np.median(res.plane_err_mm) > 10:
        print("table error above 10 mm, NOT saving. Check the lens calibration and that the tags did not move.")
        return 1
    path = cfg.data_dir / f"camera_{args.cam}.json"
    model = vision.CameraModel.load(path)
    model.rvec_base_cam = cv2.Rodrigues(T[:3, :3])[0].reshape(3).tolist()
    model.t_base_cam = T[:3, 3].tolist()
    model.calibrated = True
    model.save(path)
    print(f"saved camera pose to {path.relative_to(REPO_ROOT)}")
    return 0


__doc_touch__ = """
Fixed-camera calibration by touching tags
  1. Tags lie flat on the table; the camera sees them while the arm is out of view.
  2. The arm goes limp; you put the closed fingertips on each tag's centre and press Enter.
  3. Optionally move the tags and repeat, to cover more of the table.
The points must not all lie on one line. Do not move a tag between seeing and touching it."""


# ----------------------------------------------------------------------------- extrinsics: wrist camera


def cmd_extrinsics_wrist(cfg: Config, args) -> int:
    """Camera on the gripper: the chessboard lies still on the table, the arm looks at it from many poses."""
    daemon = Daemon(cfg)
    model = load_lens(cfg, args.cam)
    board = calib.Chessboard(cols=args.cols, rows=args.rows, square_m=args.square_mm / 1000.0)
    if args.from_dir:
        kept = json.loads((Path(args.from_dir) / "samples.json").read_text())
        print(f"{len(kept)} samples from {args.from_dir}")
        return solve_wrist(cfg, args, model, board, [np.array(k["T_base_tool"]) for k in kept], [np.array(k["T_cam_board"]) for k in kept])
    session = REPO_ROOT / "logs/calib" / f"{args.cam}-extrinsics" / datetime.now().strftime("%Y%m%d-%H%M%S")
    session.mkdir(parents=True, exist_ok=True)
    preview = Preview(args.port)
    tools, boards, kept = [], [], []
    print(f"live view: http://127.0.0.1:{args.port}   samples saved to {session.relative_to(REPO_ROOT)}")
    print("Lay the chessboard flat on the table and do not touch it until the end.")
    if args.keep_torque:
        print("Torque stays on: move the arm with the web UI or tools; a sample is taken whenever it is still.")
    else:
        confirm("The arm goes LIMP now. Hold it, then press Enter ... ")
        daemon.torque(False)
    if args.guide:
        guide(0, args.count, True)
    else:
        say(f"Move the arm so the wrist camera sees the whole board, from {args.count} clearly different angles.")
    try:
        last_hint = 0.0
        while len(tools) < args.count:
            T, move_mm, move_deg = still_pose(daemon, samples=4, dt=0.05)
            img = daemon.frame(args.cam)
            lens = model.scaled(img.shape[1], img.shape[0])
            c = calib.find_corners(img, board.cols, board.rows)
            status = "board not found"
            if c is not None:
                if move_mm > 0.6 or move_deg > 0.3:
                    status = "hold still"
                else:
                    diff = min((calib.rotation_deg(T[:3, :3], P[:3, :3]) for P in tools), default=1e9)
                    if diff < args.min_deg:
                        status = f"only {diff:.0f} deg from an earlier view: tilt or turn the gripper more"
                        if time.monotonic() - last_hint > 8:
                            say("Stärker kippen oder drehen.", GUIDE_VOICE) if args.guide else say("Tilt or turn the gripper more.")
                            last_hint = time.monotonic()
                    else:
                        # the frame must show the pose we measured: check again after grabbing it
                        T2, move2, _ = still_pose(daemon, samples=3, dt=0.04)
                        if move2 > 0.6 or np.linalg.norm(T2[:3, 3] - T[:3, 3]) > 0.001:
                            status = "hold still"
                        else:
                            Tcb, err = calib.board_pose(c, board, lens.K, lens.dist)
                            if err > 1.0:
                                status = f"board fit poor ({err:.1f} px), blurred?"
                            else:
                                n = len(tools)
                                cv2.imwrite(str(session / f"sample_{n:02d}.jpg"), img)
                                tools.append(T)
                                boards.append(Tcb)
                                kept.append({"T_base_tool": T.tolist(), "T_cam_board": Tcb.tolist(), "board_px": err})
                                (session / "samples.json").write_text(json.dumps(kept, indent=2))  # as it goes: poses cannot be taken again from the pictures
                                status = "captured"
                                if args.guide:
                                    say("Aufgenommen.", GUIDE_VOICE) if n + 1 >= args.count else guide(n + 1, args.count, True)
                                else:
                                    say(f"{n + 1}")
                                last_hint = time.monotonic()
            shown = calib.draw_corners(img, c, board.cols, board.rows)
            preview.show(banner(shown, [f"{args.cam}: {len(tools)}/{args.count} poses", status]))
    finally:
        preview.close()
        if not args.keep_torque:
            if confirm("\nHold the arm: torque comes back on where it is now. Press Enter ... "):
                daemon.torque(True)
            else:
                print("no terminal to confirm on: the arm stays limp")

    (session / "samples.json").write_text(json.dumps(kept, indent=2))
    return solve_wrist(cfg, args, model, board, tools, boards)


def solve_wrist(cfg: Config, args, model, board: calib.Chessboard, tools: list, boards: list) -> int:
    boards, turned = calib.unflip_boards(tools, boards, board)
    if turned:
        print(f"samples {turned} saw the board the other way up (it looks the same turned half round): turned back")
    res = calib.solve_hand_eye(tools, boards)
    X = res.T_tool_cam
    print(f"\nwrist camera in the tool frame ({res.method}): t = [{X[0, 3] * 1000:+.1f}, {X[1, 3] * 1000:+.1f}, {X[2, 3] * 1000:+.1f}] mm")
    print(f"consistency of the board position across samples: mean {np.mean(res.spread_mm):.1f} mm, max {max(res.spread_mm):.1f} mm, "
          f"mean {np.mean(res.spread_deg):.2f} deg")
    bz = res.T_base_board[2, 3]
    print(f"board found at z={bz * 1000:+.1f} mm (table_z {cfg.table_z * 1000:+.1f} mm; the board's own thickness adds a few mm)")
    if np.mean(res.spread_mm) > args.max_spread_mm:
        print(f"spread above {args.max_spread_mm:g} mm, NOT saving. Usual causes: the board moved, too little rotation between poses, lens not calibrated.")
        return 1
    path = cfg.data_dir / f"camera_{args.cam}.json"
    model = vision.CameraModel.load(path)
    model.rvec_tool_cam = cv2.Rodrigues(X[:3, :3])[0].reshape(3).tolist()
    model.t_tool_cam = X[:3, 3].tolist()
    model.rvec_base_cam = model.t_base_cam = None
    model.calibrated = True
    model.save(path)
    print(f"saved camera mounting to {path.relative_to(REPO_ROOT)}")
    if args.cam == "wrist":  # the agent workspace reads its own copy (workspace/tools/util/capture.py)
        copy = cfg.workspace_dir / "robot_config" / "camera_wrist.json"
        model.save(copy)
        print(f"and to {copy.relative_to(REPO_ROOT)}")
    return 0


# ----------------------------------------------------------------------------- verify


def cmd_verify(cfg: Config, args) -> int:
    """Fixed camera: find the tags, say where they are on the table, optionally tap each one."""
    daemon = Daemon(cfg)
    model = load_lens(cfg, args.cam)
    if not model.fixed:
        raise SystemExit(f"camera {args.cam!r} has no fixed pose; run `extrinsics --cam {args.cam}` first")
    input("Move the arm out of the camera's view and press Enter ... ")
    tags, lens, _ = tag_pixels(daemon, args.cam, model, calib.tag_detector(args.family))
    if not tags:
        print("no tags seen")
        return 1
    for tid in sorted(tags):
        p = lens.pixel_to_plane(*tags[tid], None, cfg.table_z)
        print(f"  tag {tid}: x={p[0]:+.3f} y={p[1]:+.3f} m")
        if args.touch:
            input(f"    Enter = tap tag {tid} (watch how far from the centre the fingertip lands) ... ")
            print("    " + daemon.tool("touch", x=float(p[0]), y=float(p[1]))["summary"])
    if args.touch:
        daemon.tool("home")
    return 0


# ----------------------------------------------------------------------------- main


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("detect", help="print the chessboard's inner-corner count")
    d.add_argument("--cam", required=True, help="camera name from config.yaml, e.g. top or wrist")

    i = sub.add_parser("intrinsics", help="calibrate the lens with a chessboard")
    i.add_argument("--cam", required=True, help="camera name from config.yaml, e.g. top or wrist")
    i.add_argument("--cols", type=int, required=True, help="inner corners along the long side")
    i.add_argument("--rows", type=int, required=True, help="inner corners along the short side")
    i.add_argument("--square-mm", type=float, required=True, help="edge length of one square in mm")
    i.add_argument("--count", type=int, default=20, help="views to capture (default 20)")
    i.add_argument("--from-dir", help="recompute from view_*.jpg saved by an earlier run")
    i.add_argument("--port", type=int, default=8790, help="live view port (default 8790)")
    i.add_argument("--hold-s", type=float, default=0.6, help="how long the board must be still (s)")
    i.add_argument("--still-px", type=float, default=1.5, help="max corner motion between frames to count as still")
    i.add_argument("--min-diff", type=float, default=0.08, help="how different a new view must be (0..1 scale)")

    e = sub.add_parser("extrinsics", help="where the camera is: touch tags (fixed camera) or hand-eye (wrist camera)")
    e.add_argument("--cam", required=True, help="camera name from config.yaml, e.g. top or wrist")
    e.add_argument("--method", choices=["touch", "handeye"], help="default: handeye for 'wrist', touch otherwise")
    e.add_argument("--family", default="tag25h9", choices=sorted(calib.TAG_FAMILIES), help="tag family (touch)")
    e.add_argument("--cols", type=int, default=8, help="chessboard inner corners, long side (handeye)")
    e.add_argument("--rows", type=int, default=6, help="chessboard inner corners, short side (handeye)")
    e.add_argument("--square-mm", type=float, default=25.0, help="chessboard square size in mm (handeye)")
    e.add_argument("--count", type=int, default=15, help="poses to capture (handeye)")
    e.add_argument("--min-deg", type=float, default=8.0, help="min rotation between poses (handeye)")
    e.add_argument("--from-dir", help="solve again from the samples.json of an earlier run (handeye)")
    e.add_argument("--max-spread-mm", type=float, default=8.0, help="do not save above this mean spread of the board position (handeye)")
    e.add_argument("--guide", action="store_true", help="say which pose to take next, in German (handeye)")
    e.add_argument("--keep-torque", action="store_true", help="do not go limp; move the arm by other means (handeye)")
    e.add_argument("--port", type=int, default=8790, help="live view port (handeye)")

    v = sub.add_parser("verify", help="fixed camera: locate the tags on the table and optionally tap them")
    v.add_argument("--cam", default="top")
    v.add_argument("--family", default="tag25h9", choices=sorted(calib.TAG_FAMILIES))
    v.add_argument("--touch", action="store_true", help="tap each tag with the fingertip")

    args = ap.parse_args()
    logging.basicConfig(level=logging.WARNING)
    cfg = Config.load()
    if args.cmd == "detect":
        return asyncio.run(cmd_detect(cfg, args))
    if args.cmd == "intrinsics":
        return asyncio.run(cmd_intrinsics(cfg, args))
    if args.cmd == "extrinsics":
        method = args.method or ("handeye" if args.cam == "wrist" else "touch")
        return cmd_extrinsics_wrist(cfg, args) if method == "handeye" else cmd_extrinsics_fixed(cfg, args)
    return cmd_verify(cfg, args)


if __name__ == "__main__":
    sys.exit(main())
