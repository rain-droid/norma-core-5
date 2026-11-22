"""Save top-camera frames while the arm is throwing: throwcam.py OUTDIR."""
import json, sys, time, urllib.request, pathlib
out = pathlib.Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
for f in out.glob("*.jpg"): f.unlink()
get = lambda p: urllib.request.urlopen("http://127.0.0.1:8765" + p, timeout=2).read()
t_end, n, seen = time.time() + 90, 0, None
while time.time() < t_end:
    act = json.loads(get("/api/state")).get("activity")
    if act == "throwing" and seen is None: seen = time.time()
    if seen:
        (out / f"{n:03d}_{time.time()-seen:.2f}.jpg").write_bytes(get("/api/camera/top/frame.jpg")); n += 1
        if time.time() - seen > 2.2: break
    else: time.sleep(0.02)
