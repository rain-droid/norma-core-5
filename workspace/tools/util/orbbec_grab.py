"""Grab one colour picture and one depth picture from the Orbbec depth camera (DaBai DCW2).

    uv run --python 3.11 --with pyorbbecsdk==1.3.2 --with numpy --with opencv-python-headless tools/util/orbbec_grab.py OUT_DIR
    uv run --python 3.12 --with pyorbbecsdk2 --with numpy --with opencv-python-headless tools/util/orbbec_grab.py OUT_DIR

Two SDK generations exist and which one talks to this camera has to be tried: the second line
is the newer one. Writes OUT_DIR/color.jpg, depth.png (16 bit, millimetres), depth_view.jpg and
info.json (sizes, intrinsics, device). The camera must be free: the Station takes every UVC
camera it finds, so plug this one in after the Station has started, or stop the Station.
"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from pyorbbecsdk import Config, Context, OBFormat, OBSensorType, Pipeline

out = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
devices = Context().query_devices()
print("devices:", devices.get_count())
if devices.get_count() == 0:
    sys.exit("no Orbbec device found: is it plugged in (lsusb shows vendor 2bc5), and may this user open it (udev rule)?")
dev = devices.get_device_by_index(0)
di = dev.get_device_info()
info = {"name": di.get_name(), "serial": di.get_serial_number(), "pid": hex(di.get_pid()), "firmware": di.get_firmware_version()}
print(info)
pipe, cfg = Pipeline(dev), Config()
for sensor, key in ((OBSensorType.DEPTH_SENSOR, "depth"), (OBSensorType.COLOR_SENSOR, "color")):
    try:
        profiles = pipe.get_stream_profile_list(sensor)
        p = profiles.get_default_video_stream_profile()
        cfg.enable_stream(p)
        info[key] = {"width": p.get_width(), "height": p.get_height(), "fps": p.get_fps(), "format": str(p.get_format())}
    except Exception as e:  # a camera without that sensor, or one the Station holds
        info[key] = {"error": str(e)}
print(info)
pipe.start(cfg)
try:
    depth = color = None
    t0 = time.time()
    while time.time() - t0 < 8 and (depth is None or color is None):
        frames = pipe.wait_for_frames(1000)
        if frames is None:
            continue
        d, c = frames.get_depth_frame(), frames.get_color_frame()
        if d is not None and time.time() - t0 > 1.0:  # the first frames are dark or empty
            depth = np.frombuffer(d.get_data(), np.uint16).reshape(d.get_height(), d.get_width()).astype(np.float32) * d.get_depth_scale()
        if c is not None and time.time() - t0 > 1.0:
            raw = np.frombuffer(c.get_data(), np.uint8)
            fmt = c.get_format()
            if fmt == OBFormat.MJPG:
                color = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            elif fmt == OBFormat.RGB:
                color = cv2.cvtColor(raw.reshape(c.get_height(), c.get_width(), 3), cv2.COLOR_RGB2BGR)
            elif fmt == OBFormat.YUYV:
                color = cv2.cvtColor(raw.reshape(c.get_height(), c.get_width(), 2), cv2.COLOR_YUV2BGR_YUY2)
            else:
                info["color"]["unhandled_format"] = str(fmt)
    try:
        cp = pipe.get_camera_param()
        di_ = cp.depth_intrinsic
        info["depth_intrinsic"] = {"fx": di_.fx, "fy": di_.fy, "cx": di_.cx, "cy": di_.cy, "width": di_.width, "height": di_.height}
        ci_ = cp.rgb_intrinsic
        info["color_intrinsic"] = {"fx": ci_.fx, "fy": ci_.fy, "cx": ci_.cx, "cy": ci_.cy, "width": ci_.width, "height": ci_.height}
    except Exception as e:
        info["intrinsics_error"] = str(e)
finally:
    pipe.stop()
if depth is not None:
    cv2.imwrite(str(out / "depth.png"), np.clip(depth, 0, 65535).astype(np.uint16))
    ok = depth > 0
    info["depth_valid_share"] = round(float(ok.mean()), 3)
    info["depth_mm_range"] = [float(depth[ok].min()), float(np.median(depth[ok])), float(depth[ok].max())] if ok.any() else None
    view = cv2.applyColorMap(np.clip(depth / 1500 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    view[~ok] = 0
    cv2.imwrite(str(out / "depth_view.jpg"), view)
if color is not None:
    cv2.imwrite(str(out / "color.jpg"), color)
info["got"] = {"depth": depth is not None, "color": color is not None}
(out / "info.json").write_text(json.dumps(info, indent=1) + "\n")
print(json.dumps(info, indent=1))
