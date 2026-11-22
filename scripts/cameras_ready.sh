#!/usr/bin/env bash
# Get the three cameras ready for the Station. Run it with the Station STOPPED, before `pixi run up`,
# after a reboot, a replug, or whenever the Station was restarted:
#
#   scripts/cameras_ready.sh [exposure]      exposure of the top camera in 100 us units (default 60 = 6 ms)
#
# 1. Hands every camera back to the kernel's uvcvideo driver. The Station takes them over through
#    libusb and does not give them back; a camera left like that refuses 1280x720 the next time
#    (libuvc error -51) and the Station silently falls back to 640x480, which is a different crop of
#    the sensor: the wrist camera's calibration (data/camera_wrist.json, 853x480) is then wrong.
# 2. Gives the UGREEN top camera a short fixed exposure. On auto exposure the mat's white corner
#    tags are washed out and tools/calibrate_top.py cannot decode them. The setting lasts until the
#    camera is unplugged.
set -euo pipefail
cd "$(dirname "$0")/.."
if pgrep -x station >/dev/null; then echo "The Station is running: stop it first (pkill -x station)."; exit 1; fi
uv run -q --with pyusb python - <<'PY'
import usb.core
for vid, pid, name in ((0x0C45, 0x2283, "UGREEN (top)"), (0x2C7F, 0x4A60, "KD-USB")):
    for dev in usb.core.find(find_all=True, idVendor=vid, idProduct=pid):
        for interface in (0, 1):
            if not dev.is_kernel_driver_active(interface):
                dev.attach_kernel_driver(interface)
        print(f"{name} on bus {dev.bus} device {dev.address}: kernel driver attached")
PY
sleep 1
dev=$(ls /dev/v4l/by-id/usb-UGREEN_Camera_*-video-index0 2>/dev/null | head -1)
[ -n "$dev" ] || { echo "UGREEN camera not found: is it plugged in?"; exit 1; }
v4l2-ctl -d "$dev" -c auto_exposure=1 -c exposure_time_absolute="${1:-60}" -c exposure_dynamic_framerate=0
v4l2-ctl -d "$dev" -C auto_exposure,exposure_time_absolute
