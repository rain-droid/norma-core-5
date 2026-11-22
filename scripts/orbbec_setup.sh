#!/usr/bin/env bash
# Build the Orbbec SDK's Python bindings for the depth camera (DaBai DCW2) into bin/orbbec/.
#
#   scripts/orbbec_setup.sh
#
# Why a build: this camera's depth sensor (USB 2bc5:06a0) speaks Orbbec's old OpenNI protocol.
# The current SDK on PyPI (pyorbbecsdk2) sees the device and matches nothing; the old SDK does read
# it, but its PyPI wheel for Linux (pyorbbecsdk 1.3.2) contains macOS libraries. So the bindings are
# built from Orbbec's v1 source, which carries the SDK's Linux libraries. Takes about a minute.
#
# The user must be allowed to open the camera. Once, as root:
#   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", MODE="0666"' > /etc/udev/rules.d/99-elrobot-orbbec.rules
#   udevadm control --reload && udevadm trigger --subsystem-match=usb --attr-match=idVendor=2bc5
#
# The kernel caps the memory of USB transfers made from user space at 16 MB, and the Station's three
# cameras use most of it: the depth stream then stops after three pictures. Raise it, once, as root:
#   echo 256 > /sys/module/usbcore/parameters/usbfs_memory_mb
#   echo 'w /sys/module/usbcore/parameters/usbfs_memory_mb - - - - 256' > /etc/tmpfiles.d/elrobot-usbfs.conf   # at every boot
set -euo pipefail
cd "$(dirname "$0")/.."
out=$PWD/bin/orbbec
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
git clone -q --depth 1 -b main https://github.com/orbbec/pyorbbecsdk.git "$work/src"
uv venv -q -p 3.12 "$work/venv"  # the same Python as the project: the module only loads in that version
uv pip install -q -p "$work/venv/bin/python" cmake 'pybind11<3'
"$work/venv/bin/cmake" -S "$work/src" -B "$work/build" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -Dpybind11_DIR="$("$work/venv/bin/python" -m pybind11 --cmakedir)" -DPython3_EXECUTABLE="$work/venv/bin/python" >/dev/null
"$work/venv/bin/cmake" --build "$work/build" -j "$(nproc)" >/dev/null
mkdir -p "$out"
cp "$work"/build/pyorbbecsdk.cpython-*.so "$out/"
cp -a "$work"/src/sdk/lib/linux_x64/*.so* "$out/"
cp "$work/src/LICENSE" "$out/LICENSE"
echo "built into $out; try: cd workspace && uv run python tools/depth_scan.py"
