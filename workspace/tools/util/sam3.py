"""Ask SAM 3 what is in a picture: every instance of a thing named in words, each with its outline.

SAM 3 runs as its own local process (util/sam3_server.py: it needs a GPU, its own libraries and
15 s to load), started from this directory and left running:

    uv run tools/util/sam3_server.py

    from util import sam3
    if sam3.available():
        for one in sam3.segment(img, ["object"]):       # or "blue cube", "roll of tape", ...
            one["polygon"], one["score"], one["prompt"]

About 0.3 s a prompt once warm, on this machine's GPU. Too slow for the tracker, which looks ten
times a second and keeps its colour segmentation; find_objects uses this when it is running.
"""

import base64
import json
import os
import urllib.error
import urllib.request

import cv2
import numpy as np

from util.arm import RobotError

URL = os.environ.get("ELROBOT_SAM3", "http://127.0.0.1:8766")


def available() -> bool:
    try:
        with urllib.request.urlopen(URL + "/health", timeout=0.5) as r:
            return bool(json.loads(r.read()).get("ok"))
    except (OSError, ValueError):
        return False


def segment(img: np.ndarray, prompts: list[str], threshold: float = 0.4) -> list[dict]:
    """Every instance of each prompt in a BGR image: `prompt`, `score` (0..1), `box` (x1, y1, x2,
    y2), `polygon` (the outline, pixels of `img`) and `area_px`. An empty list means nothing
    matched; RobotError means the service is not running or failed."""
    ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    body = json.dumps({"image_b64": base64.b64encode(jpg.tobytes()).decode(), "prompts": list(prompts), "threshold": threshold}).encode()
    req = urllib.request.Request(URL + "/segment", data=body, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=120) as r:
            return json.loads(r.read())["instances"]
    except urllib.error.HTTPError as e:
        raise RobotError(f"SAM 3 failed: {e.read().decode(errors='replace')[:300]}") from None
    except OSError as e:
        raise RobotError(f"SAM 3 is not running at {URL} ({e}). Start it from the workspace directory: `uv run tools/util/sam3_server.py`") from None
