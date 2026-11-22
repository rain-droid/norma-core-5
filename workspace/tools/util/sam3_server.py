# /// script
# requires-python = ">=3.11,<3.13"
# dependencies = ["torch>=2.7", "torchvision", "transformers>=4.57", "pillow", "numpy", "opencv-python-headless", "accelerate"]
# ///
"""SAM 3 as a local service: instance segmentation by words, for the tools in this workspace.

    uv run tools/util/sam3_server.py            # from the workspace directory; first run downloads about 7 GB

SAM 3 (facebook/sam3 on Hugging Face, a gated model: the machine's Hugging Face login must have
been granted access) takes a picture and a short noun phrase ("cube", "roll of tape") and
returns a mask for every instance of it. It needs a GPU and about 15 s to load, which a tool
that runs once and exits cannot afford, so it lives here, in its own process with its own
libraries (the header above; uv installs them), and the tools ask it over local HTTP
(util/sam3.py). Nothing here knows about the robot.

POST /segment  {"image_path": "...jpg" | "image_b64": "<jpeg>", "prompts": ["cube", ...], "threshold": 0.5}
  -> {"instances": [{"prompt", "score", "box": [x1, y1, x2, y2], "polygon": [[u, v], ...], "area_px"}], "seconds"}
GET  /health   -> {"ok": true, "model": "facebook/sam3", "device": "cuda"}

One polygon per instance: the outline of its largest connected part, in pixels of the image as
sent. Listens on 127.0.0.1 only (port 8766, or ELROBOT_SAM3_PORT).
"""

import base64
import io
import json
import os
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import Sam3Model, Sam3Processor

MODEL = "facebook/sam3"
PORT = int(os.environ.get("ELROBOT_SAM3_PORT", "8766"))
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
DTYPE = torch.float16 if DEVICE == "cuda" else torch.float32  # half precision: fits beside other things on an 8 GB card

processor = Sam3Processor.from_pretrained(MODEL)
model = Sam3Model.from_pretrained(MODEL, torch_dtype=DTYPE).to(DEVICE).eval()


def segment(image: Image.Image, prompts: list[str], threshold: float) -> list[dict]:
    out = []
    for prompt in prompts:
        inputs = processor(images=image, text=prompt, return_tensors="pt").to(DEVICE)
        inputs = {k: (v.to(DTYPE) if torch.is_tensor(v) and v.is_floating_point() else v) for k, v in inputs.items()}
        with torch.no_grad():
            outputs = model(**inputs)
        res = processor.post_process_instance_segmentation(outputs, threshold=threshold, mask_threshold=0.5, target_sizes=inputs["original_sizes"].tolist())[0]
        for mask, box, score in zip(res["masks"], res["boxes"], res["scores"]):
            m = mask.cpu().numpy().astype(np.uint8)
            contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            c = max(contours, key=cv2.contourArea)
            out.append({
                "prompt": prompt, "score": round(float(score), 3), "box": [round(float(v), 1) for v in box.tolist()],
                "polygon": c.reshape(-1, 2).tolist(), "area_px": int(m.sum()),
            })  # fmt: skip
    return out


class Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, body: dict) -> None:
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self._send(200, {"ok": True, "model": MODEL, "device": DEVICE}) if self.path == "/health" else self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/segment":
            return self._send(404, {"error": "not found"})
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))))
            image = Image.open(req["image_path"] if req.get("image_path") else io.BytesIO(base64.b64decode(req["image_b64"]))).convert("RGB")
            t0 = time.monotonic()
            found = segment(image, [str(p) for p in req["prompts"]], float(req.get("threshold", 0.5)))
            self._send(200, {"instances": found, "seconds": round(time.monotonic() - t0, 2)})
        except Exception as e:  # the caller needs the reason, the server needs to stay up
            self._send(500, {"error": f"{type(e).__name__}: {e}"})

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    print(f"sam3 ready on http://127.0.0.1:{PORT} ({DEVICE}, {DTYPE})", flush=True)
    HTTPServer(("127.0.0.1", PORT), Handler).serve_forever()
