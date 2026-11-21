"""Explicit, image-only target memory. Synchronous; never controls a robot.

Selection freshness belongs to the caller. This memory checks image ordering,
camera identity, local tracking and a monotonic selection lifetime, not depth,
world motion, object identity or grasp readiness.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
import time

import cv2
import numpy as np

from .geometry import CleanupError
from .tracking import FeatureTracker
from .visual import PatchTracker


@dataclass
class _Target:
    generation: int
    selected_at: float
    tracker_kind: str
    tracker: object
    accepted: dict
    result: dict
    history: dict = field(default_factory=dict)


class TargetMemory:
    """Remember at most 16 explicitly selected ROIs for 300 seconds each.

    Lost, expired and changed-view states latch until explicit reselection.
    Tracking never increments a generation or extends its lifetime. The optional
    ``clock`` is an injectable monotonic clock for deterministic offline tests.
    """

    LABELS = frozenset({"crumpled_paper", "snack_wrapper"})
    VALID = frozenset({"selected", "tracked", "shifted"})
    TTL_S = 300.0
    MAX_TARGETS = 16
    MAX_HISTORY = 64

    def __init__(self, *, clock=None):
        self._clock = clock or time.monotonic
        self._targets: dict[str, _Target] = {}

    @staticmethod
    def _id(value):
        if not isinstance(value, str) or not value.strip() or len(value) > 128:
            raise CleanupError("target_id must be a nonempty string of at most 128 characters")
        return value

    def _get(self, target_id):
        target_id = self._id(target_id)
        if target_id not in self._targets:
            raise CleanupError("unknown target; select an explicit image ROI first")
        return self._targets[target_id]

    @staticmethod
    def _observation_id(observation):
        ident = observation.get("observation_id") if isinstance(observation, dict) else None
        if not isinstance(ident, str) or not ident.strip():
            raise CleanupError("observation requires a nonempty observation_id")
        return ident

    @classmethod
    def _metadata(cls, observation, view):
        ident = cls._observation_id(observation)
        views = observation.get("views")
        if not isinstance(views, dict) or view not in views:
            raise CleanupError("selected camera view is missing from observation")
        data = views[view]
        if not isinstance(data, dict):
            raise CleanupError("camera view metadata must be an object")
        camera = data.get("camera_id")
        stamp = data.get("captured_ns")
        size = data.get("image_size")
        path = data.get("path")
        if not isinstance(camera, str) or not camera.strip():
            raise CleanupError("camera view requires a camera_id")
        if not isinstance(stamp, int) or isinstance(stamp, bool) or stamp < 0:
            raise CleanupError("camera captured_ns must be a nonnegative integer")
        if (not isinstance(size, (list, tuple)) or len(size) != 2
                or any(type(v) is not int or v <= 0 for v in size)):
            raise CleanupError("camera image_size must be [width, height] in positive integer pixels")
        if not isinstance(path, (str, Path)) or not str(path) or "\x00" in str(path) or "://" in str(path):
            raise CleanupError("camera evidence requires a local source image path")
        return {"observation_id": ident, "camera_id": camera, "captured_ns": stamp,
                "image_size": list(size), "source_image_path": str(Path(path).resolve())}

    @staticmethod
    def _image(metadata):
        image = cv2.imread(metadata["source_image_path"], cv2.IMREAD_COLOR)
        if image is None:
            raise CleanupError("source image cannot be decoded")
        if list(image.shape[1::-1]) != metadata["image_size"]:
            raise CleanupError("decoded image dimensions disagree with camera metadata")
        return image

    @staticmethod
    def _bbox(center, size, image_size):
        center = np.asarray(center, dtype=float)
        if center.shape != (2,) or not np.isfinite(center).all():
            raise CleanupError("tracker returned an invalid image center")
        w, h = size
        x, y = np.rint(center - [w / 2, h / 2]).astype(int)
        if x < 0 or y < 0 or x + w > image_size[0] or y + h > image_size[1]:
            raise CleanupError("translated target patch is outside the image")
        return [int(x), int(y), int(w), int(h)]

    @staticmethod
    def _latch(target, status, reason, metadata=None):
        result = deepcopy(target.result)
        if metadata is not None:
            result.update(metadata)
        result.update(status=status, reason=reason, normalized_correlation=None,
                      displacement_px=None, displacement_norm_px=None)
        result.pop("center_pixel", None)
        result.pop("patch_bbox", None)
        target.result = result
        return deepcopy(result)

    def _expire(self, target):
        if target.result["status"] in self.VALID and self._clock() - target.selected_at >= self.TTL_S:
            self._latch(target, "stale", "selection expired after 300 seconds; explicitly reselect the target")

    def select(self, target_id, label, view, observation, bbox, tracker="patch"):
        """Explicitly seed or reselect a target from an upstream-fresh observation."""
        target_id = self._id(target_id)
        if not isinstance(label, str) or label not in self.LABELS:
            raise CleanupError("target label must be crumpled_paper or snack_wrapper")
        if not isinstance(view, str) or not view.strip():
            raise CleanupError("target view must name an observation camera")
        if tracker not in {"patch", "features"}:
            raise CleanupError("tracker must be patch or features")
        old = self._targets.get(target_id)
        if old is None and len(self._targets) >= self.MAX_TARGETS:
            raise CleanupError("target memory is full (maximum 16 targets); reselect an existing target ID")
        metadata = self._metadata(observation, view)
        image = self._image(metadata)
        implementation = (PatchTracker if tracker == "patch" else FeatureTracker)(image, bbox)
        generation = old.generation + 1 if old else 1
        result = {"target_id": target_id, "generation": generation, "label": label, "view": view,
                  **metadata, "status": "selected", "tracker": tracker,
                  "center_pixel": implementation.center.tolist(),
                  "patch_bbox": self._bbox(implementation.center, implementation.size, metadata["image_size"]),
                  "normalized_correlation": None, "correlation_kind": "normalized_patch_correlation_not_probability",
                  "displacement_px": [0.0, 0.0], "displacement_norm_px": 0.0,
                  "position_space": "image_pixels", "robot_xyz_m": None, "grasp_ready": False}
        self._targets[target_id] = _Target(generation, self._clock(), tracker, implementation,
            deepcopy(result), deepcopy(result), {metadata["observation_id"]: deepcopy(result)})
        return deepcopy(result)

    def locate(self, target_id, observation):
        """Track a newer local frame, or return a latched invalid state.

        Repeating one of the last 64 accepted observations is idempotent while
        the target remains valid. It does not replace the latest observation used
        by ``resolve``. Expiry and invalid-state latches take precedence over old
        cached successes. A new ID with a non-increasing camera timestamp raises
        without changing the target.
        """
        target = self._get(target_id)
        self._expire(target)
        if target.result["status"] not in self.VALID:
            return deepcopy(target.result)
        ident = self._observation_id(observation)
        if ident in target.history:
            return deepcopy(target.history[ident])
        view = target.accepted["view"]
        views = observation.get("views")
        if not isinstance(views, dict) or view not in views:
            return self._latch(target, "view_changed", "selected camera view is missing; explicitly reselect",
                {"observation_id": ident, "camera_id": None, "captured_ns": None,
                 "image_size": None, "source_image_path": None})
        metadata = self._metadata(observation, view)
        if metadata["captured_ns"] <= target.accepted["captured_ns"]:
            raise CleanupError("out-of-order camera frame: a new observation requires a newer captured_ns")
        if (metadata["camera_id"] != target.accepted["camera_id"]
                or metadata["image_size"] != target.accepted["image_size"]):
            return self._latch(target, "view_changed", "camera identity or resolution changed; explicitly reselect", metadata)
        try:
            image = self._image(metadata)
            center, correlation = target.tracker.locate(image, radius=65)
            if not np.isfinite(correlation):
                raise CleanupError("tracker returned non-finite correlation")
            bbox = self._bbox(center, target.tracker.size, metadata["image_size"])
        except (CleanupError, cv2.error, OSError) as exc:
            return self._latch(target, "lost", f"local image tracking failed; explicitly reselect: {exc}", metadata)
        displacement = np.asarray(center) - target.accepted["center_pixel"]
        norm = float(np.linalg.norm(displacement))
        result = {**deepcopy(target.accepted), **metadata,
                  "status": "shifted" if norm > 5 else "tracked", "center_pixel": np.asarray(center).tolist(),
                  "patch_bbox": bbox, "normalized_correlation": float(correlation),
                  "displacement_px": displacement.tolist(), "displacement_norm_px": norm}
        target.accepted, target.result = deepcopy(result), deepcopy(result)
        target.history[ident] = deepcopy(result)
        if len(target.history) > self.MAX_HISTORY:
            del target.history[next(iter(target.history))]
        return deepcopy(result)

    def validate_binding(self, target_id, generation):
        """Reject bindings invalidated by reselection, expiry, loss or view change."""
        target = self._get(target_id)
        if type(generation) is not int or generation != target.generation:
            raise CleanupError("target generation changed; select and bind a new probe")
        self._expire(target)
        if target.result["status"] not in self.VALID:
            raise CleanupError(f"target is {target.result['status']}; explicitly reselect before probing")

    def resolve(self, target_id, observation_id):
        """Resolve only the latest accepted wrist ROI into an explicit probe input."""
        target = self._get(target_id)
        self.validate_binding(target_id, target.generation)
        accepted = target.accepted
        if accepted["view"] != "wrist":
            raise CleanupError("target probe requires a wrist-view selection")
        if observation_id != accepted["observation_id"]:
            raise CleanupError("target must be located in the probe's current observation first")
        return {"target_id": target_id, "generation": target.generation,
                "bbox": deepcopy(accepted["patch_bbox"]), "tracker": target.tracker_kind,
                "label": accepted["label"], "view": accepted["view"], "camera_id": accepted["camera_id"],
                "observation_id": accepted["observation_id"], "captured_ns": accepted["captured_ns"],
                "image_size": deepcopy(accepted["image_size"]), "source_image_path": accepted["source_image_path"],
                "center_pixel": deepcopy(accepted["center_pixel"]), "position_space": "image_pixels",
                "robot_xyz_m": None, "grasp_ready": False}
