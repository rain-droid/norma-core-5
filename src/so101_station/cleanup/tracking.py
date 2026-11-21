"""Reference-anchored image feature tracking; no robot or camera I/O.

``confidence`` is measured normalized patch correlation, not a probability.
This module is deliberately separate from the current visual controller default.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from .geometry import CleanupError, vector


class FeatureTracker:
    """Track one textured, locally rigid ROI using fixed reference keypoints.

    Every accepted frame needs consistent bidirectional optical flow, spatially
    distributed similarity-transform inliers and an unambiguous reference-patch
    match. Failed updates leave the last accepted center and transform unchanged.
    There is no automatic target replacement after feature loss.
    """

    MIN_POINTS = 8
    MAX_FB_ERROR_PX = 1.0
    MAX_FIT_ERROR_PX = 1.5
    MIN_CORRELATION = .70
    MIN_PEAK_GAP = .035
    MAX_CORROBORATION_ERROR_PX = 2.0
    CENTER_DEADBAND_PX = .30

    def __init__(self, image_bgr, bbox_xywh):
        gray = self._gray(image_bgr)
        box = vector(bbox_xywh, 4, "bbox [x,y,width,height]")
        if not np.equal(box, np.round(box)).all():
            raise CleanupError("bbox must use integer pixels")
        x, y, w, h = map(int, box)
        if (w < 24 or h < 24 or w > 300 or h > 300 or x < 0 or y < 0
                or x + w > gray.shape[1] or y + h > gray.shape[0]):
            raise CleanupError("bbox outside image or patch dimensions outside 24..300 pixels")
        self.shape, self.size = gray.shape, (w, h)
        self._origin = np.array([x, y], dtype=float)
        self._reference_center = self._origin + [w / 2, h / 2]
        self.center = self._reference_center.copy()
        self._reference = gray.copy()
        self._patch = gray[y:y + h, x:x + w].copy()
        if float(self._patch.std()) < 8:
            raise CleanupError("target patch lacks texture")
        # Keep LK windows away from the selected patch's immediate boundary.
        mask = np.zeros_like(gray)
        margin = min(8, max(3, min(w, h) // 10))
        mask[y + margin:y + h - margin, x + margin:x + w - margin] = 255
        points = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=.02,
                                        minDistance=5, mask=mask, blockSize=5)
        if points is None or len(points) < self.MIN_POINTS:
            raise CleanupError("target patch has too few distinct features")
        self._points = points.reshape(-1, 2).astype(np.float32)
        self._check_coverage(self._points)
        self._transform = np.array([[1., 0., 0.], [0., 1., 0.]])
        self.last_metrics = {"reference_features": len(self._points), "confidence_kind": "normalized_patch_correlation"}

    @staticmethod
    def _gray(image):
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
                or image.ndim not in {2, 3} or image.size == 0
                or (image.ndim == 3 and image.shape[2] != 3)):
            raise CleanupError("image must be a nonempty uint8 grayscale or BGR array")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        return cv2.GaussianBlur(gray, (3, 3), 0)

    @staticmethod
    def _apply(matrix, points):
        return np.asarray(points) @ matrix[:, :2].T + matrix[:, 2]

    def _check_coverage(self, points):
        span = np.ptp(points, axis=0)
        area = cv2.contourArea(cv2.convexHull(points.astype(np.float32)))
        if (np.any(span < np.array(self.size) * .30)
                or area < self.size[0] * self.size[1] * .08):
            raise CleanupError("features lack distributed support across the target patch")

    def _flow(self, gray):
        # Original-to-current LK makes each estimate independent of accumulated
        # frame-to-frame flow error. The last pose only initializes the solver.
        start = self._points.reshape(-1, 1, 2)
        guess = self._apply(self._transform, self._points).astype(np.float32).reshape(-1, 1, 2)
        options = dict(winSize=(15, 15), maxLevel=2,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01),
                       flags=cv2.OPTFLOW_USE_INITIAL_FLOW, minEigThreshold=1e-4)
        forward, forward_ok, _ = cv2.calcOpticalFlowPyrLK(self._reference, gray, start, guess.copy(), **options)
        if forward is None or forward_ok is None:
            raise CleanupError("target lost: forward optical flow failed")
        backward, backward_ok, _ = cv2.calcOpticalFlowPyrLK(gray, self._reference, forward, start.copy(), **options)
        if backward is None or backward_ok is None:
            raise CleanupError("target lost: backward optical flow failed")
        current = forward.reshape(-1, 2)
        fb_error = np.linalg.norm(backward.reshape(-1, 2) - self._points, axis=1)
        valid = ((forward_ok.ravel() != 0) & (backward_ok.ravel() != 0)
                 & np.isfinite(current).all(axis=1) & np.isfinite(fb_error)
                 & (fb_error <= self.MAX_FB_ERROR_PX)
                 & (current[:, 0] >= 1) & (current[:, 0] < self.shape[1] - 1)
                 & (current[:, 1] >= 1) & (current[:, 1] < self.shape[0] - 1))
        minimum = max(self.MIN_POINTS, math.ceil(len(self._points) * .40))
        if int(valid.sum()) < minimum:
            raise CleanupError(f"target lost: only {int(valid.sum())}/{len(valid)} consistent features")
        original, current = self._points[valid], current[valid]
        matrix, inliers = cv2.estimateAffinePartial2D(original, current, method=cv2.RANSAC,
            ransacReprojThreshold=self.MAX_FIT_ERROR_PX, maxIters=1000, confidence=.99, refineIters=10)
        if matrix is None or inliers is None or not np.isfinite(matrix).all():
            raise CleanupError("target lost: no coherent feature transform")
        inliers = inliers.ravel().astype(bool)
        if int(inliers.sum()) < minimum or float(inliers.mean()) < .65:
            raise CleanupError("incoherent feature motion: insufficient agreeing inliers")
        self._check_coverage(original[inliers])
        errors = np.linalg.norm(self._apply(matrix, original[inliers]) - current[inliers], axis=1)
        if float(np.quantile(errors, .90)) > self.MAX_FIT_ERROR_PX:
            raise CleanupError("incoherent feature motion: residual too large")
        self._check_transform(matrix)
        return matrix, original[inliers], {
            "reference_features": len(self._points), "consistent_features": int(valid.sum()),
            "inlier_features": int(inliers.sum()), "inlier_fraction": float(inliers.mean()),
            "median_forward_backward_error_px": float(np.median(fb_error[valid][inliers])),
            "max_forward_backward_error_px": float(fb_error[valid][inliers].max()),
            "median_fit_error_px": float(np.median(errors)),
        }

    def _check_transform(self, matrix):
        linear = matrix[:, :2]
        scale = float(np.sqrt(np.linalg.det(linear))) if np.linalg.det(linear) > 0 else 0
        angle = math.degrees(math.atan2(linear[1, 0], linear[0, 0]))
        step = linear @ np.linalg.inv(self._transform[:, :2])
        step_scale = float(np.sqrt(np.linalg.det(step))) if np.linalg.det(step) > 0 else 0
        step_angle = math.degrees(math.atan2(step[1, 0], step[0, 0]))
        if not .85 <= scale <= 1.18 or abs(angle) > 12 or not .93 <= step_scale <= 1.07 or abs(step_angle) > 8:
            raise CleanupError("target scale/rotation exceeds local tracking bounds")
        w, h = self.size
        corners = self._origin + np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]])
        mapped = self._apply(matrix, corners)
        if np.any(mapped < 0) or np.any(mapped >= np.array(self.shape[::-1])):
            raise CleanupError("target patch moved outside image bounds")

    @staticmethod
    def _subpixel_peak(scores, px, py):
        subpixel = np.zeros(2)
        for axis, values in enumerate((scores[py, px - 1:px + 2] if 0 < px < scores.shape[1] - 1 else [],
                                       scores[py - 1:py + 2, px] if 0 < py < scores.shape[0] - 1 else [])):
            if len(values) == 3 and np.isfinite(values).all():
                a, b, c = map(float, values)
                denominator = a - 2 * b + c
                if denominator < -1e-6:
                    subpixel[axis] = np.clip(.5 * (a - c) / denominator, -.5, .5)
        return subpixel

    def _match_reference(self, gray, linear, radius, inlier_points):
        # Rotate/scale the original patch, never replace it with a tracked crop.
        w, h = self.size
        half = np.array([w / 2, h / 2])
        corners = (np.array([[0, 0], [w, 0], [w, h], [0, h]]) - half) @ linear.T
        lower, upper = np.floor(corners.min(axis=0)), np.ceil(corners.max(axis=0))
        output_size = tuple((upper - lower).astype(int))
        warp = np.column_stack((linear, -linear @ half - lower))
        template = cv2.warpAffine(self._patch, warp, output_size, flags=cv2.INTER_LINEAR,
                                  borderValue=float(self._patch.mean()))
        # Compare original appearance near independently verified visible points.
        # A solid occluder can otherwise dominate whole-patch NCC through its
        # brightness, even when the surviving target features agree precisely.
        visible = np.zeros_like(self._patch)
        support_radius = max(5, min(10, min(w, h) // 10))
        for point in inlier_points - self._origin:
            cv2.circle(visible, tuple(np.rint(point).astype(int)), support_radius, 255, -1)
        if np.count_nonzero(visible) < w * h * .15:
            raise CleanupError("visible feature support covers too little of the reference patch")
        mask = cv2.warpAffine(visible, warp, output_size, flags=cv2.INTER_NEAREST)
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8))
        center_in_template = -lower
        left_top = np.maximum(0, np.floor(self.center - radius - center_in_template)).astype(int)
        right_bottom = np.minimum(self.shape[::-1], np.ceil(self.center + radius + np.array(output_size) - center_in_template)).astype(int)
        search = gray[left_top[1]:right_bottom[1], left_top[0]:right_bottom[0]]
        if np.any(np.array(search.shape[::-1]) < output_size):
            raise CleanupError("target search region outside image bounds")
        scores = cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED, mask=mask)
        scores[~np.isfinite(scores)] = -1
        _, peak, _, point = cv2.minMaxLoc(scores)
        px, py = point
        excluded = scores.copy()
        exclusion = max(5, int(min(w, h) * .15))
        excluded[max(0, py - exclusion):py + exclusion + 1, max(0, px - exclusion):px + exclusion + 1] = -1
        gap = float(peak - excluded.max())
        if peak < self.MIN_CORRELATION or gap < self.MIN_PEAK_GAP:
            raise CleanupError(f"target lost/ambiguous: patch correlation={peak:.3f}, peak gap={gap:.3f}")
        center = left_top + [px, py] + self._subpixel_peak(scores, px, py) + center_in_template
        return center, float(peak), gap

    def locate(self, image, radius=65):
        """Return an anchored center and measured normalized patch correlation.

        ``radius`` bounds each accepted center displacement, with a 5 px margin.
        It does not bound cumulative motion from the initial ROI. No world
        coordinates, depth, contact, object identity or camera freshness are inferred.
        """
        gray = self._gray(image)
        if gray.shape != self.shape:
            raise CleanupError("camera image dimensions changed")
        if isinstance(radius, bool) or not isinstance(radius, (float, int)) or not math.isfinite(radius) or not 6 <= radius <= 100:
            raise CleanupError("tracker radius must be finite and within 6..100 pixels")
        try:
            matrix, inlier_points, metrics = self._flow(gray)
            flow_center = self._apply(matrix, self._reference_center)
            center, confidence, gap = self._match_reference(gray, matrix[:, :2], radius, inlier_points)
        except cv2.error as exc:
            raise CleanupError("target tracking failed in image processing") from exc
        disagreement = float(np.linalg.norm(center - flow_center))
        if disagreement > self.MAX_CORROBORATION_ERROR_PX:
            raise CleanupError(f"feature/patch disagreement: {disagreement:.2f} pixels")
        if np.linalg.norm(center - self.center) > radius - 5:
            raise CleanupError("target displacement exceeds tracker search bound")
        # Translate the feature map back onto the original-patch anchor each time.
        matrix[:, 2] += center - flow_center
        if np.linalg.norm(center - self.center) < self.CENTER_DEADBAND_PX:
            center = self.center.copy()
        self._transform, self.center = matrix, center
        self.last_metrics = {**metrics, "confidence_kind": "normalized_patch_correlation",
            "patch_correlation": confidence, "second_peak_gap": gap,
            "feature_patch_disagreement_px": disagreement}
        return self.center.copy(), confidence
