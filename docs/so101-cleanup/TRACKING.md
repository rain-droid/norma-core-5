# Reference-anchored feature tracker

`FeatureTracker` is an offline image-processing alternative to `visual.PatchTracker`. It does not change the controller default, talk to Station, capture images, send motion, or write files. Its output is the selected image patch's center, not a grasp point or a world-coordinate measurement.

```python
from so101_station.cleanup.tracking import FeatureTracker

tracker = FeatureTracker(first_image_bgr, [x, y, width, height])
center, confidence = tracker.locate(next_image_bgr, radius=65)
```

The compatible interface is `center` (NumPy `[x, y]`), `shape` (`height, width`), `size` (`width, height`), and `locate(image, radius=65) -> (center, confidence)`. Returned center arrays are copies. Input images must be nonempty `uint8` BGR or grayscale arrays with unchanged dimensions. ROI coordinates are integer pixels; width and height must be 24–300 pixels.

`confidence` is the measured normalized cross-correlation of the original patch on the visible support around accepted feature inliers. It is **not a probability**, an object-identity score, or a calibration accuracy estimate. It is not directly comparable to the existing tracker's full-patch correlation because the masks differ. `last_metrics` exposes counts, forward/backward errors, fit residuals, correlation, peak separation and feature/patch disagreement for inspection. None of these values verifies robot or camera freshness.

## Acceptance checks

The tracker seeds up to 100 Shi–Tomasi corners inside the selected ROI, away from its immediate edge. It retains the original image, keypoints and patch for its lifetime. A patch needs at least eight features distributed across both axes and a convex-hull area of at least 8% of the ROI.

Each update uses pyramidal Lucas–Kanade flow from the **original frame** to the new frame. The prior accepted transform initializes the search; it is not used as a new reference image. Forward/backward error must be at most 1 pixel. RANSAC fits a similarity transform, with a 1.5-pixel residual bound, at least eight and 40% of the original features retained, and at least 65% agreement among the surviving flow points. Inliers must still be spatially distributed.

The local appearance envelope permits scale 0.85–1.18 and rotation within 12° of the reference. A single update permits scale 0.93–1.07 and rotation within 8° of the prior accepted transform. The mapped patch must remain inside the image.

An independently searched, rotated/scaled original patch corroborates the feature estimate. Its mask uses neighborhoods around the visible inlier points and must cover at least 15% of the original patch. This permits some occlusion without allowing the occluder's brightness to dominate whole-patch correlation. The match must have correlation at least 0.70, a second-peak separation of at least 0.035, and agree with the feature-derived center within 2 pixels. Repeated or similar-looking patches with competing peaks fail this check. There is no template-only fallback after feature loss.

Search is bounded around the previous accepted center. The accepted step must be within `radius - 5` pixels; `radius` must be 6–100 pixels. This is a per-update image bound, not a cumulative workspace bound. Original-patch anchoring corrects the estimated translation every frame, and a 0.30-pixel center deadband suppresses tiny reporting jitter. Genuine displacement beyond that deadband is retained. The controller's existing convergence tolerance must still prevent noisy image estimates from causing unnecessary motion.

Loss, ambiguity, inconsistent motion, unsupported scale/rotation or excessive displacement raises `CleanupError` without updating the accepted center or transform. The tracker never replaces its reference with a newly observed object. The calling controller remains responsible for stopping motion and requiring an inspected new ROI when appropriate.

## Validation and measured CPU cost

```sh
.venv/bin/python -m pytest -q tests/test_cleanup_tracking.py
```

Fifteen tests cover subpixel translations and return to the initial reference, mild rotation/scale, partial occlusion, feature loss and replacement, repeated-texture ambiguity, independently moving minority outliers, incoherent split motion, stationary noise without accumulated drift, displacement bounds and invalid input. The rotation/scale test uses 4° and 1.04× scale. The partial-occlusion case covers 23% of a synthetic textured ROI. These are specific passing cases, not guarantees for arbitrary occlusion or deformation.

An offline comparison on 2026-11-21 used six original 853×480 wrist JPEGs from run `73253fe70b6143a199b17ed17459e77d`, with ROI `[345, 55, 220, 170]` on the paper. The first frame initialized a new tracker; the next five were processed sequentially, repeated 25 times for each implementation. Frames were decoded before timing; each `locate` call, including rejected calls, was timed with `time.perf_counter`. No synthetic warp was applied to these recorded frames. Environment: arm64, OpenCV 5.1.0, eight OpenCV worker threads. These are elapsed CPU-processing call times on this host, not end-to-end control latency.

| Tracker | Calls | Accepted calls | Median | p95 |
| --- | ---: | ---: | ---: | ---: |
| Existing `PatchTracker` | 125 | 125 | 1.61 ms | 4.64 ms |
| New `FeatureTracker` | 125 | 100 | 9.09 ms | 12.45 ms |

The feature tracker accepted the first four distinct transitions and rejected the fifth for exceeding its scale/rotation envelope, on every repetition. The patch tracker accepted all five. There is no labeled ground truth for this sequence, so acceptance counts do not establish which estimate is correct, and this comparison does not measure physical accuracy or reliability.

Only first/last accepted centers and correlations were retained from the benchmark:

| Tracker | First accepted center | First correlation | Last accepted center | Last correlation |
| --- | --- | ---: | --- | ---: |
| `PatchTracker` | `[454.964, 140.185]` | 0.99630 | `[451.488, 144.628]` | 0.79861 |
| `FeatureTracker` | `[455.000, 140.000]` | 0.99642 | `[455.000, 140.000]` | 0.99722 |

The final accepted centers refer to different frames because the new tracker rejected the last transition. The reference frame is `17286a20772b4da09ac7fa2b94455190`; subsequent observation IDs, in order, are `2765f1cc487c4355a091008c4aff6d9b`, `7e777afe298c430d910bad7da89de40e`, `00f1a1d73d104958b93e7cfb056fdda2`, `c6da3514020a47e3880a6ab6f79c7d6a` and `ab908a2fc4e246738b50604334b15b8b`. The benchmark persisted no image copies, frame payloads or base64 data.

## Integration recommendation and limits

Retain `PatchTracker` as the default until both trackers have been compared on the new bounded physical-probe recordings. Select `FeatureTracker` explicitly for comparison; preserve the controller's stop handling, measured pose bounds, fresh-frame checks, calibration expiry and convergence tolerance. Treat `CleanupError` as a failed observation, not permission to switch targets automatically. Do not relax the feature check merely because template matching still finds a peak.

The new tracker can help when a selected patch has several distinct features, modest rigid rotation/scale changes, or limited occlusion. It can reject valid motion more often than the current tracker. Large viewpoint changes, perspective deformation, crumpling paper, blur, exposure changes, repetitive surfaces and insufficient visible texture remain limitations. Background features can dominate a poorly selected ROI; the tracker is not an object segmenter. Truly indistinguishable patches cannot be assigned physical identities from these images alone. Neither implementation measures depth, contact, grasp success or millimeter accuracy.
