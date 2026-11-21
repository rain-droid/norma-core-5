import cv2
import numpy as np
import pytest

from so101_station.cleanup.geometry import CleanupError
from so101_station.cleanup.tracking import FeatureTracker


BOX = [170, 130, 96, 96]
CENTER = np.array([218., 178.])


def scene(seed=42):
    image = np.full((360, 480, 3), 105, np.uint8)
    rng = np.random.default_rng(seed)
    # Multiscale texture provides stable corners, including after interpolation.
    patch = rng.integers(20, 235, (96, 96, 3), dtype=np.uint8)
    patch = cv2.GaussianBlur(patch, (3, 3), .6)
    for _ in range(25):
        x, y = rng.integers(6, 90, 2)
        cv2.circle(patch, (int(x), int(y)), 3, tuple(map(int, rng.integers(0, 255, 3))), -1)
    image[130:226, 170:266] = patch
    return image


def moved(image, dx=0., dy=0., angle=0., scale=1.):
    matrix = cv2.getRotationMatrix2D(tuple(CENTER), angle, scale)
    matrix[:, 2] += [dx, dy]
    return cv2.warpAffine(image, matrix, image.shape[1::-1], flags=cv2.INTER_LINEAR, borderValue=(105, 105, 105))


def test_subpixel_translation_and_return_are_reference_anchored():
    image = scene()
    tracker = FeatureTracker(image, BOX)
    for dx, dy in [(2.4, -1.7), (6.75, 3.25), (3.1, 1.8), (0, 0)]:
        point, confidence = tracker.locate(moved(image, dx, dy))
        assert np.linalg.norm(point - CENTER - [dx, dy]) < .30
        assert .70 <= confidence <= 1.001
        assert tracker.last_metrics["max_forward_backward_error_px"] <= 1.0
        assert tracker.last_metrics["inlier_features"] >= 8
    assert np.linalg.norm(tracker.center - CENTER) < .05


def test_mild_rotation_and_scale_keep_center_and_report_measured_confidence():
    image = scene()
    tracker = FeatureTracker(image, BOX)
    point, confidence = tracker.locate(moved(image, 4.5, -2.25, angle=4, scale=1.04))
    assert np.linalg.norm(point - CENTER - [4.5, -2.25]) < .65
    assert tracker.last_metrics["confidence_kind"] == "normalized_patch_correlation"
    assert tracker.last_metrics["patch_correlation"] == confidence
    with pytest.raises(CleanupError, match="scale/rotation|lost|incoherent|disagreement"):
        tracker.locate(moved(image, angle=25, scale=1.3))


def test_partial_occlusion_keeps_distributed_features():
    image = scene()
    target = moved(image, 3, 2)
    target[132:154, 173:269] = 70  # Top 23% occluded, remaining target is rigid.
    tracker = FeatureTracker(image, BOX)
    point, confidence = tracker.locate(target)
    assert np.linalg.norm(point - CENTER - [3, 2]) < .75
    assert confidence > .70
    assert tracker.last_metrics["inlier_features"] < tracker.last_metrics["reference_features"]


def test_feature_loss_and_replacement_fail_without_updating_center():
    image = scene()
    tracker = FeatureTracker(image, BOX)
    for bad in [np.full_like(image, 105), scene(100)]:
        with pytest.raises(CleanupError, match="lost|incoherent|disagreement|support"):
            tracker.locate(bad)
        assert np.array_equal(tracker.center, CENTER)


def test_repeated_texture_is_ambiguous_even_when_flow_has_one_solution():
    image = scene()
    # Two identical copies can both fall inside the local search window.
    crop = image[130:226, 170:206].copy()
    image[130:226, 206:242] = crop
    image[130:226, 242:278] = crop
    tracker = FeatureTracker(image, [170, 130, 36, 96])
    with pytest.raises(CleanupError, match="ambiguous"):
        tracker.locate(image)


def test_independently_moving_minority_background_is_rejected_as_outliers():
    image = scene()
    target = moved(image, 4, -2)
    # A narrow strip moves in the opposite direction to the dominant patch.
    other = moved(image, -5, 4)
    target[128:224, 174:192] = other[128:224, 174:192]
    tracker = FeatureTracker(image, BOX)
    point, _ = tracker.locate(target)
    assert np.linalg.norm(point - CENTER - [4, -2]) < .8
    assert tracker.last_metrics["inlier_features"] < tracker.last_metrics["reference_features"]


def test_split_motion_does_not_choose_an_arbitrary_half():
    image = scene()
    left, right = moved(image, -5, 0), moved(image, 5, 0)
    target = left.copy()
    target[:, 218:] = right[:, 218:]
    tracker = FeatureTracker(image, BOX)
    with pytest.raises(CleanupError, match="incoherent|lost|disagreement|support|ambiguous"):
        tracker.locate(target)


def test_stationary_noise_and_alternating_jitter_do_not_accumulate():
    image = scene()
    tracker = FeatureTracker(image, BOX)
    rng = np.random.default_rng(54)
    centers = []
    for i in range(60):
        noise = rng.normal(0, 1.5, image.shape)
        noisy = np.clip(moved(image, .12 if i % 2 else -.12, .1) + noise, 0, 255).astype(np.uint8)
        centers.append(tracker.locate(noisy)[0])
    assert np.max(np.linalg.norm(np.array(centers) - CENTER, axis=1)) < .35
    tracker.locate(image)
    assert np.linalg.norm(tracker.center - CENTER) < .35


def test_jump_and_out_of_frame_are_rejected():
    image = scene()
    tracker = FeatureTracker(image, BOX)
    with pytest.raises(CleanupError):
        tracker.locate(moved(image, 40, 0), radius=25)
    assert np.array_equal(tracker.center, CENTER)
    with pytest.raises(CleanupError):
        tracker.locate(moved(image, -190, 0))


@pytest.mark.parametrize("box", [[-1, 0, 40, 40], [1, 1, 20, 30], [1.5, 1, 30, 30], [450, 300, 60, 60]])
def test_invalid_roi_rejected(box):
    with pytest.raises(CleanupError, match="bbox"):
        FeatureTracker(scene(), box)


def test_invalid_image_texture_shape_and_radius_rejected():
    with pytest.raises(CleanupError, match="texture"):
        FeatureTracker(np.full((100, 100, 3), 100, np.uint8), [10, 10, 60, 60])
    with pytest.raises(CleanupError, match="uint8"):
        FeatureTracker(scene().astype(float), BOX)
    tracker = FeatureTracker(scene(), BOX)
    with pytest.raises(CleanupError, match="dimensions"):
        tracker.locate(scene()[:-1])
    for radius in [5, 101, float("nan"), True]:
        with pytest.raises(CleanupError, match="radius"):
            tracker.locate(scene(), radius=radius)


def test_returned_center_cannot_mutate_tracker_state():
    tracker = FeatureTracker(scene(), BOX)
    point, _ = tracker.locate(scene())
    point[:] = 0
    assert np.array_equal(tracker.center, CENTER)
