from copy import deepcopy

import cv2
import numpy as np
import pytest

from so101_station.cleanup.geometry import CleanupError
from so101_station.cleanup.targets import TargetMemory


BOX = [90, 70, 80, 80]
CENTER = np.array([130., 110.])


def image():
    picture = np.full((240, 320, 3), 110, np.uint8)
    picture[70:150, 90:170] = np.random.default_rng(12).integers(0, 256, (80, 80, 3), dtype=np.uint8)
    return picture


def observation(tmp_path, name="initial", stamp=100, dx=0, dy=0, camera="camera-wrist", view="wrist", picture=None):
    picture = image() if picture is None else picture
    if dx or dy:
        picture = cv2.warpAffine(picture, np.float32([[1, 0, dx], [0, 1, dy]]), picture.shape[1::-1], borderValue=(110, 110, 110))
    path = tmp_path / f"{name}.png"
    assert cv2.imwrite(str(path), picture)
    return {"observation_id": name, "views": {view: {"camera_id": camera, "captured_ns": stamp,
        "image_size": list(picture.shape[1::-1]), "path": str(path)}}, "state": {"q": [0] * 5}}


def select(memory, obs, target_id="paper", tracker="patch", view="wrist"):
    return memory.select(target_id, "crumpled_paper", view, obs, BOX, tracker=tracker)


def test_select_translate_and_shift_return_only_image_coordinates(tmp_path):
    memory = TargetMemory()
    selected = select(memory, observation(tmp_path))
    assert selected["status"] == "selected" and selected["generation"] == 1
    assert selected["normalized_correlation"] is None
    assert selected["center_pixel"] == CENTER.tolist() and selected["patch_bbox"] == BOX
    tracked = memory.locate("paper", observation(tmp_path, "small", 101, dx=3, dy=1))
    assert tracked["status"] == "tracked"
    assert np.allclose(tracked["center_pixel"], CENTER + [3, 1], atol=.15)
    shifted = memory.locate("paper", observation(tmp_path, "shift", 102, dx=11, dy=3))
    assert shifted["status"] == "shifted" and shifted["generation"] == 1
    assert np.allclose(shifted["displacement_px"], [8, 2], atol=.2)
    assert shifted["patch_bbox"] == [101, 73, 80, 80]
    assert shifted["normalized_correlation"] > .9
    assert shifted["position_space"] == "image_pixels"
    assert shifted["robot_xyz_m"] is None and shifted["grasp_ready"] is False
    assert shifted["source_image_path"] == str(tmp_path / "shift.png")


def test_same_observation_is_cached_without_rewinding_resolve(tmp_path):
    memory = TargetMemory()
    first = observation(tmp_path)
    selected = select(memory, first)
    newer = observation(tmp_path, "newer", 110, dx=4)
    tracked = memory.locate("paper", newer)
    newer["views"]["wrist"]["path"] = "/does/not/exist.png"
    assert memory.locate("paper", newer) == tracked
    assert memory.locate("paper", first) == selected
    assert memory.resolve("paper", "newer")["bbox"] == [94, 70, 80, 80]
    with pytest.raises(CleanupError, match="current observation"):
        memory.resolve("paper", "initial")


@pytest.mark.parametrize("stamp", [99, 100])
def test_older_or_equal_timestamp_raises_without_mutation(tmp_path, stamp):
    memory = TargetMemory()
    initial = observation(tmp_path)
    selected = select(memory, initial)
    with pytest.raises(CleanupError, match="out-of-order"):
        memory.locate("paper", observation(tmp_path, "out-of-order", stamp, dx=12))
    assert memory.locate("paper", initial) == selected
    assert memory.resolve("paper", "initial")["bbox"] == BOX
    memory.validate_binding("paper", 1)


def test_loss_latches_and_explicit_reselection_invalidates_generation(tmp_path):
    memory = TargetMemory()
    initial = observation(tmp_path)
    select(memory, initial)
    blank = observation(tmp_path, "blank", 101, picture=np.full_like(image(), 110))
    lost = memory.locate("paper", blank)
    assert lost["status"] == "lost" and lost["generation"] == 1
    assert "center_pixel" not in lost and "patch_bbox" not in lost
    assert memory.locate("paper", observation(tmp_path, "visible-again", 102)) == lost
    assert memory.locate("paper", initial) == lost  # Cached success cannot bypass a latch.
    with pytest.raises(CleanupError, match="lost"):
        memory.resolve("paper", "initial")
    with pytest.raises(CleanupError, match="lost"):
        memory.validate_binding("paper", 1)
    selected = select(memory, observation(tmp_path, "reselected", 103))
    assert selected["generation"] == 2 and selected["status"] == "selected"
    with pytest.raises(CleanupError, match="generation"):
        memory.validate_binding("paper", 1)
    memory.validate_binding("paper", 2)


def test_monotonic_expiry_latches_and_tracking_does_not_extend_ttl(tmp_path):
    now = [10.]
    memory = TargetMemory(clock=lambda: now[0])
    first = observation(tmp_path)
    select(memory, first)
    now[0] = 309.9
    memory.locate("paper", observation(tmp_path, "late", 101, dx=1))
    now[0] = 310.
    stale = memory.locate("paper", first)
    assert stale["status"] == "stale" and "center_pixel" not in stale
    with pytest.raises(CleanupError, match="stale"):
        memory.validate_binding("paper", 1)
    with pytest.raises(CleanupError, match="stale"):
        memory.resolve("paper", "late")
    assert select(memory, observation(tmp_path, "new", 102))["generation"] == 2
    memory.validate_binding("paper", 2)


@pytest.mark.parametrize("change", ["identity", "resolution", "missing"])
def test_camera_change_latches_without_coordinates(tmp_path, change):
    memory = TargetMemory()
    select(memory, observation(tmp_path))
    changed = observation(tmp_path, "changed", 101)
    if change == "identity":
        changed["views"]["wrist"]["camera_id"] = "other-camera"
    elif change == "resolution":
        changed["views"]["wrist"]["image_size"] = [640, 480]
    else:
        changed["views"] = {}
    result = memory.locate("paper", changed)
    assert result["status"] == "view_changed" and "patch_bbox" not in result
    if change == "missing":
        assert result["observation_id"] == "changed"
        assert all(result[key] is None for key in ("camera_id", "captured_ns", "image_size", "source_image_path"))
    assert memory.locate("paper", observation(tmp_path, "restored", 102)) == result
    with pytest.raises(CleanupError, match="view_changed"):
        memory.validate_binding("paper", 1)


def test_resolve_requires_latest_wrist_observation_and_copies_result(tmp_path):
    memory = TargetMemory()
    overhead = observation(tmp_path, view="overhead")
    select(memory, overhead, view="overhead")
    with pytest.raises(CleanupError, match="wrist"):
        memory.resolve("paper", "initial")
    wrist = observation(tmp_path, "wrist", 101)
    select(memory, wrist)
    with pytest.raises(CleanupError, match="current observation"):
        memory.resolve("paper", "unseen")
    binding = memory.resolve("paper", "wrist")
    assert binding["tracker"] == "patch" and binding["generation"] == 2
    binding["bbox"][0] = -999
    assert memory.resolve("paper", "wrist")["bbox"] == BOX


def test_feature_tracker_is_explicit_and_valid_binding_survives_shift(tmp_path):
    memory = TargetMemory()
    select(memory, observation(tmp_path), tracker="features")
    result = memory.locate("paper", observation(tmp_path, "move", 101, dx=6))
    assert result["status"] == "shifted"
    assert result["tracker"] == "features"
    memory.validate_binding("paper", 1)
    assert memory.resolve("paper", "move")["tracker"] == "features"


@pytest.mark.parametrize("label", ["paper", "anything", "", None])
def test_only_two_explicit_target_labels_are_allowed(tmp_path, label):
    with pytest.raises(CleanupError, match="label"):
        TargetMemory().select("target", label, "wrist", observation(tmp_path), BOX)


def test_target_limits_and_invalid_reselection_preserve_old_binding(tmp_path):
    memory = TargetMemory()
    obs = observation(tmp_path)
    for index in range(16):
        select(memory, obs, target_id=f"target-{index}")
    with pytest.raises(CleanupError, match="maximum 16"):
        select(memory, obs, target_id="extra")
    assert select(memory, obs, target_id="target-0")["generation"] == 2
    with pytest.raises(CleanupError, match="bbox"):
        memory.select("target-0", "snack_wrapper", "wrist", obs, [-1, -1, 80, 80])
    memory.validate_binding("target-0", 2)
    for invalid in ["", " ", "x" * 129, 1]:
        with pytest.raises(CleanupError, match="target_id"):
            select(memory, obs, target_id=invalid)
    with pytest.raises(CleanupError, match="unknown"):
        memory.validate_binding("unknown", 1)


def test_missing_image_latches_loss_and_metadata_mismatch_rejects_selection(tmp_path):
    memory = TargetMemory()
    obs = observation(tmp_path)
    select(memory, obs)
    bad = deepcopy(obs)
    bad["observation_id"] = "missing"
    bad["views"]["wrist"].update(captured_ns=101, path=str(tmp_path / "missing.png"))
    assert memory.locate("paper", bad)["status"] == "lost"
    invalid = deepcopy(obs)
    invalid["views"]["wrist"]["image_size"] = [10, 10]
    with pytest.raises(CleanupError, match="dimensions"):
        select(memory, invalid)
    with pytest.raises(CleanupError, match="lost"):
        memory.validate_binding("paper", 1)


def test_memory_never_accesses_robot_state_or_mutates_input(tmp_path):
    class NoRobotState(dict):
        def __getitem__(self, key):
            raise AssertionError("Image memory must not access robot state")
    memory = TargetMemory()
    obs = observation(tmp_path)
    obs["state"] = NoRobotState()
    original_views = deepcopy(obs["views"])
    selected = memory.select("wrapper", "snack_wrapper", "wrist", obs, BOX)
    selected["center_pixel"][0] = -1
    assert memory.locate("wrapper", obs)["center_pixel"] == CENTER.tolist()
    assert obs["views"] == original_views
    assert memory.resolve("wrapper", "initial")["robot_xyz_m"] is None


def test_history_is_bounded_to_last_64_accepted_observations(tmp_path):
    memory = TargetMemory()
    initial = observation(tmp_path)
    select(memory, initial)
    accepted = {}
    for index in range(1, 66):
        current = deepcopy(initial)
        current["observation_id"] = f"frame-{index}"
        current["views"]["wrist"]["captured_ns"] = 100 + index
        accepted[index] = memory.locate("paper", current)
    with pytest.raises(CleanupError, match="out-of-order"):
        memory.locate("paper", initial)
    oldest_evicted = deepcopy(initial)
    oldest_evicted["observation_id"] = "frame-1"
    oldest_evicted["views"]["wrist"]["captured_ns"] = 101
    with pytest.raises(CleanupError, match="out-of-order"):
        memory.locate("paper", oldest_evicted)
    oldest_cached = deepcopy(initial)
    oldest_cached["observation_id"] = "frame-2"
    assert memory.locate("paper", oldest_cached) == accepted[2]
    assert memory.resolve("paper", "frame-65")["generation"] == 1
