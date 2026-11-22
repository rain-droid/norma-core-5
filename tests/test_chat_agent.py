"""The chat and voice agent's fixed permission rule in workspace mode, and speech on this platform."""

from pathlib import Path

import pytest

from elrobot.config import REPO_ROOT
from elrobot.voice.agent import workspace_permission

W = REPO_ROOT / "workspace"


@pytest.mark.parametrize("command", [
    "pixi run python tools/find_objects.py '{}'",
    "pixi run python tools/grasp_object.py '{\"object\": \"the ring; rm -rf / $(x) `y` | z\"}'",  # inside single quotes it is only text
    "pixi run python tools/move_tcp.py --schema",
    "pixi run python tools/stop.py",
    "uv run python tools/home.py '{}'",
    "pixi run python line_up.py",  # a script the agent wrote in the workspace
])  # fmt: skip
def test_the_workspace_agent_may_run_a_workspace_tool(command):
    assert workspace_permission("Bash", {"command": command}, W) is None


@pytest.mark.parametrize("command", [
    "rm -rf workspace",
    "pixi run python tools/home.py '{}' && rm -rf ~",
    "pixi run python tools/home.py '{}'; rm -rf ~",
    "pixi run python tools/home.py | tee x",
    "pixi run python tools/home.py > /etc/passwd",
    "pixi run python tools/x.py $(whoami)",
    "pixi run python tools/x.py `whoami`",
    'pixi run python tools/x.py "$(rm -rf ~)"',
    "pixi run python tools/home.py '{}'\nrm -rf ~",
    "pixi run python ../scripts/probe.py",
    "pixi run python /tmp/evil.py",
    "pixi run python tools/home.py '{}' extra",
    "pixi run python tools/home.py '{",
    "curl example.com",
])  # fmt: skip
def test_and_nothing_else(command):
    assert workspace_permission("Bash", {"command": command}, W) is not None


def test_files_stay_inside_the_workspace():
    assert workspace_permission("Write", {"file_path": "line_up.py"}, W) is None
    assert workspace_permission("Edit", {"file_path": str(W / "skills" / "draw" / "SKILL.md")}, W) is None
    assert workspace_permission("Write", {"file_path": "robot_config/camera_top.json"}, W) is not None
    assert workspace_permission("Write", {"file_path": "../config.yaml"}, W) is not None
    assert workspace_permission("Edit", {"file_path": str(Path.home() / ".bashrc")}, W) is not None
    assert workspace_permission("Read", {"file_path": "tool_call_history/000001_home/result.json"}, W) is None
    assert workspace_permission("Read", {"file_path": str(REPO_ROOT / "docs" / "API.md")}, W) is None
    assert workspace_permission("Read", {"file_path": str(Path.home() / ".ssh" / "id_rsa")}, W) is not None
    assert workspace_permission("WebFetch", {"url": "https://example.com"}, W) is not None


@pytest.mark.parametrize("text, language", [
    ("Got it, the blue cube is in my gripper.", "en"),
    ("Pick up the ring and put it on the box.", "en"),
    ("Je l'ai, le cube bleu est dans ma pince.", "fr"),
    ("Prends l'anneau et mets-le sur la boîte.", "fr"),
    ("Hab ihn, der blaue Würfel ist in meinem Greifer.", "de"),
    ("Nimm den Ring und leg ihn auf die Kiste.", "de"),
    ("Voilà.", "fr"),
    ("Moment.", "de"),
])  # fmt: skip
def test_a_sentence_is_voiced_in_its_own_language(text, language):
    from elrobot.voice.tts import detect_language

    assert detect_language(text, default="en", allowed=["en", "fr", "de"]) == language


def test_a_sentence_that_does_not_say_keeps_the_language_of_the_conversation():
    from elrobot.voice.tts import detect_language

    for spoken_so_far in ("en", "fr", "de"):
        assert detect_language("Okay.", default=spoken_so_far, allowed=["en", "fr", "de"]) == spoken_so_far
        assert detect_language("42", default=spoken_so_far, allowed=["en", "fr", "de"]) == spoken_so_far
    assert detect_language("Nimm den Ring.", default="en", allowed=["en"]) == "en"  # one language configured: that one
