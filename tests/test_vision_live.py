"""The web UI's live vision layers, against the simulated cameras."""

import time

from fastapi.testclient import TestClient

from elrobot.config import Config
from elrobot.daemon import create_app


def test_layers_for_the_simulated_cameras(tmp_path):
    import shutil

    from elrobot.config import REPO_ROOT

    root = tmp_path / "workspace"
    shutil.copytree(REPO_ROOT / "workspace", root, ignore=shutil.ignore_patterns("perception", "tool_call_history", "state", "__pycache__", "assets", "*.sim.json", ".*"))
    with TestClient(create_app(Config(sim=True, voice=False, workspace_dir=root, table_z=0.010))) as client:
        for _ in range(100):  # the simulated cameras start a moment after the daemon
            if client.get("/api/camera/top/vision").status_code == 200:
                break
            time.sleep(0.1)
        top = client.get("/api/camera/top/vision").json()
        assert (top["width"], top["height"]) == (640, 480)
        colours = sorted(o["label"] for o in top["layers"]["objects"])
        assert colours == ["blue", "green", "red"], colours  # the simulated table's three blocks, and not the arm
        for o in top["layers"]["objects"]:
            assert len(o["polygon"]) >= 3 and len(o["box"]) == 4 and len(o["point"]) == 2
        assert [line["label"] for line in top["layers"]["frame"]][:2] == ["x 10 cm", "y 10 cm"]
        assert top["layers"]["tags"] == []

        wrist = client.get("/api/camera/wrist/vision").json()
        assert "frame" not in wrist["layers"] and "objects" in wrist["layers"]
        assert client.get("/api/camera/nope/vision").status_code == 404
