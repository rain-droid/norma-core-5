"""Exercise the complete harness against synthetic tabletop objects; no hardware.

Run: uv run python scripts/cleanup_demo.py
The fixed labels/pixels below are fixture inputs standing in for Codex vision.
The demo does not claim to test recognition or physical grasping.
"""
import asyncio
import json
from pathlib import Path
import numpy as np

from so101_station.cleanup.backend import MockBackend, mock_workspace
from so101_station.cleanup.geometry import ArmModel, TableMap
from so101_station.cleanup.harness import Harness


async def run():
    model = ArmModel()
    backend = MockBackend(model)
    table = TableMap(mock_workspace())
    harness = Harness(backend, model, table, Path("logs/cleanup-demo"))
    await backend.start()
    try:
        for item in list(backend.objects):
            before = await harness.get_observation()
            x, y = item["xy"]
            pixel = [80 + x * 4000, 60 + (y + .22) * 3000]
            plan = await harness.plan_pick(before["observation_id"], item["label"], .99, pixel, True)
            result = await harness.execute_plan(plan["plan_id"])
            # Simulator ground truth only; real verification must inspect the images.
            in_tray = float(np.linalg.norm(np.array(item["xy"]) - table.drop)) < .025
            report = harness.verify(result["run_id"], result["after_observation_id"], in_tray, in_tray,
                                    "Synthetic world confirms the item moved from its source into the tray.")
            print(json.dumps({"label": item["label"], "status": report["status"],
                              "before": before["image_path"], "after": result["after"]["image_path"]}))
        print(json.dumps({"mode": "sim", "verified_cleanups": harness.verified}))
    finally:
        await backend.close()


if __name__ == "__main__":
    asyncio.run(run())
