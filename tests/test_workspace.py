"""The workspace runs tools straight from their files, in the process that owns the arm."""

import asyncio
import json
import os

from elrobot.config import Config
from elrobot.daemon import Hub

TOOL = '''
async def nudge(ctx, joint: int, by: float = {by}):
    q = ctx.robot.q()
    q[joint - 1] += by
    await ctx.robot.move_joints(q)
    return {{"moved_by": by}}
'''


def test_tool_file_is_run_recorded_and_reloaded(tmp_path):
    tools = tmp_path / "tools"
    tools.mkdir()
    (tools / "nudge.py").write_text(TOOL.format(by=0.2))
    hub = Hub(Config(sim=True, voice=False, workspace_dir=tmp_path))

    async def run():
        await hub.link.start()
        try:
            assert await hub.link.wait_connected(2)
            q0 = hub.robot.q()[0]
            res = await hub.workspace.call("nudge", {"joint": 1})
            assert res["moved_by"] == 0.2 and "error" not in res
            assert abs(hub.robot.q()[0] - q0 - 0.2) < 0.02
            history = tmp_path / res["history"]
            assert json.loads((history / "input.json").read_text())["arguments"] == {"joint": 1}
            assert len((history / "joint_commands.jsonl").read_text().splitlines()) == res["joint_command_count"] == 1
            assert json.loads((tmp_path / "perception/proprioception/state.json").read_text())["connected"]

            # an edit to the file is what the next call runs
            (tools / "nudge.py").write_text(TOOL.format(by=-0.1))
            os.utime(tools / "nudge.py", (1e9, 2e9))
            assert (await hub.workspace.call("nudge", {"joint": 1}))["moved_by"] == -0.1

            assert (await hub.workspace.call("nudge", {"jiont": 1}))["error_kind"] == "robot"
            bug = await hub.workspace.call("nudge", {"joint": 99})
            assert bug["error_kind"] == "tool_bug" and "IndexError" in bug["traceback"]
            assert not hub._task_lock.locked()
        finally:
            await hub.link.stop()

    asyncio.run(run())
