"""Fold the arm into its sitting pose and switch the motors off.

Moves above the saved sitting pose (data/sit_pose.json in the repo), lowers onto it slowly, then
releases torque so the servos cool down. Refuses while holding something. Use it when the task
is finished; any later motion tool switches torque back on by itself.
"""

from util.motion import require_ready


async def rest(ctx) -> dict:
    require_ready(ctx)
    return {"pose": await ctx.skills.rest(), "torque": ctx.robot.torque_on}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(rest)
