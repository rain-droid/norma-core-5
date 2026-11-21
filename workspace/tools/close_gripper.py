"""Close the jaws on whatever is between them and report whether something stopped them.

The servo is driven towards fully closed. If the jaws stall wider than 6 % open, an object is
in the way: `holding` is true, `width_mm` is the object's width across the jaws, and the servo
goal is backed off to just past the stall point so it keeps a firm grip without overheating.
If they shut completely, `holding` is false: nothing was there.

`holding` only means the jaws stalled. It does not prove the right object is held or that it
survived a lift: look with the wrist camera, and check state.json's gripper again after moving.
"""

from util.motion import require_ready, tcp_report


async def close_gripper(ctx) -> dict:
    require_ready(ctx)
    grasp = await ctx.robot.grasp()
    return {"holding": grasp["holding"], "width_mm": grasp["width_mm"], "tcp": tcp_report(ctx)}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(close_gripper)
