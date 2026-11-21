"""Close the jaws on whatever is between them and report whether something stopped them.

A joint command with the gripper at 0: the servo is driven towards fully closed. If the jaws
stall wider than 6 % open, an object is in the way: `holding` is true, `width_mm` is the
object's width across the jaws, and the robot side eases the servo to just past the stall point
so it keeps a firm grip without overheating. If they shut completely, `holding` is false.

`holding` only means the jaws stalled. It does not prove the right object is held or that it
survived a lift: look with a camera, and read the gripper state again after moving.
"""

from util.motion import require_ready, tcp_report


def close_gripper(ctx) -> dict:
    require_ready(ctx)
    res = ctx.arm.joints(gripper=0.0, label="grasping")
    return {"holding": res["gripper"]["holding"], "width_mm": res["gripper"]["width_mm"], "tcp": tcp_report(res["tcp"], res["gripper"])}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(close_gripper)
