"""Open the jaws to `fraction` of full travel: 1 = fully open (53 mm), just above 0 = nearly shut.

One joint command for the gripper servo alone; the arm does not move. Returns the opening
actually reached. To grip something use close_gripper, which closes until the jaws stall and
reports whether an object stopped them.
"""

from util.arm import RobotError
from util.motion import require_ready, tcp_report


def open_gripper(ctx, fraction: float = 1.0) -> dict:
    require_ready(ctx)
    if not 0.0 < fraction <= 1.0:
        raise RobotError("fraction is an opening between 0 and 1; to close on something use close_gripper")
    res = ctx.arm.joints(gripper=fraction, label="gripper")
    return {"requested_open": fraction, "reached_open": res["gripper"]["open"], "width_mm": res["gripper"]["width_mm"], "tcp": tcp_report(res["tcp"], res["gripper"])}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(open_gripper)
