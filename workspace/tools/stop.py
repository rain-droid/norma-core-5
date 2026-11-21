"""Halt all motion now and hold position. Always allowed, even while another tool is running.

The joint command in progress ends with a `stopped` error. Torque stays on, so the arm does not drop.
"""

from util.motion import tcp_report


def stop(ctx) -> dict:
    ctx.arm.stop()
    state = ctx.arm.state()
    return {"stopped": True, "tcp": tcp_report(state["ee"], state["gripper"]) if state["ee"] else None}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(stop)
