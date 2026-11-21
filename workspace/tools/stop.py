"""Halt all motion now and hold position. Always allowed, even while another tool is running.

The running tool ends with a `stopped` error. Torque stays on, so the arm does not drop.
"""

from util.motion import tcp_report

MOTION = False


async def stop(ctx) -> dict:
    await ctx.robot.stop()
    return {"stopped": True, "tcp": tcp_report(ctx) if ctx.robot.ready else None}


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(stop)
