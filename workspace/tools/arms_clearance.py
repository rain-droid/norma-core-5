"""How far apart the two arms are right now, and between which parts.

Both arms as they stand, in the shared space (util/shared_space.py): each is five capsules
(base, upper arm, forearm, wrist, gripper) and `gap_mm` is the smallest distance between any
capsule of one and any of the other; negative means the model has them overlapping, as the two
grippers do at a hand-over. `gap_without_grippers_mm` leaves the gripper pair out. Moves of
either arm are refused below 20 mm. No motion.
"""

from util import second_arm, shared_space


def arms_clearance(ctx) -> dict:
    space = shared_space.space(ctx.root)
    q1, q2 = ctx.arm.q(), second_arm.joints_now()
    gap, where = space.between(q1, q2)
    rest, where_rest = space.between(q1, q2, grippers_may_meet=True)
    return {"gap_mm": round(gap * 1000), "between": where, "gap_without_grippers_mm": round(rest * 1000), "between_without_grippers": where_rest,
            "this_tool_point": [round(float(v), 3) for v in ctx.arm.state()["ee"]["pos"]], "second_tool_point": [round(float(v), 3) for v in space.second_tool(q2)]}  # fmt: skip


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(arms_clearance)
