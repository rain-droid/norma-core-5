"""Where on the table can the arm hover and descend for a pick? Offline, from the kinematics alone."""
import numpy as np
from elrobot.config import Config
from elrobot.daemon import Hub
from elrobot.robot import RobotError
cfg = Config.load(); cfg.sim = True
hub = Hub(cfg); sk, r = hub.skills, hub.robot
r._cmd_q = np.radians(sk.__class__.__dict__.get("READY", [0, -45, 45, 75, 0])) if False else None
print("     x: " + " ".join(f"{x:+.2f}" for x in np.arange(-0.25, 0.26, 0.05)))
for y in np.arange(0.40, 0.04, -0.05):
    row = []
    for x in np.arange(-0.25, 0.26, 0.05):
        t = np.array([x, y, 0.0])
        try:
            a, pitch, q_hover, hover_pos, grasp = sk._approach_for(t, 0.018, None)
        except RobotError:
            row.append("  .  "); continue
        try:
            r.plan_linear(q_hover, grasp, approach=a, allow_low=True); row.append("  O  ")
        except RobotError as e:
            sol = r.solve(grasp, approach=a, allow_low=True, q_seed=q_hover) if "q_seed" in r.solve.__code__.co_varnames else r.solve(grasp, approach=a, allow_low=True)
            row.append("  j  " if sol.ok else "  x  ")
    print(f"y {y:.2f}: " + "".join(row))
print("O straight descent ok, j only by joint move (the new fallback), x hover ok but no descent, . no approach at all")
