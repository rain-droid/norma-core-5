"""Along a wind-up -> follow-through swing with all joints at speed W, print launch velocity."""
import math, sys
import numpy as np
from elrobot.config import Config
from elrobot.kinematics import Kinematics
kin = Kinematics(Config.load().urdf)
W = 3.5
def show(qa, qb):
    qa, qb = np.radians(qa), np.radians(qb); d = qb - qa
    print("swing", np.degrees(qa).round(0), "->", np.degrees(qb).round(0), "collide:", kin.self_collision(qa), kin.self_collision(qb))
    for t in np.arange(0, np.max(np.abs(d)) / W + 0.02, 0.04):
        q = qa + np.sign(d) * np.minimum(np.abs(d), W * t)
        qd = np.where(np.abs(d) > W * t, np.sign(d) * W, 0.0)
        v = kin.jacobian(q)[:3] @ qd; p = kin.fk(q)[:3, 3]
        print(f"  t={t:.2f} q={np.degrees(q[1:4]).round(0)} p=({p[1]:+.2f},{p[2]:.2f}) v=({v[1]:+.2f},{v[2]:+.2f}) |v|={np.hypot(v[1],v[2]):.2f} el={math.degrees(math.atan2(v[2],v[1])):.0f}")
for a, b in eval(sys.argv[1]):
    show(a, b)
