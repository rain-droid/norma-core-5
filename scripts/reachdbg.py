import numpy as np, sys
from elrobot.config import Config
from elrobot.daemon import Hub
from elrobot.robot import RobotError
cfg = Config.load(); cfg.sim = True
hub = Hub(cfg); sk, r = hub.skills, hub.robot
np.set_printoptions(precision=1, suppress=True)
for x in (0.10, -0.10):
    t = np.array([x, 0.30, 0.0])
    a, pitch, qh, hp, g = sk._approach_for(t, 0.018, None)
    print("x", x, "pitch", pitch, "q_hover", np.degrees(qh), "a", a.round(2))
    T = r.kin.fk(qh); print("   hover fk pos err mm", ((T[:3,3]-hp)*1000).round(1), "approach got", T[:3,2].round(2))
    try:
        p = r.plan_linear(qh, g, approach=a, allow_low=True); print("   linear ok, end q", np.degrees(p[-1]))
    except RobotError as e: print("   linear FAIL:", e)
    for k in (0.25, 0.5, 0.75, 1.0):
        pt = hp + (g - hp) * k
        res = r.kin.ik(pt, approach=a, q_seed=qh, limit_margin=cfg.limit_margin, random_seeds=0, ref_weight=0.05, pos_tol=0.002, ang_tol=np.radians(7))
        print("   k", k, res.ok, np.degrees(res.q), res.describe() if not res.ok else "")
print("limits", np.degrees(r.kin.lower), np.degrees(r.kin.upper))
print("---- tips along the descent")
for x in (0.10, -0.10):
    t = np.array([x, 0.30, 0.0])
    a, pitch, qh, hp, g = sk._approach_for(t, 0.018, None)
    q = qh
    for k in (0.5, 0.8, 1.0):
        pt = hp + (g - hp) * k
        res = r.kin.ik(pt, approach=a, q_seed=q, limit_margin=cfg.limit_margin, random_seeds=0, ref_weight=0.05, pos_tol=0.002, ang_tol=np.radians(7)); q = res.q
        geo = r.kin.geometry(q)
        print(" x", x, "k", k, "q", np.degrees(q), "tcp z mm", round(geo.tcp[2]*1000,1), "tips z mm", (geo.tips[:,2]*1000).round(1), "approach", r.kin.fk(q)[:3,2].round(2), "why", r.check_config(q, allow_low=True))
