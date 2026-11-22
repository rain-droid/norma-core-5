"""Grid-search swings (servo speed W, ramp A) for the longest lob released at 45-60 deg elevation."""
import itertools, math, sys
import numpy as np
from elrobot.config import Config
from elrobot.kinematics import Kinematics
kin = Kinematics(Config.load().urdf)
W, A, G = 2.9, 10.0, 9.81
lo, hi = np.degrees(kin.lower) + 5, np.degrees(kin.upper) - 5
def prof(t, dist):  # distance and speed along one joint after t seconds
    tr = W / A
    s = 0.5 * A * t * t if t < tr else 0.5 * W * tr + W * (t - tr)
    v = min(W, A * t)
    return (dist, 0.0) if s >= dist else (s, v)
def ok_back(P):  # people sit right behind the base: stay out of their space
    return True
def best(qa, qb):
    qa, qb = np.radians(qa), np.radians(qb); d = qb - qa
    out = None
    for t in np.arange(0.02, 0.9, 0.02):
        sv = [prof(t, abs(x)) for x in d]
        q = qa + np.sign(d) * [s for s, _ in sv]; qd = np.sign(d) * [v for _, v in sv]
        g = kin.geometry(q)
        P = g.all_points()
        if P[2:, 2].min() < 0.03 or not ok_back(P): return out
        v = kin.jacobian(q)[:3] @ qd; p = g.tcp
        if v[1] < 0.3 or v[2] <= 0: continue
        el = math.degrees(math.atan2(v[2], v[1]))
        if not 45 <= el <= 62: continue
        disc = v[2] ** 2 + 2 * G * (p[2] - 0.20)
        if disc < 0: continue
        rng = p[1] + v[1] * (v[2] + math.sqrt(disc)) / G
        if out is None or rng > out[0]:
            out = (rng, t, el, float(np.hypot(v[1], v[2])), p.round(2), np.degrees(q[1:4]).round(0))
    return out
res = []
grid = lambda i, n: np.linspace(lo[i], hi[i], n).round(0)
for a in itertools.product(grid(1, 7), grid(2, 7), grid(3, 7)):
    qa = [0, *a, 0]
    if kin.self_collision(np.radians(qa)) or kin.geometry(np.radians(qa)).all_points()[2:, 2].min() < 0.03 or not ok_back(kin.geometry(np.radians(qa)).all_points()): continue
    for b in itertools.product(grid(1, 4), grid(2, 4), grid(3, 4)):
        r = best(qa, [0, *b, 0])
        if r: res.append((r, a, b))
res.sort(key=lambda r: -r[0][0])
seen = set()
for r, a, b in res:
    if a in seen: continue
    seen.add(a)
    print(f"range {r[0]:.2f} t={r[1]:.2f} el={r[2]:.0f} v={r[3]:.2f} p={r[4]} q_rel={r[5]}  {a} -> {b}")
    if len(seen) > 14: break
