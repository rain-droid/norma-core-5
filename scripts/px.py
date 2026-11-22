"""Where wrist-camera pixels land on horizontal planes, at the arm's current pose: px.py u,v[,z] ..."""
import json, sys, urllib.request
import numpy as np
from elrobot.config import Config
from elrobot.link import SimLink
from elrobot.robot import Robot
from elrobot.skills import Skills

cfg = Config.load()
st = json.load(urllib.request.urlopen("http://127.0.0.1:8765/api/state"))
q = np.array([j["q"] for j in st["joints"][:-1]])
robot = Robot(SimLink({j["id"]: (j["lower"], j["upper"]) for j in st["joints"]}), cfg)
sk = Skills(robot, cfg)
T = robot.kin.fk(q)
cam = sk.cam.scaled(853, 480)
print("cam at", (T @ cam.T_tool_cam())[:3, 3].round(3))
for a in sys.argv[1:]:
    p = [float(v) for v in a.split(",")]
    for z in ([p[2]] if len(p) > 2 else [0.0, 0.05, 0.08, 0.10, 0.12, 0.14]):
        r = cam.pixel_to_plane(p[0], p[1], T, z)
        print(f"({p[0]:.0f},{p[1]:.0f}) z={z:.2f} ->", None if r is None else r[:2].round(3))
