import json, sys, threading, time, urllib.request
def state():
    d = json.load(urllib.request.urlopen("http://127.0.0.1:8765/api/state")); j = d["joints"][-1]; return j["tick"], j["load"], j["current"], round(j["q_goal"], 2)
def call(o):
    req = urllib.request.Request("http://127.0.0.1:8765/api/tools/gripper", json.dumps({"args": {"open": o}}).encode(), {"Content-Type": "application/json"})
    r = json.load(urllib.request.urlopen(req)); print("   returned:", r["summary"], f"at {time.time()-t0:.2f}s")
for o in [float(a) for a in sys.argv[1:]]:
    t0 = time.time(); print("cmd", o); threading.Thread(target=call, args=(o,)).start()
    for _ in range(16):
        print(f"  {time.time()-t0:4.2f}", *state()); time.sleep(0.25)
