"""Sample commanded vs measured joint angles while a tool call runs: `track.py TOOL JSON`."""
import json, math, sys, threading, time, urllib.request

def state():
    return json.load(urllib.request.urlopen("http://127.0.0.1:8765/api/state"))

def call():
    req = urllib.request.Request(f"http://127.0.0.1:8765/api/tools/{sys.argv[1]}", json.dumps({"args": json.loads(sys.argv[2])}).encode(), {"Content-Type": "application/json"})
    out.append(json.load(urllib.request.urlopen(req)))

out = []
th = threading.Thread(target=call); th.start()
t0 = time.time(); worst = {}
while th.is_alive():
    s = state(); row = []
    for j in s["joints"][:-1]:
        lag = math.degrees(j["q_goal"] - j["q"]); row.append(f"{math.degrees(j['q']):6.1f}({lag:+5.1f},{j['load']:4d})")
        worst[j["id"]] = max(worst.get(j["id"], 0), abs(lag))
    print(f"{time.time()-t0:5.2f}", " ".join(row)); time.sleep(0.1)
print({k: round(v, 1) for k, v in worst.items()}); print(out[0].get("summary"))
