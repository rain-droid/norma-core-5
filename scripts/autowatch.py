"""Send an instruction to the autopilot and print its decisions until it ends: autowatch.py "prompt" [timeout]."""
import asyncio, json, sys
import httpx, websockets
async def main():
    prompt, limit = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 90
    async with websockets.connect("ws://127.0.0.1:8765/ws", max_size=None) as ws:
        await ws.recv()
        print(httpx.post("http://127.0.0.1:8765/api/autopilot", json={"prompt": prompt}, timeout=30).json())
        async def watch():
            async for m in ws:
                d = json.loads(m)
                if d.get("type") != "event": continue
                if d["kind"] == "autopilot":
                    if d["phase"] == "decision":
                        w = d["waypoints"]
                        print(f"#{d['n']} {d['latency_ms']}ms {d['status']} skill={d['skill']} wps={len(w)} {[(p['x'],p['y'],p['z'],p['pitch'],p['grip']) for p in w[:4]]} say={d['say']!r}")
                    elif d["phase"] == "end":
                        print("END", d["status"], d["say"], d["seconds"], "s", d["decisions"], "decisions"); return
                elif d["kind"] == "tool_end" and d.get("source") == "autopilot":
                    print("  tool", d["tool"], "ok" if d["ok"] else "FAILED", d["summary"][:150])
        try: await asyncio.wait_for(watch(), limit)
        except asyncio.TimeoutError:
            print("TIMEOUT -> stop"); httpx.post("http://127.0.0.1:8765/api/autopilot/stop")
asyncio.run(main())
