"""Send a recorded utterance to the autopilot and print what it heard, did and said: voicetest.py FILE [timeout]."""
import asyncio, json, sys
import httpx, websockets
async def main():
    async with websockets.connect("ws://127.0.0.1:8765/ws", max_size=None) as ws:
        await ws.recv()
        r = httpx.post("http://127.0.0.1:8765/api/autopilot/voice", content=open(sys.argv[1], "rb").read(), headers={"content-type": "audio/aiff"}, timeout=60).json()
        print("heard:", r)
        async def watch():
            async for m in ws:
                d = json.loads(m)
                if d.get("type") != "event": continue
                if d["kind"] == "autopilot" and d["phase"] == "decision": print(f"  #{d['n']} {d['latency_ms']}ms {d['status']} skill={d['skill']} say={d['say']!r}")
                elif d["kind"] == "autopilot" and d["phase"] == "speech":
                    a = httpx.get("http://127.0.0.1:8765" + d["url"]) if d["url"] else None
                    print("  SPOKE:", d["text"], "| audio", a.status_code if a else None, len(a.content) if a else 0, "bytes")
                elif d["kind"] == "tool_end" and d.get("source") == "autopilot": print("  tool", d["tool"], d["ok"], d["summary"][:100])
                elif d["kind"] == "autopilot" and d["phase"] == "end":
                    print("  END", d["status"], d["seconds"], "s"); await asyncio.sleep(2.5); return
        try: await asyncio.wait_for(watch(), float(sys.argv[2]) if len(sys.argv) > 2 else 40)
        except asyncio.TimeoutError: print("TIMEOUT")
asyncio.run(main())
