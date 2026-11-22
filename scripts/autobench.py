"""Time the decision call on several models with a fixed scene."""
import asyncio, json, time, httpx
from elrobot.autopilot import SCHEMA, SYSTEM, api_key
from elrobot.skills import GESTURES
USER = """Instruction: point at the blue cube with the gripper 10 cm above it

Running for 1s.

Your earlier actions, oldest first:
none yet

Waypoints still queued: 0

Last two seconds of perception, oldest first:
""" + "\n".join(f"t{-2+i*0.25:+.1f}s ee=(+0.00,+0.19,0.08) pitch=73 grip=0.99 holding=false still | sees: blue@(+0.06,+0.27) age 0.1s" for i in range(8))
async def main():
    async with httpx.AsyncClient(timeout=40) as c:
        for model, eff, tier in (("gpt-5.4-mini", "none", ""), ("gpt-5.4-mini", "none", "priority"), ("gpt-6-luna", "none", ""), ("gpt-6-luna", "none", "priority"), ("gpt-5.4-nano", "none", "")):
            ts, out = [], ""
            for _ in range(4):
                body = {"model": model, "messages": [{"role": "system", "content": SYSTEM.format(reach=0.36, gestures=", ".join(GESTURES))}, {"role": "user", "content": USER}],
                        "response_format": {"type": "json_schema", "json_schema": {"name": "action_chunk", "strict": True, "schema": SCHEMA}}}
                if eff: body["reasoning_effort"] = eff
                if tier: body["service_tier"] = tier
                t0 = time.monotonic()
                r = await c.post("https://api.openai.com/v1/chat/completions", json=body, headers={"Authorization": f"Bearer {api_key()}"})
                ts.append(round(time.monotonic() - t0, 2))
                if r.status_code != 200: out = r.text[:260]; break
                u = r.json()["usage"]; d = json.loads(r.json()["choices"][0]["message"]["content"]); out = f"tok in {u['prompt_tokens']} out {u['completion_tokens']} tier={r.json().get('service_tier')} " + json.dumps(d)[:150]
            print(model, eff, tier, ts, out)
asyncio.run(main())
