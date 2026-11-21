"""Keep control of the arm across several tool calls: `acquire`, `release`, or `status`.

The arm is shared: people use it from the web UI on other laptops, and other scripts may be
running. The robot side moves it only for whoever holds control. Every tool that moves the
arm asks for control by itself and gives it back when the call ends, so a single call needs
nothing from you. Between two of your calls, though, someone else may take the arm and move
things. For a task of several steps (look, pick, place, look again), `acquire` first: the
ticket is kept in `.control` and every tool uses it until you `release`, so nobody else moves
the arm in between.

acquire  waits up to `wait_s` for your turn (others may be ahead in the queue). The ticket
         lapses after `ttl_s` seconds without a command, so the arm is not blocked forever if
         you stop; acquire again if a tool says the ticket is no longer valid.
release  gives the arm back. Do it when the task is done.
status   who holds control and who is waiting.

A person at the web UI can take control from you at any time; your motion in progress then
stops, and you are first in line to get it back. Stop never needs control.
"""

import time

from util.arm import SESSION_FILE, RobotError, _request, read_session


def control(ctx, action: str = "status", wait_s: float = 120.0, ttl_s: float = 180.0) -> dict:
    token = read_session()
    if action == "status":
        res = _request("/api/control", token=token)
        return {"holder": res["holder"], "queue": res["queue"], "you": res.get("you", {"state": "none"})}
    if action == "release":
        if token:
            _request("/api/control/release", {}, token=token)
        SESSION_FILE.unlink(missing_ok=True)
        return {"released": bool(token)}
    if action != "acquire":
        raise RobotError("action is one of acquire, release, status")

    t0 = time.monotonic()
    body = {"name": ctx.arm.source, "kind": "agent", "ttl_s": ttl_s, "wait": 0}
    res = _request("/api/control/request", body, token=token)
    while res["you"]["state"] == "waiting" and time.monotonic() - t0 < wait_s:
        wait = max(0.5, min(20.0, wait_s - (time.monotonic() - t0)))
        res = _request("/api/control/request", {**body, "wait": wait}, timeout=wait + 10, token=res["token"])
    if res["you"]["state"] != "granted":
        _request("/api/control/release", {}, token=res["token"])
        SESSION_FILE.unlink(missing_ok=True)
        who = (res.get("holder") or {}).get("name") or "someone"
        raise RobotError(f"no control after {wait_s:.0f} s: {who} has the arm and {len(res['queue'])} are waiting. Try again later.")
    SESSION_FILE.write_text(res["token"] + "\n")
    return {"granted": True, "waited_s": round(time.monotonic() - t0, 1), "lapses_after_idle_s": res["you"]["ttl_s"],
            "queue": res["queue"]}  # fmt: skip


if __name__ == "__main__":
    from util.cli import run_cli

    run_cli(control)
