"""Sharing the arm: the control queue, and the daemon refusing motion from whoever does not hold it."""

import asyncio
import shutil

import pytest
from fastapi.testclient import TestClient

from elrobot.config import REPO_ROOT, Config
from elrobot.control import ControlQueue, NotYourTurn
from elrobot.daemon import create_app


def test_queue_order_release_and_takeover():
    async def run():
        log = []
        q = ControlQueue(on_change=log.append)
        q.check(None)  # free arm: a one-off command may run
        a = q.request("agent")
        assert q.holder is a
        b = q.request("alice", "human")
        assert q.status(b.token)["you"] == {"state": "waiting", "position": 1, "ttl_s": 20.0}
        with pytest.raises(NotYourTurn, match="agent has control"):
            q.check(None)
        with pytest.raises(NotYourTurn, match="number 1"):
            q.check(b.token)
        assert q.check(a.token) is a

        c, prev = q.take("bob")  # a person jumps the queue; the holder goes first in line
        assert q.holder is c and prev is a and [t.name for t in q.waiting] == ["agent", "alice"]
        q.release(c.token)
        assert q.holder is a
        q.release(a.token)
        assert q.holder is b
        q.release(b.token)
        assert q.free
        assert any("took control" in line for line in log)

    asyncio.run(run())


def test_quiet_tickets_lapse_but_not_mid_motion():
    async def run():
        moving = [True]
        q = ControlQueue(is_moving=lambda: moving[0])
        a = q.request("agent", ttl_s=5)
        b = q.request("alice", ttl_s=600)
        a.seen -= 10
        q.expire()
        assert q.holder is a  # still moving: keeps control
        moving[0] = False
        q.expire()
        assert q.holder is b
        with pytest.raises(NotYourTurn, match="not valid any more"):
            q.check(a.token)

    asyncio.run(run())


@pytest.fixture()
def client(tmp_path):
    root = tmp_path / "workspace"
    shutil.copytree(REPO_ROOT / "workspace", root, ignore=shutil.ignore_patterns("perception", "tool_call_history", "__pycache__", "assets", "*.sim.json"))
    app = create_app(Config(sim=True, voice=False, workspace_dir=root, access_key="s3cret"))
    with TestClient(app) as c:
        yield c, app


def gripper(c, token=None, open_=1.0):
    headers = {"X-Control-Token": token} if token else {}
    return c.post("/api/joints", json={"gripper": open_, "source": "test"}, headers=headers).json()


def test_daemon_enforces_control(client):
    c, _ = client
    assert gripper(c)["ok"]  # nobody holds control: one-off commands still work

    a = c.post("/api/control/request", json={"name": "agent"}).json()
    assert a["you"]["state"] == "granted"
    res = gripper(c)
    assert not res["ok"] and res["kind"] == "control" and "agent has control" in res["error"]
    tool = c.post("/api/tools/gripper", json={"args": {"open": 0.5}}).json()
    assert not tool["ok"] and tool["error"] == "control"
    assert c.post("/api/stop").json()["ok"]  # stop is never refused
    assert gripper(c, a["token"])["ok"]

    b = c.post("/api/control/request", json={"name": "alice", "kind": "human"}).json()
    assert b["you"] == {"state": "waiting", "position": 1, "ttl_s": 20.0}
    state = c.get("/api/state").json()["control"]
    assert state["holder"]["name"] == "agent" and [t["name"] for t in state["queue"]] == ["alice"]

    c.post("/api/control/release", headers={"X-Control-Token": a["token"]})
    b = c.post("/api/control/request", json={}, headers={"X-Control-Token": b["token"]}).json()
    assert b["you"]["state"] == "granted"

    t = c.post("/api/control/take", json={"name": "bob"}).json()
    assert t["you"]["state"] == "granted"
    assert c.get("/api/control", headers={"X-Control-Token": b["token"]}).json()["you"]["state"] == "waiting"


def test_other_machines_need_the_key(tmp_path):
    app = create_app(Config(sim=True, voice=False, workspace_dir=tmp_path / "none", access_key="s3cret"))
    with TestClient(app, client=("192.168.1.20", 50000)) as remote:
        assert remote.get("/api/state").status_code == 401
        assert remote.get("/").status_code == 401
        assert remote.get("/api/state", headers={"X-Elrobot-Key": "s3cret"}).status_code == 200
        r = remote.get("/api/meta?key=s3cret")
        assert r.status_code == 200 and "elrobot_key" in r.cookies
        assert remote.get("/api/state").status_code == 200  # the cookie carries it from now on
