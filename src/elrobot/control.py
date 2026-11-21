"""Who may move the arm: one control lease at a time, and a queue of those waiting for it.

Several clients share the arm (the web UI on other laptops, an agent in the workspace, the
MCP shim, scripts). Locking each command is not enough: a pick is many joint commands, and
another client's command slipped in between them is how things get hit. So a client asks
for control, gets a ticket, waits its turn, and holds the arm until it releases it or goes
quiet. Stop is never subject to any of this.

A request without a ticket is still accepted while nobody holds control and nobody waits:
then it runs on its own, as before, which keeps single commands from the UI or the MCP shim
free of ceremony.
"""

from __future__ import annotations

import asyncio
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable

KINDS = ("human", "agent")
MIN_TTL_S, MAX_TTL_S = 5.0, 600.0


@dataclass
class Ticket:
    token: str
    name: str
    kind: str  # "human" or "agent": who to show it as; any client may take over
    ttl_s: float = 20.0  # silence after which the ticket lapses
    since: float = field(default_factory=time.monotonic)  # when it joined the queue / got control
    seen: float = field(default_factory=time.monotonic)  # last sign of life

    def public(self, now: float) -> dict:
        return {"id": self.token[:6], "name": self.name, "kind": self.kind, "for_s": round(now - self.since, 1)}


class NotYourTurn(Exception):
    """A motion request from someone who does not hold control while someone else does."""


class ControlQueue:
    def __init__(self, ttl_s: float = 20.0, on_change: Callable[[str], None] | None = None,
                 is_moving: Callable[[], bool] | None = None):  # fmt: skip
        self.ttl_s = ttl_s
        self.holder: Ticket | None = None
        self.waiting: list[Ticket] = []
        self._on_change = on_change or (lambda text: None)
        self._is_moving = is_moving or (lambda: False)
        self._changed = asyncio.Event()

    # ---- queries

    def find(self, token: str | None) -> Ticket | None:
        if not token:
            return None
        if self.holder is not None and self.holder.token == token:
            return self.holder
        return next((t for t in self.waiting if t.token == token), None)

    def status(self, token: str | None = None) -> dict:
        """The public picture (who holds control, who waits) plus, for a ticket, where it stands."""
        self.expire()
        now = time.monotonic()
        out: dict = {
            "holder": self.holder.public(now) if self.holder else None,
            "queue": [t.public(now) for t in self.waiting],
        }
        if token is not None:
            t = self.find(token)
            if t is None:
                out["you"] = {"state": "none"}
            elif t is self.holder:
                out["you"] = {"state": "granted", "ttl_s": t.ttl_s}
            else:
                out["you"] = {"state": "waiting", "position": self.waiting.index(t) + 1, "ttl_s": t.ttl_s}
        return out

    @property
    def free(self) -> bool:
        self.expire()
        return self.holder is None and not self.waiting

    # ---- changes

    def _ttl(self, ttl_s: float | None) -> float:
        return self.ttl_s if ttl_s is None else min(MAX_TTL_S, max(MIN_TTL_S, float(ttl_s)))

    def request(self, name: str, kind: str = "agent", token: str | None = None, ttl_s: float | None = None) -> Ticket:
        """Join the queue (or get control at once if it is free). Asking again with a ticket
        you already hold just refreshes it. `ttl_s` is how long the ticket survives without a
        sign of life: an agent that thinks between commands needs longer than a UI that pings."""
        self.expire()
        t = self.find(token)
        if t is not None:
            t.seen = time.monotonic()
            return t
        t = Ticket(token=uuid.uuid4().hex, name=(name or "someone")[:40], kind=kind if kind in KINDS else "agent", ttl_s=self._ttl(ttl_s))
        self.waiting.append(t)
        self._advance()
        if t is not self.holder:
            self._notify(f"{t.name} is waiting for control of the arm (position {len(self.waiting)})")
        return t

    def touch(self, token: str | None) -> Ticket | None:
        t = self.find(token)
        if t is not None:
            t.seen = time.monotonic()
        return t

    def release(self, token: str | None) -> bool:
        t = self.find(token)
        if t is None:
            return False
        if t is self.holder:
            self.holder = None
            self._notify(f"{t.name} released control of the arm")
        else:
            self.waiting.remove(t)
            self._notify(f"{t.name} stopped waiting for control")
        self._advance()
        return True

    def take(self, name: str, kind: str = "human", token: str | None = None, ttl_s: float | None = None) -> tuple[Ticket, Ticket | None]:
        """Take control now, ahead of everyone. The previous holder goes to the front of the
        queue, so it gets the arm back afterwards. Returns (new holder, previous holder)."""
        self.expire()
        t = self.find(token)
        if t is not None and t is self.holder:
            t.seen = time.monotonic()
            return t, None
        if t is not None:
            self.waiting.remove(t)
        else:
            t = Ticket(token=uuid.uuid4().hex, name=(name or "someone")[:40], kind=kind if kind in KINDS else "human", ttl_s=self._ttl(ttl_s))
        prev = self.holder
        if prev is not None:
            prev.since = prev.seen = time.monotonic()
            self.waiting.insert(0, prev)
        t.since = t.seen = time.monotonic()
        self.holder = t
        self._notify(f"{t.name} took control of the arm" + (f" from {prev.name}" if prev else ""))
        return t, prev

    def check(self, token: str | None) -> Ticket | None:
        """May a motion request with this ticket (or none) run? Returns the holder's ticket
        when it is theirs, None for an unticketed request on a free arm; raises otherwise."""
        self.expire()
        t = self.touch(token)
        if self.holder is not None and t is self.holder:
            return t
        if t is None and token:
            raise NotYourTurn("that control ticket is not valid any more (released, expired or taken over): ask for control again")
        if self.holder is None and not self.waiting:
            return None  # free arm, one-off command
        if self.holder is None:
            raise NotYourTurn(f"{self.waiting[0].name} is next in line for control of the arm; ask for control to queue up")
        who = self.holder.name
        if t is not None:
            raise NotYourTurn(f"{who} has control of the arm; you are number {self.waiting.index(t) + 1} in the queue")
        raise NotYourTurn(f"{who} has control of the arm. Ask for control (POST /api/control/request) to queue up, or wait")

    def expire(self) -> None:
        """Drop tickets that have gone quiet. The holder does not expire while the arm is moving."""
        now = time.monotonic()
        gone = [t for t in self.waiting if now - t.seen > t.ttl_s]
        for t in gone:
            self.waiting.remove(t)
            self._notify(f"{t.name} stopped answering and left the queue")
        h = self.holder
        if h is not None and now - h.seen > h.ttl_s and not self._is_moving():
            self.holder = None
            self._notify(f"{h.name} went quiet for {h.ttl_s:.0f} s: control released")
        if gone or (h is not None and self.holder is None):
            self._advance()

    async def wait_change(self, timeout: float) -> None:
        try:
            await asyncio.wait_for(self._changed.wait(), timeout)
        except asyncio.TimeoutError:
            pass

    # ---- internals

    def _advance(self) -> None:
        if self.holder is None and self.waiting:
            t = self.waiting.pop(0)
            t.since = t.seen = time.monotonic()
            self.holder = t
            self._notify(f"{t.name} has control of the arm")
        self._changed.set()
        self._changed = asyncio.Event()

    def _notify(self, text: str) -> None:
        self._on_change(text)
        self._changed.set()
        self._changed = asyncio.Event()
