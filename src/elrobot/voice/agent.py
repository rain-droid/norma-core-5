"""The brain: one persistent Claude session that talks and drives the arm.

    brain = Brain(mcp_command=["uv", "run", "--project", root, "elrobot-mcp"])
    await brain.start()
    async for event in brain.turn("dap me up"):
        ...   # TextDelta, TextEnd, ToolStart, ToolEnd, then exactly one TurnEnd
    await brain.interrupt()        # from another task: ends the running turn promptly

Built on the Claude Agent SDK's `ClaudeSDKClient`, which drives the locally
logged-in Claude Code CLI as a subprocess (no API key involved). The session
only ever sees the `elrobot` MCP server's tools: built-in tools are switched
off, other MCP configuration is ignored, and anything not pre-approved is
denied without asking, so nothing can block waiting for a human.

Conversation memory lives in the CLI session. If the CLI process dies the
brain reconnects and resumes that session by id, so memory survives.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import shutil
import time
from collections.abc import AsyncIterator
from dataclasses import dataclass, field
from typing import Any, Callable

from claude_agent_sdk import (
    AssistantMessage,
    ClaudeAgentOptions,
    ClaudeSDKClient,
    ResultMessage,
    SystemMessage,
    TextBlock,
    ToolResultBlock,
    ToolUseBlock,
    UserMessage,
)
from claude_agent_sdk.types import StreamEvent

from . import config

log = logging.getLogger("elrobot.voice.agent")

# How long a turn may keep running after interrupt() before the CLI is
# considered wedged and replaced. An interrupt normally lands in ~10 ms.
_INTERRUPT_GRACE_S = 6.0
_INTERRUPT_NUDGE_S = 1.0


# ---------------------------------------------------------------------------
# Events a turn produces
# ---------------------------------------------------------------------------


@dataclass
class TextDelta:
    """A piece of assistant text, as soon as the model produced it."""

    text: str


@dataclass
class TextEnd:
    """The current stretch of assistant text is complete (a tool call or the end follows)."""


@dataclass
class ToolStart:
    id: str
    name: str  # short name, without the mcp__elrobot__ prefix
    input: dict[str, Any]


@dataclass
class ToolEnd:
    id: str
    name: str
    ok: bool | None  # None when the result does not say
    summary: str
    has_image: bool
    duration_ms: float


@dataclass
class TurnEnd:
    status: str  # "completed" | "interrupted" | "error" | "refused"
    error: str | None = None
    first_text_ms: float | None = None  # prompt sent -> first assistant text
    total_ms: float = 0.0
    api_ms: float | None = None
    steps: int = 0  # model requests in the turn
    cost_usd: float | None = None
    usage: dict[str, Any] = field(default_factory=dict)
    session_id: str | None = None


BrainEvent = TextDelta | TextEnd | ToolStart | ToolEnd | TurnEnd


class _Closed:
    """Marker the reader leaves in the inbox when the CLI connection ends."""

    def __init__(self, error: BaseException | None) -> None:
        self.error = error


class _ConnectionLost(Exception):
    pass


@dataclass
class _Turn:
    started: float
    cancelled: bool = False
    cancelled_at: float = 0.0
    nudges: int = 0
    sent_at: float = 0.0
    first_text_at: float | None = None
    stream_buffer: str = ""  # text delivered as deltas, not yet seen as a whole block
    in_text_block: bool = False
    tools: dict[str, tuple[str, float]] = field(default_factory=dict)
    api_error: str | None = None
    produced: bool = False  # anything reached the caller (so no silent replay)
    result: ResultMessage | None = None


def _short_tool_name(name: str) -> str:
    prefix = f"mcp__{config.MCP_SERVER_NAME}__"
    return name[len(prefix) :] if name.startswith(prefix) else name


def _unwrap_json(text: str) -> Any:
    """Parse a tool result's text, looking through `{"result": "<json>"}` wrappers."""
    value: Any = text
    for _ in range(3):
        if not isinstance(value, str):
            break
        stripped = value.strip()
        if not stripped or stripped[0] not in "{[":
            break
        try:
            value = json.loads(stripped)
        except ValueError:
            break
        if isinstance(value, dict) and set(value) == {"result"}:
            value = value["result"]
    return value


def summarise_tool_result(block: ToolResultBlock) -> tuple[bool | None, str, bool]:
    """(ok, one-line summary, carries an image) for a tool result block.

    Purely for display and state: the model reads the result itself.
    """
    texts: list[str] = []
    has_image = False
    content = block.content
    if isinstance(content, str):
        texts.append(content)
    elif isinstance(content, list):
        for item in content:
            if not isinstance(item, dict):
                continue
            if item.get("type") == "text":
                texts.append(str(item.get("text", "")))
            elif item.get("type") == "image":
                has_image = True
    ok: bool | None = False if block.is_error else None
    summary = ""
    for text in texts:
        payload = _unwrap_json(text)
        if isinstance(payload, dict):
            if isinstance(payload.get("ok"), bool) and ok is None:
                ok = payload["ok"]
            summary = str(payload.get("summary") or payload.get("error") or "")
            if summary or "ok" in payload:
                break
        elif isinstance(payload, str) and not summary:
            summary = payload
    if not summary and texts:
        summary = texts[0]
    return ok, " ".join(summary.split())[:240], has_image


def load_persona() -> str:
    return config.PERSONA_PATH.read_text(encoding="utf-8").strip()


class Brain:
    def __init__(
        self,
        mcp_command: list[str] | None = None,
        *,
        model: str | None = None,
        effort: str | None = None,
        persona: str | None = None,
        cli_path: str | None = None,
        on_log: Callable[[str, str], None] | None = None,
    ) -> None:
        command = list(mcp_command or config.default_mcp_command())
        if not command:
            raise ValueError("mcp_command must not be empty")
        # Resolve now so a later change of PATH or cwd cannot break the spawn.
        if os.sep not in command[0]:
            command[0] = shutil.which(command[0]) or command[0]
        self.mcp_command = command
        self.model = model or config.VOICE_MODEL
        self.effort = effort or config.VOICE_EFFORT
        self.persona = persona if persona is not None else load_persona()
        self.cli_path = cli_path or config.claude_cli_path()
        self._on_log = on_log

        self.session_id: str | None = None
        self.mcp_status: str | None = None  # last reported status of the elrobot server
        self.tools: list[str] = []
        self.connect_ms: float | None = None

        self._client: ClaudeSDKClient | None = None
        self._inbox: asyncio.Queue[Any] = asyncio.Queue()
        self._reader: asyncio.Task[None] | None = None
        self._alive = False
        self._outstanding = 0  # prompts written whose result has not been read yet
        self._conn_lock = asyncio.Lock()
        self._turn_lock = asyncio.Lock()
        self._current: _Turn | None = None
        self._background: set[asyncio.Task[Any]] = set()
        self._stopped = False

    # -- lifecycle --------------------------------------------------------

    async def start(self) -> None:
        """Spawn the CLI and wait (briefly) for the robot's MCP server to connect."""
        self._stopped = False
        await self._ensure_connected()
        deadline = time.perf_counter() + 20.0
        while self.mcp_status in (None, "pending") and time.perf_counter() < deadline:
            await asyncio.sleep(0.05)

    async def stop(self) -> None:
        self._stopped = True
        for task in list(self._background):
            task.cancel()
        async with self._conn_lock:
            await self._drop_connection()

    @property
    def busy(self) -> bool:
        return self._current is not None

    @property
    def connected(self) -> bool:
        return self._alive and self._client is not None

    # -- one turn ---------------------------------------------------------

    async def turn(self, text: str) -> AsyncIterator[BrainEvent]:
        """Run one user turn. Yields events as they happen and ends with a TurnEnd.

        Turns run one at a time; a second call waits for the first to finish
        (callers that want barge-in call `interrupt()` first).
        """
        async with self._turn_lock:
            turn = _Turn(started=time.perf_counter())
            self._current = turn
            try:
                async for event in self._run(turn, text):
                    yield event
            finally:
                self._current = None

    async def interrupt(self) -> bool:
        """Stop the running turn. Returns False if there was nothing to stop."""
        turn = self._current
        if turn is None or turn.result is not None:
            return False
        if not turn.cancelled:
            turn.cancelled = True
            turn.cancelled_at = time.perf_counter()
        await self._send_interrupt()
        return True

    async def _send_interrupt(self) -> None:
        client = self._client
        if client is None or not self._alive:
            return
        try:
            await asyncio.wait_for(client.interrupt(), timeout=2.0)
        except Exception as exc:  # a dead connection is handled by the turn loop
            log.debug("interrupt request failed: %s", exc)

    async def _run(self, turn: _Turn, text: str) -> AsyncIterator[BrainEvent]:
        attempt = 0
        while True:
            attempt += 1
            try:
                await self._ensure_connected()
                await self._settle()
                if turn.cancelled:
                    yield self._end(turn, "interrupted")
                    return
                client, inbox = self._client, self._inbox
                assert client is not None
                turn.sent_at = time.perf_counter()
                await client.query(text)
                self._outstanding += 1
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._log("warn", f"brain connection failed ({type(exc).__name__}: {exc})")
                async with self._conn_lock:
                    await self._drop_connection()
                if attempt >= 2:
                    yield self._end(turn, "error", f"could not reach Claude: {exc}")
                    return
                continue

            try:
                async for event in self._pump(turn, client, inbox):
                    turn.produced = True
                    yield event
            except _ConnectionLost as exc:
                self._log("warn", f"brain connection lost mid-turn ({exc}); reconnecting")
                async with self._conn_lock:
                    await self._drop_connection()
                # Replaying is only safe when nothing was said or done yet:
                # replaying after a tool call could run a motion twice.
                if not turn.produced and not turn.cancelled and attempt < 2:
                    continue
                self._spawn(self._reconnect_quietly())
                status = "interrupted" if turn.cancelled else "error"
                yield self._end(turn, status, None if turn.cancelled else f"connection lost: {exc}")
                return

            result = turn.result
            assert result is not None
            reason = result.terminal_reason or ""
            if turn.cancelled or reason.startswith("aborted"):
                yield self._end(turn, "interrupted")
            elif result.stop_reason == "refusal":
                yield self._end(turn, "refused", "the model declined this request")
            elif result.is_error or turn.api_error:
                detail = turn.api_error or "; ".join(result.errors or []) or result.subtype
                if result.api_error_status:
                    detail = f"{detail} (HTTP {result.api_error_status})"
                yield self._end(turn, "error", detail)
            else:
                yield self._end(turn, "completed")
            return

    async def _pump(
        self, turn: _Turn, client: ClaudeSDKClient, inbox: asyncio.Queue[Any]
    ) -> AsyncIterator[BrainEvent]:
        """Translate CLI messages into events until this turn's result arrives."""
        while True:
            timeout = _INTERRUPT_NUDGE_S if turn.cancelled else config.VOICE_STALL_TIMEOUT
            try:
                message = await asyncio.wait_for(inbox.get(), timeout=timeout)
            except asyncio.TimeoutError:
                message = None
                if not turn.cancelled:
                    # Nothing from the CLI for a long time: treat it as hung.
                    self._log("warn", "brain turn stalled; interrupting it")
                    turn.cancelled = True
                    turn.cancelled_at = time.perf_counter()
                    turn.api_error = "turn stalled"
                    await self._send_interrupt()
                    continue
            if turn.cancelled:
                waited = time.perf_counter() - turn.cancelled_at
                if waited > _INTERRUPT_GRACE_S:
                    raise _ConnectionLost("interrupt was not acknowledged")
                if waited > _INTERRUPT_NUDGE_S * (turn.nudges + 1):
                    # The first interrupt can race the prompt and be ignored.
                    turn.nudges += 1
                    await self._send_interrupt()
            if message is None:
                continue
            if isinstance(message, _Closed):
                raise _ConnectionLost(str(message.error or "CLI process exited"))
            if isinstance(message, ResultMessage):
                self._outstanding = max(0, self._outstanding - 1)
                self.session_id = message.session_id or self.session_id
                turn.result = message
                return
            for event in self._translate(turn, message):
                yield event

    def _translate(self, turn: _Turn, message: Any) -> list[BrainEvent]:
        out: list[BrainEvent] = []
        if isinstance(message, StreamEvent):
            if message.parent_tool_use_id:
                return out
            event = message.event
            kind = event.get("type")
            if kind == "content_block_start":
                block = event.get("content_block") or {}
                turn.in_text_block = block.get("type") == "text"
                if turn.in_text_block and block.get("text"):
                    out.append(self._text(turn, block["text"]))
            elif kind == "content_block_delta":
                delta = event.get("delta") or {}
                if delta.get("type") == "text_delta" and delta.get("text"):
                    out.append(self._text(turn, delta["text"]))
            elif kind == "content_block_stop":
                if turn.in_text_block:
                    turn.in_text_block = False
                    out.append(TextEnd())
        elif isinstance(message, AssistantMessage):
            if message.parent_tool_use_id:
                return out
            if message.error:
                # The CLI reports API failures as assistant text ("API Error: ...").
                # That is for the log, not for the loudspeaker.
                turn.api_error = str(message.error)
            for block in message.content:
                if isinstance(block, TextBlock):
                    if message.error:
                        self._log("warn", f"brain API error: {block.text[:200]}")
                        continue
                    rest = self._unstreamed(turn, block.text)
                    if rest:
                        out.append(self._text(turn, rest, streamed=False))
                    out.append(TextEnd())
                elif isinstance(block, ToolUseBlock):
                    name = _short_tool_name(block.name)
                    turn.tools[block.id] = (name, time.perf_counter())
                    out.append(ToolStart(id=block.id, name=name, input=dict(block.input or {})))
        elif isinstance(message, UserMessage):
            if message.parent_tool_use_id or not isinstance(message.content, list):
                return out
            for block in message.content:
                if isinstance(block, ToolResultBlock):
                    name, started = turn.tools.pop(
                        block.tool_use_id, ("tool", time.perf_counter())
                    )
                    ok, summary, has_image = summarise_tool_result(block)
                    out.append(
                        ToolEnd(
                            id=block.tool_use_id,
                            name=name,
                            ok=ok,
                            summary=summary,
                            has_image=has_image,
                            duration_ms=(time.perf_counter() - started) * 1000.0,
                        )
                    )
        elif isinstance(message, SystemMessage) and message.subtype == "init":
            self._note_init(message.data)
        return out

    def _text(self, turn: _Turn, text: str, streamed: bool = True) -> TextDelta:
        if turn.first_text_at is None:
            turn.first_text_at = time.perf_counter()
        if streamed:
            turn.stream_buffer += text
        return TextDelta(text)

    def _unstreamed(self, turn: _Turn, block_text: str) -> str:
        """The part of a finished text block that never arrived as deltas."""
        streamed, turn.stream_buffer = turn.stream_buffer, ""
        if not streamed:
            return block_text
        if block_text.startswith(streamed):
            return block_text[len(streamed) :]
        if streamed.startswith(block_text):
            turn.stream_buffer = streamed[len(block_text) :]
            return ""
        return ""  # deltas and block disagree; what was streamed has been spoken

    def _end(self, turn: _Turn, status: str, error: str | None = None) -> TurnEnd:
        now = time.perf_counter()
        result = turn.result
        first = None
        if turn.first_text_at is not None and turn.sent_at:
            first = (turn.first_text_at - turn.sent_at) * 1000.0
        return TurnEnd(
            status=status,
            error=error,
            first_text_ms=first,
            total_ms=(now - turn.started) * 1000.0,
            api_ms=float(result.duration_api_ms) if result else None,
            steps=result.num_turns if result else 0,
            cost_usd=result.total_cost_usd if result else None,
            usage=dict(result.usage or {}) if result else {},
            session_id=self.session_id,
        )

    # -- connection -------------------------------------------------------

    def _options(self, resume: str | None) -> ClaudeAgentOptions:
        server = config.MCP_SERVER_NAME
        mcp_env = {k: v for k, v in os.environ.items() if k.startswith("ELROBOT_")}
        mcp_env["ELROBOT_SOURCE"] = "voice"  # lets the MCP server tag its tool calls
        cli_env = {
            "DISABLE_AUTOUPDATER": "1",
            # One motion can take a while; the CLI's per-call limit must not be
            # shorter than what we are prepared to wait for ourselves.
            "MCP_TOOL_TIMEOUT": os.environ.get(
                "MCP_TOOL_TIMEOUT", str(int(config.VOICE_STALL_TIMEOUT * 1000))
            ),
            "MCP_TIMEOUT": os.environ.get("MCP_TIMEOUT", "60000"),
        }
        fallback = config.VOICE_FALLBACK_MODEL
        if fallback.lower() in ("", "none", "off") or fallback == self.model:
            fallback = None
        return ClaudeAgentOptions(
            # No built-in tools at all: no files, no shell, no web.
            tools=[],
            # Only the robot's MCP server, whatever else is configured on this machine.
            mcp_servers={
                server: {
                    "type": "stdio",
                    "command": self.mcp_command[0],
                    "args": self.mcp_command[1:],
                    "env": mcp_env,
                }
            },
            strict_mcp_config=True,
            # Every tool of that server is pre-approved; anything else is denied
            # outright instead of prompting, because nobody is there to answer.
            allowed_tools=[f"mcp__{server}"],
            permission_mode="dontAsk",
            # Ignore user/project settings (hooks, CLAUDE.md, a default model or
            # effort level): the voice agent must behave the same everywhere.
            setting_sources=[],
            extra_args={"disable-slash-commands": None},
            system_prompt=self.persona,
            model=self.model,
            fallback_model=fallback,
            effort=self.effort,  # type: ignore[arg-type]
            include_partial_messages=True,  # token-level text, so speech starts early
            max_turns=config.VOICE_MAX_STEPS,
            cwd=str(config.SESSION_DIR),
            cli_path=self.cli_path,
            resume=resume,
            env=cli_env,
            stderr=self._on_stderr,
        )

    async def _ensure_connected(self) -> None:
        if self._stopped:
            raise RuntimeError("brain is stopped")
        async with self._conn_lock:
            if self._alive and self._client is not None:
                return
            await self._drop_connection()
            config.SESSION_DIR.mkdir(parents=True, exist_ok=True)
            started = time.perf_counter()
            resume = self.session_id
            try:
                client = await self._open(resume)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if not resume:
                    raise
                # The saved session may be gone or unreadable; a fresh one beats none.
                self._log("warn", f"could not resume session ({exc}); starting a new one")
                self.session_id = None
                client = await self._open(None)
            self._client = client
            self._inbox = asyncio.Queue()
            self._outstanding = 0
            self._alive = True
            self._reader = asyncio.create_task(self._read(client, self._inbox))
            self.connect_ms = (time.perf_counter() - started) * 1000.0
            self._spawn(self._check_mcp(client))
            if resume and self.session_id:
                self._log("info", "brain reconnected and resumed the conversation")

    async def _open(self, resume: str | None) -> ClaudeSDKClient:
        client = ClaudeSDKClient(self._options(resume))
        await client.connect()
        return client

    async def _drop_connection(self) -> None:
        """Tear the current CLI process down. Caller holds the connection lock."""
        self._alive = False
        reader, self._reader = self._reader, None
        client, self._client = self._client, None
        if reader is not None:
            reader.cancel()
            self._inbox.put_nowait(_Closed(None))  # wake a turn still waiting on this connection
        if client is not None:
            try:
                await asyncio.wait_for(client.disconnect(), timeout=25.0)
            except Exception as exc:
                log.debug("disconnect failed: %s", exc)

    async def _read(self, client: ClaudeSDKClient, inbox: asyncio.Queue[Any]) -> None:
        """Move every message from the CLI into the inbox, for as long as it lives."""
        error: BaseException | None = None
        while True:
            try:
                async for message in client.receive_messages():
                    inbox.put_nowait(message)
                break
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if type(exc).__name__ == "MessageParseError":
                    log.debug("skipping an unparseable CLI message: %s", exc)
                    continue  # one odd message must not end the session
                error = exc
                break
        if self._client is client:
            self._alive = False
            inbox.put_nowait(_Closed(error))
            if self._current is None and not self._stopped:
                # Died while idle: reconnect now so the next turn does not pay for it.
                self._log("warn", f"brain CLI process ended ({error or 'exit'}); reconnecting")
                self._spawn(self._reconnect_quietly())

    async def _reconnect_quietly(self) -> None:
        try:
            await self._ensure_connected()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self._log("error", f"brain reconnect failed: {exc}")

    async def _settle(self) -> None:
        """Make sure the CLI is idle and the inbox is clean before a new prompt.

        A turn whose consumer went away can leave its result unread; sending a
        new prompt on top would let that stale result end the new turn early.
        """
        inbox = self._inbox
        deadline = time.perf_counter() + _INTERRUPT_GRACE_S
        if self._outstanding > 0:
            await self._send_interrupt()
        while self._outstanding > 0:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                raise _ConnectionLost("previous turn never finished")
            try:
                message = await asyncio.wait_for(inbox.get(), timeout=remaining)
            except asyncio.TimeoutError:
                raise _ConnectionLost("previous turn never finished") from None
            if isinstance(message, _Closed):
                raise _ConnectionLost(str(message.error or "CLI process exited"))
            if isinstance(message, ResultMessage):
                self._outstanding -= 1
        while not inbox.empty():
            message = inbox.get_nowait()
            if isinstance(message, _Closed):
                raise _ConnectionLost(str(message.error or "CLI process exited"))

    async def _check_mcp(self, client: ClaudeSDKClient) -> None:
        """Follow the robot MCP server until it is up, and say so if it is not."""
        deadline = time.perf_counter() + 60.0
        while self._client is client and time.perf_counter() < deadline:
            try:
                status = await asyncio.wait_for(client.get_mcp_status(), timeout=30.0)
            except Exception as exc:
                log.debug("mcp status unavailable: %s", exc)
                return
            server = next(
                (s for s in status.get("mcpServers", []) if s.get("name") == config.MCP_SERVER_NAME),
                None,
            )
            if server is None:
                self.mcp_status = "missing"
                return
            state = server.get("status")
            if state == "pending":
                self.mcp_status = state
                await asyncio.sleep(0.1)
                continue
            self.tools = [_short_tool_name(t.get("name", "")) for t in server.get("tools") or []]
            self.mcp_status = state
            if state != "connected":
                self._log(
                    "error",
                    f"robot MCP server is {state}: {server.get('error') or 'no detail'} "
                    f"(command: {' '.join(self.mcp_command)})",
                )
            return

    def _note_init(self, data: dict[str, Any]) -> None:
        self.session_id = data.get("session_id") or self.session_id
        for server in data.get("mcp_servers") or []:
            if server.get("name") == config.MCP_SERVER_NAME:
                status = server.get("status")
                if status != self.mcp_status and status in ("failed", "needs-auth"):
                    self._log("error", f"robot MCP server is {status}; trying to reconnect it")
                    client = self._client
                    if client is not None:
                        self._spawn(self._revive_mcp(client))
                self.mcp_status = status
        tools = data.get("tools")
        if isinstance(tools, list):
            self.tools = [_short_tool_name(str(t)) for t in tools]

    async def _revive_mcp(self, client: ClaudeSDKClient) -> None:
        try:
            await asyncio.wait_for(
                client.reconnect_mcp_server(config.MCP_SERVER_NAME), timeout=60.0
            )
            self.mcp_status = "connected"
            self._log("info", "robot MCP server reconnected")
        except Exception as exc:
            self._log("error", f"robot MCP server reconnect failed: {exc}")

    # -- helpers ----------------------------------------------------------

    def _spawn(self, coro: Any) -> None:
        task = asyncio.ensure_future(coro)
        self._background.add(task)
        task.add_done_callback(self._background.discard)

    def _on_stderr(self, line: str) -> None:
        log.debug("claude: %s", line.rstrip())

    def _log(self, level: str, text: str) -> None:
        getattr(log, {"warn": "warning"}.get(level, level), log.info)(text)
        if self._on_log is not None:
            try:
                self._on_log(level, text)
            except Exception:
                log.exception("log callback failed")
