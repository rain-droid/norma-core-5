"""The integration surface the daemon mounts: `VoiceService` and `create_router`.

    service = VoiceService(emit, mcp_command=[...], stop_robot=daemon.stop)
    await service.start()
    app.include_router(create_router(service))

`emit` receives event dicts without `type`/`id`/`t` (the daemon adds those).
Three kinds come out of here, as described in docs/API.md:

    {"kind": "chat", "role": "user"|"assistant", "text", "turn_id", "final", "via"}
    {"kind": "speech", "turn_id", "seq", "text", "url", "duration_ms"}
    {"kind": "voice_state", "state": "idle"|"transcribing"|"thinking"|"speaking"|"acting"}

plus `{"kind": "log", "level", "text"}` when something goes wrong.
"""

from __future__ import annotations

import asyncio
import difflib
import logging
import re
import time
import uuid
from collections import deque
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from fastapi import APIRouter, File, HTTPException, UploadFile
from fastapi.responses import FileResponse
from pydantic import BaseModel

from . import config
from .agent import Brain, TextDelta, TextEnd, ToolEnd, ToolStart, TurnEnd
from .stt import STT, AudioDecodeError, Transcript
from .tts import TTS, SentenceChunker, speakable, detect_language

log = logging.getLogger("elrobot.voice.service")

# Lines spoken without the model, for the moments the model is the problem.
LINE_STOPPED = "Stopped."
# Said when the model starts a tool call without a word (it is asked to
# acknowledge first, and nearly always does). Rotated so it does not repeat.
FILLERS = ("On it.", "One sec.", "Bet.", "Got you.")
LINE_BRAIN_ERROR = "Ugh, my brain glitched. Say that again?"
LINE_REFUSED = "Nah, I can't help with that one."

# --- stop commands ---------------------------------------------------------
# Checked on the raw text before the model is involved: when someone says
# "stop", the arm has to stop now, not after a round trip to Claude.
_STOP_WORDS = frozenset({"stop", "freeze", "halt", "abort"})
_STOP_PHRASES = (("hold", "up"), ("hold", "it"), ("hold", "on"), ("cut", "it", "out"))
_STOP_FILLER = frozenset(
    "hey yo ok okay no nope whoa woah wait please now just right there it that "
    "moving everything robot arm bro dude man stopping".split()
)


def is_stop_command(text: str) -> bool:
    """True when the utterance is nothing but a stop command.

    "stop", "stop stop", "freeze!", "hold up", "whoa, stop it" all count.
    "stop by the cup" or "don't stop" do not: anything with other content goes
    to the model, which can stop the arm itself and deal with the rest.
    """
    words = re.sub(r"[^a-z\s]", " ", text.lower().replace("'", "")).split()
    if not words or len(words) > 6:
        return False
    found = False
    i = 0
    while i < len(words):
        phrase = next((p for p in _STOP_PHRASES if tuple(words[i : i + len(p)]) == p), None)
        if phrase:
            found = True
            i += len(phrase)
        elif words[i] in _STOP_WORDS:
            found = True
            i += 1
        elif words[i] in _STOP_FILLER:
            i += 1
        else:
            return False
    return found


def _words(text: str) -> str:
    return " ".join(re.sub(r"[^a-z0-9\s]", "", text.lower().replace("'", "")).split())


@dataclass
class _Turn:
    id: str
    via: str
    text: str
    started: float
    task: asyncio.Task[None] | None = None
    cancelled: bool = False
    done: bool = False
    brain_done: bool = False
    acting: int = 0  # tool calls in flight
    blocks: list[str] = field(default_factory=lambda: [""])
    seq: int = 0
    speech: asyncio.Queue[str | None] = field(default_factory=asyncio.Queue)
    # Sentences being synthesized on a guess, before the splitter has confirmed them.
    early: dict[str, asyncio.Task[Any]] = field(default_factory=dict)
    last_interrupt: float = 0.0

    def full_text(self) -> str:
        return " ".join(block.strip() for block in self.blocks if block.strip())


class VoiceService:
    def __init__(
        self,
        emit: Callable[[dict], None],
        *,
        mcp_command: list[str] | None = None,
        stop_robot: Callable[[], Awaitable[None]] | None = None,
    ) -> None:
        self._emit_raw = emit
        self._stop_robot = stop_robot
        self.brain = Brain(mcp_command, on_log=self._log_event)
        self.stt = STT()
        self.tts = TTS()
        # Optional local observer for development (the REPL sets it). Receives
        # dicts about tool calls, timings and rejected audio. Nothing here is
        # sent to the daemon.
        self.trace: Callable[[dict], None] | None = None

        self.state = "idle"
        self.warm_ms: dict[str, float] = {}
        self._turn: _Turn | None = None
        self._gate = asyncio.Lock()  # one "replace the running turn" at a time
        self._transcribing = 0
        self._speaking_until = 0.0  # monotonic time the emitted clips finish playing
        self._state_timer: asyncio.TimerHandle | None = None
        self._notes: list[str] = []  # context to hand the model with the next prompt
        self._filler_index = 0
        # What was said recently, to recognise the microphone picking it back up.
        self._recent_speech: deque[tuple[float, str]] = deque(maxlen=12)
        self._warm: dict[str, asyncio.Task[None]] = {}
        self._tasks: set[asyncio.Task[Any]] = set()
        self._started = False

    # -- lifecycle --------------------------------------------------------

    async def start(self) -> None:
        """Kick off model warm-up and the brain connection, then return.

        Nothing heavy runs on the event loop: Whisper and Kokoro load in their
        own threads and the brain is a subprocess. Requests that arrive before
        warm-up finishes simply wait for the part they need. `await ready()`
        if you want to block until everything is warm.
        """
        if self._started:
            return
        self._started = True
        self._warm = {
            "brain": asyncio.create_task(self._warm_one("brain", self.brain.start)),
            "stt": asyncio.create_task(self._warm_one("stt", self.stt.awarm)),
            "tts": asyncio.create_task(self._warm_one("tts", self.tts.awarm)),
        }

    async def ready(self) -> dict[str, float]:
        """Wait for warm-up; returns milliseconds each part took."""
        if self._warm:
            await asyncio.gather(*self._warm.values())
        return dict(self.warm_ms)

    async def stop(self) -> None:
        self._started = False
        turn = self._turn
        if turn is not None and not turn.done:
            await self._cancel(turn)
        pending = [t for t in (*self._warm.values(), *self._tasks) if not t.done()]
        if turn is not None and turn.task is not None and not turn.task.done():
            pending.append(turn.task)
        for task in pending:
            task.cancel()
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)
        if self._state_timer is not None:
            self._state_timer.cancel()
        await self.brain.stop()
        self.stt.close()
        self.tts.close()

    async def _warm_one(self, name: str, fn: Callable[[], Awaitable[Any]]) -> None:
        started = time.perf_counter()
        try:
            await fn()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            log.exception("%s warm-up failed", name)
            self._log_event("error", f"voice: {name} failed to start: {exc}")
        self.warm_ms[name] = (time.perf_counter() - started) * 1000.0
        self._trace({"kind": "warm", "part": name, "ms": self.warm_ms[name]})

    # -- input ------------------------------------------------------------

    async def handle_text(self, text: str, via: str = "text", language: str | None = None) -> str:
        """Start a turn for `text` and return its id at once.

        A turn already running is interrupted first (barge-in). A bare stop
        command stops everything instead of starting a turn.
        """
        text = " ".join((text or "").split())
        if not text:
            raise ValueError("text is empty")
        # Replies are voiced in the language the person used: what speech to text heard, or what
        # the typed words look like; a message that does not say keeps the language so far.
        if language in config.LANGUAGES:
            self.tts.language = language
        else:
            self.tts.language = detect_language(text, self.tts.language)
        turn_id = uuid.uuid4().hex[:12]
        self._emit(
            {"kind": "chat", "role": "user", "text": text, "turn_id": turn_id,
             "final": True, "via": via}
        )  # fmt: skip

        if is_stop_command(text):
            await self.interrupt()
            self._notes.append(
                f'The person said "{text}" a moment ago, so the arm was halted where it was; '
                "whatever it was doing may be half done."
            )
            self._spawn(self._say(turn_id, LINE_STOPPED, via))
            return turn_id

        async with self._gate:
            previous = self._turn
            if previous is not None and not previous.done:
                # Only halt the arm if it is actually mid-motion for the old turn;
                # otherwise the new turn's first motion would be refused as "busy".
                await self._cancel(previous, stop_robot=previous.acting > 0)
            turn = _Turn(id=turn_id, via=via, text=text, started=time.perf_counter())
            self._turn = turn
            self._speaking_until = 0.0  # the person spoke over whatever was still playing
            turn.task = asyncio.create_task(self._run_turn(turn, previous))
        self._refresh_state()
        return turn_id

    async def handle_audio(self, data: bytes, mime: str | None = None) -> tuple[str, str | None]:
        """Transcribe an utterance and treat it like typed text with via="voice".

        Returns ("", None) for silence, noise or undecodable audio; no turn starts.
        """
        self._transcribing += 1
        self._refresh_state()
        result: Transcript | None = None
        try:
            result = await self.stt.atranscribe(data, mime)
        except AudioDecodeError as exc:
            self._log_event("warn", f"voice: {exc}")
        except Exception as exc:
            log.exception("transcription failed")
            self._log_event("error", f"voice: transcription failed: {exc}")
        finally:
            self._transcribing -= 1
        if result is not None:
            self._trace(
                {"kind": "stt", "text": result.text, "rejected": result.rejected,
                 "raw_text": result.raw_text, "audio_s": result.audio_s,
                 "decode_ms": result.decode_ms, "infer_ms": result.infer_ms,
                 "detail": result.detail}
            )  # fmt: skip
        if result is not None and result.text and self._is_own_voice(result.text):
            self._trace({"kind": "stt", "text": "", "rejected": "echo", "raw_text": result.text,
                         "audio_s": result.audio_s, "decode_ms": 0.0, "infer_ms": 0.0, "detail": {}})  # fmt: skip
            result = None
        if result is None or not result.text:
            self._refresh_state()
            return "", None
        turn_id = await self.handle_text(result.text, via="voice", language=result.language)
        return result.text, turn_id

    def _is_own_voice(self, transcript: str) -> bool:
        """True when the "utterance" is the robot's own speech coming back through the mic.

        Without this an open microphone next to the loudspeaker makes the agent
        answer itself forever. Only multi-word transcripts that closely match
        something said in the last few seconds are dropped.
        """
        heard = _words(transcript)
        if len(heard.split()) < 2:
            return False
        now = time.monotonic()
        recent = [text for ended, text in self._recent_speech if now - ended < 4.0]
        if not recent:
            return False
        for candidate in (*recent, " ".join(recent)):
            if len(heard.split()) >= 3 and heard in candidate:
                return True
            if difflib.SequenceMatcher(None, heard, candidate).ratio() >= 0.85:
                return True
        return False

    async def interrupt(self) -> None:
        """Stop the arm, stop the turn, drop speech that has not been sent yet."""
        # The robot first, and not behind anything else that could be slow.
        halt = asyncio.ensure_future(self._halt_robot())
        turn = self._turn
        if turn is not None and not turn.done:
            await self._cancel(turn, stop_robot=False)
        await halt
        self._refresh_state()

    def audio_path(self, clip_id: str) -> Path | None:
        return self.tts.audio_path(clip_id)

    async def wait_idle(self, timeout: float | None = None) -> bool:
        """Wait until no turn is running and the last clip has (by estimate) played out."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            turn = self._turn
            busy = (
                (turn is not None and not turn.done)
                or bool(self._tasks)
                or self._transcribing > 0
                or time.monotonic() < self._speaking_until
            )
            if not busy:
                return True
            if deadline is not None and time.monotonic() >= deadline:
                return False
            await asyncio.sleep(0.03)

    # -- a turn -----------------------------------------------------------

    async def _run_turn(self, turn: _Turn, previous: _Turn | None) -> None:
        speaker = asyncio.create_task(self._speak_queue(turn))
        try:
            if previous is not None and previous.task is not None and not previous.task.done():
                # Let the interrupted turn close its transcript before this one opens.
                await asyncio.wait([previous.task], timeout=10.0)
            end: TurnEnd | None = None
            if not turn.cancelled:
                chunker = SentenceChunker()
                async for event in self.brain.turn(self._compose_prompt(turn.text)):
                    if isinstance(event, TurnEnd):
                        end = event
                        continue  # the stream ends right after this
                    if turn.cancelled:
                        await self._insist_on_interrupt(turn)
                        continue
                    if isinstance(event, TextDelta):
                        turn.blocks[-1] += event.text
                        self._emit_assistant(turn, final=False)
                        for sentence in chunker.feed(event.text):
                            self._queue_speech(turn, sentence)
                        self._synthesize_early(turn, chunker.peek())
                    elif isinstance(event, TextEnd):
                        for sentence in chunker.flush():
                            self._queue_speech(turn, sentence)
                        if turn.blocks[-1].strip():
                            turn.blocks.append("")
                    elif isinstance(event, ToolStart):
                        if config.VOICE_FILLER and not turn.full_text() and turn.seq == 0:
                            self._fill_silence(turn)
                        turn.acting += 1
                        self._refresh_state()
                        self._trace(
                            {"kind": "tool_start", "turn_id": turn.id, "tool": event.name,
                             "args": event.input}
                        )  # fmt: skip
                    elif isinstance(event, ToolEnd):
                        turn.acting = max(0, turn.acting - 1)
                        self._refresh_state()
                        self._trace(
                            {"kind": "tool_end", "turn_id": turn.id, "tool": event.name,
                             "ok": event.ok, "summary": event.summary,
                             "image": event.has_image, "duration_ms": event.duration_ms}
                        )  # fmt: skip
                if not turn.cancelled:
                    for sentence in chunker.flush():
                        self._queue_speech(turn, sentence)
            turn.acting = 0
            turn.brain_done = True
            if end is not None:
                self._trace(
                    {"kind": "turn_end", "turn_id": turn.id, "status": end.status,
                     "error": end.error, "first_text_ms": end.first_text_ms,
                     "total_ms": end.total_ms, "steps": end.steps, "cost_usd": end.cost_usd}
                )  # fmt: skip
                if not turn.cancelled and end.status in ("error", "refused"):
                    self._log_event("error", f"voice: turn {end.status}: {end.error}")
                    line = LINE_REFUSED if end.status == "refused" else LINE_BRAIN_ERROR
                    turn.blocks.append(line)
                    self._queue_speech(turn, line)
            # Every turn closes with exactly one final assistant event, even an
            # interrupted one (its text is whatever was said before the cut).
            self._emit_assistant(turn, final=True)
            turn.speech.put_nowait(None)
            self._refresh_state()
            await speaker
        except asyncio.CancelledError:
            speaker.cancel()
            raise
        except Exception as exc:
            log.exception("voice turn crashed")
            self._log_event("error", f"voice: turn crashed: {exc}")
            speaker.cancel()
        finally:
            turn.acting = 0
            turn.brain_done = True
            turn.done = True
            self._refresh_state()

    def _fill_silence(self, turn: _Turn) -> None:
        """The arm is about to move and nothing has been said: say something short."""
        line = FILLERS[self._filler_index % len(FILLERS)]
        self._filler_index += 1
        turn.blocks[-1] = line
        turn.blocks.append("")
        self._emit_assistant(turn, final=False)
        self._queue_speech(turn, line)

    def _compose_prompt(self, text: str) -> str:
        if not self._notes:
            return text
        notes, self._notes = " ".join(self._notes), []
        return f"[{notes}]\n\n{text}"

    async def _insist_on_interrupt(self, turn: _Turn) -> None:
        """A cancelled turn is still producing: make sure the brain knows."""
        now = time.perf_counter()
        if now - turn.last_interrupt > 0.5:
            turn.last_interrupt = now
            await self.brain.interrupt()

    async def _cancel(self, turn: _Turn, stop_robot: bool = False) -> None:
        turn.cancelled = True
        turn.last_interrupt = time.perf_counter()
        self._speaking_until = 0.0
        jobs: list[Awaitable[Any]] = [self.brain.interrupt()]
        if stop_robot:
            jobs.append(self._halt_robot())
        await asyncio.gather(*jobs, return_exceptions=True)

    async def _halt_robot(self) -> None:
        if self._stop_robot is None:
            return
        started = time.perf_counter()
        try:
            await self._stop_robot()
        except Exception as exc:
            log.exception("stop_robot failed")
            self._log_event("error", f"voice: could not stop the robot: {exc}")
        self._trace({"kind": "robot_stop", "ms": (time.perf_counter() - started) * 1000.0})

    # -- speech -----------------------------------------------------------

    def _queue_speech(self, turn: _Turn, sentence: str) -> None:
        text = speakable(sentence)
        if text and not turn.cancelled:
            turn.speech.put_nowait(text)

    def _synthesize_early(self, turn: _Turn, guess: str | None) -> None:
        """Start on a sentence that looks finished before the splitter is sure.

        The model's text often pauses right after the acknowledgement while it
        writes the tool call; waiting for that to confirm the sentence would
        add a few hundred milliseconds before the first word. The clip is only
        used if exactly this sentence is released afterwards.
        """
        text = speakable(guess) if guess else ""
        if not text or turn.cancelled or text in turn.early:
            return
        self._drop_early(turn)  # at most one guess at a time
        turn.early[text] = asyncio.ensure_future(self.tts.asynthesize(text))

    def _drop_early(self, turn: _Turn) -> None:
        for task in turn.early.values():
            task.add_done_callback(_discard_clip)
            task.cancel()
        turn.early.clear()

    async def _speak_queue(self, turn: _Turn) -> None:
        """Synthesize this turn's sentences in order; one `speech` event each."""
        try:
            while True:
                text = await turn.speech.get()
                if text is None or turn.cancelled:
                    return
                try:
                    early = turn.early.pop(text, None)
                    clip = await (early if early is not None else self.tts.asynthesize(text))
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    log.exception("speech synthesis failed")
                    self._log_event("error", f"voice: could not synthesize speech: {exc}")
                    continue
                if turn.cancelled:
                    clip.path.unlink(missing_ok=True)  # interrupted while synthesizing
                    return
                self._emit_speech(turn.id, turn.seq, clip)
                turn.seq += 1
        finally:
            self._drop_early(turn)

    def _emit_speech(self, turn_id: str, seq: int, clip: Any) -> None:
        now = time.monotonic()
        # The browser plays clips back to back, so estimate when it will be done.
        self._speaking_until = max(now, self._speaking_until) + clip.duration
        self._recent_speech.append((self._speaking_until, _words(clip.text)))
        self._emit(
            {"kind": "speech", "turn_id": turn_id, "seq": seq, "text": clip.text,
             "url": f"/api/voice/audio/{clip.id}.wav", "duration_ms": clip.duration_ms}
        )  # fmt: skip
        self._trace(
            {"kind": "tts", "turn_id": turn_id, "seq": seq, "provider": clip.provider,
             "synth_ms": clip.synth_ms, "duration_ms": clip.duration_ms, "path": str(clip.path)}
        )  # fmt: skip
        self._refresh_state()

    async def _say(self, turn_id: str, line: str, via: str) -> None:
        """Speak a fixed line outside any model turn."""
        self._emit(
            {"kind": "chat", "role": "assistant", "text": line, "turn_id": turn_id,
             "final": True, "via": via}
        )  # fmt: skip
        try:
            clip = await self.tts.asynthesize(line)
        except Exception as exc:
            self._log_event("error", f"voice: could not synthesize speech: {exc}")
            return
        self._emit_speech(turn_id, 0, clip)

    # -- events and state -------------------------------------------------

    def _emit_assistant(self, turn: _Turn, final: bool) -> None:
        self._emit(
            {"kind": "chat", "role": "assistant", "text": turn.full_text(),
             "turn_id": turn.id, "final": final, "via": turn.via}
        )  # fmt: skip

    def _compute_state(self) -> str:
        if self._transcribing > 0:
            return "transcribing"
        turn = self._turn
        # A cancelled turn is only winding down; it no longer speaks or acts.
        active = turn is not None and not turn.done and not turn.cancelled
        if active and turn.acting > 0:
            return "acting"
        if time.monotonic() < self._speaking_until - 0.02:
            return "speaking"
        if active:
            # After the model is done the only thing left is speech on its way out.
            return "speaking" if turn.brain_done else "thinking"
        return "idle"

    def _refresh_state(self) -> None:
        state = self._compute_state()
        if state != self.state:
            self.state = state
            self._emit({"kind": "voice_state", "state": state})
        if self._state_timer is not None:
            self._state_timer.cancel()
            self._state_timer = None
        remaining = self._speaking_until - time.monotonic()
        if remaining > 0:
            try:
                loop = asyncio.get_running_loop()
            except RuntimeError:
                return
            self._state_timer = loop.call_later(remaining + 0.03, self._refresh_state)

    def _emit(self, event: dict) -> None:
        try:
            self._emit_raw(event)
        except Exception:
            log.exception("emit failed for %s event", event.get("kind"))

    def _log_event(self, level: str, text: str) -> None:
        self._emit({"kind": "log", "level": level, "text": text})

    def _trace(self, info: dict) -> None:
        if self.trace is not None:
            try:
                self.trace(info)
            except Exception:
                log.exception("trace callback failed")

    def _spawn(self, coro: Any) -> None:
        task = asyncio.ensure_future(coro)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)


def _discard_clip(task: asyncio.Future) -> None:
    """Delete the file of a clip nobody will play."""
    if task.cancelled() or task.exception() is not None:
        return
    task.result().path.unlink(missing_ok=True)


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------


class ChatRequest(BaseModel):
    text: str


def create_router(service: VoiceService) -> APIRouter:
    """The four conversation/voice routes from docs/API.md, with their full paths."""
    router = APIRouter()

    @router.post("/api/chat")
    async def chat(body: ChatRequest) -> dict:
        if not body.text.strip():
            raise HTTPException(status_code=400, detail="text is empty")
        return {"turn_id": await service.handle_text(body.text, via="text")}

    @router.post("/api/chat/interrupt")
    async def chat_interrupt() -> dict:
        await service.interrupt()
        return {"ok": True}

    @router.post("/api/voice/utterance")
    async def voice_utterance(audio: UploadFile = File(...)) -> dict:
        data = await audio.read()
        text, turn_id = await service.handle_audio(data, audio.content_type)
        return {"text": text, "turn_id": turn_id}

    @router.get("/api/voice/audio/{clip_id}.wav")
    async def voice_audio(clip_id: str) -> FileResponse:
        path = service.audio_path(clip_id)
        if path is None:
            raise HTTPException(status_code=404, detail="no such clip")
        return FileResponse(path, media_type="audio/wav", headers={"Cache-Control": "no-store"})

    return router
