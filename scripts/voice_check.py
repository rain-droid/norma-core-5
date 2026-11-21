"""End-to-end check and latency benchmark for the voice layer (stub robot).

    uv run --project . python scripts/voice_check.py            # everything
    uv run --project . python scripts/voice_check.py --offline  # no Claude calls
    uv run --project . python scripts/voice_check.py --turns 10 # more latency samples

Runs the real service (Whisper, TTS, Claude through the local CLI) against
`elrobot.voice.stub_mcp`, drives it through its own HTTP router, and prints
PASS/FAIL per behaviour plus the measured latencies.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import statistics
import subprocess
import sys
import tempfile
import time
import wave
from pathlib import Path

import numpy as np

FAILED: list[str] = []
NUMBERS: list[tuple[str, str]] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}", flush=True)
    if not ok:
        FAILED.append(name)
    return ok


def record(name: str, value: str) -> None:
    NUMBERS.append((name, value))
    print(f"  ....  {name}: {value}", flush=True)


def med(values: list[float]) -> float:
    return statistics.median(values) if values else float("nan")


def spread(values: list[float], unit: str = "ms") -> str:
    if not values:
        return "n/a"
    return f"median {med(values):.0f} {unit} (min {min(values):.0f}, max {max(values):.0f}, n={len(values)})"


# ---------------------------------------------------------------------------


def offline_checks() -> None:
    from elrobot.voice.service import is_stop_command
    from elrobot.voice.tts import SentenceChunker, speakable

    print("\n== text handling")

    def chunks(text: str, step: int) -> list[str]:
        chunker, out = SentenceChunker(), []
        for i in range(0, len(text), step):
            out += chunker.feed(text[i : i + step])
        return out + chunker.flush()

    cases = {
        "Alright bet. Lemme grab it.": ["Alright bet.", "Lemme grab it."],
        "It weighs 3.5 grams, e.g. nothing. Easy!": ["It weighs 3.5 grams, e.g. nothing.", "Easy!"],
        "Wait... what? Oh shit, I messed up.": ["Wait... what?", "Oh shit, I messed up."],
        "Dr. Smith said \"stop.\" Then I stopped!": ["Dr. Smith said \"stop.\"", "Then I stopped!"],
        "Line one\nLine two. Three": ["Line one", "Line two.", "Three"],
    }
    good = all(chunks(t, s) == want for t, want in cases.items() for s in (1, 3, 7, 99))
    check("sentence splitting (decimals, e.g., ellipsis, quotes, any delta size)", good)
    check("markdown and emoji are not spoken", speakable("**Alright** bet 🤙") == "Alright bet")

    stops = ["stop", "Stop!", "stop stop", "STOP STOP STOP", "freeze", "Hold up.", "hey, stop it", "whoa whoa stop"]
    not_stops = ["stop by the red cup", "don't stop", "can you stop the music", "hold the cup up",
                 "pick up the phone", "okay", "hold up the banana", "what does freeze mean to a robot"]  # fmt: skip
    check("stop commands recognised", all(is_stop_command(s) for s in stops),
          str([s for s in stops if not is_stop_command(s)] or ""))  # fmt: skip
    check("ordinary sentences are not stop commands", not any(is_stop_command(s) for s in not_stops),
          str([s for s in not_stops if is_stop_command(s)] or ""))  # fmt: skip


def encode(wav_path: Path, fmt: str, out_dir: Path) -> bytes:
    from elrobot.voice import config

    out = out_dir / f"utterance.{fmt}"
    codec = {"webm": ["-c:a", "libopus", "-b:a", "32k"], "mp4": ["-c:a", "aac", "-b:a", "64k"]}[fmt]
    subprocess.run(
        [config.FFMPEG, "-y", "-loglevel", "error", "-i", str(wav_path), *codec, str(out)],
        check=True,
    )
    return out.read_bytes()


def pcm_to_webm(samples: np.ndarray, out_dir: Path) -> bytes:
    path = out_dir / "synthetic.wav"
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(16000)
        wav.writeframes((np.clip(samples, -1, 1) * 32767).astype("<i2").tobytes())
    return encode(path, "webm", out_dir)


def norm(text: str) -> str:
    return " ".join("".join(ch for ch in text.lower() if ch.isalnum() or ch.isspace()).split())


def speech_checks(tmp: Path, repeats: int) -> dict[str, bytes]:
    """TTS and STT on their own. Returns encoded utterances for the later stages."""
    from elrobot.voice.stt import STT
    from elrobot.voice.tts import TTS

    print("\n== text to speech")
    tts = TTS()
    started = time.perf_counter()
    provider = tts.warm()
    record("TTS provider / warm-up", f"{provider}, {time.perf_counter() - started:.1f} s")
    samples = {
        "short ack": "Alright, bet.",
        "failure line": "Oh shit, I messed up, lemme try again.",
        "long sentence": "Got it this time, the phone's in my gripper now, want me to hand it over?",
    }
    rtfs = []
    for label, text in samples.items():
        clips = [tts.synthesize(text) for _ in range(repeats)]
        synth = [c.synth_ms for c in clips]
        rtfs += [c.rtf for c in clips]
        record(f"TTS {label} ({clips[0].duration:.2f} s of audio)", spread(synth))
        check(f"clip file for {label!r} is a readable wav", tts.audio_path(clips[0].id) is not None)
    record("TTS real-time factor", f"median {med(rtfs):.2f} (min {min(rtfs):.2f}, max {max(rtfs):.2f})")
    check("clip ids cannot escape the audio directory", tts.audio_path("../../etc/passwd") is None)

    from elrobot.voice.tts import Provider, SayProvider

    class Broken(Provider):
        name = "broken"

        def synthesize_to(self, text, path):
            raise RuntimeError("simulated outage")

    chain = TTS(providers=[Broken(), SayProvider()])
    fallback = chain.synthesize("Alright, bet.")
    check("a failing provider falls through to the next one", fallback.provider == "say" and fallback.duration > 0.2,
          f"{fallback.provider}, {fallback.duration:.2f} s")  # fmt: skip
    chain.close()

    print("\n== speech to text")
    stt = STT()
    started = time.perf_counter()
    stt.warm()
    record("STT model / warm-up", f"{Path(stt.model_path()).name}, {time.perf_counter() - started:.1f} s")

    utterances = {
        "dap": "Yo, dap me up.",
        "pick": "Hey, can you pick up the phone and hand it to me please?",
        "thanks": "Thank you.",
        "stop": "Stop!",
    }
    encoded: dict[str, bytes] = {}
    for key, text in utterances.items():
        clip = tts.synthesize(text)
        for fmt, mime in (("webm", "audio/webm;codecs=opus"), ("mp4", "audio/mp4")):
            data = encode(clip.path, fmt, tmp)
            encoded[f"{key}.{fmt}"] = data
            runs = [stt.transcribe(data, mime) for _ in range(repeats)]
            ok = norm(runs[0].text) == norm(text)
            check(f"{fmt}: {text!r} -> {runs[0].text!r}", ok)
            if key == "pick":
                record(
                    f"STT {runs[0].audio_s:.1f} s utterance, {fmt}",
                    f"decode {spread([r.decode_ms for r in runs])}; "
                    f"whisper {spread([r.infer_ms for r in runs])}",
                )

    rng = np.random.default_rng(7)
    rate, seconds = 16000, 3
    t = np.arange(rate * seconds) / rate
    knock = np.zeros(rate * seconds)
    knock[rate : rate + rate // 2] = rng.standard_normal(rate // 2) * 0.2
    typing = np.zeros(rate * seconds)
    for k in range(14):
        at = int(rate * (0.2 + k * 0.19))
        typing[at : at + 400] = rng.standard_normal(400) * 0.4 * np.exp(-np.arange(400) / 80)
    whine = 0.1 * np.sin(2 * np.pi * (300 + 200 * np.sin(2 * np.pi * 1.5 * t)) * t)
    noises = {
        "digital silence": np.zeros(rate * seconds),
        "room hiss": rng.standard_normal(rate * seconds) * 0.002,
        "loud white noise": rng.standard_normal(rate * seconds) * 0.05,
        "mains hum": 0.05 * np.sin(2 * np.pi * 120 * t),
        "knock on the table": knock,
        "keyboard typing": typing,
        "servo whine": whine * (np.sin(2 * np.pi * 0.7 * t) > 0),
    }
    for label, samples_ in noises.items():
        result = stt.transcribe(pcm_to_webm(samples_, tmp), "audio/webm")
        check(f"{label} starts no turn", result.text == "", f"{result.rejected}, raw {result.raw_text!r}")
    empty = stt.transcribe(b"", "audio/webm")
    check("empty upload starts no turn", empty.text == "")
    stt.close()
    tts.close()
    return encoded


class Recorder:
    """Collects emitted events with arrival times."""

    def __init__(self) -> None:
        self.events: list[tuple[float, dict]] = []
        self.trace: list[tuple[float, dict]] = []

    def emit(self, event: dict) -> None:
        self.events.append((time.perf_counter(), dict(event)))

    def on_trace(self, info: dict) -> None:
        self.trace.append((time.perf_counter(), dict(info)))

    def of(self, turn_id: str, kind: str, **match) -> list[tuple[float, dict]]:
        return [
            (t, e) for t, e in self.events
            if e.get("turn_id") == turn_id and e.get("kind") == kind
            and all(e.get(k) == v for k, v in match.items())
        ]  # fmt: skip

    def traced(self, turn_id: str, kind: str) -> list[tuple[float, dict]]:
        return [(t, e) for t, e in self.trace if e.get("turn_id") == turn_id and e.get("kind") == kind]

    async def final(self, turn_id: str, timeout: float = 120.0) -> dict:
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            finals = self.of(turn_id, "chat", role="assistant", final=True)
            if finals:
                return finals[-1][1]
            await asyncio.sleep(0.02)
        raise TimeoutError(f"turn {turn_id} never finished")


async def scripted_checks() -> None:
    """Service behaviour with a scripted brain: no Claude involved, real TTS."""
    from elrobot.voice import VoiceService, config
    from elrobot.voice.agent import TextDelta, TextEnd, ToolEnd, ToolStart, TurnEnd
    from elrobot.voice.service import FILLERS, LINE_BRAIN_ERROR

    print("\n== service logic with a scripted brain")
    rec = Recorder()
    stops: list[float] = []

    async def stop_robot() -> None:
        stops.append(time.perf_counter())

    service = VoiceService(rec.emit, mcp_command=config.stub_mcp_command(), stop_robot=stop_robot)
    service.trace = rec.on_trace
    script: list[tuple[float, object]] = []

    async def scripted_turn(prompt: str):
        for delay, event in list(script):
            await asyncio.sleep(delay)
            yield event

    interrupted: list[float] = []

    async def scripted_interrupt() -> bool:
        interrupted.append(time.perf_counter())
        return True

    service.brain.turn = scripted_turn  # type: ignore[method-assign]
    service.brain.interrupt = scripted_interrupt  # type: ignore[method-assign]

    async def run(events: list[tuple[float, object]]) -> tuple[str, dict]:
        script[:] = events
        turn_id = await service.handle_text("scripted")
        final = await rec.final(turn_id, timeout=30)
        await service.wait_idle(timeout=30)
        return turn_id, final

    # The model calls a tool without a word: the service fills the silence.
    turn_id, final = await run([
        (0.05, ToolStart("t1", "place", {})),
        (1.2, ToolEnd("t1", "place", True, "Released the banana.", False, 1200.0)),
        (0.05, TextDelta("Banana's down.")), (0.0, TextEnd()), (0.0, TurnEnd("completed")),
    ])  # fmt: skip
    speech = rec.of(turn_id, "speech")
    tool_end = rec.traced(turn_id, "tool_end")
    check("a silent tool call gets a spoken filler before the tool finishes",
          bool(speech) and speech[0][1]["text"] in FILLERS and speech[0][0] < tool_end[0][0],
          repr([e["text"] for _, e in speech]))  # fmt: skip
    check("the filler is part of the transcript", final["text"].endswith("Banana's down.")
          and final["text"].split(" Banana")[0] in FILLERS, repr(final["text"]))  # fmt: skip

    # The model fails outright: say so in character instead of going silent.
    turn_id, final = await run([(0.05, TurnEnd("error", error="simulated API failure"))])
    check("a failed turn is spoken about, not silent", final["text"] == LINE_BRAIN_ERROR
          and len(rec.of(turn_id, "speech")) == 1, repr(final["text"]))  # fmt: skip
    check("and it is logged", any(e.get("kind") == "log" and "simulated API failure" in e.get("text", "")
                                  for _, e in rec.events))  # fmt: skip

    # A guess at the sentence end that turns out wrong must not be spoken.
    turn_id, final = await run([
        (0.05, TextDelta("Wait.")), (0.4, TextDelta(".. what? It weighs 3.")), (0.3, TextDelta("5 grams.")),
        (0.0, TextEnd()), (0.0, TurnEnd("completed")),
    ])  # fmt: skip
    spoken = [e["text"] for _, e in rec.of(turn_id, "speech")]
    check("early synthesis never speaks a half sentence", spoken == ["Wait... what?", "It weighs 3.5 grams."], repr(spoken))

    # Text before and after a tool call is one transcript, growing monotonically.
    turn_id, final = await run([
        (0.05, TextDelta("Alright, ")), (0.05, TextDelta("bet.")), (0.0, TextEnd()),
        (0.0, ToolStart("t2", "gesture", {"name": "dap"})),
        (1.0, ToolEnd("t2", "gesture", True, "Did the dap gesture.", False, 1000.0)),
        (0.05, TextDelta("Boom.")), (0.0, TextEnd()), (0.0, TurnEnd("completed")),
    ])  # fmt: skip
    texts = [e["text"] for _, e in rec.of(turn_id, "chat", role="assistant")]
    check("chat text accumulates across the tool call", texts[-1] == "Alright, bet. Boom."
          and all(b.startswith(a) for a, b in zip(texts, texts[1:])), repr(texts))  # fmt: skip
    speech = rec.of(turn_id, "speech")
    check("the acknowledgement is ready while the tool is still running",
          bool(speech) and speech[0][0] < rec.traced(turn_id, "tool_end")[0][0])  # fmt: skip

    # The microphone hears the robot's own sentence: that must not start a turn.
    from elrobot.voice.stt import Transcript

    heard: list[str] = ["Alright, bet. Boom."]

    async def scripted_stt(data: bytes, mime: str | None = None) -> Transcript:
        return Transcript(text=heard[0], raw_text=heard[0], audio_s=1.0)

    service.stt.atranscribe = scripted_stt  # type: ignore[method-assign]
    await run([(0.0, TextDelta("Alright, bet. Boom.")), (0.0, TextEnd()), (0.0, TurnEnd("completed"))])
    echo = await service.handle_audio(b"x", "audio/webm")
    check("hearing its own last sentence back starts no turn", echo == ("", None), str(echo))
    heard[0] = "nice one, now wave"
    script[:] = [(0.0, TurnEnd("completed"))]
    real = await service.handle_audio(b"x", "audio/webm")
    check("a different sentence right after still starts a turn", real[0] == heard[0] and bool(real[1]), str(real))
    await rec.final(real[1], timeout=10)
    await service.wait_idle(timeout=10)

    # Interrupt drops speech that has not gone out yet.
    script[:] = [(0.05, TextDelta("One. Two. Three. Four. Five. Six. Seven. Eight. ")), (30.0, TurnEnd("completed"))]

    async def stop_script_on_interrupt() -> bool:
        interrupted.append(time.perf_counter())
        script[:] = []
        return True

    turn_id = await service.handle_text("count")
    for _ in range(300):
        if rec.of(turn_id, "speech"):
            break
        await asyncio.sleep(0.01)
    before = len(rec.of(turn_id, "speech"))
    stops.clear()
    began = time.perf_counter()
    await service.interrupt()
    took = (time.perf_counter() - began) * 1000.0
    await asyncio.sleep(1.0)
    after = len(rec.of(turn_id, "speech"))
    check("interrupt() drops queued speech", 1 <= before == after < 8, f"{before} clips out before, {after} after")
    check("interrupt() calls stop_robot and the brain", len(stops) == 1 and bool(interrupted), f"{took:.0f} ms")
    check("voice_state is idle right after an interrupt", service.state == "idle", service.state)
    script[:] = [(0.0, TurnEnd("interrupted"))]
    await service.stop()


async def live_checks(encoded: dict[str, bytes], turns: int) -> None:
    import httpx
    from fastapi import FastAPI

    from elrobot.voice import VoiceService, create_router
    from elrobot.voice import config

    print("\n== service + router against the stub robot")
    rec = Recorder()
    stops: list[float] = []

    async def stop_robot() -> None:
        stops.append(time.perf_counter())

    service = VoiceService(rec.emit, mcp_command=config.stub_mcp_command(), stop_robot=stop_robot)
    service.trace = rec.on_trace
    app = FastAPI()
    app.include_router(create_router(service))
    started = time.perf_counter()
    await service.start()
    check("start() returns without blocking on model loads", time.perf_counter() - started < 1.0,
          f"{(time.perf_counter() - started) * 1000:.0f} ms")  # fmt: skip
    warm = await service.ready()
    record("service warm-up", ", ".join(f"{k} {v / 1000:.1f} s" for k, v in warm.items()))
    brain = service.brain
    check("robot MCP server connected", brain.mcp_status == "connected", str(brain.mcp_status))
    check("only the robot's tools are available", bool(brain.tools) and not any(
        t in brain.tools for t in ("Bash", "Read", "Write", "Edit", "WebFetch", "WebSearch", "Agent")
    ), ", ".join(brain.tools))  # fmt: skip

    async with httpx.AsyncClient(
        transport=httpx.ASGITransport(app=app), base_url="http://voice.test", timeout=180
    ) as http:

        async def say(text: str) -> tuple[str, float, dict]:
            sent = time.perf_counter()
            response = await http.post("/api/chat", json={"text": text})
            response.raise_for_status()
            turn_id = response.json()["turn_id"]
            final = await rec.final(turn_id)
            await service.wait_idle(timeout=60)
            return turn_id, sent, final

        def first(turn_id: str, kind: str, sent: float, **match) -> float | None:
            hits = rec.of(turn_id, kind, **match)
            return (hits[0][0] - sent) * 1000.0 if hits else None

        # --- dap me up ---------------------------------------------------
        print("\n-- 'dap me up'")
        turn_id, sent, final = await say("dap me up")
        speech = rec.of(turn_id, "speech")
        tool_end = rec.traced(turn_id, "tool_end")
        tool_start = rec.traced(turn_id, "tool_start")
        check("the fist-bump gesture was called", any(e["tool"] == "gesture" for _, e in tool_start),
              str([(e["tool"], e["args"]) for _, e in tool_start]))  # fmt: skip
        check("acknowledgement speech event arrives before the gesture finishes",
              bool(speech and tool_end) and speech[0][0] < tool_end[0][0],
              f"speech at {(speech[0][0] - sent):.2f} s, gesture done at {(tool_end[0][0] - sent):.2f} s"
              if speech and tool_end else "missing events")  # fmt: skip
        check("speech events are numbered in order", [e["seq"] for _, e in speech] == list(range(len(speech))))
        check("user chat event emitted", len(rec.of(turn_id, "chat", role="user")) == 1)
        partial = rec.of(turn_id, "chat", role="assistant")
        texts = [e["text"] for _, e in partial]
        check("assistant chat events carry the full text so far and end with final",
              all(b.startswith(a) for a, b in zip(texts, texts[1:])) and partial[-1][1]["final"]
              and sum(1 for _, e in partial if e["final"]) == 1, repr(texts[-1]))  # fmt: skip
        clip_url = speech[0][1]["url"] if speech else "/api/voice/audio/missing.wav"
        audio = await http.get(clip_url)
        check("GET audio url returns a wav", audio.status_code == 200 and audio.content[:4] == b"RIFF",
              f"{clip_url} -> {audio.status_code}, {len(audio.content)} bytes")  # fmt: skip
        check("unknown clip is a 404", (await http.get("/api/voice/audio/0000000000000000.wav")).status_code == 404)
        print(f"     said: {final['text']!r}")

        # --- pick up the phone: fail, say so, retry, succeed --------------
        print("\n-- 'pick up the phone'")
        turn_id, sent, final = await say("pick up the phone")
        ends = rec.traced(turn_id, "tool_end")
        picks = [(t, e) for t, e in ends if e["tool"] == "pick"]
        check("first pick failed and a later pick succeeded",
              len(picks) >= 2 and picks[0][1]["ok"] is False and picks[-1][1]["ok"] is True,
              str([(e["tool"], e["ok"]) for _, e in ends]))  # fmt: skip
        if len(picks) >= 2:
            between = [e["text"] for t, e in rec.of(turn_id, "speech") if picks[0][0] < t < picks[-1][0]]
            check("it said something between the failure and the successful retry", bool(between), repr(between))
        print(f"     said: {final['text']!r}")

        # --- camera image reaches the model -------------------------------
        print("\n-- camera")
        turn_id, sent, final = await say("take a look and tell me what colour the cup is")
        looked = [e for _, e in rec.traced(turn_id, "tool_end") if e["tool"] == "look"]
        check("look() returned an image to the session", bool(looked) and looked[0]["image"])
        check("the model answered from the picture (the cup is red)", "red" in final["text"].lower(), repr(final["text"]))

        # --- interrupt mid-turn, then keep going --------------------------
        print("\n-- interrupt")
        response = await http.post("/api/chat", json={"text": "wave at me"})
        turn_id = response.json()["turn_id"]
        for _ in range(400):
            if rec.traced(turn_id, "tool_start"):
                break
            await asyncio.sleep(0.02)
        await asyncio.sleep(0.4)  # mid-motion
        stops.clear()
        began = time.perf_counter()
        ok = (await http.post("/api/chat/interrupt")).json()
        took = (time.perf_counter() - began) * 1000.0
        check("POST /api/chat/interrupt answers ok", ok == {"ok": True})
        check("stop_robot was called", len(stops) == 1)
        record("interrupt: request to turn stopped", f"{took:.0f} ms"
               + (f" (stop_robot called after {(stops[0] - began) * 1000:.1f} ms)" if stops else ""))  # fmt: skip
        final = await rec.final(turn_id, timeout=10)
        ended = rec.traced(turn_id, "turn_end")
        check("the turn ended as interrupted", bool(ended) and ended[-1][1]["status"] == "interrupted")
        await service.wait_idle(timeout=10)
        check("voice_state returned to idle", service.state == "idle", service.state)
        turn_id, sent, final = await say("my name is Rayane by the way. what was I asking you to do just now?")
        check("session still works after the interrupt and remembers the conversation",
              bool(final["text"]) and "wav" in final["text"].lower(), repr(final["text"]))  # fmt: skip

        # --- stop command bypasses the model ------------------------------
        print("\n-- stop command")
        response = await http.post("/api/chat", json={"text": "pick up the banana"})
        busy_turn = response.json()["turn_id"]
        for _ in range(400):
            if rec.traced(busy_turn, "tool_start"):
                break
            await asyncio.sleep(0.02)
        stops.clear()
        began = time.perf_counter()
        stop_turn = (await http.post("/api/chat", json={"text": "stop stop!"})).json()["turn_id"]
        check("'stop stop!' calls stop_robot without a model round trip", len(stops) == 1,
              f"{(stops[0] - began) * 1000:.1f} ms after the request" if stops else "not called")  # fmt: skip
        await rec.final(busy_turn, timeout=10)
        await service.wait_idle(timeout=10)
        check("no model turn ran for the stop command", not rec.traced(stop_turn, "turn_end"))

        # --- barge-in ------------------------------------------------------
        print("\n-- barge-in")
        first_turn = (await http.post("/api/chat", json={"text": "do a little dance"})).json()["turn_id"]
        for _ in range(400):
            if rec.traced(first_turn, "tool_start"):
                break
            await asyncio.sleep(0.02)
        await asyncio.sleep(0.3)  # mid-motion
        stops.clear()
        turn_id, sent, final = await say("never mind. what's my name?")
        check("the earlier turn was interrupted by the new message",
              rec.traced(first_turn, "turn_end")[-1][1]["status"] == "interrupted")  # fmt: skip
        check("the motion in flight was stopped for the new turn", len(stops) == 1)
        check("the new turn was answered (and memory holds)", "rayane" in final["text"].lower(), repr(final["text"]))

        # --- voice in ------------------------------------------------------
        print("\n-- voice utterances")
        sent = time.perf_counter()
        response = await http.post(
            "/api/voice/utterance", files={"audio": ("utt.webm", encoded["dap.webm"], "audio/webm;codecs=opus")}
        )
        body = response.json()
        check("utterance returns the transcript and a turn id",
              response.status_code == 200 and norm(body["text"]) == "yo dap me up" and bool(body["turn_id"]), str(body))  # fmt: skip
        turn_id = body["turn_id"]
        await rec.final(turn_id)
        await service.wait_idle(timeout=60)
        user = rec.of(turn_id, "chat", role="user")
        check("the voice turn's chat events say via=voice", bool(user) and user[0][1]["via"] == "voice")
        loop_ms = [first(turn_id, "speech", sent)]
        for _ in range(3):
            sent = time.perf_counter()
            again = (await http.post(
                "/api/voice/utterance", files={"audio": ("utt.webm", encoded["dap.webm"], "audio/webm")}
            )).json()  # fmt: skip
            await rec.final(again["turn_id"])
            await service.wait_idle(timeout=60)
            loop_ms.append(first(again["turn_id"], "speech", sent))
        record("voice loop: audio upload -> first speech event (0.9 s utterance)",
               spread([x for x in loop_ms if x is not None]))  # fmt: skip
        silent = await http.post(
            "/api/voice/utterance",
            files={"audio": ("s.webm", pcm_to_webm(np.zeros(48000), Path(tempfile.gettempdir())), "audio/webm")},
        )
        check("silence returns empty text and a null turn id", silent.json() == {"text": "", "turn_id": None}, str(silent.json()))
        stops.clear()
        spoken_stop = await http.post(
            "/api/voice/utterance", files={"audio": ("stop.webm", encoded["stop.webm"], "audio/webm")}
        )
        check("a spoken 'stop' halts the robot", len(stops) == 1, str(spoken_stop.json()))
        await service.wait_idle(timeout=20)
        states = [e["state"] for _, e in rec.events if e.get("kind") == "voice_state"]
        check("voice_state covered transcribing, thinking, acting, speaking, idle",
              {"transcribing", "thinking", "acting", "speaking", "idle"} <= set(states), str(sorted(set(states))))  # fmt: skip
        check("empty chat text is rejected with 400", (await http.post("/api/chat", json={"text": "  "})).status_code == 400)

        # --- latency -------------------------------------------------------
        print(f"\n-- latency over {turns} chat turns and {turns} action turns")
        chat_prompts = ["yo what's good", "how's the table treating you", "you ever get bored", "what's your favourite thing to grab",
                        "who built you", "tell me something cocky", "you tired", "what do you think of humans",
                        "are you stronger than me", "say something nice"]  # fmt: skip
        act_prompts = ["dap me up", "wave at me", "nod for me", "go home", "dap me up again", "give me a wave",
                       "touch the red cup", "do a little dance", "point at something", "one more dap"]  # fmt: skip
        first_text, first_speech, model_first = [], [], []
        for prompt in chat_prompts[:turns]:
            turn_id, sent, final = await say(prompt)
            first_text.append(first(turn_id, "chat", sent, role="assistant"))
            first_speech.append(first(turn_id, "speech", sent))
            model_first += [e["first_text_ms"] for _, e in rec.traced(turn_id, "turn_end") if e["first_text_ms"]]
        act_text, act_speech, act_tool, words, spoke_first = [], [], [], [], 0
        for prompt in act_prompts[:turns]:
            turn_id, sent, final = await say(prompt)
            act_text.append(first(turn_id, "chat", sent, role="assistant"))
            act_speech.append(first(turn_id, "speech", sent))
            starts = rec.traced(turn_id, "tool_start")
            ends = rec.traced(turn_id, "tool_end")
            speech = rec.of(turn_id, "speech")
            if starts:
                act_tool.append((starts[0][0] - sent) * 1000.0)
            spoke_first += bool(speech and ends and speech[0][0] < ends[0][0])
            words.append(len(final["text"].split()))
        clean = lambda xs: [x for x in xs if x is not None]  # noqa: E731
        record("chat turn: text sent -> first assistant text", spread(clean(first_text)))
        record("chat turn: text sent -> first speech event (audio ready to play)", spread(clean(first_speech)))
        record("action turn: text sent -> first assistant text", spread(clean(act_text)))
        record("action turn: text sent -> first speech event", spread(clean(act_speech)))
        record("action turn: text sent -> tool call starts", spread(act_tool))
        record("model time to first text (prompt written -> first token)", spread(model_first))
        check("every action turn had speech out before its first tool finished",
              spoke_first == len(act_prompts[:turns]), f"{spoke_first}/{len(act_prompts[:turns])}")  # fmt: skip
        record("words spoken per action turn", spread([float(w) for w in words], "words"))

        # --- the CLI process dies ------------------------------------------
        print("\n-- recovery")
        process = getattr(getattr(brain._client, "_transport", None), "_process", None)
        if process is None:
            check("could reach the CLI process to kill it", False)
        else:
            process.kill()
            await asyncio.sleep(1.5)
            turn_id, sent, final = await say("you still with me? what's my name?")
            check("after the CLI was killed the next turn still works and remembers",
                  "rayane" in final["text"].lower(), repr(final["text"]))  # fmt: skip

    await service.stop()
    logs = [e for _, e in rec.events if e.get("kind") == "log"]
    for entry in logs:
        print(f"  log {entry['level']}: {entry['text']}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--offline", action="store_true", help="skip everything that calls Claude")
    parser.add_argument("--turns", type=int, default=6, help="latency samples per turn type (max 10)")
    parser.add_argument("--repeats", type=int, default=5, help="repetitions for STT/TTS timing")
    args = parser.parse_args()
    os.environ.setdefault("ELROBOT_STUB_DELAY", "1")

    offline_checks()
    with tempfile.TemporaryDirectory(prefix="voice-check-") as tmp:
        encoded = speech_checks(Path(tmp), args.repeats)
        asyncio.run(scripted_checks())
        if not args.offline:
            asyncio.run(live_checks(encoded, max(1, min(args.turns, 10))))

    print("\n== measured")
    for name, value in NUMBERS:
        print(f"  {name}: {value}")
    print(f"\n{'ALL CHECKS PASSED' if not FAILED else 'FAILED: ' + '; '.join(FAILED)}")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
