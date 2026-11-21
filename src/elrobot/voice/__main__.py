"""Terminal harness for the voice layer.

    python -m elrobot.voice                       # REPL against the stub robot
    python -m elrobot.voice --say "dap me up"     # one turn, then exit
    python -m elrobot.voice --wav utterance.webm  # push an audio file through STT first
    python -m elrobot.voice --real                # use the real elrobot-mcp server
    python -m elrobot.voice --mcp "uv run --project . elrobot-mcp"

Steps run in the order given and can be combined to script a conversation:

    --say "pick up the phone" --interrupt-after 2.5 --say "what just happened?"
    --say "pick up the phone" --after 2 --say "nah, wave instead"     # barge-in

Every event the service emits is printed with its time since the turn began,
and speech is played through the speakers with afplay.

In the REPL: type to talk, `/stop` to interrupt, `/wav FILE` to send audio,
`/rec [seconds]` to record from the microphone, `/q` to quit.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import os
import shlex
import shutil
import sys
import tempfile
import time
from pathlib import Path

DIM, BOLD, RED, GREEN, YELLOW, CYAN, RESET = (
    ("\033[2m", "\033[1m", "\033[31m", "\033[32m", "\033[33m", "\033[36m", "\033[0m")
    if sys.stdout.isatty()
    else ("",) * 7
)


class _Step(argparse.Action):
    """Collect --say/--wav/--after/--interrupt-after in the order they were given."""

    def __call__(self, parser, namespace, values, option_string=None):
        steps = getattr(namespace, "steps", None) or []
        steps.append((self.dest, values))
        namespace.steps = steps


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="python -m elrobot.voice",
        description="Talk to the robot's voice agent from a terminal.",
    )
    parser.add_argument("--say", action=_Step, metavar="TEXT", help="run one typed turn")
    parser.add_argument("--wav", action=_Step, metavar="FILE", help="run one turn from an audio file")
    parser.add_argument(
        "--after", action=_Step, type=float, metavar="SECONDS",
        help="start the next step this long after the previous one began, without waiting "
        "for it to finish (barge-in)",
    )  # fmt: skip
    parser.add_argument(
        "--interrupt-after", action=_Step, type=float, metavar="SECONDS", dest="interrupt_after",
        help="interrupt the previous step this long after it began",
    )  # fmt: skip
    parser.add_argument("--mcp", metavar="COMMAND", help="MCP server command line (default: the stub)")
    parser.add_argument("--real", action="store_true", help="use the real elrobot-mcp server")
    parser.add_argument(
        "--daemon", metavar="URL",
        help="daemon base URL; interrupts then POST {URL}/api/stop "
        "(default with --real: http://127.0.0.1:8765)",
    )  # fmt: skip
    parser.add_argument("--model", help="override ELROBOT_VOICE_MODEL")
    parser.add_argument("--effort", help="override ELROBOT_VOICE_EFFORT")
    parser.add_argument("--tts", choices=["elevenlabs", "kokoro", "say"], help="force a TTS provider")
    parser.add_argument("--no-play", action="store_true", help="do not play audio")
    parser.add_argument("-v", "--verbose", action="store_true", help="print every partial chat event")
    parser.add_argument("--debug", action="store_true", help="debug logging, including CLI stderr")
    args = parser.parse_args(argv)
    args.steps = getattr(args, "steps", None) or []
    return args


class Player:
    """Plays speech clips back to back with afplay, like the browser will.

    Clips are scheduled by their length rather than by waiting for afplay to
    exit: afplay lingers for most of a second after the sound has ended, which
    would put a gap between every two sentences.
    """

    def __init__(self, enabled: bool, on_start) -> None:
        self.enabled = enabled and shutil.which("afplay") is not None
        self._queue: asyncio.Queue[tuple[int, str, int, Path, float]] = asyncio.Queue()
        self._procs: set[asyncio.subprocess.Process] = set()
        self._free_at = 0.0  # monotonic time the speaker is free again
        self._generation = 0
        self._on_start = on_start
        self._task: asyncio.Task[None] | None = None
        self._reapers: set[asyncio.Task[int]] = set()

    def start(self) -> None:
        self._task = asyncio.create_task(self._run())

    def add(self, turn_id: str, seq: int, path: Path, duration: float) -> None:
        if self.enabled:
            self._queue.put_nowait((self._generation, turn_id, seq, path, duration))

    def cut(self) -> None:
        """Stop talking right now and forget what was queued."""
        self._generation += 1
        self._free_at = 0.0
        for proc in list(self._procs):
            if proc.returncode is None:
                with contextlib.suppress(ProcessLookupError):
                    proc.kill()

    @property
    def idle(self) -> bool:
        return self._queue.empty() and time.monotonic() >= self._free_at

    async def close(self) -> None:
        self.cut()
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
        if self._reapers:
            await asyncio.gather(*self._reapers, return_exceptions=True)

    async def _run(self) -> None:
        while True:
            generation, turn_id, seq, path, duration = await self._queue.get()
            while generation == self._generation and time.monotonic() < self._free_at:
                await asyncio.sleep(0.01)
            if generation != self._generation:
                continue  # cut while waiting its turn
            try:
                proc = await asyncio.create_subprocess_exec(
                    "afplay", str(path),
                    stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL,
                )  # fmt: skip
            except FileNotFoundError:
                self.enabled = False
                continue
            self._free_at = time.monotonic() + duration + 0.05
            self._procs.add(proc)
            reaper = asyncio.create_task(proc.wait())
            self._reapers.add(reaper)
            reaper.add_done_callback(lambda _t, p=proc: self._procs.discard(p))
            reaper.add_done_callback(self._reapers.discard)
            self._on_start(turn_id, seq)


class Harness:
    def __init__(self, args: argparse.Namespace) -> None:
        from . import config
        from .service import VoiceService

        self.args = args
        if args.mcp:
            command = shlex.split(args.mcp)
        elif args.real:
            command = config.default_mcp_command()
        else:
            command = config.stub_mcp_command()
        self.command = command
        self.daemon = args.daemon or ("http://127.0.0.1:8765" if args.real else None)
        self.turn_started: dict[str, float] = {}
        self.clock = time.perf_counter()  # start of the most recent step
        self.finals: dict[str, asyncio.Event] = {}
        self.first_partial: set[str] = set()
        self.player = Player(not args.no_play, self._on_play)
        self.service = VoiceService(self.emit, mcp_command=command, stop_robot=self.stop_robot)
        self.service.trace = self.trace

    # -- output -----------------------------------------------------------

    def _stamp(self, turn_id: str | None = None) -> str:
        origin = self.turn_started.get(turn_id or "", self.clock)
        return f"{DIM}[+{time.perf_counter() - origin:6.2f}s]{RESET}"

    def out(self, turn_id: str | None, text: str) -> None:
        print(f"{self._stamp(turn_id)} {text}", flush=True)

    def emit(self, event: dict) -> None:
        kind = event.get("kind")
        turn_id = event.get("turn_id")
        if kind == "chat":
            if event["role"] == "user":
                # A new user line is a new turn: the clock restarts and old speech is cut.
                self.turn_started.setdefault(turn_id, self.clock)
                self.player.cut()
                self.out(turn_id, f"{BOLD}chat user{RESET} ({event['via']}): {event['text']!r}")
                return
            if event["final"]:
                self.out(turn_id, f"{BOLD}chat assistant{RESET} final: {event['text']!r}")
                self.finals.setdefault(turn_id, asyncio.Event()).set()
            elif self.args.verbose or turn_id not in self.first_partial:
                self.first_partial.add(turn_id)
                suffix = "" if self.args.verbose else f" {DIM}(first text; -v shows all){RESET}"
                self.out(turn_id, f"chat assistant …: {event['text']!r}{suffix}")
        elif kind == "speech":
            path = self.service.audio_path(event["url"].rsplit("/", 1)[-1].removesuffix(".wav"))
            self.out(
                turn_id,
                f"{GREEN}speech #{event['seq']}{RESET} {event['text']!r} "
                f"{DIM}{event['duration_ms']} ms audio, {event['url']}{RESET}",
            )
            if path is not None:
                self.player.add(turn_id, event["seq"], path, event["duration_ms"] / 1000.0)
        elif kind == "voice_state":
            self.out(None, f"{CYAN}voice_state{RESET} {event['state']}")
        elif kind == "log":
            colour = RED if event.get("level") == "error" else YELLOW
            self.out(None, f"{colour}log {event.get('level')}{RESET}: {event.get('text')}")
        else:
            self.out(turn_id, f"event {event}")

    def trace(self, info: dict) -> None:
        kind = info.get("kind")
        turn_id = info.get("turn_id")
        if kind == "tool_start":
            self.out(turn_id, f"{YELLOW}tool →{RESET} {info['tool']}({_args(info['args'])})")
        elif kind == "tool_end":
            mark = {True: f"{GREEN}ok{RESET}", False: f"{RED}FAILED{RESET}"}.get(info["ok"], "?")
            image = " +image" if info["image"] else ""
            self.out(
                turn_id,
                f"{YELLOW}tool ←{RESET} {info['tool']} {mark}{image} "
                f"{DIM}{info['duration_ms']:.0f} ms{RESET}: {info['summary']}",
            )
        elif kind == "tts":
            self.out(
                turn_id,
                f"{DIM}  tts #{info['seq']} {info['provider']}: {info['synth_ms']:.0f} ms for "
                f"{info['duration_ms']} ms of audio{RESET}",
            )
        elif kind == "stt":
            verdict = repr(info["text"]) if info["text"] else f"rejected ({info['rejected']})"
            self.out(
                None,
                f"{DIM}  stt: {verdict}; {info['audio_s']:.2f} s audio, decode "
                f"{info['decode_ms']:.0f} ms + whisper {info['infer_ms']:.0f} ms{RESET}",
            )
        elif kind == "turn_end":
            first = info.get("first_text_ms")
            first_text = f"{first:.0f} ms" if first is not None else "n/a"
            cost = f", ${info['cost_usd']:.4f}" if info.get("cost_usd") else ""
            error = f" {RED}{info['error']}{RESET}" if info.get("error") else ""
            self.out(
                turn_id,
                f"{DIM}  turn {info['status']}: model first text after {first_text}, "
                f"{info['steps']} model step(s){cost}{RESET}{error}",
            )
        elif kind == "robot_stop":
            pass  # stop_robot prints for itself
        elif kind == "warm":
            pass

    def _on_play(self, turn_id: str, seq: int) -> None:
        self.out(turn_id, f"{DIM}  ♪ playing #{seq}{RESET}")

    async def stop_robot(self) -> None:
        if not self.daemon:
            self.out(None, f"{RED}stop_robot(){RESET} called (no daemon in this harness)")
            return
        import httpx

        async with httpx.AsyncClient(timeout=3.0) as client:
            response = await client.post(f"{self.daemon}/api/stop")
        self.out(None, f"{RED}stop_robot(){RESET} → HTTP {response.status_code}")

    # -- driving ----------------------------------------------------------

    async def run(self) -> int:
        from . import config

        print(f"{DIM}mcp: {' '.join(self.command)}{RESET}")
        print(f"{DIM}model: {config.VOICE_MODEL} (effort {config.VOICE_EFFORT}), warming up ...{RESET}")
        self.player.start()
        started = time.perf_counter()
        await self.service.start()
        warm = await self.service.ready()
        brain = self.service.brain
        print(
            f"{DIM}ready in {time.perf_counter() - started:.1f} s "
            f"(brain {warm.get('brain', 0):.0f} ms, stt {warm.get('stt', 0):.0f} ms, "
            f"tts {warm.get('tts', 0):.0f} ms via {self.service.tts.active}){RESET}"
        )
        print(
            f"{DIM}robot tools ({brain.mcp_status or 'unknown'}): "
            f"{', '.join(brain.tools) or 'none yet'}{RESET}\n"
        )
        try:
            if self.args.steps:
                await self.run_steps(self.args.steps)
            else:
                await self.repl()
        finally:
            await self.player.close()
            await self.service.stop()
        return 0

    async def submit(self, kind: str, value: str) -> str | None:
        self.clock = time.perf_counter()
        if kind == "say":
            return await self.service.handle_text(value, via="text")
        path = Path(value).expanduser()
        data = path.read_bytes()
        mime = {
            ".webm": "audio/webm", ".ogg": "audio/ogg", ".mp4": "audio/mp4", ".m4a": "audio/mp4",
            ".wav": "audio/wav", ".mp3": "audio/mpeg",
        }.get(path.suffix.lower())  # fmt: skip
        self.out(None, f"{BOLD}audio{RESET} {path.name} ({len(data) / 1024:.0f} kB)")
        text, turn_id = await self.service.handle_audio(data, mime)
        if turn_id is None:
            self.out(None, "no speech recognised; no turn started")
        return turn_id

    async def settle(self, turn_id: str | None) -> None:
        """Wait until the turn is finished and its speech has played out."""
        if turn_id is not None:
            await self.finals.setdefault(turn_id, asyncio.Event()).wait()
        await self.service.wait_idle()
        while not self.player.idle:
            await asyncio.sleep(0.05)
        await asyncio.sleep(0.1)

    async def run_steps(self, steps: list[tuple[str, object]]) -> None:
        turn_id: str | None = None
        pending = False  # a submitted step we have not waited for yet
        for kind, value in steps:
            if kind == "after":
                await self._sleep_since_step(float(value))  # type: ignore[arg-type]
                pending = False  # the next step barges in on purpose
                continue
            if kind == "interrupt_after":
                await self._sleep_since_step(float(value))  # type: ignore[arg-type]
                self.out(None, f"{RED}interrupt(){RESET}")
                began = time.perf_counter()
                await self.service.interrupt()
                self.player.cut()
                self.out(None, f"{DIM}  interrupt returned in {(time.perf_counter() - began) * 1000:.0f} ms{RESET}")
                continue
            if pending:
                await self.settle(turn_id)
                print()
            turn_id = await self.submit(kind, str(value))
            pending = True
        if pending:
            await self.settle(turn_id)

    async def _sleep_since_step(self, seconds: float) -> None:
        await asyncio.sleep(max(0.0, self.clock + seconds - time.perf_counter()))

    async def repl(self) -> None:
        print("type to talk · /stop · /wav FILE · /rec [seconds] · /q\n")
        loop = asyncio.get_running_loop()
        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                break
            line = line.strip()
            if not line:
                continue
            if line in ("/q", "/quit", "/exit"):
                break
            try:
                if line in ("/stop", "/i", "/interrupt"):
                    self.clock = time.perf_counter()
                    await self.service.interrupt()
                    self.player.cut()
                elif line.startswith("/wav "):
                    await self.submit("wav", line[5:].strip())
                elif line.startswith("/rec"):
                    seconds = float(line[4:].strip() or 4)
                    path = await self.record(seconds)
                    if path:
                        await self.submit("wav", str(path))
                else:
                    await self.submit("say", line)
            except Exception as exc:
                print(f"{RED}error: {exc}{RESET}")

    async def record(self, seconds: float) -> Path | None:
        """Record from the default microphone as webm/opus, like a browser would."""
        from . import config

        path = Path(tempfile.gettempdir()) / f"elrobot-rec-{os.getpid()}.webm"
        print(f"{RED}● recording {seconds:.0f} s ...{RESET}", flush=True)
        proc = await asyncio.create_subprocess_exec(
            config.FFMPEG, "-y", "-loglevel", "error", "-f", "avfoundation", "-i", ":default",
            "-t", str(seconds), "-c:a", "libopus", "-b:a", "32k", str(path),
            stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.PIPE,
        )  # fmt: skip
        _, err = await proc.communicate()
        if proc.returncode != 0:
            print(f"{RED}recording failed: {err.decode(errors='replace').strip()[-300:]}{RESET}")
            return None
        return path


def _args(args: dict) -> str:
    return ", ".join(f"{key}={value!r}" for key, value in args.items())


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    # These are read when the config module is first imported, so set them first.
    if args.model:
        os.environ["ELROBOT_VOICE_MODEL"] = args.model
    if args.effort:
        os.environ["ELROBOT_VOICE_EFFORT"] = args.effort
    if args.tts:
        os.environ["ELROBOT_TTS_PROVIDER"] = args.tts
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    try:
        return asyncio.run(Harness(args).run())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
