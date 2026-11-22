"""Text to speech: sentence splitting plus a provider chain behind one call.

    tts = TTS()
    clip = tts.synthesize("alright bet")      # blocking -> Clip(path, duration, ...)
    clip = await tts.asynthesize("...")       # same, off the event loop

Providers, first usable one wins and a failing one falls through to the next:
ElevenLabs (if ELEVENLABS_API_KEY is set) -> local Phonon -> local Kokoro -> macOS `say`.
Every clip is a 16-bit mono WAV under data/audio/, pruned by age and count.
"""

from __future__ import annotations

import asyncio
import logging
import os
import re
import subprocess
import tempfile
import threading
import time
import uuid
import wave
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from . import config

log = logging.getLogger("elrobot.voice.tts")
# espeak's word-count sanity check warns on ordinary slang; it is not actionable.
logging.getLogger("phonemizer").setLevel(logging.ERROR)

CLIP_ID_RE = re.compile(r"^[0-9a-f]{16}$")


# ---------------------------------------------------------------------------
# Splitting streamed text into speakable sentences
# ---------------------------------------------------------------------------

# A period after one of these is part of the word, not the end of a sentence.
_ABBREVIATIONS = frozenset(
    "e.g i.e eg ie mr mrs ms dr prof sr jr st vs etc approx no fig vol inc ltd "
    "a.m p.m u.s u.k".split()
)
_TERMINATORS = ".!?…"
_CLOSERS = "\"')]}”’*_"
_SOFT_BREAK = re.compile(r"[,;:—–]\s")
# Past this many characters without a sentence end we cut at a clause boundary
# so a run-on never holds speech back.
_MAX_SENTENCE_CHARS = 220


class SentenceChunker:
    """Turns a stream of text deltas into whole sentences, as early as possible.

    A sentence is released once its terminator has been followed by whitespace
    and the start of the next word, so "e.g. this", "3.5" and "wait... what" do
    not split. Call `flush()` when the text block ends to get the remainder.
    """

    def __init__(self) -> None:
        self._buf = ""

    def feed(self, delta: str) -> list[str]:
        self._buf += delta
        out: list[str] = []
        while True:
            cut = self._find_cut()
            if cut is None:
                break
            sentence, self._buf = self._buf[:cut].strip(), self._buf[cut:].lstrip()
            if sentence:
                out.append(sentence)
        return out

    def flush(self) -> list[str]:
        rest, self._buf = self._buf.strip(), ""
        return [rest] if rest else []

    def peek(self) -> str | None:
        """The buffered text, if it already looks like a finished sentence.

        `feed` cannot release a sentence until it has seen what follows, but the
        text often stops arriving right after "Alright, bet." while the model
        writes a tool call. The caller can start synthesizing this guess early
        and keep the audio only if the same sentence is released later.
        """
        text = self._buf.strip()
        if not text or "\n" in text:
            return None
        end = len(text)
        while end > 0 and text[end - 1] in _CLOSERS:
            end -= 1
        start = end
        while start > 0 and text[start - 1] in _TERMINATORS:
            start -= 1
        if start == end:
            return None  # does not end in a terminator
        if any(ch in _TERMINATORS and i + 1 < start and text[i + 1].isspace()
               for i, ch in enumerate(text[:start])):
            return None  # an earlier boundary is still undecided; do not guess past it
        if text[start:end] == ".":
            if start > 0 and text[start - 1].isdigit():
                return None  # could be the middle of "3.5"
            self_buf, self._buf = self._buf, text
            try:
                if not self._period_ends_sentence(start, len(text), "A"):
                    return None
            finally:
                self._buf = self_buf
        return text

    def _find_cut(self) -> int | None:
        buf = self._buf
        newline = buf.find("\n")
        limit = len(buf) if newline == -1 else newline
        i = 0
        while i < limit:
            if buf[i] not in _TERMINATORS:
                i += 1
                continue
            end = i + 1
            while end < limit and buf[end] in _TERMINATORS:
                end += 1
            while end < limit and buf[end] in _CLOSERS:
                end += 1
            # Need to see what follows before deciding.
            if end >= len(buf):
                return None
            if not buf[end].isspace():
                i = end  # "3.5", "elrobot.voice", "e.g.x": not a boundary
                continue
            nxt = end
            while nxt < len(buf) and buf[nxt].isspace() and buf[nxt] != "\n":
                nxt += 1
            if nxt >= len(buf):
                return None  # only whitespace so far; wait for the next word
            if buf[i] == "." and not self._period_ends_sentence(i, end, buf[nxt]):
                i = end
                continue
            return end
        if newline != -1:
            return newline + 1  # a line break always ends the sentence
        if len(buf) > _MAX_SENTENCE_CHARS:
            soft = [m.end() for m in _SOFT_BREAK.finditer(buf, 40, _MAX_SENTENCE_CHARS)]
            if soft:
                return soft[-1]
            space = buf.rfind(" ", 40, _MAX_SENTENCE_CHARS)
            if space != -1:
                return space + 1
        return None

    def _period_ends_sentence(self, i: int, end: int, next_char: str) -> bool:
        buf = self._buf
        run = buf[i:end].rstrip(_CLOSERS)
        if next_char.islower():
            return False  # "approx. five", "wait... what": the sentence carries on
        if len(run) > 1:
            return True  # an ellipsis followed by a capital
        start = i
        while start > 0 and not buf[start - 1].isspace():
            start -= 1
        word = buf[start:i].lstrip("\"'([{“‘*_").lower()
        if word in _ABBREVIATIONS:
            return False
        if len(word) == 1 and word.isalpha() and word != "i":
            return False  # an initial, as in "J. Smith"
        return True


_MARKUP = re.compile(r"[*_`#~>|]+")
_LINK = re.compile(r"\[([^\]]+)\]\([^)]*\)")
_EMOJI = re.compile(
    "[\U0001f000-\U0001faff\U00002600-\U000027bf\U0001f1e6-\U0001f1ff‍️←-⇿⬀-⯿]+"
)


def speakable(text: str) -> str:
    """What should actually be read out: no markdown, no emoji, single spaces.

    Returns "" when nothing pronounceable is left (so "..." or "🤙" is skipped).
    """
    text = _LINK.sub(r"\1", text)
    text = _EMOJI.sub(" ", text)
    text = _MARKUP.sub(" ", text)
    text = re.sub(r"^\s*(?:[-•]|\d+[.)])\s+", "", text)
    text = re.sub(r"\s+", " ", text).strip()
    return text if any(ch.isalnum() for ch in text) else ""


# ---------------------------------------------------------------------------
# Providers
# ---------------------------------------------------------------------------


@dataclass
class Clip:
    id: str
    path: Path
    duration: float  # seconds of audio
    text: str
    provider: str
    synth_ms: float  # wall time spent producing it

    @property
    def duration_ms(self) -> int:
        return int(round(self.duration * 1000))

    @property
    def rtf(self) -> float:
        """Real-time factor: synthesis time / audio time (lower is faster)."""
        return (self.synth_ms / 1000.0) / self.duration if self.duration else 0.0


def _wav_duration(path: Path) -> float:
    with wave.open(str(path), "rb") as wav:
        return wav.getnframes() / float(wav.getframerate() or 1)


def _write_pcm16_wav(path: Path, pcm: bytes, sample_rate: int) -> float:
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm)
    return len(pcm) / 2 / sample_rate


def download(url: str, dest: Path, on_progress: Callable[[str], None] | None = None) -> None:
    """Fetch `url` to `dest` atomically (a partial file is never left in place)."""
    import httpx

    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_name(dest.name + ".part")
    if on_progress:
        on_progress(f"downloading {dest.name} ...")
    with httpx.stream("GET", url, follow_redirects=True, timeout=httpx.Timeout(30, read=120)) as r:
        r.raise_for_status()
        with open(tmp, "wb") as fh:
            for chunk in r.iter_bytes(1 << 20):
                fh.write(chunk)
    os.replace(tmp, dest)
    if on_progress:
        on_progress(f"downloaded {dest.name} ({dest.stat().st_size / 1e6:.0f} MB)")


# Words that give a language away in a short spoken sentence. Not a language identifier: it only
# has to tell the configured languages apart, and when a sentence has none of these words the
# language of the conversation so far is kept.
_LANGUAGE_WORDS = {
    "en": "the a an i you it is are and not with on for my your yes no hello hi thanks please what where how pick put got "
          "now that this to of me up i'm it's let's done here there can will do did have its one sec",  # "okay" is said in all three
    "fr": "le la les un une des je tu il elle nous vous est sont et que qui dans pour pas sur avec ce cette mon ton oui non "
          "bonjour salut merci c'est j'ai ça voilà prends mets où quoi au du ne suis fait bien très d'accord je l'ai "
          "voici maintenant encore peux vais cube boîte anneau pince désolé attends regarde",
    "de": "der die das ein eine ich du er sie wir ihr ist sind und nicht mit auf für den dem ja nein hallo danke bitte was wo "
          "wie nimm leg stell mein dein es zu im am habe hab bin gut sehr jetzt noch mal auch schon kann würfel greifer "
          "moment fertig hier dort kiste ring tut mir leid schau",
}
_LANGUAGE_WORDS = {code: set(words.split()) for code, words in _LANGUAGE_WORDS.items()}
_LANGUAGE_LETTERS = {"de": set("äöüß"), "fr": set("éèêëàâçùûîïôœ")}


def detect_language(text: str, default: str = "en", allowed: list[str] | None = None) -> str:
    """Which of the configured languages a sentence is in; `default` when it does not say."""
    allowed = allowed or config.LANGUAGES
    if len(allowed) < 2:
        return allowed[0] if allowed else default
    lowered = text.lower()
    words = re.findall(r"[a-zà-ÿœ']+", lowered)
    scores = {}
    for code in allowed:
        score = sum(1.0 for w in words if w in _LANGUAGE_WORDS.get(code, ()))
        score += 1.5 * sum(1 for ch in lowered if ch in _LANGUAGE_LETTERS.get(code, ()))
        scores[code] = score
    best = max(scores, key=scores.get)
    runner_up = max((v for k, v in scores.items() if k != best), default=0.0)
    if scores[best] >= 1.0 and scores[best] > runner_up:
        return best
    return default if default in allowed else allowed[0]


class Provider:
    name = "base"

    def available(self) -> bool:
        return True

    def warm(self) -> None:
        """Load whatever makes the first real call slow. May download models."""

    def synthesize_to(self, text: str, path: Path) -> float:
        """Write a mono 16-bit WAV at `path`; return its duration in seconds."""
        raise NotImplementedError

    def speak(self, text: str, path: Path, lang: str) -> float:
        """As synthesize_to, in the language `lang`. A provider with one language ignores it."""
        return self.synthesize_to(text, path)


class ElevenLabsProvider(Provider):
    """ElevenLabs streaming endpoint, asking for raw PCM so no decoder is needed."""

    name = "elevenlabs"
    SAMPLE_RATE = 24000

    def __init__(self) -> None:
        self._client = None
        self._rejected = False  # the key was refused; stop trying for this process

    def available(self) -> bool:
        return bool(config.ELEVENLABS_API_KEY) and not self._rejected

    def _http(self):
        if self._client is None:
            import httpx

            self._client = httpx.Client(
                base_url="https://api.elevenlabs.io",
                headers={"xi-api-key": config.ELEVENLABS_API_KEY},
                timeout=httpx.Timeout(10.0, read=30.0),
            )
        return self._client

    def warm(self) -> None:
        # A free call that opens the connection (so the first sentence does not
        # pay for the TLS handshake) and finds a bad key now, not mid-demo.
        response = self._http().get("/v1/models")
        if response.status_code in (401, 403):
            self._rejected = True
            raise RuntimeError(f"ElevenLabs rejected the API key (HTTP {response.status_code})")

    def synthesize_to(self, text: str, path: Path) -> float:
        pcm = bytearray()
        with self._http().stream(
            "POST",
            f"/v1/text-to-speech/{config.ELEVENLABS_VOICE_ID}/stream",
            params={"output_format": f"pcm_{self.SAMPLE_RATE}"},
            json={"text": text, "model_id": config.ELEVENLABS_MODEL_ID},
        ) as response:
            if response.status_code >= 400:
                body = response.read().decode("utf-8", "replace")[:300]
                self._rejected = response.status_code in (401, 403)
                raise RuntimeError(f"ElevenLabs HTTP {response.status_code}: {body}")
            for chunk in response.iter_bytes():
                pcm.extend(chunk)
        if len(pcm) < 2:
            raise RuntimeError("ElevenLabs returned no audio")
        if len(pcm) % 2:
            pcm = pcm[:-1]
        return _write_pcm16_wav(path, bytes(pcm), self.SAMPLE_RATE)


class KokoroProvider(Provider):
    """Local Kokoro-82M through onnxruntime. About 350 MB of model files."""

    name = "kokoro"

    def __init__(self) -> None:
        self._kokoro = None
        self._lock = threading.Lock()
        self.model_path = config.KOKORO_DIR / "kokoro-v1.0.onnx"
        self.voices_path = config.KOKORO_DIR / "voices-v1.0.bin"

    def available(self) -> bool:
        try:
            import kokoro_onnx  # noqa: F401
            import soundfile  # noqa: F401
        except Exception:
            return False
        return True

    def _load(self):
        with self._lock:
            if self._kokoro is None:
                if not self.voices_path.exists():
                    download(config.KOKORO_VOICES_URL, self.voices_path, log.info)
                if not self.model_path.exists():
                    download(config.KOKORO_MODEL_URL, self.model_path, log.info)
                from kokoro_onnx import Kokoro

                self._kokoro = Kokoro(str(self.model_path), str(self.voices_path))
            return self._kokoro

    def warm(self) -> None:
        kokoro = self._load()
        # The first run pays for espeak start-up and onnxruntime's lazy allocations.
        kokoro.create("Yo.", voice=config.TTS_VOICE, speed=config.TTS_SPEED, lang="en-us")

    def synthesize_to(self, text: str, path: Path) -> float:
        import soundfile as sf

        kokoro = self._load()
        samples, sample_rate = kokoro.create(
            text, voice=config.TTS_VOICE, speed=config.TTS_SPEED, lang="en-us"
        )
        if len(samples) == 0:
            raise RuntimeError("Kokoro produced no audio")
        sf.write(str(path), samples, sample_rate, subtype="PCM_16", format="WAV")
        return len(samples) / float(sample_rate)


class PhononProvider(Provider):
    """Gradium's Phonon through the `ptts` runtime: a small on-device model, several times faster
    than real time on a laptop processor. Weights come from Hugging Face on first use.

    The runtime is told a language when it is loaded (it decides how numbers and dates are
    read out), so there is one instance per language, made the first time that language is
    spoken; they share the weights file on disk. Which model, and the licence that goes with
    it, is in config.py."""

    name = "phonon"

    def __init__(self) -> None:
        self._voices: dict[str, Any] = {}
        self._lock = threading.Lock()
        self._model = config.PHONON_MODEL

    def available(self) -> bool:
        try:
            import ptts  # noqa: F401
        except Exception:
            return False
        return True

    def _voice(self, lang: str):
        """(runtime, voice name) for a language; English if that language cannot be loaded."""
        with self._lock:
            if lang not in self._voices:
                import ptts

                try:
                    self._voices[lang] = ptts.TTS(self._model, lang=lang, quant=config.PHONON_QUANT)
                except Exception as exc:
                    if self._model == config.PHONON_MODEL_EN:
                        raise
                    # not logged in to Hugging Face, or the licence not accepted: English only
                    log.warning("Phonon model %s unavailable (%s); using the English model", self._model, str(exc).splitlines()[0][:160])
                    self._model = config.PHONON_MODEL_EN
                    self._voices.clear()
                    self._voices["en"] = ptts.TTS(self._model, lang="en", quant=config.PHONON_QUANT)
            if lang not in self._voices or self._model == config.PHONON_MODEL_EN:
                lang = "en"
                if "en" not in self._voices:
                    import ptts

                    self._voices["en"] = ptts.TTS(self._model, lang="en", quant=config.PHONON_QUANT)
            return self._voices[lang], (config.PHONON_VOICES.get(lang) or None)

    def warm(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            for lang, line in (("en", "Yo."), ("fr", "Salut."), ("de", "Hallo.")):
                if lang in config.LANGUAGES:  # the first sentence in each language is the slow one
                    self.speak(line, Path(tmp) / f"{lang}.wav", lang)

    def synthesize_to(self, text: str, path: Path) -> float:
        return self.speak(text, path, config.LANGUAGES[0])

    def speak(self, text: str, path: Path, lang: str) -> float:
        tts, voice = self._voice(lang)
        with self._lock:  # one sentence at a time
            tts.save(str(path), text, voice=voice)
        return _wav_duration(path)


class SayProvider(Provider):
    """macOS `say`, the robotic last resort that needs no model at all."""

    name = "say"

    def available(self) -> bool:
        return Path("/usr/bin/say").exists()

    def synthesize_to(self, text: str, path: Path) -> float:
        cmd = ["/usr/bin/say", "--file-format=WAVE", "--data-format=LEI16@24000", "-o", str(path)]
        if config.SAY_VOICE:
            cmd += ["-v", config.SAY_VOICE]
        subprocess.run(cmd + ["--", text], check=True, capture_output=True, timeout=30)
        return _wav_duration(path)


def default_providers() -> list[Provider]:
    chain: list[Provider] = [ElevenLabsProvider(), PhononProvider(), KokoroProvider(), SayProvider()]
    if config.TTS_PROVIDER:
        chain = [p for p in chain if p.name == config.TTS_PROVIDER] or chain
    return chain


# ---------------------------------------------------------------------------
# The chain
# ---------------------------------------------------------------------------


class TTS:
    def __init__(
        self, audio_dir: Path | None = None, providers: list[Provider] | None = None
    ) -> None:
        self.audio_dir = Path(audio_dir or config.AUDIO_DIR)
        self.providers = providers if providers is not None else default_providers()
        self._broken: dict[str, float] = {}  # provider name -> time it last failed
        # One worker: clips are made in order, and the model is never run twice at once.
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="tts")
        self._last_prune = 0.0
        self.active: str | None = None  # provider that produced the last clip
        # The language of the conversation: what a sentence is taken to be in when its own words
        # do not say. The service sets it from what the person last said or typed.
        self.language: str = config.LANGUAGES[0]

    # -- lifecycle --------------------------------------------------------

    def warm(self) -> str:
        """Blocking. Prepare the first provider that works and return its name."""
        self.audio_dir.mkdir(parents=True, exist_ok=True)
        self.prune(force=True)
        ready: list[str] = []
        for provider in self._usable():
            # Every provider is warmed, not just the first: if the preferred
            # one drops out mid-conversation the fallback must not start cold.
            try:
                provider.warm()
                ready.append(provider.name)
            except Exception as exc:
                self._mark_broken(provider, exc)
        if not ready:
            raise RuntimeError("no text-to-speech provider is usable")
        self.active = ready[0]
        return ready[0]

    async def awarm(self) -> str:
        return await asyncio.get_running_loop().run_in_executor(self._executor, self.warm)

    def close(self) -> None:
        self._executor.shutdown(wait=False, cancel_futures=True)

    # -- synthesis --------------------------------------------------------

    def synthesize(self, text: str) -> Clip:
        """Blocking. Text in, WAV file path plus duration out."""
        text = speakable(text)
        if not text:
            raise ValueError("nothing speakable in the text")
        self.audio_dir.mkdir(parents=True, exist_ok=True)
        clip_id = uuid.uuid4().hex[:16]
        path = self.audio_dir / f"{clip_id}.wav"
        last_error: Exception | None = None
        for provider in self._usable():
            started = time.perf_counter()
            try:
                duration = provider.speak(text, path, detect_language(text, self.language))
            except Exception as exc:
                last_error = exc
                self._mark_broken(provider, exc)
                path.unlink(missing_ok=True)
                continue
            self.active = provider.name
            self.prune()
            return Clip(
                id=clip_id,
                path=path,
                duration=duration,
                text=text,
                provider=provider.name,
                synth_ms=(time.perf_counter() - started) * 1000.0,
            )
        raise RuntimeError(f"all text-to-speech providers failed: {last_error}")

    async def asynthesize(self, text: str) -> Clip:
        return await asyncio.get_running_loop().run_in_executor(
            self._executor, self.synthesize, text
        )

    # -- clip files -------------------------------------------------------

    def audio_path(self, clip_id: str) -> Path | None:
        if not CLIP_ID_RE.match(clip_id or ""):
            return None  # also keeps path separators out
        path = self.audio_dir / f"{clip_id}.wav"
        return path if path.is_file() else None

    def prune(self, force: bool = False) -> int:
        """Delete clips older than AUDIO_MAX_AGE_S or beyond the newest AUDIO_MAX_CLIPS."""
        now = time.time()
        if not force and now - self._last_prune < 30.0:
            return 0
        self._last_prune = now
        removed = 0
        try:
            clips = sorted(
                (p for p in self.audio_dir.glob("*.wav") if CLIP_ID_RE.match(p.stem)),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
        except OSError:
            return 0
        for index, path in enumerate(clips):
            try:
                if index >= config.AUDIO_MAX_CLIPS or now - path.stat().st_mtime > config.AUDIO_MAX_AGE_S:
                    path.unlink()
                    removed += 1
            except OSError:
                pass
        return removed

    # -- internals --------------------------------------------------------

    def _usable(self) -> list[Provider]:
        """Available providers, ones that failed in the last minute moved to the back."""
        now = time.time()
        usable = [p for p in self.providers if p.available()]
        healthy = [p for p in usable if now - self._broken.get(p.name, 0.0) > 60.0]
        return healthy + [p for p in usable if p not in healthy]

    def _mark_broken(self, provider: Provider, exc: Exception) -> None:
        self._broken[provider.name] = time.time()
        log.warning("tts provider %s failed: %s", provider.name, exc)
