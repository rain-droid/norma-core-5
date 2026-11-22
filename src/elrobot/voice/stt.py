"""Speech to text: browser audio bytes in, clean transcript (or "") out.

    stt = STT()
    stt.warm()                                  # blocking, loads + compiles the model
    result = stt.transcribe(webm_bytes)         # blocking -> Transcript
    result = await stt.atranscribe(webm_bytes)  # same, off the event loop

Audio is decoded with ffmpeg (any container it reads), then has to get past
three checks before it can start a turn: an energy gate (silence, hum, hiss),
a voice-activity model (knocks, typing, servo whine: loud but not speech),
and a filter on Whisper's own output for the phrases it invents on empty audio
("thank you", "thanks for watching"). `Transcript.text` is "" when nothing
real was said, and `Transcript.rejected` says why.
"""

from __future__ import annotations

import asyncio
import logging
import os
import re
import subprocess
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import config

log = logging.getLogger("elrobot.voice.stt")

SAMPLE_RATE = 16000

def _phrases(block: str) -> frozenset[str]:
    return frozenset(line.strip() for line in block.strip().splitlines() if line.strip())


# What Whisper says when there is nothing to hear. The first set is never
# something a person says to a robot arm, so it is always dropped.
_ALWAYS_HALLUCINATED = _phrases(
    """
    thanks for watching
    thank you for watching
    thank you so much for watching
    thank you very much for watching
    thanks for watching and see you next time
    please subscribe
    please subscribe to my channel
    like and subscribe
    dont forget to subscribe
    subtitles by the amaraorg community
    subtitles by amaraorg
    see you in the next video
    ill see you in the next video
    see you next time
    well see you next time
    music
    applause
    laughter
    silence
    blank audio
    blankaudio
    inaudible
    no audio
    foreign
    """
)

# These are both classic hallucinations and things people really say ("thank
# you" to the robot is fine). They are dropped only when the audio or the
# decoder says the evidence for speech is weak.
_WEAK_HALLUCINATED = _phrases(
    """
    thank you
    thank you very much
    thank you so much
    thanks
    bye
    bye bye
    goodbye
    you
    so
    oh
    okay
    ok
    yeah
    yes
    no
    hmm
    uh
    um
    i
    the
    and
    it
    """
)

_BRACKETED = re.compile(r"[\[(（【♪*][^\])）】♪*]*[\])）】♪*]")


def _normalise(text: str) -> str:
    text = text.lower().replace("’", "'")
    text = re.sub(r"[^a-z0-9\s]", "", text)
    return re.sub(r"\s+", " ", text).strip()


@dataclass
class Transcript:
    text: str  # "" when rejected
    raw_text: str = ""  # what Whisper produced before filtering
    rejected: str | None = None  # reason, when text is ""
    audio_s: float = 0.0  # decoded audio length
    speech_s: float = 0.0  # how much of it was above the energy gate
    decode_ms: float = 0.0  # ffmpeg
    infer_ms: float = 0.0  # whisper
    language: str | None = None
    detail: dict = field(default_factory=dict)

    @property
    def total_ms(self) -> float:
        return self.decode_ms + self.infer_ms


class AudioDecodeError(RuntimeError):
    pass


_EXTENSIONS = {
    "audio/webm": ".webm",
    "video/webm": ".webm",
    "audio/ogg": ".ogg",
    "audio/mp4": ".mp4",
    "video/mp4": ".mp4",
    "audio/x-m4a": ".m4a",
    "audio/aac": ".aac",
    "audio/mpeg": ".mp3",
    "audio/wav": ".wav",
    "audio/x-wav": ".wav",
    "audio/wave": ".wav",
    "audio/flac": ".flac",
}


def decode_audio(data: bytes, mime: str | None = None) -> np.ndarray:
    """Decode any audio ffmpeg understands to 16 kHz mono float32 in [-1, 1].

    The bytes go through a temp file rather than a pipe because MP4 (Safari's
    MediaRecorder) often keeps its index at the end and cannot be read from a
    stream.
    """
    if not data:
        return np.zeros(0, dtype=np.float32)
    suffix = _EXTENSIONS.get((mime or "").split(";")[0].strip().lower(), ".bin")
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as tmp:
        tmp.write(data)
        tmp_path = tmp.name
    try:
        proc = subprocess.run(
            [
                config.FFMPEG, "-nostdin", "-hide_banner", "-loglevel", "error",
                "-i", tmp_path,
                "-vn", "-ac", "1", "-ar", str(SAMPLE_RATE), "-f", "f32le", "pipe:1",
            ],  # fmt: skip
            capture_output=True,
            timeout=30,
        )
    except FileNotFoundError as exc:
        raise AudioDecodeError(f"ffmpeg not found at {config.FFMPEG}") from exc
    except subprocess.TimeoutExpired as exc:
        raise AudioDecodeError("ffmpeg timed out decoding the audio") from exc
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
    if proc.returncode != 0 and not proc.stdout:
        message = proc.stderr.decode("utf-8", "replace").strip().splitlines()
        raise AudioDecodeError(f"ffmpeg could not decode the audio: {' | '.join(message[-2:])}")
    return np.frombuffer(proc.stdout, dtype=np.float32).copy()


def speech_stats(audio: np.ndarray) -> tuple[float, float, float]:
    """(seconds above the gate, loud-frame level in dBFS, noise floor in dBFS).

    30 ms frames. A frame counts as speech when it is clearly above both an
    absolute floor and the clip's own noise floor, so a constant hum or an
    open-mic hiss does not count however long it lasts.
    """
    frame = int(SAMPLE_RATE * 0.03)
    count = len(audio) // frame
    if count == 0:
        return 0.0, -120.0, -120.0
    frames = audio[: count * frame].reshape(count, frame)
    rms = np.sqrt(np.mean(frames.astype(np.float64) ** 2, axis=1))
    db = 20.0 * np.log10(np.maximum(rms, 1e-7))
    floor = float(np.percentile(db, 10))
    loud = float(np.percentile(db, 95))
    threshold = max(-50.0, floor + 9.0)
    return float(np.sum(db > threshold)) * 0.03, loud, floor


class SpeechDetector:
    """Silero VAD run directly through onnxruntime (a 2 MB model, ~20 ms per utterance).

    Whisper will happily turn a knock on the table into words, so "is anyone
    actually talking" is decided here, by a model trained for exactly that.
    """

    WINDOW = 512  # samples per step at 16 kHz (32 ms)
    CONTEXT = 64  # samples of the previous window the model wants prepended

    def __init__(self) -> None:
        self._session = None
        self._failed = False
        self.path = config.MODELS_DIR / "silero-vad" / "silero_vad.onnx"

    def _load(self):
        if self._session is None and not self._failed:
            try:
                if not self.path.exists():
                    from .tts import download

                    download(config.VAD_MODEL_URL, self.path, log.info)
                import onnxruntime as rt

                options = rt.SessionOptions()
                options.intra_op_num_threads = 1
                options.inter_op_num_threads = 1
                options.log_severity_level = 3
                self._session = rt.InferenceSession(
                    str(self.path), options, providers=["CPUExecutionProvider"]
                )
            except Exception as exc:  # no network, no onnxruntime: degrade to the energy gate
                self._failed = True
                log.warning("voice-activity model unavailable, using the energy gate only: %s", exc)
        return self._session

    def speech_seconds(self, audio: np.ndarray, threshold: float = 0.5) -> tuple[float, float] | None:
        """(seconds of audio the model calls speech, peak probability), or None if unavailable."""
        session = self._load()
        if session is None:
            return None
        window, context_size = self.WINDOW, self.CONTEXT
        state = np.zeros((2, 1, 128), dtype=np.float32)
        context = np.zeros((1, context_size), dtype=np.float32)
        rate = np.array(SAMPLE_RATE, dtype=np.int64)
        hits, peak = 0, 0.0
        for start in range(0, len(audio), window):
            chunk = audio[start : start + window]
            if len(chunk) < window:
                chunk = np.pad(chunk, (0, window - len(chunk)))
            frame = np.concatenate([context, chunk[None, :].astype(np.float32)], axis=1)
            out, state = session.run(None, {"input": frame, "state": state, "sr": rate})
            context = frame[:, -context_size:]
            probability = float(out[0][0])
            peak = max(peak, probability)
            hits += probability >= threshold
        return hits * window / SAMPLE_RATE, peak


class STT:
    def __init__(
        self,
        model: str | None = None,
        language: str | None = None,
        initial_prompt: str | None = None,
    ) -> None:
        self.model_id = model or config.STT_MODEL
        language = (language or config.STT_LANGUAGE or "en").strip().lower()
        self.language: str | None = None if language in ("auto", "detect", "") else language
        prompt = config.STT_PROMPT if initial_prompt is None else initial_prompt
        self.initial_prompt: str | None = None if prompt.strip().lower() in ("", "none") else prompt
        self._model_path: str | None = None
        self._faster_model = None  # loaded on first use, off Apple silicon
        self._parakeet_model = None  # likewise; False once it has failed to load
        self.detector = SpeechDetector()
        # MLX keeps per-thread state, so load, warm-up and every transcription
        # all run on this one thread.
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="stt")
        self.warm_ms: float | None = None

    # -- model ------------------------------------------------------------

    def model_path(self) -> str:
        """Local directory of the MLX weights, downloading them on first use."""
        if self._model_path:
            return self._model_path
        as_path = Path(self.model_id).expanduser()
        if as_path.is_dir():
            self._model_path = str(as_path)
            return self._model_path
        local = config.MODELS_DIR / self.model_id.split("/")[-1]
        if not (local / "config.json").exists() or not any(local.glob("weights.*")):
            from huggingface_hub import snapshot_download

            log.info("downloading %s to %s ...", self.model_id, local)
            snapshot_download(
                repo_id=self.model_id,
                local_dir=str(local),
                allow_patterns=["config.json", "weights.safetensors", "weights.npz"],
            )
        self._model_path = str(local)
        return self._model_path

    def warm(self) -> float:
        """Blocking. Load the weights and run one decode so the first real
        utterance does not pay for it. Returns seconds taken."""
        started = time.perf_counter()
        # Low noise rather than pure zeros so the whole encoder/decoder path runs.
        rng = np.random.default_rng(0)
        noise = (rng.standard_normal(SAMPLE_RATE) * 1e-3).astype(np.float32)
        self.detector.speech_seconds(noise)
        self._infer(noise)
        self.warm_ms = (time.perf_counter() - started) * 1000.0
        return self.warm_ms / 1000.0

    async def awarm(self) -> float:
        return await asyncio.get_running_loop().run_in_executor(self._executor, self.warm)

    def close(self) -> None:
        self._executor.shutdown(wait=False, cancel_futures=True)

    # -- transcription ----------------------------------------------------

    def _faster(self):
        """The faster-whisper model (CTranslate2): what runs where MLX does not, which is
        everywhere but Apple silicon. On the graphics card if CUDA and its libraries are
        there, otherwise on the processor with 8-bit weights."""
        if self._faster_model is None:
            from faster_whisper import WhisperModel

            name, root = config.STT_MODEL_FASTER, str(config.MODELS_DIR / "faster-whisper")
            tries = [("cpu", "int8")] if config.STT_DEVICE == "cpu" else [("cuda", "float16"), ("cpu", "int8")]
            for device, compute in tries:
                try:
                    model = WhisperModel(name, device=device, compute_type=compute, download_root=root)
                    list(model.transcribe(np.zeros(SAMPLE_RATE // 2, np.float32), language="en", beam_size=1)[0])
                except Exception as e:  # no CUDA, or its libraries missing: the next one down
                    log.info("faster-whisper %s on %s not usable: %s", name, device, str(e).splitlines()[0][:120])
                    continue
                log.info("speech to text: faster-whisper %s on %s (%s)", name, device, compute)
                self._faster_model = model
                break
            else:
                raise RuntimeError(f"faster-whisper could not load '{name}' on any device")
        return self._faster_model

    def _parakeet(self):
        """NVIDIA Parakeet v3 through onnx-asr, or None if it is not wanted or cannot be loaded."""
        if self._parakeet_model is None and config.STT_BACKEND in ("auto", "parakeet"):
            try:
                import onnx_asr

                model = onnx_asr.load_model(config.STT_MODEL_PARAKEET, quantization=config.STT_PARAKEET_QUANT or None)
                self._parakeet_model = model.with_timestamps()  # that form also gives each token's log-probability
                log.info("speech to text: %s (%s) through onnxruntime", config.STT_MODEL_PARAKEET, config.STT_PARAKEET_QUANT or "f32")
            except Exception as e:
                log.warning("Parakeet not usable (%s); using Whisper", str(e).splitlines()[0][:160])
                self._parakeet_model = False
        return self._parakeet_model or None

    def _infer_parakeet(self, model, audio: np.ndarray) -> dict:
        result = model.recognize(audio, sample_rate=SAMPLE_RATE)
        text = (result.text or "").strip()
        logprobs = [float(p) for p in (result.logprobs or [])]
        # The model writes the language it hears without naming it. The words name it, when they
        # can: two guesses from opposite starting points that agree. Otherwise it is left open and
        # the conversation's language so far stands.
        from .tts import detect_language

        guesses = {detect_language(text, default=code) for code in config.LANGUAGES[:2]} if text else set()
        language = guesses.pop() if len(guesses) == 1 else None
        if self.language is not None:
            language = self.language
        segment = {"text": text, "no_speech_prob": 0.0, "avg_logprob": sum(logprobs) / len(logprobs) if logprobs else 0.0, "compression_ratio": 1.0}
        return {"text": text, "language": language, "segments": [segment] if text else []}

    def _infer(self, audio: np.ndarray) -> dict:
        try:
            import mlx_whisper
        except ModuleNotFoundError:
            parakeet = self._parakeet()
            if parakeet is not None:
                return self._infer_parakeet(parakeet, audio)
            model = self._faster()
            language = self.language
            if language is None:  # the most likely of the configured languages, not of all hundred
                _, _, ranked = model.detect_language(audio)
                allowed = [(code, p) for code, p in ranked if code in config.LANGUAGES]
                language = max(allowed, key=lambda t: t[1])[0] if allowed else config.LANGUAGES[0]
            segments, info = model.transcribe(
                audio, language=language, task="transcribe", beam_size=1, temperature=0.0,
                condition_on_previous_text=False, without_timestamps=True,
                initial_prompt=self.initial_prompt if language == "en" else None,  # the prompt is English
            )  # fmt: skip
            found = [{"text": s.text, "no_speech_prob": s.no_speech_prob, "avg_logprob": s.avg_logprob, "compression_ratio": s.compression_ratio} for s in segments]
            return {"text": "".join(s["text"] for s in found), "language": info.language, "segments": found}

        return mlx_whisper.transcribe(
            audio,
            path_or_hf_repo=self.model_path(),
            language=self.language,
            task="transcribe",
            # One greedy pass. The temperature ladder re-decodes up to six times
            # on hard audio, which is exactly when latency matters least to
            # quality and most to the person waiting.
            temperature=0.0,
            condition_on_previous_text=False,
            initial_prompt=self.initial_prompt,
            without_timestamps=True,
            verbose=None,
        )

    def transcribe(self, data: bytes, mime: str | None = None) -> Transcript:
        """Blocking: decode, gate, transcribe, filter."""
        started = time.perf_counter()
        audio = decode_audio(data, mime)
        decode_ms = (time.perf_counter() - started) * 1000.0
        return self.transcribe_array(audio, decode_ms=decode_ms)

    def transcribe_array(self, audio: np.ndarray, decode_ms: float = 0.0) -> Transcript:
        result = Transcript(text="", audio_s=len(audio) / SAMPLE_RATE, decode_ms=decode_ms)
        if result.audio_s < 0.25:
            result.rejected = "too_short"
            return result
        speech_s, loud_db, floor_db = speech_stats(audio)
        result.speech_s = speech_s
        result.detail.update(loud_db=round(loud_db, 1), floor_db=round(floor_db, 1))
        if loud_db < -50.0 or speech_s < 0.15:
            result.rejected = "silence"
            return result
        started = time.perf_counter()
        voiced = self.detector.speech_seconds(audio)
        if voiced is not None:
            result.speech_s = voiced[0]
            result.detail.update(vad_s=round(voiced[0], 2), vad_peak=round(voiced[1], 2))
            result.detail["vad_ms"] = round((time.perf_counter() - started) * 1000.0, 1)
            if voiced[0] < 0.12:
                result.rejected = "no_voice"
                result.infer_ms = (time.perf_counter() - started) * 1000.0
                return result
            speech_s = voiced[0]

        output = self._infer(audio)
        result.infer_ms = (time.perf_counter() - started) * 1000.0
        result.language = output.get("language")
        result.raw_text = (output.get("text") or "").strip()

        kept: list[str] = []
        worst_no_speech, worst_logprob = 0.0, 0.0
        for segment in output.get("segments") or []:
            no_speech = float(segment.get("no_speech_prob", 0.0))
            logprob = float(segment.get("avg_logprob", 0.0))
            ratio = float(segment.get("compression_ratio", 1.0))
            text = (segment.get("text") or "").strip()
            if not text:
                continue
            if no_speech > 0.8 or logprob < -1.5 or ratio > 2.6:
                continue  # decoder itself doubts this is speech, or it is looping
            kept.append(text)
            worst_no_speech = max(worst_no_speech, no_speech)
            worst_logprob = min(worst_logprob, logprob)
        result.detail.update(
            no_speech_prob=round(worst_no_speech, 3), avg_logprob=round(worst_logprob, 3)
        )
        text = _BRACKETED.sub(" ", " ".join(kept))
        text = re.sub(r"\s+", " ", text).strip()
        if not text:
            result.rejected = "no_speech"
            return result

        reason = self._hallucination(text, speech_s, worst_no_speech, worst_logprob)
        if reason:
            result.rejected = reason
            return result
        result.text = text
        return result

    def _hallucination(
        self, text: str, speech_s: float, no_speech: float, logprob: float
    ) -> str | None:
        norm = _normalise(text)
        if not norm:
            return "no_words"
        if norm in _ALWAYS_HALLUCINATED:
            return "hallucination"
        if self.initial_prompt and len(norm) > 6 and norm in _normalise(self.initial_prompt):
            # On unintelligible audio Whisper sometimes just recites its prompt.
            # A person really saying those words decodes with high confidence.
            if speech_s < 0.4 or logprob < -0.35:
                return "prompt_echo"
        if norm in _WEAK_HALLUCINATED:
            weak = speech_s < 0.25 or no_speech > 0.35 or logprob < -0.8
            if weak:
                return "hallucination"
        words = norm.split()
        if len(words) >= 6 and len(set(words)) <= 2:
            return "repetition"
        return None

    async def atranscribe(self, data: bytes, mime: str | None = None) -> Transcript:
        return await asyncio.get_running_loop().run_in_executor(
            self._executor, self.transcribe, data, mime
        )
