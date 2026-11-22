"""Paths and environment knobs for the voice layer, in one place.

Everything here is read once at import. All of it can be overridden by env so
the daemon never needs to pass configuration through `VoiceService`.
"""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path


def _env(name: str, default: str) -> str:
    value = os.environ.get(name, "").strip()
    return value or default


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


# src/elrobot/voice/config.py -> repo root is three levels above the package dir.
REPO_ROOT = Path(_env("ELROBOT_ROOT", str(Path(__file__).resolve().parents[3])))
MODELS_DIR = Path(_env("ELROBOT_MODELS_DIR", str(REPO_ROOT / "models")))
AUDIO_DIR = Path(_env("ELROBOT_AUDIO_DIR", str(REPO_ROOT / "data" / "audio")))
# The Claude CLI files its transcripts under the cwd it runs in. A dedicated
# directory keeps the voice agent's sessions out of the repo's own Claude Code
# session list, and it must stay the same across restarts for resume to work.
SESSION_DIR = Path(_env("ELROBOT_VOICE_SESSION_DIR", str(REPO_ROOT / "data" / "voice-session")))

FFMPEG = _env("ELROBOT_FFMPEG", shutil.which("ffmpeg") or "/opt/homebrew/bin/ffmpeg")

# --- brain ---------------------------------------------------------------
VOICE_MODEL = _env("ELROBOT_VOICE_MODEL", "claude-sonnet-5-5")
# "low" is what keeps time-to-first-word under a second: from "medium" up the
# model thinks briefly before almost every reply, even a greeting.
VOICE_EFFORT = _env("ELROBOT_VOICE_EFFORT", "low")
# Used only when the main model is overloaded or unavailable. "none" disables.
VOICE_FALLBACK_MODEL = _env("ELROBOT_VOICE_FALLBACK_MODEL", "claude-haiku-5-5")
# Model requests allowed inside one user turn (ack, tool, retry, ... wrap-up).
VOICE_MAX_STEPS = int(_env_float("ELROBOT_VOICE_MAX_STEPS", 16))
# Longest silence from the CLI inside a turn before we call it hung. A single
# tool call (one motion) has to fit in this.
VOICE_STALL_TIMEOUT = _env_float("ELROBOT_VOICE_STALL_TIMEOUT", 180.0)
# If the model goes straight to a tool without saying anything, the service
# says a short filler itself so the arm never starts moving in silence.
VOICE_FILLER = _env("ELROBOT_VOICE_FILLER", "1").lower() not in ("0", "false", "no", "off")
PERSONA_PATH = Path(_env("ELROBOT_VOICE_PERSONA", str(Path(__file__).with_name("persona.md"))))
MCP_SERVER_NAME = "elrobot"

# Which hands the chat and voice agent has. "workspace" (the default when workspace/ exists): it
# runs in the agent workspace and uses its tools, skills and world model, the same harness as a
# Claude Code session started there. "mcp": the daemon's own tool registry through the MCP shim.
CHAT_AGENT = _env("ELROBOT_CHAT_AGENT", "workspace").lower()
WORKSPACE_DIR = Path(_env("ELROBOT_WORKSPACE_DIR", str(REPO_ROOT / "workspace")))
WORKSPACE_PERSONA_PATH = Path(_env("ELROBOT_WORKSPACE_PERSONA", str(Path(__file__).with_name("persona_workspace.md"))))
# A workspace task is many small steps (look, open the picture, name, grasp, check, put, look).
WORKSPACE_MAX_STEPS = int(_env_float("ELROBOT_WORKSPACE_MAX_STEPS", 80))


def workspace_agent() -> bool:
    return CHAT_AGENT == "workspace" and (WORKSPACE_DIR / "AGENTS.md").is_file()


def claude_cli_path() -> str | None:
    """The locally logged-in Claude Code CLI; None lets the SDK use its bundled copy."""
    explicit = os.environ.get("ELROBOT_CLAUDE_CLI", "").strip()
    if explicit:
        return explicit
    for candidate in (Path.home() / ".local/bin/claude", shutil.which("claude")):
        if candidate and Path(candidate).is_file():
            return str(candidate)
    return None


def default_mcp_command() -> list[str]:
    """The real robot MCP server, as documented for the daemon."""
    return ["uv", "run", "--project", str(REPO_ROOT), "elrobot-mcp"]


def stub_mcp_command() -> list[str]:
    return [sys.executable, "-m", "elrobot.voice.stub_mcp"]


# --- languages -----------------------------------------------------------
# The languages people may speak and type to the robot, and it answers in. With more than one,
# speech to text works out which is being spoken, and each reply is voiced in its own language.
LANGUAGES = [code.strip() for code in _env("ELROBOT_LANGUAGES", "en,fr,de").lower().split(",") if code.strip()] or ["en"]
MULTILINGUAL = len(LANGUAGES) > 1

# --- speech to text ------------------------------------------------------
STT_MODEL = _env("ELROBOT_STT_MODEL", "mlx-community/whisper-large-v3-turbo")
# Which recogniser runs where MLX does not exist. "parakeet": NVIDIA Parakeet TDT 0.6B v3 through
# onnxruntime (25 European languages, the language found by itself, CC BY 4.0; a command takes about
# a third of a second on a laptop processor). "whisper": faster-whisper, below. "auto": Parakeet
# when its package is installed, else Whisper; Whisper also takes over if Parakeet fails to load.
STT_BACKEND = _env("ELROBOT_STT_BACKEND", "auto").lower()
STT_MODEL_PARAKEET = _env("ELROBOT_STT_MODEL_PARAKEET", "nemo-parakeet-tdt-0.6b-v3")
STT_PARAKEET_QUANT = _env("ELROBOT_STT_PARAKEET_QUANT", "int8")
# faster-whisper runs this model: a
# model size or a Hugging Face repository in CTranslate2 format. "small.en" answers a short command
# in about a second on a laptop processor. ELROBOT_STT_DEVICE=cpu skips the attempt to use CUDA.
# With several languages it has to be a multilingual model ("small", not "small.en").
STT_MODEL_FASTER = _env("ELROBOT_STT_MODEL_FASTER", "small" if MULTILINGUAL else "small.en")
STT_DEVICE = _env("ELROBOT_STT_DEVICE", "auto").lower()
STT_LANGUAGE = _env("ELROBOT_STT_LANGUAGE", "auto" if MULTILINGUAL else "en")  # "auto": the one of LANGUAGES being spoken
# Biases Whisper toward the one word it reliably gets wrong here ("dap" comes
# out as "dat", "dab" or "step" without it). Kept short and harmless on
# purpose: on unintelligible audio Whisper may recite its prompt, and reciting
# this costs a fist bump, not a grab. Set to "none" to disable.
STT_PROMPT = _env("ELROBOT_STT_PROMPT", "Yo, dap me up! Nice fist bump.")
VAD_MODEL_URL = _env(
    "ELROBOT_VAD_MODEL_URL",
    "https://github.com/snakers4/silero-vad/raw/master/src/silero_vad/data/silero_vad.onnx",
)

# --- text to speech ------------------------------------------------------
ELEVENLABS_API_KEY = os.environ.get("ELEVENLABS_API_KEY", "").strip()
ELEVENLABS_VOICE_ID = _env("ELEVENLABS_VOICE_ID", "nPczCjzI2devNBz1zQrb")  # "Brian"
ELEVENLABS_MODEL_ID = _env("ELEVENLABS_MODEL_ID", "eleven_flash_v2_5")
# Gradium's Phonon (the `ptts` package): the preferred local voice. Any name from its catalogue of
# about a hundred voices works; `uvx ptts --help` and huggingface.co/gradium/phonon-voices list them.
# English alone uses the 31M English model (Apache 2.0). With more languages it is the 40M
# multilingual model (English, French, German, Spanish, Portuguese): CC BY-NC, not for commercial
# use, and gated on Hugging Face, so the machine must be logged in to an account that accepted it.
# If that model cannot be loaded, the English one stands in and speaks English only.
PHONON_MODEL_EN = "hf://gradium/phonon-31m-en"
PHONON_MODEL = _env("ELROBOT_PHONON_MODEL", "hf://gradium/phonon-40m-multilingual" if MULTILINGUAL else PHONON_MODEL_EN)
# One voice per language; ELROBOT_PHONON_VOICE_FR and so on choose another from the catalogue.
PHONON_VOICES = {"en": "Easton", "fr": "Baptiste", "de": "Mirko", "es": "", "pt": ""}
PHONON_VOICES["en"] = _env("ELROBOT_PHONON_VOICE", PHONON_VOICES["en"])
for _code in list(PHONON_VOICES):
    PHONON_VOICES[_code] = _env(f"ELROBOT_PHONON_VOICE_{_code.upper()}", PHONON_VOICES[_code])
PHONON_QUANT = _env("ELROBOT_PHONON_QUANT", "q8")
KOKORO_DIR = MODELS_DIR / "kokoro"
KOKORO_MODEL_URL = _env(
    "ELROBOT_KOKORO_MODEL_URL",
    "https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.0/kokoro-v1.0.onnx",
)
KOKORO_VOICES_URL = _env(
    "ELROBOT_KOKORO_VOICES_URL",
    "https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.0/voices-v1.0.bin",
)
TTS_VOICE = _env("ELROBOT_TTS_VOICE", "am_puck")
TTS_SPEED = _env_float("ELROBOT_TTS_SPEED", 1.1)
# Force one provider ("elevenlabs", "phonon", "kokoro", "say") instead of the chain.
TTS_PROVIDER = os.environ.get("ELROBOT_TTS_PROVIDER", "").strip().lower()
SAY_VOICE = os.environ.get("ELROBOT_SAY_VOICE", "").strip()

# Generated clips are throwaway: keep them long enough for a browser to fetch.
AUDIO_MAX_AGE_S = _env_float("ELROBOT_AUDIO_MAX_AGE_S", 900.0)
AUDIO_MAX_CLIPS = int(_env_float("ELROBOT_AUDIO_MAX_CLIPS", 200))
