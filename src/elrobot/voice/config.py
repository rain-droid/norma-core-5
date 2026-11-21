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


# --- speech to text ------------------------------------------------------
STT_MODEL = _env("ELROBOT_STT_MODEL", "mlx-community/whisper-large-v3-turbo")
STT_LANGUAGE = _env("ELROBOT_STT_LANGUAGE", "en")  # "auto" lets Whisper detect it
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
# Force one provider ("elevenlabs", "kokoro", "say") instead of the chain.
TTS_PROVIDER = os.environ.get("ELROBOT_TTS_PROVIDER", "").strip().lower()
SAY_VOICE = os.environ.get("ELROBOT_SAY_VOICE", "").strip()

# Generated clips are throwaway: keep them long enough for a browser to fetch.
AUDIO_MAX_AGE_S = _env_float("ELROBOT_AUDIO_MAX_AGE_S", 900.0)
AUDIO_MAX_CLIPS = int(_env_float("ELROBOT_AUDIO_MAX_CLIPS", 200))
