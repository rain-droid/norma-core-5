"""Conversational voice layer: hear the user, answer out loud, drive the arm.

Public surface (see `service.py`):

    from elrobot.voice import VoiceService, create_router
"""

from __future__ import annotations

__all__ = ["VoiceService", "create_router"]


def __getattr__(name: str):
    # Lazy so `import elrobot.voice.stub_mcp` (the MCP subprocess) does not pull
    # in fastapi, whisper and the rest of the service.
    if name in __all__:
        from . import service

        return getattr(service, name)
    raise AttributeError(name)
