import type { CameraSource, Transport, TransportHandlers } from './transport.ts';
import type { ChatResult, ServerMessage, ToolResult, UtteranceResult, VisionFrame } from './types.ts';

const BACKOFF_MS = [500, 1000, 2000, 4000, 8000];
/** `state` arrives at ~30 Hz, so this much silence means the socket is dead. */
const SILENCE_MS = 4000;

async function post<T>(path: string, body?: unknown, form?: FormData): Promise<T> {
  const res = await fetch(path, {
    method: 'POST',
    headers: form ? undefined : { 'Content-Type': 'application/json' },
    body: form ?? JSON.stringify(body ?? {}),
  });
  const text = await res.text();
  let json: unknown = null;
  try {
    json = text ? JSON.parse(text) : null;
  } catch {
    /* not JSON */
  }
  if (!res.ok) {
    const detail =
      json && typeof json === 'object'
        ? ((json as Record<string, unknown>).error ?? (json as Record<string, unknown>).detail)
        : null;
    throw new Error(typeof detail === 'string' ? detail : `${path} answered ${res.status}`);
  }
  return json as T;
}

export class LiveTransport implements Transport {
  readonly kind = 'live' as const;
  private handlers: TransportHandlers | null = null;
  private ws: WebSocket | null = null;
  private attempt = 0;
  private retryTimer: number | undefined;
  private watchdog: number | undefined;
  private lastMessageAt = 0;
  private disposed = false;

  start(handlers: TransportHandlers): void {
    this.handlers = handlers;
    this.connect();
    this.watchdog = window.setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN && Date.now() - this.lastMessageAt > SILENCE_MS) {
        this.ws.close();
      }
    }, 1000);
  }

  dispose(): void {
    this.disposed = true;
    window.clearTimeout(this.retryTimer);
    window.clearInterval(this.watchdog);
    const ws = this.ws;
    this.ws = null;
    if (ws) {
      ws.onopen = ws.onclose = ws.onerror = ws.onmessage = null;
      ws.close();
    }
  }

  retryNow(): void {
    if (this.disposed || this.ws) return;
    window.clearTimeout(this.retryTimer);
    this.connect();
  }

  private connect(): void {
    if (this.disposed) return;
    this.handlers?.onLink('connecting', null);
    const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
    let ws: WebSocket;
    try {
      ws = new WebSocket(`${scheme}://${location.host}/ws`);
    } catch {
      this.scheduleRetry();
      return;
    }
    this.ws = ws;

    ws.onopen = () => {
      this.lastMessageAt = Date.now();
    };
    ws.onmessage = (e) => {
      this.lastMessageAt = Date.now();
      if (typeof e.data !== 'string') return;
      let msg: ServerMessage;
      try {
        msg = JSON.parse(e.data) as ServerMessage;
      } catch {
        return;
      }
      // The link counts as open once the daemon has introduced itself.
      if (msg.type === 'hello') this.attempt = 0;
      this.handlers?.onMessage(msg);
    };
    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.ws = null;
      this.scheduleRetry();
    };
    ws.onerror = () => {
      /* onclose follows and handles the retry */
    };
  }

  private scheduleRetry(): void {
    if (this.disposed) return;
    const base = BACKOFF_MS[Math.min(this.attempt, BACKOFF_MS.length - 1)];
    const delay = base * (0.85 + Math.random() * 0.3);
    this.attempt++;
    this.handlers?.onLink('closed', Date.now() + delay);
    this.retryTimer = window.setTimeout(() => this.connect(), delay);
  }

  async callTool(tool: string, args: Record<string, unknown>): Promise<ToolResult> {
    try {
      return await post<ToolResult>(`/api/tools/${encodeURIComponent(tool)}`, { args, source: 'ui' });
    } catch (err) {
      return { ok: false, error: err instanceof Error ? err.message : String(err) };
    }
  }

  async stop(): Promise<void> {
    await post('/api/stop');
  }

  async interrupt(): Promise<void> {
    await post('/api/chat/interrupt');
  }

  async torque(enable: boolean): Promise<void> {
    await post('/api/torque', { enable });
  }

  async collab(enable: boolean): Promise<void> {
    await post('/api/collab', { enable });
  }

  chat(text: string): Promise<ChatResult> {
    return post<ChatResult>('/api/chat', { text });
  }

  utterance(audio: Blob): Promise<UtteranceResult> {
    const form = new FormData();
    const ext = audio.type.includes('mp4') ? 'mp4' : audio.type.includes('ogg') ? 'ogg' : 'webm';
    form.append('audio', audio, `utterance.${ext}`);
    return post<UtteranceResult>('/api/voice/utterance', undefined, form);
  }

  camera(name: string): CameraSource {
    return { kind: 'mjpeg', url: `/api/camera/${encodeURIComponent(name)}/stream.mjpg` };
  }

  async vision(name: string): Promise<VisionFrame | null> {
    try {
      const res = await fetch(`/api/camera/${encodeURIComponent(name)}/vision`, { cache: 'no-store' });
      return res.ok ? ((await res.json()) as VisionFrame) : null;
    } catch {
      return null;
    }
  }
}
