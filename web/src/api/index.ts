import { app, ingest, resetSession } from '../state/app.ts';
import { LiveTransport } from './live.ts';
import { MockTransport } from './mock/mock.ts';
import type { Transport } from './transport.ts';

// Survive hot module reloads in dev: the page keeps its connection.
let transport: Transport | null = (import.meta.hot?.data.transport as Transport | undefined) ?? null;
import.meta.hot?.dispose((data) => {
  data.transport = transport;
});

/** Connect to the daemon (`live`) or start the in-browser demo (`mock`). */
export function startTransport(kind: 'live' | 'mock'): void {
  transport?.dispose();
  resetSession();
  transport = kind === 'mock' ? new MockTransport() : new LiveTransport();
  if (import.meta.env.DEV && kind === 'mock') {
    (window as unknown as { __elrobotMock?: Transport }).__elrobotMock = transport;
  }
  app.set({ transport: kind, link: 'connecting', retryAt: null, everOpened: false });
  transport.start({
    onMessage: ingest,
    onLink: (link, retryAt) => app.set({ link, retryAt }),
  });
}

/** Switch to demo mode and keep it across reloads. */
export function enterDemoMode(): void {
  const url = new URL(location.href);
  url.searchParams.set('mock', '1');
  history.replaceState(null, '', url);
  startTransport('mock');
}

function active(): Transport {
  if (!transport) throw new Error('transport not started');
  return transport;
}

/** The daemon API as the UI uses it; every call goes to the active transport. */
export const api = {
  callTool: (tool: string, args: Record<string, unknown> = {}) => active().callTool(tool, args),
  stop: () => active().stop(),
  interrupt: () => active().interrupt(),
  torque: (enable: boolean) => active().torque(enable),
  chat: (text: string) => active().chat(text),
  utterance: (audio: Blob) => active().utterance(audio),
  camera: (name: string) => active().camera(name),
  retryNow: () => active().retryNow(),
};
