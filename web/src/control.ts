/**
 * This page's place in the line for the arm (docs/API.md, "Control").
 *
 * The daemon moves the arm only for whoever holds control. While nobody does, single commands
 * from any page still run, so the buttons work without asking first. Asking for control puts
 * this page in the queue; once granted, nobody else (the agent included) can move the arm
 * until it is released here, this page goes quiet, or someone takes it over.
 */
import { api } from './api/index.ts';
import { controlToken } from './api/controlToken.ts';
import type { ControlReply } from './api/types.ts';
import { createStore } from './state/store.ts';

const NAME_KEY = 'elrobot.name';
/** The daemon drops a ticket after 20 s of silence; this keeps ours alive with room to spare. */
const HEARTBEAT_MS = 4000;

export interface MyControl {
  name: string;
  state: 'none' | 'waiting' | 'granted';
  position: number | null;
  pending: boolean;
  error: string | null;
}

function savedName(): string {
  try {
    const n = localStorage.getItem(NAME_KEY);
    if (n) return n;
  } catch {
    /* private mode */
  }
  return `browser ${Math.random().toString(36).slice(2, 5)}`;
}

export const myControl = createStore<MyControl>({ name: savedName(), state: 'none', position: null, pending: false, error: null });

let beat: number | undefined;

function apply(reply: ControlReply): void {
  const state = reply.you?.state ?? 'none';
  if (state === 'none') {
    forget();
  } else {
    if (reply.token) controlToken.set(reply.token);
    if (beat === undefined) beat = window.setInterval(() => void heartbeat(), HEARTBEAT_MS);
  }
  myControl.set({ state, position: reply.you?.position ?? null, error: null });
}

function forget(): void {
  controlToken.set(null);
  window.clearInterval(beat);
  beat = undefined;
  myControl.set({ state: 'none', position: null });
}

async function heartbeat(): Promise<void> {
  if (!controlToken.get()) return forget();
  try {
    apply(await api.control('request', { name: myControl.get().name, kind: 'human' }));
  } catch {
    /* the link is down; the next beat tries again, and the daemon forgets us if it stays down */
  }
}

async function run(action: 'request' | 'take'): Promise<void> {
  if (myControl.get().pending) return;
  myControl.set({ pending: true, error: null });
  try {
    apply(await api.control(action, { name: myControl.get().name, kind: 'human' }));
  } catch (err) {
    myControl.set({ error: err instanceof Error ? err.message : 'The daemon did not answer.' });
  } finally {
    myControl.set({ pending: false });
  }
}

/** Get in line for the arm (or get it at once if nobody holds it). */
export const requestControl = () => run('request');

/** Take the arm now, ahead of the queue; whatever the holder has running stops. */
export const takeControl = () => run('take');

/** Give the arm back, or leave the queue. */
export async function releaseControl(): Promise<void> {
  const had = controlToken.get();
  forget();
  if (!had) return;
  try {
    await api.control('release', { control: had });
  } catch {
    /* it lapses on its own once the heartbeat stops */
  }
}

export function setControlName(name: string): void {
  const n = name.trim().slice(0, 40);
  if (!n) return;
  myControl.set({ name: n });
  try {
    localStorage.setItem(NAME_KEY, n);
  } catch {
    /* private mode: just not remembered */
  }
}

// Closing the tab gives the arm back at once instead of after the timeout.
window.addEventListener('pagehide', () => {
  const t = controlToken.get();
  if (t) navigator.sendBeacon('/api/control/release', new Blob([JSON.stringify({ control: t })], { type: 'application/json' }));
});
