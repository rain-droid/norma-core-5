import { api } from './api/index.ts';
import type { Source, ToolResult } from './api/types.ts';
import { createStore } from './state/store.ts';
import { voice } from './voice/controller.ts';

export const SOURCE_LABEL: Record<Source, string> = {
  'claude-code': 'Claude Code',
  voice: 'voice',
  ui: 'UI',
};

export function sourceLabel(source: string | null | undefined): string {
  if (!source) return '';
  return SOURCE_LABEL[source as Source] ?? source;
}

/* ------------------------------------------------------------------ */
/* Stop                                                                */
/* ------------------------------------------------------------------ */

export interface StopState {
  /** Bumps on every press, so the button can flash. */
  presses: number;
  note: { ok: boolean; text: string; at: number } | null;
}

export const stopStore = createStore<StopState>({ presses: 0, note: null });

/**
 * The one stop path, used by the button and by Escape. Halts motion and the
 * agent's turn, and silences speech locally without waiting for either.
 */
export async function stopEverything(): Promise<void> {
  stopStore.set((s) => ({ presses: s.presses + 1 }));
  voice.halt();
  const [motion, turn] = await Promise.allSettled([api.stop(), api.interrupt()]);
  if (motion.status === 'fulfilled') {
    stopStore.set({ note: { ok: true, text: 'Stopped. Holding position.', at: Date.now() } });
  } else {
    stopStore.set({
      note: {
        ok: false,
        text: 'Stop did not reach the daemon. Cut power to the arm if it is still moving.',
        at: Date.now(),
      },
    });
  }
  void turn;
}

/* ------------------------------------------------------------------ */
/* Manual tool calls                                                   */
/* ------------------------------------------------------------------ */

export interface ManualState {
  /** Name of the call in flight, if any. */
  pending: string | null;
  result: { ok: boolean; text: string } | null;
}

export const manualStore = createStore<ManualState>({ pending: null, result: null });

function describe(res: ToolResult): { ok: boolean; text: string } {
  if (res.ok) return { ok: true, text: res.summary || 'done' };
  if (res.error === 'busy') return { ok: false, text: 'Busy: the arm is still doing something else.' };
  if (res.error === 'control') return { ok: false, text: `${res.summary ?? 'Someone else has control of the arm.'}` };
  return { ok: false, text: res.summary || res.error || 'That did not work.' };
}

/** Run a tool from the manual panel (`source: "ui"`), one at a time. */
export async function runTool(label: string, tool: string, args: Record<string, unknown> = {}): Promise<void> {
  if (manualStore.get().pending) return;
  manualStore.set({ pending: label });
  try {
    const res = await api.callTool(tool, args);
    manualStore.set({ pending: null, result: describe(res) });
  } catch (err) {
    manualStore.set({
      pending: null,
      result: { ok: false, text: err instanceof Error ? err.message : 'That did not work.' },
    });
  }
}

export async function setTorque(enable: boolean): Promise<void> {
  if (manualStore.get().pending) return;
  manualStore.set({ pending: 'torque' });
  try {
    await api.torque(enable);
    manualStore.set({
      pending: null,
      result: { ok: true, text: enable ? 'Torque on: the arm holds itself.' : 'Torque off: the arm is limp.' },
    });
  } catch (err) {
    manualStore.set({
      pending: null,
      result: { ok: false, text: err instanceof Error ? err.message : 'Could not change torque.' },
    });
  }
}
