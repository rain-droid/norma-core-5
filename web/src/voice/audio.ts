/** One shared AudioContext, created only when audio is first needed. */

let ctx: AudioContext | null = null;

export function getAudioContext(): AudioContext {
  if (!ctx) ctx = new AudioContext({ latencyHint: 'interactive' });
  return ctx;
}

export function hasAudioContext(): boolean {
  return ctx !== null;
}

/** Browsers keep audio suspended until the person interacts with the page. */
export async function resumeAudio(): Promise<boolean> {
  const c = getAudioContext();
  if (c.state === 'running') return true;
  try {
    await Promise.race([c.resume(), new Promise((r) => setTimeout(r, 250))]);
  } catch {
    /* still blocked */
  }
  // resume() changed the state behind TypeScript's back, so re-read it untyped.
  return (c.state as AudioContextState) === 'running';
}
