/**
 * Talking to the robot: push-to-talk, the hands-free voice detector, and the
 * glue that feeds `speech` events to the player.
 */
import { api } from '../api/index.ts';
import { app, onLiveEvent } from '../state/app.ts';
import { hasAudioContext, resumeAudio } from './audio.ts';
import { Mic, type Recording } from './mic.ts';
import { speech } from './speech.ts';
import { voiceLive, voiceStore } from './state.ts';

/** Shorter than this is a slip of the finger, not an utterance. */
const MIN_UTTERANCE_MS = 280;
const MAX_UTTERANCE_MS = 15000;

/* Hands-free voice activity detection (energy with hysteresis and a hangover). */
const VAD_TICK_MS = 20;
/** Level must exceed the noise floor by this factor to start an utterance... */
const VAD_ON_FACTOR = 3.2;
/** ...and stay above this factor to count as still talking. */
const VAD_OFF_FACTOR = 2.0;
const VAD_ON_MIN = 0.02;
const VAD_OFF_MIN = 0.012;
/** Consecutive loud ticks needed to start (60 ms). */
const VAD_ATTACK_TICKS = 3;
/** Silence that ends an utterance. */
const VAD_HANGOVER_MS = 800;
/** An utterance needs at least this much voiced audio, or it was a click or a bump. */
const VAD_MIN_VOICED_MS = 240;
/** Stay deaf this long after the robot stops talking (speaker tail, room echo). */
const VAD_AFTER_SPEECH_MS = 450;

const mic = new Mic();
let pttHeld = false;
let pttRecording = false;
let vadTimer: number | undefined;
let vad = { floor: 0.006, loud: 0, active: false, startedAt: 0, lastVoiceAt: 0, voicedMs: 0, deafUntil: 0 };
let started = false;

function fail(err: unknown): void {
  voiceStore.set({
    micError: err instanceof Error ? err.message : 'Could not open the microphone.',
    recording: false,
    handsFree: false,
  });
}

async function send(rec: Recording | null): Promise<void> {
  if (!rec) return;
  voiceStore.set({ uploading: true, heard: null });
  try {
    const res = await api.utterance(rec.blob);
    voiceStore.set({ uploading: false, heard: res.text ? res.text : "didn't catch that" });
  } catch (err) {
    voiceStore.set({
      uploading: false,
      heard: null,
      micError: `Could not send that to the robot (${err instanceof Error ? err.message : 'network error'}).`,
    });
  }
}

/** Runs whenever the mic is open: meters the level and, in hands-free mode, segments speech. */
function tick(): void {
  const level = mic.level();
  voiceLive.micLevel = Math.min(1, level * 9);
  voiceLive.speechLevel = speech.level();

  const s = voiceStore.get();
  if (!s.handsFree || pttRecording || s.uploading) return;
  const now = performance.now();

  // Never listen to the robot's own voice.
  const robotTalking = speech.active || s.speaking || app.get().voiceState === 'speaking';
  if (robotTalking) vad.deafUntil = now + VAD_AFTER_SPEECH_MS;
  if (now < vad.deafUntil) {
    if (vad.active) {
      vad.active = false;
      mic.cancel();
      voiceStore.set({ recording: false });
    }
    vad.loud = 0;
    return;
  }

  if (!vad.active) {
    // Track the room's noise floor: quick to fall, slow to rise.
    vad.floor += (level - vad.floor) * (level < vad.floor ? 0.2 : 0.012);
    const on = Math.max(VAD_ON_MIN, vad.floor * VAD_ON_FACTOR);
    vad.loud = level > on ? vad.loud + 1 : 0;
    if (vad.loud >= VAD_ATTACK_TICKS) {
      vad = { ...vad, active: true, startedAt: now, lastVoiceAt: now, voicedMs: VAD_ATTACK_TICKS * VAD_TICK_MS, loud: 0 };
      mic.start();
      voiceStore.set({ recording: true, micError: null });
    }
    return;
  }

  const off = Math.max(VAD_OFF_MIN, vad.floor * VAD_OFF_FACTOR);
  if (level > off) {
    vad.lastVoiceAt = now;
    vad.voicedMs += VAD_TICK_MS;
  }
  const silent = now - vad.lastVoiceAt > VAD_HANGOVER_MS;
  const tooLong = now - vad.startedAt > MAX_UTTERANCE_MS;
  if (silent || tooLong) {
    vad.active = false;
    voiceStore.set({ recording: false });
    const voiced = vad.voicedMs;
    void mic.stop().then((rec) => (voiced >= VAD_MIN_VOICED_MS ? send(rec) : undefined));
  }
}

function ensureTicking(): void {
  if (vadTimer === undefined) vadTimer = window.setInterval(tick, VAD_TICK_MS);
}

export const voice = {
  /** Wire speech playback to the event stream. Call once. */
  start(): void {
    if (started) return;
    started = true;
    onLiveEvent((ev) => {
      if (ev.kind === 'speech') speech.enqueue(ev);
      else if (ev.kind === 'chat' && ev.role === 'user') speech.supersede(ev.turn_id);
    });
    // The first click or key press unlocks audio if the browser was holding it back.
    const unlock = () => {
      if (!hasAudioContext()) return;
      void resumeAudio().then((ok) => ok && voiceStore.set({ soundBlocked: false }));
    };
    window.addEventListener('pointerdown', unlock, true);
    window.addEventListener('keydown', unlock, true);
  },

  /** Push-to-talk pressed (Space or the talk button). */
  async pttDown(): Promise<void> {
    if (pttHeld) return;
    pttHeld = true;
    speech.stopAll(); // talking over the robot shuts it up
    if (vad.active) {
      vad.active = false;
      mic.cancel();
    }
    try {
      await mic.open();
    } catch (err) {
      pttHeld = false;
      fail(err);
      return;
    }
    ensureTicking();
    if (!pttHeld) return; // released while the permission prompt was up
    mic.start();
    pttRecording = true;
    voiceStore.set({ recording: true, micError: null, heard: null });
  },

  /** Push-to-talk released: send what was said. */
  async pttUp(): Promise<void> {
    if (!pttHeld) return;
    pttHeld = false;
    if (!pttRecording) return;
    pttRecording = false;
    voiceStore.set({ recording: false });
    const rec = await mic.stop();
    if (rec && rec.durationMs < MIN_UTTERANCE_MS) {
      voiceStore.set({ heard: 'hold it down while you talk' });
      return;
    }
    await send(rec);
  },

  async setHandsFree(on: boolean): Promise<void> {
    if (!on) {
      if (vad.active) {
        vad.active = false;
        mic.cancel();
      }
      voiceStore.set({ handsFree: false, recording: pttRecording });
      return;
    }
    try {
      await mic.open();
    } catch (err) {
      fail(err);
      return;
    }
    ensureTicking();
    vad = { ...vad, active: false, loud: 0, deafUntil: performance.now() + 300 };
    voiceStore.set({ handsFree: true, micError: null });
  },

  /** Stop button / Escape: silence the robot and drop any half-recorded utterance. */
  halt(): void {
    speech.stopAll();
    pttHeld = false;
    pttRecording = false;
    if (vad.active) vad.active = false;
    mic.cancel();
    voiceStore.set({ recording: false });
  },
};
