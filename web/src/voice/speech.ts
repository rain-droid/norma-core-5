/**
 * Plays the robot's synthesised sentences. `speech` events carry a URL per
 * sentence; they are played strictly in `seq` order within a turn, one turn
 * at a time, and everything stops at once on interrupt.
 */
import type { SpeechEvent } from '../api/types.ts';
import { getAudioContext, resumeAudio } from './audio.ts';
import { voiceLive, voiceStore } from './state.ts';

interface Item {
  seq: number;
  text: string;
  buffer: Promise<AudioBuffer | null>;
}

interface Turn {
  id: string;
  items: Map<number, Item>;
  /** Next seq to play; null until the first sentence has been chosen. */
  next: number | null;
  firstAt: number;
  /** When we started waiting for a missing seq. */
  gapSince: number | null;
}

/** Sentence 0 may arrive just after sentence 1; give it this long before starting. */
const FIRST_GRACE_MS = 350;
/** A sentence that never arrives is skipped after this long. */
const GAP_TIMEOUT_MS = 2500;
/** Keep reporting "speaking" briefly after the last sample, for the mic gate. */
const TAIL_MS = 350;

class SpeechPlayer {
  private turns: Turn[] = [];
  private dead = new Set<string>();
  private current: AudioBufferSourceNode | null = null;
  private analyser: AnalyserNode | null = null;
  private samples: Uint8Array<ArrayBuffer> | null = null;
  private timer: number | undefined;
  private tailTimer: number | undefined;
  private generation = 0;

  /** True while audio is playing or queued. The hands-free mic stays shut. */
  get active(): boolean {
    return this.current !== null || this.turns.some((t) => t.items.size > 0);
  }

  enqueue(ev: SpeechEvent): void {
    if (!ev.url || this.dead.has(ev.turn_id)) return;
    let turn = this.turns.find((t) => t.id === ev.turn_id);
    if (!turn) {
      turn = { id: ev.turn_id, items: new Map(), next: null, firstAt: performance.now(), gapSince: null };
      this.turns.push(turn);
    }
    if (turn.next !== null && ev.seq < turn.next) return; // already past it
    if (turn.items.has(ev.seq)) return;
    turn.items.set(ev.seq, { seq: ev.seq, text: ev.text ?? '', buffer: this.fetchBuffer(ev.url) });
    this.pump();
  }

  /** A new user turn began: anything older is no longer worth saying. */
  supersede(turnId: string): void {
    const stale = this.turns.filter((t) => t.id !== turnId);
    if (stale.length === 0) return;
    stale.forEach((t) => this.dead.add(t.id));
    this.turns = this.turns.filter((t) => t.id === turnId);
    this.halt();
    this.pump();
  }

  /** Interrupt: silence now, and ignore late sentences from these turns. */
  stopAll(): void {
    this.turns.forEach((t) => this.dead.add(t.id));
    this.turns = [];
    this.halt();
    this.settle();
  }

  /** 0..1 loudness of what is playing right now (drives the face's mouth). */
  level(): number {
    if (!this.analyser || !this.samples || !this.current) return 0;
    this.analyser.getByteTimeDomainData(this.samples);
    let sum = 0;
    for (let i = 0; i < this.samples.length; i++) {
      const v = (this.samples[i] - 128) / 128;
      sum += v * v;
    }
    return Math.min(1, Math.sqrt(sum / this.samples.length) * 3.2);
  }

  private halt(): void {
    this.generation++;
    window.clearTimeout(this.timer);
    const src = this.current;
    this.current = null;
    if (src) {
      src.onended = null;
      try {
        src.stop();
      } catch {
        /* already stopped */
      }
    }
  }

  private async fetchBuffer(url: string): Promise<AudioBuffer | null> {
    try {
      const res = await fetch(url);
      if (!res.ok) return null;
      const data = await res.arrayBuffer();
      return await getAudioContext().decodeAudioData(data);
    } catch {
      return null;
    }
  }

  private later(ms: number): void {
    window.clearTimeout(this.timer);
    this.timer = window.setTimeout(() => this.pump(), ms);
  }

  private pump(): void {
    if (this.current) return;
    const now = performance.now();

    // Finished turns leave the queue, except the newest (more may be coming).
    while (this.turns.length > 1 && this.turns[0].items.size === 0) this.turns.shift();
    const turn = this.turns[0];
    if (!turn || turn.items.size === 0) {
      this.settle();
      return;
    }

    const lowest = Math.min(...turn.items.keys());
    if (turn.next === null) {
      if (lowest > 0 && now - turn.firstAt < FIRST_GRACE_MS) {
        this.later(FIRST_GRACE_MS - (now - turn.firstAt) + 5);
        return;
      }
      turn.next = lowest;
    }

    let item = turn.items.get(turn.next);
    if (!item) {
      turn.gapSince ??= now;
      if (now - turn.gapSince < GAP_TIMEOUT_MS) {
        this.later(GAP_TIMEOUT_MS - (now - turn.gapSince) + 5);
        return;
      }
      turn.next = lowest;
      item = turn.items.get(lowest)!;
    }
    turn.gapSince = null;
    turn.items.delete(item.seq);
    turn.next = item.seq + 1;
    void this.play(item);
  }

  private async play(item: Item): Promise<void> {
    const generation = this.generation;
    // Hold the slot so pump() does not start a second sentence while this one decodes.
    const placeholder = {} as AudioBufferSourceNode;
    this.current = placeholder;
    window.clearTimeout(this.tailTimer);
    voiceStore.set({ speaking: true, caption: item.text });

    const buffer = await item.buffer;
    if (generation !== this.generation) return;
    const ctx = getAudioContext();
    if (ctx.state !== 'running') {
      await resumeAudio();
      if (generation !== this.generation) return;
    }
    if (!buffer || ctx.state !== 'running') {
      // No audio (blocked or failed to load): show the words for as long as
      // they would have taken, so captions still keep pace with the turn.
      if (ctx.state !== 'running') voiceStore.set({ soundBlocked: true });
      const ms = buffer ? buffer.duration * 1000 : Math.max(900, item.text.length * 55);
      this.timer = window.setTimeout(() => {
        if (generation !== this.generation) return;
        this.current = null;
        this.pump();
      }, ms);
      return;
    }
    voiceStore.set({ soundBlocked: false });

    if (!this.analyser) {
      this.analyser = ctx.createAnalyser();
      this.analyser.fftSize = 512;
      this.samples = new Uint8Array(new ArrayBuffer(this.analyser.fftSize));
      this.analyser.connect(ctx.destination);
    }
    const src = ctx.createBufferSource();
    src.buffer = buffer;
    src.connect(this.analyser);
    src.onended = () => {
      if (this.current !== src) return;
      this.current = null;
      this.pump();
    };
    this.current = src;
    src.start();
  }

  /** Nothing left to say. */
  private settle(): void {
    if (this.current) return;
    voiceLive.speechLevel = 0;
    window.clearTimeout(this.tailTimer);
    this.tailTimer = window.setTimeout(() => {
      if (!this.active) voiceStore.set({ speaking: false, caption: '' });
    }, TAIL_MS);
  }
}

export const speech = new SpeechPlayer();
