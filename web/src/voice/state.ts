import { createStore } from '../state/store.ts';

export interface VoiceUiState {
  /** The local mic is capturing an utterance right now. */
  recording: boolean;
  /** An utterance is on its way to the daemon. */
  uploading: boolean;
  handsFree: boolean;
  /** Why the mic is unavailable, in words the person can act on. */
  micError: string | null;
  /** What the daemon last heard, or a note when it heard nothing. */
  heard: string | null;
  /** Speech audio is playing (or queued). */
  speaking: boolean;
  /** The sentence being spoken. */
  caption: string;
  /** The browser refused to play audio until the page is clicked. */
  soundBlocked: boolean;
}

export const voiceStore = createStore<VoiceUiState>({
  recording: false,
  uploading: false,
  handsFree: false,
  micError: null,
  heard: null,
  speaking: false,
  caption: '',
  soundBlocked: false,
});

/** Per-frame levels (0..1), read by the face without going through React. */
export const voiceLive = { micLevel: 0, speechLevel: 0 };
