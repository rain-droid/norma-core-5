import { useEffect, useRef, useState } from 'react';
import { sourceLabel } from '../actions.ts';
import { app, type ToolItem } from '../state/app.ts';
import { shallowEqual, useStore } from '../state/store.ts';
import { voice } from '../voice/controller.ts';
import { voiceLive, voiceStore } from '../voice/state.ts';
import { formatArgs } from './Timeline.tsx';
import { MicIcon, SoundOffIcon } from './icons.tsx';

type Mood = 'idle' | 'listening' | 'transcribing' | 'thinking' | 'speaking' | 'acting' | 'oops' | 'offline';

/** How long the face winces after a tool call fails. */
const OOPS_MS = 2200;

/** The robot's own words for each state. */
const MOOD_WORD: Record<Mood, string> = {
  idle: 'chillin',
  listening: 'listening',
  transcribing: 'one sec',
  thinking: 'thinking',
  speaking: 'talking',
  acting: 'on it',
  oops: 'my bad',
  offline: 'offline',
};

function lastSentence(text: string): string {
  const parts = text.trim().split(/(?<=[.!?])\s+/);
  return parts[parts.length - 1] ?? '';
}

export function VoiceDock() {
  const link = useStore(app, (s) => s.link);
  const voiceState = useStore(app, (s) => s.voiceState);
  const busy = useStore(app, (s) => s.status?.busy ?? false);
  const activity = useStore(app, (s) => s.status?.activity ?? '');
  const source = useStore(app, (s) => s.status?.source ?? null);
  const failure = useStore(app, (s) => s.failure);
  const v = useStore(voiceStore, (s) => s, shallowEqual);

  // What is being worked on / said, for the caption line.
  const context = useStore(
    app,
    (s) => {
      let running: ToolItem | null = null;
      let failed: ToolItem | null = null;
      let said = '';
      let asked = '';
      for (let i = s.timeline.length - 1; i >= 0 && i >= s.timeline.length - 40; i--) {
        const it = s.timeline[i];
        if (it.kind === 'tool') {
          if (!running && it.status === 'running') running = it;
          if (!failed && it.status === 'failed') failed = it;
        } else if (it.kind === 'chat') {
          if (!said && it.role === 'assistant') said = it.text;
          if (!asked && it.role === 'user') asked = it.text;
        }
      }
      return {
        running: running ? `${running.tool} ${formatArgs(running.args)}`.trim() : '',
        failed: failed ? failed.summary || failed.error || '' : '',
        said,
        asked,
      };
    },
    shallowEqual,
  );

  const [oops, setOops] = useState(false);
  useEffect(() => {
    if (!failure) return;
    const left = OOPS_MS - (Date.now() - failure.at);
    if (left <= 0) return;
    setOops(true);
    const id = window.setTimeout(() => setOops(false), left);
    return () => window.clearTimeout(id);
  }, [failure]);

  let mood: Mood;
  if (link !== 'open') mood = 'offline';
  else if (v.recording) mood = 'listening';
  else if (oops) mood = 'oops';
  else if (v.uploading) mood = 'transcribing';
  else if (v.speaking) mood = 'speaking';
  else if (voiceState !== 'idle') mood = voiceState;
  else if (busy) mood = 'acting';
  else mood = 'idle';

  let caption: string;
  let tone: 'error' | undefined;
  if (v.micError && mood !== 'listening') {
    caption = v.micError;
    tone = 'error';
  } else {
    switch (mood) {
      case 'offline':
        caption = 'no link to the daemon';
        break;
      case 'listening':
        caption = v.handsFree ? "go ahead, I'm all ears" : "let go when you're done";
        break;
      case 'transcribing':
        caption = 'catching what you said';
        break;
      case 'thinking':
        caption = context.asked ? `“${context.asked}”` : 'working it out';
        break;
      case 'speaking':
        caption = v.caption || lastSentence(context.said);
        break;
      case 'acting':
        caption = context.running || activity || 'moving';
        if (source && source !== 'voice') caption += ` (for ${sourceLabel(source)})`;
        break;
      case 'oops':
        caption = context.failed || 'that did not work';
        break;
      default:
        caption = v.heard
          ? `heard: ${v.heard}`
          : v.handsFree
            ? 'hands-free is on, just talk'
            : 'hold space and talk to me';
    }
  }

  // Audio levels drive the eyes and mouth every frame, outside React.
  const faceRef = useRef<HTMLDivElement>(null);
  const talkRef = useRef<HTMLButtonElement>(null);
  const moodRef = useRef(mood);
  moodRef.current = mood;
  useEffect(() => {
    let raf = 0;
    let smooth = 0;
    const loop = (t: number) => {
      raf = requestAnimationFrame(loop);
      const m = moodRef.current;
      let level = 0;
      if (m === 'listening') level = voiceLive.micLevel;
      else if (m === 'speaking') {
        // With real speech audio the mouth follows it; without (demo mode), it just chatters.
        level = voiceLive.speechLevel > 0.01 ? voiceLive.speechLevel : 0.35 + 0.35 * Math.sin(t / 70) * Math.sin(t / 190);
      }
      smooth += (level - smooth) * 0.45;
      faceRef.current?.style.setProperty('--level', smooth.toFixed(3));
      talkRef.current?.style.setProperty('--mic', voiceLive.micLevel.toFixed(3));
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, []);

  // The on-screen talk button behaves like the space bar: hold to record.
  const press = (e: React.PointerEvent<HTMLButtonElement>) => {
    if (e.button !== 0) return;
    e.currentTarget.setPointerCapture(e.pointerId);
    void voice.pttDown();
  };
  const release = () => void voice.pttUp();

  return (
    <>
      {v.soundBlocked && (
        <div className="notice" role="status">
          <SoundOffIcon />
          The browser is holding the sound back. Click anywhere to hear the robot.
        </div>
      )}
      <div className="dock">
        <div className="face" data-mood={mood} ref={faceRef} role="img" aria-label={`Robot is ${mood}`}>
          <div className="face__eyes">
            <span className="eye eye--l" />
            <span className="eye eye--r" />
          </div>
          <span className="face__mouth" />
        </div>

        <div className="dock__words" aria-live="polite">
          <div className="dock__mood">{MOOD_WORD[mood]}</div>
          <div className="dock__caption" data-tone={tone} title={caption}>
            {caption}
          </div>
        </div>

        <div className="dock__controls">
          <button
            ref={talkRef}
            type="button"
            className="talk"
            data-live={v.recording}
            disabled={link !== 'open'}
            onPointerDown={press}
            onPointerUp={release}
            onPointerCancel={release}
            onContextMenu={(e) => e.preventDefault()}
            onKeyDown={(e) => {
              if ((e.key === 'Enter' || e.key === ' ') && !e.repeat) {
                e.preventDefault();
                void voice.pttDown();
              }
            }}
            onKeyUp={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault();
                void voice.pttUp();
              }
            }}
            aria-label="Hold to talk to the robot"
          >
            <span className="talk__mic">
              <MicIcon />
            </span>
            <span>
              {v.recording ? 'Listening' : 'Hold to talk'}
              <small>or hold space</small>
            </span>
          </button>

          <button
            type="button"
            className="switch"
            role="switch"
            aria-checked={v.handsFree}
            disabled={link !== 'open'}
            onClick={() => void voice.setHandsFree(!v.handsFree)}
          >
            <span className="switch__track" />
            hands-free
          </button>
        </div>
      </div>
    </>
  );
}
