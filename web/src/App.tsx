import { useCallback, useEffect, useLayoutEffect, useRef, useState } from 'react';
import { stopEverything } from './actions.ts';
import { startTransport } from './api/index.ts';
import type { Insets, RobotScene } from './scene/RobotScene.ts';
import { app } from './state/app.ts';
import { useStore } from './state/store.ts';
import { CameraPanel } from './ui/CameraPanel.tsx';
import { LinkNotices } from './ui/LinkNotices.tsx';
import { SceneView } from './ui/SceneView.tsx';
import { SidePanel } from './ui/SidePanel.tsx';
import { StatusStrip } from './ui/StatusStrip.tsx';
import { StepsPanel } from './ui/StepsPanel.tsx';
import { StopButton } from './ui/StopButton.tsx';
import { Composer, Timeline } from './ui/Timeline.tsx';
import { VoiceDock } from './ui/VoiceDock.tsx';
import { GhostIcon, StepsIcon, ViewIcon } from './ui/icons.tsx';
import { voice } from './voice/controller.ts';

function isTextField(el: Element | null): boolean {
  if (!(el instanceof HTMLElement)) return false;
  return el instanceof HTMLInputElement || el instanceof HTMLTextAreaElement || el instanceof HTMLSelectElement || el.isContentEditable;
}

/** A control someone reached with the keyboard keeps Space for itself. */
function wantsSpace(el: Element | null): boolean {
  if (!(el instanceof HTMLElement) || el === document.body) return false;
  return el.matches('button, a, [role="button"], [role="switch"], [role="tab"]') && el.matches(':focus-visible');
}

/** Escape stops the robot from anywhere; Space is push-to-talk outside text fields. */
function useGlobalKeys(): void {
  useEffect(() => {
    let spaceHeld = false;
    const down = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        e.preventDefault();
        if (!e.repeat) void stopEverything();
        return;
      }
      if (e.code !== 'Space' || e.metaKey || e.ctrlKey || e.altKey) return;
      if (spaceHeld) {
        e.preventDefault();
        return;
      }
      const target = document.activeElement;
      if (isTextField(target) || wantsSpace(target)) return;
      e.preventDefault();
      if (e.repeat) return;
      spaceHeld = true;
      void voice.pttDown();
    };
    const up = (e: KeyboardEvent) => {
      if (e.code !== 'Space' || !spaceHeld) return;
      e.preventDefault();
      spaceHeld = false;
      void voice.pttUp();
    };
    const blur = () => {
      if (!spaceHeld) return;
      spaceHeld = false;
      void voice.pttUp();
    };
    window.addEventListener('keydown', down, true);
    window.addEventListener('keyup', up, true);
    window.addEventListener('blur', blur);
    return () => {
      window.removeEventListener('keydown', down, true);
      window.removeEventListener('keyup', up, true);
      window.removeEventListener('blur', blur);
    };
  }, []);
}

export function App() {
  const transport = useStore(app, (s) => s.transport);
  const sceneRef = useRef<RobotScene | null>(null);
  const topRef = useRef<HTMLDivElement>(null);
  const talkRef = useRef<HTMLDivElement>(null);
  const sideRef = useRef<HTMLDivElement>(null);
  const dockRef = useRef<HTMLDivElement>(null);
  const [insets, setInsets] = useState<Insets>({ left: 0, right: 0, top: 0, bottom: 0 });
  const [ghost, setGhost] = useState(() => {
    try {
      return localStorage.getItem('elrobot.ghost') !== 'off';
    } catch {
      return true;
    }
  });

  useEffect(() => {
    sceneRef.current?.setGhostEnabled(ghost);
    try {
      localStorage.setItem('elrobot.ghost', ghost ? 'on' : 'off');
    } catch {
      /* private mode: the choice just does not persist */
    }
  }, [ghost]);

  const [steps, setSteps] = useState(() => {
    try {
      return localStorage.getItem('elrobot.steps') !== 'off';
    } catch {
      return true;
    }
  });
  useEffect(() => {
    try {
      localStorage.setItem('elrobot.steps', steps ? 'on' : 'off');
    } catch {
      /* private mode: the choice just does not persist */
    }
  }, [steps]);

  useEffect(() => {
    voice.start();
    const mock = new URLSearchParams(location.search).get('mock') === '1';
    startTransport(mock ? 'mock' : 'live');
  }, []);

  useGlobalKeys();

  // Tell the 3D view how much of the screen the panels cover, so it frames
  // the arm in the space that is left.
  useLayoutEffect(() => {
    const measure = () => {
      const w = window.innerWidth;
      const h = window.innerHeight;
      const top = topRef.current?.getBoundingClientRect();
      const talk = talkRef.current?.getBoundingClientRect();
      const side = sideRef.current?.getBoundingClientRect();
      const dock = dockRef.current?.getBoundingClientRect();
      const next: Insets = {
        left: talk && talk.width > 0 ? Math.round(talk.right) : 0,
        right: side && side.width > 0 ? Math.round(w - side.left) : 0,
        top: top ? Math.round(top.bottom) : 0,
        bottom: dock && dock.height > 0 ? Math.round(h - dock.top) : 0,
      };
      setInsets((prev) =>
        prev.left === next.left && prev.right === next.right && prev.top === next.top && prev.bottom === next.bottom
          ? prev
          : next,
      );
    };
    measure();
    const ro = new ResizeObserver(measure);
    for (const ref of [topRef, talkRef, sideRef, dockRef]) if (ref.current) ro.observe(ref.current);
    window.addEventListener('resize', measure);
    return () => {
      ro.disconnect();
      window.removeEventListener('resize', measure);
    };
  }, []);

  const ghostRef = useRef(ghost);
  ghostRef.current = ghost;
  const onScene = useCallback((scene: RobotScene | null) => {
    sceneRef.current = scene;
    scene?.setGhostEnabled(ghostRef.current);
  }, []);

  return (
    <>
      <SceneView insets={insets} onScene={onScene} />

      <div className="hud">
        <header className="hud__top" ref={topRef}>
          <span className="wordmark">
            el<b>robot</b>
          </span>
          <StatusStrip />
          <span className="top-spacer" />
          {transport === 'mock' && (
            <span className="demo-tag" title="Simulated arm and scripted conversation. Remove ?mock=1 to use the daemon.">
              demo mode, nothing here is real
            </span>
          )}
          <button
            type="button"
            className="chip-btn"
            aria-pressed={steps}
            onClick={() => setSteps((s) => !s)}
            title="What the robot is doing right now and the steps it has planned next"
          >
            <StepsIcon />
            Steps {steps ? 'on' : 'off'}
          </button>
          <button
            type="button"
            className="chip-btn"
            aria-pressed={ghost}
            onClick={() => setGhost((g) => !g)}
            title="Outline of the pose the arm is being told to reach, shown while it lags behind"
          >
            <GhostIcon />
            Goal ghost {ghost ? 'on' : 'off'}
          </button>
          <button type="button" className="chip-btn" onClick={() => sceneRef.current?.resetView()}>
            <ViewIcon />
            Reset view
          </button>
        </header>

        <div className="hud__talk" ref={talkRef}>
          {steps && <StepsPanel />}
          <Timeline />
          <Composer />
        </div>

        <div className="hud__side" ref={sideRef}>
          <CameraPanel />
          <SidePanel />
        </div>

        <div className="hud__dock" ref={dockRef}>
          <VoiceDock />
        </div>

        <div className="hud__stop">
          <StopButton />
        </div>
      </div>

      <LinkNotices />
    </>
  );
}
