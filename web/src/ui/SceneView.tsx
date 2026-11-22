import { useEffect, useRef, useState } from 'react';
import { app } from '../state/app.ts';
import { useStore } from '../state/store.ts';
import { RobotScene, type Insets } from '../scene/RobotScene.ts';

const DEFAULT_URDF = '/assets/elrobot/elrobot_follower.urdf';
const LAYERS = 12;

type Load =
  | { phase: 'loading'; loaded: number; total: number }
  | { phase: 'ready' }
  | { phase: 'error'; message: string };

interface Props {
  insets: Insets;
  /** Hands the scene to the app (for "reset view"); null when it is gone. */
  onScene(scene: RobotScene | null): void;
}

/** The full-viewport 3D twin, its floating labels and its loading state. */
export function SceneView({ insets, onScene }: Props) {
  const hostRef = useRef<HTMLDivElement>(null);
  const labelRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<RobotScene | null>(null);
  const [scene, setScene] = useState<RobotScene | null>(null);
  const [webglError, setWebglError] = useState<string | null>(null);
  const [load, setLoad] = useState<Load>({ phase: 'loading', loaded: 0, total: 0 });
  const [attempt, setAttempt] = useState(0);
  const loadedRef = useRef(false);
  const [loaderGone, setLoaderGone] = useState(false);

  const urdf = useStore(app, (s) => s.meta?.urdf ?? null);
  const tableZ = useStore(app, (s) => s.meta?.table_z ?? 0);
  const plan = useStore(app, (s) => s.plan);
  const markers = useStore(app, (s) => s.markers);
  const obstacles = useStore(app, (s) => s.obstacles);
  const stale = useStore(app, (s) => s.everOpened && s.link !== 'open');
  const linkOpen = useStore(app, (s) => s.link === 'open');

  useEffect(() => {
    if (!hostRef.current || !labelRef.current) return;
    let created: RobotScene;
    try {
      created = new RobotScene(hostRef.current, labelRef.current);
    } catch (err) {
      setWebglError(err instanceof Error ? err.message : 'WebGL is not available.');
      return;
    }
    sceneRef.current = created;
    setScene(created);
    onScene(created);
    return () => {
      onScene(null);
      sceneRef.current = null;
      loadedRef.current = false;
      created.dispose();
    };
    // The scene is created once for the lifetime of the page.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Load the arm once. The URDF path comes from `hello`, but the well-known
  // default lets the model start loading before the socket is even open.
  useEffect(() => {
    if (!scene || loadedRef.current) return;
    let cancelled = false;
    setLoad({ phase: 'loading', loaded: 0, total: 0 });
    scene
      .loadRobot(urdf ?? DEFAULT_URDF, (p) => {
        if (!cancelled) setLoad({ phase: 'loading', loaded: p.loaded, total: p.total });
      })
      .then(() => {
        if (cancelled) return;
        loadedRef.current = true;
        setLoad({ phase: 'ready' });
      })
      .catch((err: unknown) => {
        if (cancelled) return;
        setLoad({ phase: 'error', message: err instanceof Error ? err.message : 'The robot model did not load.' });
      });
    return () => {
      cancelled = true;
    };
    // `urdf` is read when an attempt starts; a late `hello` must not restart a load in flight.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scene, attempt]);

  // The model is served by the daemon: if it failed while the daemon was
  // down, try again as soon as the link comes up.
  useEffect(() => {
    if (linkOpen && load.phase === 'error') setAttempt((a) => a + 1);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [linkOpen]);

  // Once the arm is in, let the loader fade and then take it out of the page.
  useEffect(() => {
    if (load.phase !== 'ready') {
      setLoaderGone(false);
      return;
    }
    const id = window.setTimeout(() => setLoaderGone(true), 700);
    return () => window.clearTimeout(id);
  }, [load.phase]);

  useEffect(() => scene?.setTableZ(tableZ), [scene, tableZ]);
  useEffect(() => scene?.setPlan(plan), [scene, plan]);
  useEffect(() => scene?.setMarkers(markers), [scene, markers]);
  useEffect(() => scene?.setObstacles(obstacles), [scene, obstacles]);
  useEffect(() => scene?.setInsets(insets), [scene, insets]);

  const cx = insets.left + (window.innerWidth - insets.left - insets.right) / 2;
  const cy = insets.top + (window.innerHeight - insets.top - insets.bottom) / 2;
  const fraction = load.phase === 'loading' && load.total > 0 ? load.loaded / load.total : load.phase === 'ready' ? 1 : 0;

  return (
    <>
      <div ref={hostRef} className={`scene${stale ? ' is-stale' : ''}`} />
      <div ref={labelRef} className="labels" aria-hidden="true" />

      {webglError ? (
        <div className="loader loader--error" style={{ left: cx, top: cy }} role="alert">
          <div className="loader__title">no 3D here</div>
          <div className="loader__sub">
            This browser could not start WebGL, so the arm cannot be drawn. Try Chrome with hardware acceleration on.
          </div>
        </div>
      ) : load.phase === 'error' ? (
        <div className="loader loader--error" style={{ left: cx, top: cy }} role="alert">
          <div className="loader__title">the arm did not load</div>
          <div className="loader__sub">
            {load.message} The model comes from the daemon at /assets/elrobot, so check that it is running.
          </div>
          <button type="button" className="btn btn--robot" onClick={() => setAttempt((a) => a + 1)}>
            Load it again
          </button>
        </div>
      ) : loaderGone ? null : (
        <div className="loader" style={{ left: cx, top: cy }} data-done={load.phase === 'ready'} role="status">
          <div className="loader__layers" aria-hidden="true">
            {Array.from({ length: LAYERS }, (_, i) => (
              <i key={i} data-on={fraction * LAYERS > i} />
            ))}
          </div>
          <div className="loader__title">printing the arm</div>
          <div className="loader__sub">
            {load.phase === 'loading' && load.total > 0
              ? `${load.loaded} of ${load.total} parts`
              : load.phase === 'ready'
                ? 'done'
                : 'fetching the model'}
          </div>
        </div>
      )}
    </>
  );
}
