import { useEffect, useMemo, useRef, useState, type CSSProperties } from 'react';
import { api } from '../api/index.ts';
import type { CameraSource } from '../api/transport.ts';
import type { VisionFrame, VisionLine } from '../api/types.ts';
import { app, type DetectionSet } from '../state/app.ts';
import { useStore } from '../state/store.ts';
import { CameraOffIcon, CrossIcon } from './icons.tsx';

/** A frame older than this means the stream has frozen. */
const FROZEN_MS = 2500;
const RETRY_MS = 3000;
/** Boxes describe one frame; the wrist camera moves, so they fade once they are old. */
const FRESH_MS = 4000;

/** How often the daemon is asked what its vision sees, while a live layer is on. */
const VISION_MS = 300;

/** What can be drawn over the picture. `tools` is what the agent's tools and the tracker report. */
const LAYERS = [
  { id: 'objects', label: 'Objects', hint: 'Live segmentation: outlines of whatever stands out from the table' },
  { id: 'tags', label: 'Tags', hint: 'AprilTags in view: the mat corners used for calibration' },
  { id: 'frame', label: 'Frame', hint: "The robot's base frame and reach, from the saved calibration (top camera)" },
  { id: 'arm', label: 'Arm', hint: 'The arm as the kinematics place it (top camera): it should lie on the real arm' },
  { id: 'tools', label: 'Tools', hint: 'What the tools and the tracker last reported: names, boxes, grasp points' },
] as const;
type LayerId = (typeof LAYERS)[number]['id'];
type LayerSet = Record<LayerId, boolean>;
const DEFAULT_LAYERS: LayerSet = { objects: true, tags: false, frame: false, arm: false, tools: true };
const LINE_COLORS: Record<VisionLine['color'], string> = { x: '#ff5a4f', y: '#58e07a', reach: '#ffc916', arm: '#ff8be0' };

function remembered<T>(key: string, fallback: T): T {
  try {
    const raw = window.localStorage.getItem(key);
    return raw ? ({ ...(fallback as object), ...JSON.parse(raw) } as T) : fallback;
  } catch {
    return fallback;
  }
}

/** `?camera=top&layers=objects,frame,arm` opens the panel that way: handy for a link or a recording. */
function fromUrl(): { camera: string | null; layers: LayerSet | null } {
  const q = new URLSearchParams(window.location.search);
  const wanted = q.get('layers');
  const layers = wanted === null ? null : (Object.fromEntries(LAYERS.map((l) => [l.id, wanted.split(',').includes(l.id)])) as LayerSet);
  return { camera: q.get('camera'), layers };
}

function remember(key: string, value: unknown): void {
  try {
    window.localStorage.setItem(key, JSON.stringify(value));
  } catch {
    /* private mode: the choice just does not survive a reload */
  }
}

const BOX_COLORS = ['#ffc916', '#5ef0ff', '#ff8be0', '#b9ff6a', '#ffffff', '#ffa25c'];
const INK = '#0b1240';

function colorFor(label: string): string {
  let h = 0;
  for (let i = 0; i < label.length; i++) h = (h * 31 + label.charCodeAt(i)) >>> 0;
  return BOX_COLORS[h % BOX_COLORS.length];
}

/** Sizes the overlay canvas to the picture and clears it. */
function prepare(canvas: HTMLCanvasElement): { ctx: CanvasRenderingContext2D; w: number; h: number; dpr: number } | null {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.max(1, Math.round(rect.width * dpr));
  const h = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
  const ctx = canvas.getContext('2d');
  if (!ctx) return null;
  ctx.clearRect(0, 0, w, h);
  return { ctx, w, h, dpr };
}

function tag(ctx: CanvasRenderingContext2D, text: string, x: number, y: number, color: string, font: number, w: number, h: number): void {
  const padX = font * 0.4;
  const tw = ctx.measureText(text).width + padX * 2;
  const th = font * 1.4;
  const tx = Math.min(Math.max(0, x), w - tw);
  const ty = Math.min(Math.max(0, y), h - th);
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.roundRect(tx, ty, tw, th, th * 0.28);
  ctx.fill();
  ctx.fillStyle = INK;
  ctx.fillText(text, tx + padX, ty + th / 2 + font * 0.06);
}

/** The live layers: object outlines, tags, the robot's frame and the arm. */
function drawVision(canvas: HTMLCanvasElement, vision: VisionFrame | null, on: LayerSet): void {
  const c = prepare(canvas);
  if (!c || !vision || !vision.width || !vision.height) return;
  const { ctx, w, h, dpr } = c;
  const sx = w / vision.width;
  const sy = h / vision.height;
  const line = Math.max(1.5, w / 260);
  const font = Math.min(Math.max(10 * dpr, w / 34), 17 * dpr);
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.textBaseline = 'middle';
  ctx.font = `700 ${font}px "Overpass Variable", system-ui, sans-serif`;
  const path = (points: [number, number][], close: boolean) => {
    ctx.beginPath();
    points.forEach(([u, v], i) => (i ? ctx.lineTo(u * sx, v * sy) : ctx.moveTo(u * sx, v * sy)));
    if (close) ctx.closePath();
  };
  const stroke = (color: string, width: number) => {
    ctx.lineWidth = width * 2.2;
    ctx.strokeStyle = 'rgba(11, 18, 64, 0.6)';
    ctx.stroke();
    ctx.lineWidth = width;
    ctx.strokeStyle = color;
    ctx.stroke();
  };

  if (on.frame) {
    for (const l of vision.layers.frame ?? []) {
      const color = LINE_COLORS[l.color];
      ctx.setLineDash(l.color === 'reach' ? [line * 4, line * 3] : []);
      path(l.points, false);
      stroke(color, l.arrow ? line * 1.6 : line);
      ctx.setLineDash([]);
      const [ex, ey] = l.points[l.arrow ? l.points.length - 1 : Math.floor(l.points.length / 2)];
      if (l.arrow) {
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(ex * sx, ey * sy, line * 2.4, 0, Math.PI * 2);
        ctx.fill();
      }
      if (l.label) tag(ctx, l.label, ex * sx + line * 4, ey * sy - font * 0.7, color, font * 0.85, w, h);
    }
  }
  if (on.objects) {
    for (const o of vision.layers.objects ?? []) {
      const color = colorFor(o.label ?? '');
      if (!o.polygon || o.polygon.length < 3) continue;
      path(o.polygon, true);
      ctx.globalAlpha = 0.3;
      ctx.fillStyle = color;
      ctx.fill();
      ctx.globalAlpha = 1;
      stroke(color, line);
      if (o.point) {
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(o.point[0] * sx, o.point[1] * sy, line * 1.6, 0, Math.PI * 2);
        ctx.fill();
      }
      // The tools' own names take the label's place when they are being shown.
      if (!on.tools && o.box) tag(ctx, o.label, o.box[0] * sx, o.box[1] * sy - font * 1.5, color, font * 0.85, w, h);
    }
  }
  if (on.tags) {
    for (const t of vision.layers.tags ?? []) {
      path(t.polygon, true);
      stroke('#5ef0ff', line);
      tag(ctx, t.label, t.polygon[1][0] * sx + line * 3, t.polygon[1][1] * sy - font * 0.7, '#5ef0ff', font * 0.85, w, h);
    }
  }
  if (on.arm) {
    for (const l of vision.layers.arm ?? []) {
      path(l.points, false);
      stroke(LINE_COLORS.arm, line * 1.4);
      if (l.joints) {
        ctx.fillStyle = '#ffffff';
        for (const [u, v] of l.points) {
          ctx.beginPath();
          ctx.arc(u * sx, v * sy, line * 1.8, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }
  }
}

/** Draws detections scaled from frame pixels to the size the picture is shown at. */
function drawDetections(canvas: HTMLCanvasElement, det: DetectionSet | null): void {
  const c = prepare(canvas);
  if (!c) return;
  const { ctx, w, h, dpr } = c;
  if (!det || !det.width || !det.height) return;

  const sx = w / det.width;
  const sy = h / det.height;
  const line = Math.max(2, w / 190);
  const font = Math.max(11 * dpr, w / 27);
  ctx.lineJoin = 'round';
  ctx.textBaseline = 'middle';
  ctx.font = `700 ${font}px "Overpass Variable", system-ui, sans-serif`;

  for (const item of det.items) {
    const color = colorFor(item.label ?? '');

    if (item.polygon && item.polygon.length >= 3) {
      ctx.beginPath();
      item.polygon.forEach(([u, v], i) => (i ? ctx.lineTo(u * sx, v * sy) : ctx.moveTo(u * sx, v * sy)));
      ctx.closePath();
      ctx.globalAlpha = 0.28;
      ctx.fillStyle = color;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.lineWidth = line * 0.8;
      ctx.strokeStyle = color;
      ctx.stroke();
    }

    if (item.box) {
      const [x1, y1, x2, y2] = item.box;
      const bx = x1 * sx;
      const by = y1 * sy;
      const bw = (x2 - x1) * sx;
      const bh = (y2 - y1) * sy;
      // Ink underlay keeps the box readable on any picture.
      ctx.lineWidth = line * 2;
      ctx.strokeStyle = 'rgba(11, 18, 64, 0.55)';
      ctx.beginPath();
      ctx.roundRect(bx, by, bw, bh, line * 2);
      ctx.stroke();
      ctx.lineWidth = line;
      ctx.strokeStyle = color;
      ctx.stroke();

      const score = Number.isFinite(item.score) ? ` ${Math.round(item.score * 100)}%` : '';
      const text = `${item.label ?? ''}${score}`;
      const padX = font * 0.45;
      const tw = ctx.measureText(text).width + padX * 2;
      const th = font * 1.5;
      let tx = Math.min(Math.max(0, bx - line / 2), w - tw);
      let ty = by - th - line * 0.2;
      if (ty < 0) ty = Math.min(h - th, by + bh + line * 0.2);
      if (tx < 0) tx = 0;
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.roundRect(tx, ty, tw, th, th * 0.28);
      ctx.fill();
      ctx.fillStyle = INK;
      ctx.fillText(text, tx + padX, ty + th / 2 + font * 0.06);
    }

    if (item.point) {
      const px = item.point[0] * sx;
      const py = item.point[1] * sy;
      const r = Math.max(5 * dpr, w / 70);
      // Grasp point: a crosshair.
      for (const [width, style] of [
        [line * 2.2, 'rgba(11, 18, 64, 0.7)'],
        [line, '#ffffff'],
      ] as const) {
        ctx.lineWidth = width;
        ctx.strokeStyle = style;
        ctx.beginPath();
        ctx.arc(px, py, r, 0, Math.PI * 2);
        ctx.moveTo(px - r * 1.7, py);
        ctx.lineTo(px - r * 0.45, py);
        ctx.moveTo(px + r * 0.45, py);
        ctx.lineTo(px + r * 1.7, py);
        ctx.moveTo(px, py - r * 1.7);
        ctx.lineTo(px, py - r * 0.45);
        ctx.moveTo(px, py + r * 0.45);
        ctx.lineTo(px, py + r * 1.7);
        ctx.stroke();
      }
    }
  }
}

/** The demo's pretend camera, painted at camera-like frame rate. */
function CanvasFeed({ source }: { source: Extract<CameraSource, { kind: 'canvas' }> }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const ctx = ref.current?.getContext('2d');
    if (!ctx) return;
    let raf = 0;
    let last = 0;
    const loop = (t: number) => {
      raf = requestAnimationFrame(loop);
      if (t - last < 66) return;
      last = t;
      source.draw(ctx);
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, [source]);
  return <canvas ref={ref} width={source.width} height={source.height} />;
}

export function CameraPanel() {
  const transport = useStore(app, (s) => s.transport);
  const linkOpen = useStore(app, (s) => s.link === 'open');
  const camsKey = useStore(app, (s) => (s.meta?.cameras ?? []).join(','));
  // Which cameras are delivering frames, as a compact key (telemetry arrives at 10 Hz).
  const freshKey = useStore(app, (s) =>
    (s.telemetry?.cameras ?? [])
      .filter((c) => c.age_ms <= FROZEN_MS)
      .map((c) => c.name)
      .join(','),
  );
  const [chosen, setChosen] = useState<string | null>(
    () => fromUrl().camera ?? remembered<{ name: string | null }>('elrobot.camera', { name: null }).name,
  );
  const [layers, setLayers] = useState<LayerSet>(() => fromUrl().layers ?? remembered('elrobot.camera.layers', DEFAULT_LAYERS));
  const [vision, setVision] = useState<VisionFrame | null>(null);
  const [visionMissing, setVisionMissing] = useState(false);

  const cams = useMemo(() => (camsKey ? camsKey.split(',') : []), [camsKey]);
  const fresh = useMemo(() => new Set(freshKey ? freshKey.split(',') : []), [freshKey]);
  // The top camera sees the whole table; fall back to whatever there is.
  const name = chosen && cams.includes(chosen) ? chosen : cams.includes('top') ? 'top' : cams.includes('wrist') ? 'wrist' : (cams[0] ?? null);
  // A compact key, so 10 Hz telemetry only re-renders this panel when something visible changes.
  const infoKey = useStore(app, (s) => {
    const c = s.telemetry?.cameras.find((x) => x.name === name);
    return c ? `${c.width}x${c.height}@${Math.round(c.fps)}:${c.age_ms > FROZEN_MS ? 'frozen' : 'ok'}` : '';
  });
  const det = useStore(app, (s) => (name ? (s.detections[name] ?? null) : null));

  const pick = (cam: string) => {
    setChosen(cam);
    remember('elrobot.camera', { name: cam });
  };
  const toggle = (id: LayerId) =>
    setLayers((cur) => {
      const next = { ...cur, [id]: !cur[id] };
      remember('elrobot.camera.layers', next);
      return next;
    });
  const wantsVision = layers.objects || layers.tags || layers.frame || layers.arm;

  const [large, setLarge] = useState(false);
  const [failed, setFailed] = useState(false);
  const [nonce, setNonce] = useState(0);
  const [stale, setStale] = useState(false);
  const overlayRef = useRef<HTMLCanvasElement>(null);
  const visionRef = useRef<HTMLCanvasElement>(null);

  const source = useMemo(
    () => (name && linkOpen ? api.camera(name) : null),
    // A new transport means a new source, even for the same camera name.
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [name, linkOpen, transport],
  );

  const m = /^(\d+)x(\d+)@(\d+):(\w+)$/.exec(infoKey);
  const width = m ? Number(m[1]) : (det?.width ?? 640);
  const height = m ? Number(m[2]) : (det?.height ?? 480);
  const fps = m ? Number(m[3]) : null;
  const frozen = m ? m[4] === 'frozen' : false;
  const live = source !== null && !failed;

  // An MJPEG <img> that errored or froze has to be pointed at the stream again.
  useEffect(() => {
    if (!source || source.kind !== 'mjpeg' || !(failed || frozen)) return;
    const id = window.setTimeout(() => {
      setFailed(false);
      setNonce((n) => n + 1);
    }, RETRY_MS);
    return () => window.clearTimeout(id);
  }, [source, failed, frozen, nonce]);

  useEffect(() => setFailed(false), [source]);

  useEffect(() => {
    const canvas = overlayRef.current;
    if (!canvas) return;
    const draw = () => drawDetections(canvas, live && layers.tools ? det : null);
    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(canvas);
    return () => ro.disconnect();
  }, [det, live, large, layers.tools]);

  // Ask the daemon what its vision sees, one request after the other, while a live layer is on.
  useEffect(() => {
    setVision(null);
    setVisionMissing(false);
    if (!name || !live || !wantsVision || frozen) return;
    let stop = false;
    let timer = 0;
    let misses = 0;
    const ask = async () => {
      const got = await api.vision(name);
      if (stop) return;
      misses = got ? 0 : misses + 1;
      if (got) setVision(got);
      setVisionMissing(misses >= 3);
      timer = window.setTimeout(ask, got ? VISION_MS : 2000);
    };
    void ask();
    return () => {
      stop = true;
      window.clearTimeout(timer);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [name, live, wantsVision, frozen, transport]);

  useEffect(() => {
    const canvas = visionRef.current;
    if (!canvas) return;
    const draw = () => drawVision(canvas, live ? vision : null, layers);
    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(canvas);
    return () => ro.disconnect();
  }, [vision, live, large, layers]);

  useEffect(() => {
    setStale(false);
    if (!det) return;
    const id = window.setTimeout(() => setStale(true), Math.max(0, FRESH_MS - (Date.now() - det.at)));
    return () => window.clearTimeout(id);
  }, [det]);

  const ratio = width / height;
  const style = { '--cam-aspect': `${width} / ${height}`, '--cam-ratio': String(ratio) } as CSSProperties;

  let empty: { title: string; body: string } | null = null;
  if (!linkOpen) empty = { title: 'Camera offline', body: 'It comes back when the daemon does.' };
  else if (!name) {
    empty = {
      title: 'No camera',
      body: 'The daemon reports no cameras. Plug in the wrist camera, then restart the daemon.',
    };
  } else if (failed) {
    empty = { title: `No picture from the ${name} camera`, body: 'Check its USB cable. Trying again every few seconds.' };
  }

  return (
    <div className="camera-slot" style={style}>
      {large && <div className="camera__scrim" onClick={() => setLarge(false)} />}
      <section className={`camera${large ? ' is-large' : ''}`} aria-label="Cameras">
        <div
          className="camera__view"
          role={empty ? undefined : 'button'}
          tabIndex={empty ? undefined : 0}
          aria-label={empty ? undefined : large ? 'Shrink the camera view' : 'Enlarge the camera view'}
          onClick={() => !empty && setLarge((v) => !v)}
          onKeyDown={(e) => {
            if (!empty && (e.key === 'Enter' || e.key === ' ')) {
              e.preventDefault();
              setLarge((v) => !v);
            }
          }}
        >
          {source?.kind === 'mjpeg' && !failed && (
            <img
              key={nonce}
              src={nonce ? `${source.url}?r=${nonce}` : source.url}
              alt={`Live view from the ${name} camera`}
              onError={() => setFailed(true)}
              draggable={false}
            />
          )}
          {source?.kind === 'canvas' && <CanvasFeed source={source} />}
          <canvas ref={visionRef} style={{ pointerEvents: 'none' }} />
          <canvas
            ref={overlayRef}
            style={{ opacity: stale ? 0.35 : 1, transition: 'opacity 1.2s ease', pointerEvents: 'none' }}
          />

          {empty ? (
            <div className="camera__empty">
              <CameraOffIcon />
              <strong>{empty.title}</strong>
              <span>{empty.body}</span>
            </div>
          ) : (
            <>
              <span className="camera__badge camera__badge--fps">
                {frozen ? 'frozen' : fps !== null ? `${fps} fps` : 'live'}
              </span>
            </>
          )}
          {cams.length > 0 && linkOpen && (
            <div className="camera__bar camera__bar--top" onClick={(e) => e.stopPropagation()} onKeyDown={(e) => e.stopPropagation()}>
              <div className="camera__tabs" role="tablist" aria-label="Camera">
                {cams.map((cam) => (
                  <button
                    key={cam}
                    type="button"
                    role="tab"
                    aria-selected={cam === name}
                    data-fresh={fresh.has(cam)}
                    title={fresh.has(cam) ? `Show the ${cam} camera` : `The ${cam} camera is not delivering frames`}
                    onClick={() => pick(cam)}
                  >
                    <span className="dot" data-pulse={cam === name && !frozen} />
                    {cam}
                  </button>
                ))}
              </div>
            </div>
          )}
          {!empty && (
            <div className="camera__bar camera__bar--bottom" onClick={(e) => e.stopPropagation()} onKeyDown={(e) => e.stopPropagation()}>
              <div className="camera__layers" role="group" aria-label="What to draw over the picture">
                {LAYERS.map((l) => (
                  <button key={l.id} type="button" aria-pressed={layers[l.id]} title={l.hint} onClick={() => toggle(l.id)}>
                    {l.label}
                  </button>
                ))}
              </div>
              <span className="camera__note">
                {visionMissing && wantsVision
                  ? 'no live vision from this camera'
                  : [
                      vision && layers.objects ? `${vision.layers.objects?.length ?? 0} objects` : '',
                      vision && layers.tags ? `${vision.layers.tags?.length ?? 0} tags` : '',
                      vision && (layers.frame || layers.arm) && !vision.layers.frame ? 'frame and arm: top camera only' : '',
                      layers.tools && det ? `tools: ${det.items.length}${stale ? ' (old)' : ''}` : '',
                      vision && wantsVision ? `${vision.ms} ms` : '',
                    ]
                      .filter(Boolean)
                      .join(' · ')}
              </span>
            </div>
          )}
        </div>
        {large && (
          <button type="button" className="camera__close" onClick={() => setLarge(false)} aria-label="Close the large camera view">
            <CrossIcon />
          </button>
        )}
      </section>
    </div>
  );
}
