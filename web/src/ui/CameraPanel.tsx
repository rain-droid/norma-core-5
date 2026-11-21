import { useEffect, useMemo, useRef, useState, type CSSProperties } from 'react';
import { api } from '../api/index.ts';
import type { CameraSource } from '../api/transport.ts';
import { app, type DetectionSet } from '../state/app.ts';
import { useStore } from '../state/store.ts';
import { CameraOffIcon, CrossIcon } from './icons.tsx';

/** A frame older than this means the stream has frozen. */
const FROZEN_MS = 2500;
const RETRY_MS = 3000;
/** Boxes describe one frame; the wrist camera moves, so they fade once they are old. */
const FRESH_MS = 4000;

const BOX_COLORS = ['#ffc916', '#5ef0ff', '#ff8be0', '#b9ff6a', '#ffffff', '#ffa25c'];
const INK = '#0b1240';

function colorFor(label: string): string {
  let h = 0;
  for (let i = 0; i < label.length; i++) h = (h * 31 + label.charCodeAt(i)) >>> 0;
  return BOX_COLORS[h % BOX_COLORS.length];
}

/** Draws detections scaled from frame pixels to the size the picture is shown at. */
function drawDetections(canvas: HTMLCanvasElement, det: DetectionSet | null): void {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = Math.max(1, Math.round(rect.width * dpr));
  const h = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  ctx.clearRect(0, 0, w, h);
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
  const name = useStore(app, (s) => {
    const cams = s.meta?.cameras ?? [];
    return cams.includes('wrist') ? 'wrist' : (cams[0] ?? null);
  });
  // A compact key, so 10 Hz telemetry only re-renders this panel when something visible changes.
  const infoKey = useStore(app, (s) => {
    const c = s.telemetry?.cameras.find((x) => x.name === name);
    return c ? `${c.width}x${c.height}@${Math.round(c.fps)}:${c.age_ms > FROZEN_MS ? 'frozen' : 'ok'}` : '';
  });
  const det = useStore(app, (s) => (name ? (s.detections[name] ?? null) : null));

  const [large, setLarge] = useState(false);
  const [failed, setFailed] = useState(false);
  const [nonce, setNonce] = useState(0);
  const [stale, setStale] = useState(false);
  const overlayRef = useRef<HTMLCanvasElement>(null);

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
    const draw = () => drawDetections(canvas, live ? det : null);
    draw();
    const ro = new ResizeObserver(draw);
    ro.observe(canvas);
    return () => ro.disconnect();
  }, [det, live, large]);

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
      <section className={`camera${large ? ' is-large' : ''}`} aria-label="Wrist camera">
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
              <span className="camera__badge camera__badge--name">
                <span className="dot" data-pulse={!frozen} />
                {name} cam
              </span>
              <span className="camera__badge camera__badge--fps">
                {frozen ? 'frozen' : fps !== null ? `${fps} fps` : 'live'}
              </span>
            </>
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
