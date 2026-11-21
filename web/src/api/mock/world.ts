/**
 * The mock's pretend table: a few objects, a pinhole "wrist camera" that
 * looks along the gripper, and a painter for the fake camera feed. Detections
 * come from the same projection, so the boxes line up with the picture.
 */
import type { Detection, Vec3, WorldMarker } from '../types.ts';

export const CAM_W = 640;
export const CAM_H = 480;
const FOCAL = 520;
/** The lens sits a little behind the tool point. */
const CAM_SETBACK = 0.075;

export interface WorldObject {
  id: string;
  label: string;
  color: string;
  radius: number;
  shape: 'block' | 'ball' | 'cup';
  home: Vec3;
  pos: Vec3;
  /** Hidden from markers (for example the block once it is inside the cup). */
  hidden: boolean;
}

export function makeObjects(): WorldObject[] {
  const mk = (
    id: string,
    label: string,
    color: string,
    radius: number,
    shape: WorldObject['shape'],
    home: Vec3,
  ): WorldObject => ({ id, label, color, radius, shape, home, pos: [...home], hidden: false });
  return [
    mk('block', 'red block', '#ff5340', 0.017, 'block', [0.105, 0.255, 0.017]),
    mk('ball', 'tennis ball', '#d6f23c', 0.032, 'ball', [0.01, 0.315, 0.032]),
    mk('cup', 'paper cup', '#f4f6fb', 0.038, 'cup', [-0.125, 0.235, 0.04]),
  ];
}

export function toMarkers(objects: WorldObject[]): WorldMarker[] {
  return objects
    .filter((o) => !o.hidden)
    .map((o) => ({ id: o.id, label: o.label, pos: [...o.pos] as Vec3, color: o.color, radius: o.radius }));
}

interface Camera {
  c: Vec3;
  a: Vec3;
  right: Vec3;
  up: Vec3;
}

const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a: Vec3): Vec3 => {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
};
const smooth = (lo: number, hi: number, x: number) => {
  const t = Math.min(1, Math.max(0, (x - lo) / (hi - lo)));
  return t * t * (3 - 2 * t);
};

function camera(ee: Vec3, approach: Vec3): Camera {
  const a = norm(approach);
  // Image "up" is the robot's forward when looking down, world up when looking ahead.
  const w = smooth(0.55, 0.95, Math.abs(a[1]));
  const upRef = norm([0, 1 - w, w]);
  const right = norm(cross(a, upRef));
  const up = cross(right, a);
  const c: Vec3 = [ee[0] - a[0] * CAM_SETBACK, ee[1] - a[1] * CAM_SETBACK, ee[2] - a[2] * CAM_SETBACK];
  return { c, a, right, up };
}

function project(cam: Camera, p: Vec3): { u: number; v: number; depth: number } | null {
  const d: Vec3 = [p[0] - cam.c[0], p[1] - cam.c[1], p[2] - cam.c[2]];
  const depth = dot(d, cam.a);
  if (depth < 0.03) return null;
  return {
    u: CAM_W / 2 + (FOCAL * dot(d, cam.right)) / depth,
    v: CAM_H / 2 - (FOCAL * dot(d, cam.up)) / depth,
    depth,
  };
}

/** 2D detections for everything the pretend camera can currently see. */
export function detect(objects: WorldObject[], ee: Vec3, approach: Vec3): Detection[] {
  const cam = camera(ee, approach);
  const out: Detection[] = [];
  for (const o of objects) {
    if (o.hidden) continue;
    const p = project(cam, o.pos);
    if (!p) continue;
    const s = ((FOCAL * o.radius) / p.depth) * (o.shape === 'block' ? 1.45 : 1.12);
    const box: Detection['box'] = [p.u - s, p.v - s, p.u + s, p.v + s];
    if (box[2] < 8 || box[0] > CAM_W - 8 || box[3] < 8 || box[1] > CAM_H - 8) continue;
    const det: Detection = {
      label: o.label,
      score: 0.97 - (o.id.length % 4) * 0.035,
      box: [Math.max(0, box[0]), Math.max(0, box[1]), Math.min(CAM_W, box[2]), Math.min(CAM_H, box[3])],
      point: [p.u, p.v],
    };
    if (o.shape === 'block') {
      const r = ((FOCAL * o.radius) / p.depth) * 1.38;
      const tilt = 0.35;
      det.polygon = [0, 1, 2, 3].map((k) => {
        const ang = tilt + Math.PI / 4 + (k * Math.PI) / 2;
        return [p.u + r * Math.cos(ang), p.v + r * Math.sin(ang)] as [number, number];
      });
    }
    out.push(det);
  }
  return out;
}

/** Paints one frame of the pretend wrist camera. */
export function paintCamera(
  ctx: CanvasRenderingContext2D,
  objects: WorldObject[],
  ee: Vec3,
  approach: Vec3,
  gripperOpen: number,
  tableZ: number,
): void {
  const cam = camera(ee, approach);
  const W = CAM_W;
  const H = CAM_H;

  // Room above the horizon, cutting mat below it.
  const room = ctx.createLinearGradient(0, 0, 0, H);
  room.addColorStop(0, '#cfd6e6');
  room.addColorStop(1, '#9aa6c4');
  ctx.fillStyle = room;
  ctx.fillRect(0, 0, W, H);

  const upZ = cam.up[2];
  let horizon = cam.a[2] < 0 ? -1 : H + 1;
  if (Math.abs(upZ) > 1e-3) horizon = H / 2 + (cam.a[2] * FOCAL) / upZ;
  if (upZ >= -1e-3) {
    const top = Math.max(0, Math.min(H, horizon));
    const mat = ctx.createLinearGradient(0, top, 0, H);
    mat.addColorStop(0, '#2c4ad0');
    mat.addColorStop(1, '#3a5cf0');
    ctx.fillStyle = mat;
    ctx.fillRect(0, top, W, H - top);
  }

  // Printed grid of the mat: a lattice of small crosses, every 5 cm.
  ctx.strokeStyle = 'rgba(220, 230, 255, 0.55)';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  for (let gx = -0.5; gx <= 0.5001; gx += 0.05) {
    for (let gy = -0.25; gy <= 0.6001; gy += 0.05) {
      const p = project(cam, [gx, gy, tableZ]);
      if (!p || p.u < -10 || p.u > W + 10 || p.v < -10 || p.v > H + 10) continue;
      const s = Math.min(9, Math.max(1.5, (FOCAL * 0.004) / p.depth));
      ctx.moveTo(p.u - s, p.v);
      ctx.lineTo(p.u + s, p.v);
      ctx.moveTo(p.u, p.v - s);
      ctx.lineTo(p.u, p.v + s);
    }
  }
  ctx.stroke();

  const visible = objects
    .filter((o) => !o.hidden)
    .map((o) => ({ o, p: project(cam, o.pos) }))
    .filter((e): e is { o: WorldObject; p: { u: number; v: number; depth: number } } => e.p !== null)
    .sort((a, b) => b.p.depth - a.p.depth);

  for (const { o, p } of visible) {
    const r = (FOCAL * o.radius) / p.depth;
    if (r < 1 || r > 900) continue;
    // Contact shadow.
    ctx.fillStyle = 'rgba(8, 14, 60, 0.28)';
    ctx.beginPath();
    ctx.ellipse(p.u + r * 0.18, p.v + r * 0.22, r * 1.08, r * 1.0, 0, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = o.color;
    if (o.shape === 'block') {
      ctx.save();
      ctx.translate(p.u, p.v);
      ctx.rotate(0.35);
      ctx.beginPath();
      ctx.roundRect(-r, -r, r * 2, r * 2, r * 0.12);
      ctx.fill();
      ctx.fillStyle = 'rgba(255,255,255,0.2)';
      ctx.fillRect(-r, -r, r * 2, r * 0.5);
      ctx.restore();
    } else if (o.shape === 'ball') {
      const g = ctx.createRadialGradient(p.u - r * 0.3, p.v - r * 0.35, r * 0.1, p.u, p.v, r);
      g.addColorStop(0, '#f0ff8c');
      g.addColorStop(1, o.color);
      ctx.fillStyle = g;
      ctx.beginPath();
      ctx.arc(p.u, p.v, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = 'rgba(255,255,255,0.85)';
      ctx.lineWidth = Math.max(1, r * 0.09);
      ctx.beginPath();
      ctx.arc(p.u - r * 0.9, p.v, r * 0.95, -0.9, 0.9);
      ctx.stroke();
    } else {
      ctx.beginPath();
      ctx.arc(p.u, p.v, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#b9c0d4';
      ctx.beginPath();
      ctx.arc(p.u, p.v, r * 0.8, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#7d859f';
      ctx.beginPath();
      ctx.arc(p.u, p.v + r * 0.08, r * 0.62, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // The camera sees its own fingers at the bottom edge.
  const gap = 70 + gripperOpen * 150;
  ctx.fillStyle = '#e9ecf4';
  ctx.strokeStyle = 'rgba(11, 18, 64, 0.35)';
  ctx.lineWidth = 2;
  for (const side of [-1, 1]) {
    ctx.beginPath();
    const x = W / 2 + side * gap;
    ctx.moveTo(x, H + 4);
    ctx.lineTo(x + side * 6, H - 92);
    ctx.quadraticCurveTo(x + side * 20, H - 112, x + side * 44, H - 96);
    ctx.lineTo(x + side * 78, H + 4);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }

  const vig = ctx.createRadialGradient(W / 2, H / 2, H * 0.35, W / 2, H / 2, H * 0.85);
  vig.addColorStop(0, 'rgba(0,0,0,0)');
  vig.addColorStop(1, 'rgba(4,8,40,0.38)');
  ctx.fillStyle = vig;
  ctx.fillRect(0, 0, W, H);
}
