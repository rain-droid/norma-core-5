/**
 * What the arm intends and what it knows about the table, drawn in 3D:
 * the planned tool path with its target, a short trail behind the tool
 * point, and labelled markers for detected objects. All positions are in
 * the URDF base frame; the objects live in the scene's `world` group.
 */
import {
  CircleGeometry,
  Color,
  ConeGeometry,
  CylinderGeometry,
  DoubleSide,
  DynamicDrawUsage,
  Group,
  InstancedMesh,
  Matrix4,
  Mesh,
  MeshBasicMaterial,
  MeshStandardMaterial,
  Quaternion,
  RingGeometry,
  SphereGeometry,
  Vector2,
  Vector3,
  type Camera,
} from 'three';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import type { Vec3, WorldMarker } from '../api/types.ts';
import type { Plan } from '../state/app.ts';

const CHALK = 0xffffff;
const INK = 0x0b1240;
const Z_AXIS = new Vector3(0, 0, 1);

/* ------------------------------------------------------------------ */
/* Screen-space labels                                                 */
/* ------------------------------------------------------------------ */

interface Label {
  el: HTMLDivElement;
  anchor: Vector3;
  /** Extra pixels above the anchor. */
  lift: number;
  x: number;
  y: number;
  w: number;
  h: number;
  visible: boolean;
}

/** HTML chips pinned to 3D points: always upright, always the same size. */
export class LabelLayer {
  private labels = new Set<Label>();
  private v = new Vector3();

  constructor(private host: HTMLElement) {}

  add(className: string): Label {
    const el = document.createElement('div');
    el.className = `tag3d ${className}`;
    this.host.appendChild(el);
    const label: Label = { el, anchor: new Vector3(), lift: 0, x: 0, y: 0, w: 0, h: 0, visible: false };
    this.labels.add(label);
    return label;
  }

  remove(label: Label): void {
    label.el.remove();
    this.labels.delete(label);
  }

  /** `anchor`s are world-space (three) positions. */
  update(camera: Camera, width: number, height: number): void {
    const placed: Label[] = [];
    for (const l of this.labels) {
      if (l.el.hidden) continue;
      this.v.copy(l.anchor).project(camera);
      l.visible = this.v.z > -1 && this.v.z < 1;
      if (!l.visible) {
        l.el.style.opacity = '0';
        continue;
      }
      if (l.w === 0) {
        l.w = l.el.offsetWidth;
        l.h = l.el.offsetHeight;
      }
      l.x = (this.v.x * 0.5 + 0.5) * width;
      l.y = (-this.v.y * 0.5 + 0.5) * height - l.lift;
      placed.push(l);
    }
    // Keep chips from sitting on top of each other: lower ones win their spot,
    // anything overlapping gets pushed up.
    placed.sort((a, b) => b.y - a.y);
    for (let i = 0; i < placed.length; i++) {
      const a = placed[i];
      for (let j = 0; j < i; j++) {
        const b = placed[j];
        const overlapX = Math.abs(a.x - b.x) < (a.w + b.w) / 2 + 6;
        const overlapY = a.y > b.y - b.h - 6 && a.y - a.h < b.y + 6;
        if (overlapX && overlapY) a.y = b.y - b.h - 6;
      }
      a.el.style.opacity = '1';
      a.el.style.transform = `translate3d(${a.x.toFixed(1)}px, ${a.y.toFixed(1)}px, 0) translate(-50%, -100%)`;
    }
  }

  /** Call after changing a label's text. */
  static changed(label: Label): void {
    label.w = 0;
  }

  dispose(): void {
    for (const l of this.labels) l.el.remove();
    this.labels.clear();
  }
}

/* ------------------------------------------------------------------ */
/* Planned path + target                                               */
/* ------------------------------------------------------------------ */

export class PlanOverlay {
  readonly group = new Group();
  private casing = new LineMaterial({ color: INK, linewidth: 7, transparent: true, opacity: 0.55, depthTest: false });
  private core = new LineMaterial({
    color: CHALK,
    linewidth: 3.2,
    dashed: true,
    dashSize: 0.014,
    gapSize: 0.009,
    transparent: true,
    opacity: 0.98,
    depthTest: false,
  });
  private line: Line2 | null = null;
  private lineCasing: Line2 | null = null;
  private target = new Group();
  private drop: Mesh;
  private foot: Mesh;
  private label: Label;
  private flat = new MeshBasicMaterial({ color: CHALK, transparent: true, opacity: 0.95, depthTest: false, side: DoubleSide });
  private dim = new MeshBasicMaterial({ color: CHALK, transparent: true, opacity: 0.45, depthTest: false, side: DoubleSide });
  private dark = new MeshBasicMaterial({ color: INK, transparent: true, opacity: 0.6, depthTest: false, side: DoubleSide });
  private anchorLocal = new Vector3();
  private spin: Mesh;
  private tableZ = 0;

  constructor(labels: LabelLayer) {
    // Bullseye facing along the approach direction, with an arrow flying into it.
    const ringDark = new Mesh(new RingGeometry(0.0165, 0.0255, 48), this.dark);
    const ring = new Mesh(new RingGeometry(0.0185, 0.0235, 48), this.flat);
    const dot = new Mesh(new CircleGeometry(0.0052, 24), this.flat);
    this.spin = new Mesh(new RingGeometry(0.03, 0.0325, 48, 1, 0, Math.PI * 0.5), this.dim);
    const shaft = new Mesh(new CylinderGeometry(0.0018, 0.0018, 0.05, 10), this.flat);
    shaft.rotation.x = Math.PI / 2;
    shaft.position.z = -0.073;
    const head = new Mesh(new ConeGeometry(0.0075, 0.02, 16), this.flat);
    head.rotation.x = Math.PI / 2;
    head.position.z = -0.04;
    this.target.add(ringDark, ring, dot, this.spin, shaft, head);
    for (const m of this.target.children) m.renderOrder = 6;

    // Plumb line to the table, so the target's height reads at a glance.
    this.drop = new Mesh(new CylinderGeometry(0.0009, 0.0009, 1, 6), this.dim);
    this.drop.rotation.x = Math.PI / 2;
    this.drop.renderOrder = 5;
    this.foot = new Mesh(new RingGeometry(0.009, 0.0115, 32), this.dim);
    this.foot.renderOrder = 5;

    this.group.add(this.target, this.drop, this.foot);
    this.group.visible = false;
    this.label = labels.add('tag3d--target');
    this.label.lift = 26;
    this.label.el.hidden = true;
  }

  setResolution(size: Vector2): void {
    this.casing.resolution.copy(size);
    this.core.resolution.copy(size);
  }

  setTableZ(z: number): void {
    this.tableZ = z;
  }

  set(plan: Plan | null): void {
    for (const l of [this.line, this.lineCasing]) {
      if (l) {
        this.group.remove(l);
        l.geometry.dispose();
      }
    }
    this.line = this.lineCasing = null;
    if (!plan || plan.path.length < 2) {
      this.group.visible = false;
      this.label.el.hidden = true;
      return;
    }
    this.group.visible = true;

    const flat = plan.path.flat();
    const geoA = new LineGeometry().setPositions(flat);
    const geoB = new LineGeometry().setPositions(flat);
    this.lineCasing = new Line2(geoA, this.casing);
    this.line = new Line2(geoB, this.core);
    this.line.computeLineDistances();
    this.lineCasing.renderOrder = 4;
    this.line.renderOrder = 5;
    this.lineCasing.frustumCulled = this.line.frustumCulled = false;
    this.group.add(this.lineCasing, this.line);

    const t = plan.target;
    this.target.visible = this.drop.visible = this.foot.visible = t !== null;
    this.label.el.hidden = t === null;
    if (t) {
      const p = t.pos;
      const dir = new Vector3(...t.approach);
      if (dir.lengthSq() < 1e-6) dir.set(0, 0, -1);
      dir.normalize();
      this.target.position.set(p[0], p[1], p[2]);
      // Local +Z of the target group is the approach direction.
      this.target.quaternion.copy(new Quaternion().setFromUnitVectors(Z_AXIS, dir));
      const height = Math.max(0.001, p[2] - this.tableZ);
      this.drop.scale.set(1, height, 1);
      this.drop.position.set(p[0], p[1], this.tableZ + height / 2);
      this.foot.position.set(p[0], p[1], this.tableZ + 0.0008);
      this.anchorLocal.set(p[0], p[1], p[2]);
      const cm = (v: number) => (v * 100).toFixed(0);
      this.label.el.textContent = `target  ${cm(p[0])}, ${cm(p[1])}, ${cm(p[2])} cm`;
      LabelLayer.changed(this.label);
    }
  }

  update(dt: number, worldMatrix: Matrix4): void {
    if (!this.group.visible) return;
    this.spin.rotation.z += dt * 1.6;
    this.label.anchor.copy(this.anchorLocal).applyMatrix4(worldMatrix);
  }

  dispose(): void {
    this.set(null);
    this.casing.dispose();
    this.core.dispose();
    this.flat.dispose();
    this.dim.dispose();
    this.dark.dispose();
    this.group.traverse((o) => (o as Mesh).geometry?.dispose());
  }
}

/* ------------------------------------------------------------------ */
/* Tool-point trail                                                    */
/* ------------------------------------------------------------------ */

const TRAIL_MAX = 110;
const TRAIL_LIFE_MS = 1700;
const TRAIL_SPACING = 0.0045;

export class Trail {
  readonly mesh: InstancedMesh;
  readonly tip: Mesh;
  private pos: Vector3[] = [];
  private born: number[] = [];
  private head = 0;
  private count = 0;
  private last = new Vector3(Number.NaN, 0, 0);
  private m = new Matrix4();
  private material = new MeshBasicMaterial({ color: CHALK, transparent: true, opacity: 0.9 });
  private tipMaterial = new MeshBasicMaterial({ color: CHALK, depthTest: false, transparent: true, opacity: 0.95 });

  constructor() {
    this.mesh = new InstancedMesh(new SphereGeometry(1, 10, 8), this.material, TRAIL_MAX);
    this.mesh.instanceMatrix.setUsage(DynamicDrawUsage);
    this.mesh.frustumCulled = false;
    this.mesh.count = 0;
    for (let i = 0; i < TRAIL_MAX; i++) {
      this.pos.push(new Vector3());
      this.born.push(0);
    }
    this.tip = new Mesh(new SphereGeometry(0.0042, 16, 12), this.tipMaterial);
    this.tip.renderOrder = 7;
    this.tip.visible = false;
  }

  /** Feed the current tool point every frame. */
  push(p: Vec3, now: number): void {
    this.tip.visible = true;
    this.tip.position.set(p[0], p[1], p[2]);
    const cur = this.tip.position;
    if (Number.isNaN(this.last.x)) {
      this.last.copy(cur);
      return;
    }
    const d = cur.distanceTo(this.last);
    if (d > 0.25) {
      // A jump (reconnect, teleport): not motion worth drawing.
      this.last.copy(cur);
      return;
    }
    if (d < TRAIL_SPACING) return;
    this.pos[this.head].copy(cur);
    this.born[this.head] = now;
    this.head = (this.head + 1) % TRAIL_MAX;
    this.count = Math.min(TRAIL_MAX, this.count + 1);
    this.last.copy(cur);
  }

  update(now: number): void {
    let n = 0;
    for (let k = 0; k < this.count; k++) {
      const i = (this.head - 1 - k + TRAIL_MAX * 2) % TRAIL_MAX;
      const age = (now - this.born[i]) / TRAIL_LIFE_MS;
      if (age >= 1) break;
      const s = 0.0034 * Math.pow(1 - age, 0.8);
      this.m.makeScale(s, s, s).setPosition(this.pos[i]);
      this.mesh.setMatrixAt(n++, this.m);
    }
    this.mesh.count = n;
    this.mesh.instanceMatrix.needsUpdate = true;
  }

  dispose(): void {
    this.mesh.geometry.dispose();
    this.tip.geometry.dispose();
    this.material.dispose();
    this.tipMaterial.dispose();
  }
}

/* ------------------------------------------------------------------ */
/* World markers                                                       */
/* ------------------------------------------------------------------ */

interface MarkerVis {
  group: Group;
  body: Mesh;
  ring: Mesh;
  material: MeshStandardMaterial;
  ringMaterial: MeshBasicMaterial;
  label: Label;
  target: Vector3;
  radius: number;
  text: string;
  color: string;
  born: number;
}

const DEFAULT_MARKER_COLOR = '#ffffff';
const DEFAULT_MARKER_RADIUS = 0.02;

export class Markers {
  readonly group = new Group();
  private items = new Map<string, MarkerVis>();
  private sphere = new SphereGeometry(1, 28, 20);
  private ringGeo = new RingGeometry(1.0, 1.14, 48);
  private tableZ = 0;
  private v = new Vector3();

  constructor(private labels: LabelLayer) {}

  setTableZ(z: number): void {
    this.tableZ = z;
  }

  set(markers: WorldMarker[], now: number): void {
    const seen = new Set<string>();
    for (const mk of markers) {
      if (!mk || !Array.isArray(mk.pos)) continue;
      const id = String(mk.id ?? mk.label);
      seen.add(id);
      const color = mk.color || DEFAULT_MARKER_COLOR;
      const radius = mk.radius && mk.radius > 0 ? mk.radius : DEFAULT_MARKER_RADIUS;
      let vis = this.items.get(id);
      if (!vis) {
        const material = new MeshStandardMaterial({ roughness: 0.45, metalness: 0, transparent: true, opacity: 0.92 });
        const ringMaterial = new MeshBasicMaterial({ color: CHALK, transparent: true, opacity: 0.85, side: DoubleSide, depthWrite: false });
        const body = new Mesh(this.sphere, material);
        body.castShadow = true;
        const ring = new Mesh(this.ringGeo, ringMaterial);
        ring.renderOrder = 2;
        const group = new Group();
        group.add(body);
        this.group.add(group, ring);
        const label = this.labels.add('tag3d--marker');
        label.el.innerHTML = '<i class="tag3d__dot"></i><span class="tag3d__text"></span>';
        vis = {
          group,
          body,
          ring,
          material,
          ringMaterial,
          label,
          target: new Vector3(mk.pos[0], mk.pos[1], mk.pos[2]),
          radius,
          text: '',
          color: '',
          born: now,
        };
        group.position.copy(vis.target);
        this.items.set(id, vis);
      }
      vis.target.set(mk.pos[0], mk.pos[1], mk.pos[2]);
      vis.radius = radius;
      if (vis.color !== color) {
        vis.color = color;
        try {
          vis.material.color.set(color);
        } catch {
          vis.material.color.set(DEFAULT_MARKER_COLOR);
        }
        (vis.label.el.firstElementChild as HTMLElement).style.background = `#${new Color(vis.material.color).getHexString()}`;
      }
      const text = mk.label || id;
      if (vis.text !== text) {
        vis.text = text;
        (vis.label.el.lastElementChild as HTMLElement).textContent = text;
        LabelLayer.changed(vis.label);
      }
    }
    for (const [id, vis] of this.items) {
      if (seen.has(id)) continue;
      this.group.remove(vis.group, vis.ring);
      vis.material.dispose();
      vis.ringMaterial.dispose();
      this.labels.remove(vis.label);
      this.items.delete(id);
    }
  }

  update(now: number, dt: number, worldMatrix: Matrix4): void {
    const k = 1 - Math.exp(-dt / 0.07);
    for (const vis of this.items.values()) {
      vis.group.position.lerp(vis.target, k);
      const pop = Math.min(1, (now - vis.born) / 260);
      const s = vis.radius * (0.6 + 0.4 * (1 - Math.pow(1 - pop, 3)));
      vis.body.scale.setScalar(s);
      const p = vis.group.position;
      // The ring stays on the table under the object, wherever it is carried.
      vis.ring.position.set(p.x, p.y, this.tableZ + 0.0007);
      vis.ring.scale.setScalar(vis.radius + 0.008);
      const height = p.z - this.tableZ;
      vis.ringMaterial.opacity = height > vis.radius * 1.6 ? 0.4 : 0.85;
      vis.label.anchor.copy(this.v.set(p.x, p.y, p.z + vis.radius)).applyMatrix4(worldMatrix);
      vis.label.lift = 18;
    }
  }

  dispose(): void {
    this.set([], 0);
    this.sphere.dispose();
    this.ringGeo.dispose();
  }
}
