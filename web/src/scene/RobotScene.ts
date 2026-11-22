import {
  Group,
  NeutralToneMapping,
  PCFShadowMap,
  PerspectiveCamera,
  Plane,
  Scene,
  Vector2,
  Vector3,
  WebGLRenderer,
  type ShaderMaterial,
} from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { URDFRobot } from 'urdf-loader';
import type { RobotState, WorldMarker } from '../api/types.ts';
import { live, type Plan, type Sample } from '../state/app.ts';
import { LabelLayer, Markers, PlanOverlay, Trail } from './overlays.ts';
import {
  GRIPPER_JOINT,
  applyGripper,
  loadRobot,
  makeGhost,
  makeGhostMaterial,
  setClipping,
  setServoFault,
  type LoadProgress,
  type RobotInfo,
  type RobotModel,
} from './robot.ts';
import { buildLights, buildStage, DEFAULT_REACH, type Stage } from './stage.ts';

/**
 * Scene orientation. The daemon, the URDF, plans and markers all use the
 * robot base frame: x = right, y = forward, z = up. three.js is Y-up, so the
 * robot, the mat and every overlay live in one group (`world`) rotated by
 * this much about X. Nothing else converts coordinates.
 */
const WORLD_ROTATION_X = -Math.PI / 2;

/**
 * Render this far behind the newest state so there are always two samples to
 * blend. State arrives every 33 ms; this covers one late message.
 */
const INTERP_DELAY_MS = 55;
/** Time constant of the last-stage smoothing that rounds off the corners between samples. */
const SMOOTH_TAU_S = 0.022;

const FOV = 30;
/** Direction from the target to the camera: front-right of the robot, looking down a little. */
const VIEW_AZIMUTH = (124 * Math.PI) / 180;
const VIEW_POLAR = (63 * Math.PI) / 180;
/**
 * The default view frames the arm's working volume, sized from its reach:
 * the point looked at (above the table, ahead of the shoulder axis) and the
 * half extents that must stay visible between the panels, all in reaches.
 */
const VIEW_HEIGHT = 0.36;
const VIEW_AHEAD = 0.22;
const FIT_HALF_WIDTH = 0.84;
const FIT_HALF_HEIGHT = 0.62;
/** Cap on rendered pixels per CSS pixel: past this the picture does not get sharper, only slower. */
const MAX_PIXEL_RATIO = 2;

export interface Insets {
  left: number;
  right: number;
  top: number;
  bottom: number;
}

const lerp = (a: number, b: number, t: number) => a + (b - a) * t;

export class RobotScene {
  private renderer: WebGLRenderer;
  private scene = new Scene();
  private camera = new PerspectiveCamera(FOV, 1, 0.02, 30);
  private controls: OrbitControls;
  private world = new Group();
  private stage: Stage;
  private lights: { dispose(): void };
  private labels: LabelLayer;
  private plan: PlanOverlay;
  private trail = new Trail();
  private markers: Markers;

  private model: RobotModel | null = null;
  private ghost: URDFRobot | null = null;
  private ghostMaterial: ShaderMaterial = makeGhostMaterial();
  private ghostOpacity = 0;
  private ghostEnabled = true;
  private faults = new Map<number, boolean>();
  /** Joint values currently drawn, by URDF joint name (measured, and commanded for the ghost). */
  private shown = new Map<string, { q: number; goal: number }>();
  private shownOpen = Number.NaN;
  private shownEe: [number, number, number] = [Number.NaN, Number.NaN, Number.NaN];
  private reach = DEFAULT_REACH;
  private reachFromDaemon = false;
  private baseAxis: [number, number] = [0, 0];
  private loadToken = 0;

  private clip = new Plane(new Vector3(0, -1, 0), 0);
  private reveal: { t0: number; from: number; to: number } | null = null;

  private size = new Vector2(1, 1);
  private insets: Insets = { left: 0, right: 0, top: 0, bottom: 0 };
  private tableZ = 0;
  private lastFrame = performance.now();
  private flight: { t0: number; fromPos: Vector3; fromTarget: Vector3; toPos: Vector3; toTarget: Vector3 } | null = null;
  private userMoved = false;
  private resizeObserver: ResizeObserver;
  private disposed = false;

  constructor(
    private host: HTMLElement,
    labelHost: HTMLElement,
  ) {
    const renderer = new WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, MAX_PIXEL_RATIO));
    renderer.setClearColor(0x000000, 0);
    renderer.toneMapping = NeutralToneMapping;
    renderer.toneMappingExposure = 1.06;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = PCFShadowMap;
    renderer.localClippingEnabled = true;
    host.appendChild(renderer.domElement);
    this.renderer = renderer;

    this.world.rotation.x = WORLD_ROTATION_X;
    this.scene.add(this.world);

    this.stage = buildStage();
    this.stage.setPixelRatio(renderer.getPixelRatio());
    this.world.add(this.stage.group);
    this.lights = buildLights(this.scene, renderer);

    this.labels = new LabelLayer(labelHost);
    this.plan = new PlanOverlay(this.labels);
    this.markers = new Markers(this.labels);
    this.world.add(this.markers.group, this.trail.mesh, this.trail.tip, this.plan.group);

    const controls = new OrbitControls(this.camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.09;
    controls.minDistance = 0.22;
    controls.maxDistance = 3.4;
    controls.minPolarAngle = 0.12;
    controls.maxPolarAngle = 1.5; // never dip under the table
    controls.rotateSpeed = 0.75;
    controls.zoomSpeed = 0.8;
    controls.addEventListener('start', () => {
      this.userMoved = true;
      this.flight = null;
    });
    controls.addEventListener('change', () => {
      // Panning may wander, but not so far that the arm is lost.
      const t = controls.target;
      t.x = Math.min(0.55, Math.max(-0.55, t.x));
      t.y = Math.min(0.5, Math.max(0, t.y));
      t.z = Math.min(0.35, Math.max(-0.65, t.z));
    });
    this.controls = controls;

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(host);
    this.resize();
    this.placeCamera(this.defaultView());

    renderer.setAnimationLoop(this.frame);
  }

  /* ------------------------------ public ------------------------------ */

  /** Loads the arm the daemon describes, replacing any arm already on the mat. */
  async loadRobot(urdfUrl: string, info: RobotInfo, onProgress: (p: LoadProgress) => void): Promise<void> {
    const token = ++this.loadToken;
    const model = await loadRobot(urdfUrl, info, onProgress);
    if (this.disposed || token !== this.loadToken) return;
    this.removeModel();
    this.model = model;
    this.world.add(model.robot);
    this.faults.clear();
    this.shown.clear();
    this.shownOpen = Number.NaN;
    this.baseAxis = model.baseAxis;
    if (!this.reachFromDaemon) this.reach = model.reach;
    this.stage.setReach(this.reach, this.baseAxis);
    if (!this.userMoved && !this.flight) this.placeCamera(this.defaultView());

    this.ghost = makeGhost(model.robot, this.ghostMaterial);
    this.ghost.visible = false;
    this.world.add(this.ghost);

    // Build the arm up from the table, like a print coming off the bed.
    const still = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    if (!still) {
      setClipping(model, this.clip);
      this.clip.constant = this.tableZ;
      this.reveal = { t0: performance.now(), from: this.tableZ - 0.01, to: this.tableZ + 0.56 };
    }
  }

  /** The reach the daemon plans with, when it says; otherwise the model's own estimate is used. */
  setReach(reach: number | null): void {
    this.reachFromDaemon = reach !== null && reach > 0;
    if (this.reachFromDaemon) this.reach = reach as number;
    else if (this.model) this.reach = this.model.reach;
    this.stage.setReach(this.reach, this.baseAxis);
    if (!this.userMoved && !this.flight) this.placeCamera(this.defaultView());
  }

  private removeModel(): void {
    if (this.model) {
      this.world.remove(this.model.robot);
      this.model.materials.forEach((m) => m.dispose());
      this.model = null;
    }
    if (this.ghost) {
      this.world.remove(this.ghost);
      this.ghost = null;
    }
    this.reveal = null;
  }

  setTableZ(z: number): void {
    this.tableZ = z;
    this.stage.setTableZ(z);
    this.plan.setTableZ(z);
    this.markers.setTableZ(z);
  }

  setPlan(plan: Plan | null): void {
    this.plan.set(plan);
  }

  setMarkers(markers: WorldMarker[]): void {
    this.markers.set(markers, performance.now());
  }

  /** Show or hide the commanded-pose ghost. */
  setGhostEnabled(on: boolean): void {
    this.ghostEnabled = on;
  }

  /** Screen space covered by panels; the default view centres in what is left. */
  setInsets(insets: Insets): void {
    this.insets = insets;
    this.applyViewOffset();
    if (!this.userMoved && !this.flight) this.placeCamera(this.defaultView());
  }

  resetView(): void {
    const to = this.defaultView();
    this.userMoved = false;
    this.flight = {
      t0: performance.now(),
      fromPos: this.camera.position.clone(),
      fromTarget: this.controls.target.clone(),
      toPos: to.pos,
      toTarget: to.target,
    };
  }

  dispose(): void {
    this.disposed = true;
    this.renderer.setAnimationLoop(null);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.plan.dispose();
    this.trail.dispose();
    this.markers.dispose();
    this.labels.dispose();
    this.stage.dispose();
    this.lights.dispose();
    this.ghostMaterial.dispose();
    if (this.model) {
      const seen = new Set<unknown>();
      this.model.robot.traverse((o) => {
        const g = (o as { geometry?: { dispose(): void } }).geometry;
        if (g && !seen.has(g)) {
          seen.add(g);
          g.dispose();
        }
      });
      this.model.materials.forEach((m) => m.dispose());
    }
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }

  /* ------------------------------ camera ------------------------------ */

  private free(): { w: number; h: number; cx: number; cy: number } {
    const { left, right, top, bottom } = this.insets;
    const w = Math.max(160, this.size.x - left - right);
    const h = Math.max(160, this.size.y - top - bottom);
    return { w, h, cx: left + w / 2, cy: top + h / 2 };
  }

  private defaultView(): { pos: Vector3; target: Vector3 } {
    const free = this.free();
    const tan = Math.tan((FOV * Math.PI) / 360);
    const r = this.reach;
    const d = Math.max(
      (FIT_HALF_WIDTH * r * this.size.y) / (free.w * tan),
      (FIT_HALF_HEIGHT * r * this.size.y) / (free.h * tan),
    );
    const dist = Math.min(3.2, Math.max(0.5, d));
    const dir = new Vector3(
      Math.sin(VIEW_POLAR) * Math.sin(VIEW_AZIMUTH),
      Math.cos(VIEW_POLAR),
      Math.sin(VIEW_POLAR) * Math.cos(VIEW_AZIMUTH),
    );
    // Base frame (x right, y forward, z up) to three (x, z, -y).
    const target = new Vector3(this.baseAxis[0], this.tableZ + VIEW_HEIGHT * r, -(this.baseAxis[1] + VIEW_AHEAD * r));
    return { pos: target.clone().addScaledVector(dir, dist), target };
  }

  private placeCamera(view: { pos: Vector3; target: Vector3 }): void {
    this.camera.position.copy(view.pos);
    this.controls.target.copy(view.target);
    this.controls.update();
  }

  /** Shift the optical centre into the middle of the space the panels leave free. */
  private applyViewOffset(): void {
    const { x: w, y: h } = this.size;
    const free = this.free();
    const dx = Math.round(free.cx - w / 2);
    const dy = Math.round(free.cy - h / 2);
    if (dx === 0 && dy === 0) this.camera.clearViewOffset();
    else this.camera.setViewOffset(w, h, -dx, -dy, w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  private resize(): void {
    const w = Math.max(1, this.host.clientWidth);
    const h = Math.max(1, this.host.clientHeight);
    if (w === this.size.x && h === this.size.y) return;
    this.size.set(w, h);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, MAX_PIXEL_RATIO));
    this.renderer.setSize(w, h);
    this.stage.setPixelRatio(this.renderer.getPixelRatio());
    this.plan.setResolution(this.size);
    this.applyViewOffset();
    if (!this.userMoved && !this.flight) this.placeCamera(this.defaultView());
  }

  /* ------------------------------- frame ------------------------------ */

  private frame = (): void => {
    const now = performance.now();
    const dt = Math.min(0.1, (now - this.lastFrame) / 1000);
    this.lastFrame = now;

    this.applyState(now, dt);

    if (this.reveal) {
      const u = Math.min(1, (now - this.reveal.t0) / 1500);
      const e = 1 - Math.pow(1 - u, 3);
      this.clip.constant = lerp(this.reveal.from, this.reveal.to, e);
      if (u >= 1) {
        this.reveal = null;
        if (this.model) setClipping(this.model, null);
      }
    }

    if (this.flight) {
      const u = Math.min(1, (now - this.flight.t0) / 650);
      const e = u * u * (3 - 2 * u);
      this.camera.position.lerpVectors(this.flight.fromPos, this.flight.toPos, e);
      this.controls.target.lerpVectors(this.flight.fromTarget, this.flight.toTarget, e);
      if (u >= 1) this.flight = null;
    }
    this.controls.update();

    this.world.updateMatrixWorld();
    this.trail.update(now);
    this.markers.update(now, dt, this.world.matrixWorld);
    this.plan.update(dt, this.world.matrixWorld);
    this.camera.updateMatrixWorld();
    this.labels.update(this.camera, this.size.x, this.size.y);

    this.renderer.render(this.scene, this.camera);
  };

  /** Pose the twin from the two state samples that bracket "a moment ago". */
  private applyState(now: number, dt: number): void {
    const model = this.model;
    const samples = live.samples;
    if (samples.length === 0) return;

    const t = now - INTERP_DELAY_MS;
    let a: Sample = samples[0];
    let b: Sample = samples[0];
    if (t >= samples[samples.length - 1].at) {
      a = b = samples[samples.length - 1];
    } else {
      for (let i = samples.length - 1; i > 0; i--) {
        if (samples[i - 1].at <= t) {
          a = samples[i - 1];
          b = samples[i];
          break;
        }
      }
    }
    const span = b.at - a.at;
    const k = span > 0 ? Math.min(1, Math.max(0, (t - a.at) / span)) : 0;
    const sa: RobotState = a.s;
    const sb: RobotState = b.s;

    if (!model) return;

    // Follow the interpolated sample through a short low-pass: motion stays
    // continuous even when a message is late or two carry the same reading.
    const follow = 1 - Math.exp(-dt / SMOOTH_TAU_S);
    const ease = (from: number, to: number) =>
      Number.isFinite(from) && Math.abs(to - from) < 0.6 ? from + (to - from) * follow : to;

    let lag = 0;
    const n = Math.min(sa.joints?.length ?? 0, sb.joints?.length ?? 0);
    for (let i = 0; i < n; i++) {
      const ja = sa.joints[i];
      const jb = sb.joints[i];
      const fault = jb.error !== 0;
      if (this.faults.get(jb.id) !== fault) {
        this.faults.set(jb.id, fault);
        setServoFault(model, jb.id, fault);
      }
      // Joints are matched by URDF name. The gripper reports an open fraction,
      // not an angle; its URDF joints are driven below.
      if (jb.name === GRIPPER_JOINT || !(jb.name in model.robot.joints)) continue;
      let cur = this.shown.get(jb.name);
      if (!cur) this.shown.set(jb.name, (cur = { q: Number.NaN, goal: Number.NaN }));
      cur.q = ease(cur.q, lerp(ja.q, jb.q, k));
      const goal = lerp(ja.q_goal, jb.q_goal, k);
      cur.goal = Number.isFinite(goal) ? ease(cur.goal, goal) : cur.q;
      model.robot.setJointValue(jb.name, cur.q);
      this.ghost?.setJointValue(jb.name, cur.goal);
      lag = Math.max(lag, Math.abs(cur.goal - cur.q));
    }
    const open = ease(this.shownOpen, lerp(sa.gripper?.open ?? 1, sb.gripper?.open ?? 1, k));
    this.shownOpen = open;
    applyGripper(model.robot, open);

    // The tool point gets the same smoothing so it stays glued to the gripper.
    if (sa.ee?.pos && sb.ee?.pos) {
      const e = this.shownEe;
      for (let i = 0; i < 3; i++) e[i] = ease(e[i], lerp(sa.ee.pos[i], sb.ee.pos[i], k));
      this.trail.push([e[0], e[1], e[2]], now);
    }

    // The ghost shows the commanded pose only while it differs from the measured one.
    if (this.ghost) {
      applyGripper(this.ghost, open);
      const want = this.reveal || !this.ghostEnabled ? 0 : Math.min(1, Math.max(0, (lag - 0.035) / 0.09));
      this.ghostOpacity += (want - this.ghostOpacity) * (1 - Math.exp(-dt / 0.12));
      this.ghostMaterial.uniforms.uOpacity.value = this.ghostOpacity;
      this.ghost.visible = this.ghostOpacity > 0.01;
    }
  }
}
