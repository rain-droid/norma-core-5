/**
 * In-browser stand-in for the robot daemon. It speaks the same contract as
 * docs/API.md (`hello`, `state` at 30 Hz, `event`s, and the HTTP calls) so the
 * UI can be developed and demoed with no backend. It loops a scripted scene:
 * a look, a pick that fails once and is retried, a place, a wave from Claude
 * Code and a dap. Speech events are never emitted (there is no audio here).
 */
import type { CameraSource, Transport, TransportHandlers } from '../transport.ts';
import type {
  ChatResult,
  Hello,
  JointMeta,
  JointState,
  RobotEvent,
  RobotState,
  Source,
  ToolResult,
  UtteranceResult,
  Vec3,
  VoiceState,
} from '../types.ts';
import { Kinematics } from './kin.ts';
import { CAM_H, CAM_W, detect, makeObjects, paintCamera, toMarkers, type WorldObject } from './world.ts';

const URDF_URL = '/assets/elrobot/elrobot_follower.urdf';
const TICK_MS = 33;
const GEAR_MAX = 2.2028;
const JAW_TRAVEL_MM = 51;
const GESTURES = ['dap', 'wave', 'nod'];

const HOME = [0, 0, 0, 0, 0, 0, 0];
const REST = [0, -1.25, -0.55, 1.62, 0, 0.95, 0];
/** Used only if the URDF could not be fetched (then there is no IK either). */
const FALLBACK_LIMITS: [number, number][] = [
  [-1.5509, 1.5509],
  [-1.6122, 1.6122],
  [-1.761, 1.761],
  [-1.7533, 1.7533],
  [-2.62, 3.252],
  [-1.3775, 1.7641],
  [-3.2014, 2.7336],
];
/** Raw servo units (about 6.5 mA each). */
const IDLE_CURRENT = [5, 34, 27, 15, 4, 10, 4, 3];
const IDLE_TEMP = [33, 38, 41, 39, 34, 36, 33, 35];

const TOOLS = [
  ['home', 'Move to the home pose.'],
  ['rest', 'Fold down into the rest pose.'],
  ['stop', 'Stop all motion and hold position.'],
  ['gesture', 'Play a named gesture.'],
  ['gripper', 'Open or close the gripper (0 closed, 1 open).'],
  ['move_joints', 'Move joints by id, absolute or relative, in radians.'],
  ['look', 'Look at the table and detect objects.'],
  ['pick', 'Pick up a detected object.'],
  ['place', 'Place the held object on a target.'],
] as const;

class Cancelled extends Error {}
class ToolFailure extends Error {
  constructor(
    readonly code: string,
    summary: string,
  ) {
    super(summary);
  }
}

interface Motion {
  qs: number[][];
  t0: number;
  dur: number;
  done(): void;
}

interface Waypoint {
  pos: Vec3;
  approach: Vec3;
}

const lerp = (a: number, b: number, t: number) => a + (b - a) * t;
const ease = (u: number) => u * u * u * (u * (u * 6 - 15) + 10);
const unit = (v: Vec3): Vec3 => {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
};
const DOWN: Vec3 = [0, 0, -1];
const TILTED: Vec3 = unit([0, 0.3, -0.95]);

export class MockTransport implements Transport {
  readonly kind = 'mock' as const;

  private handlers: TransportHandlers | null = null;
  private kin: Kinematics | null = null;
  private disposed = false;
  private ticker: number | undefined;
  private resumeTimer: number | undefined;

  private q = [...HOME];
  private qGoal = [...HOME];
  private vel = new Array<number>(8).fill(0);
  private grip = 0.85;
  private gripGoal = 0.85;
  private holding: WorldObject | null = null;
  private motion: Motion | null = null;
  private temps = [...IDLE_TEMP];
  private gripError = 0;

  private torqueOn = true;
  private busy = false;
  private activity = 'idle';
  private source: Source | null = null;
  private objects = makeObjects();

  /** Bumping this cancels whatever script is running. */
  private epoch = 0;
  private uiBusy = false;
  private eventId = 0;
  private callSeq = 0;
  private turnSeq = 0;
  private utteranceSeq = 0;
  private lastTick = 0;

  /* ----------------------------- lifecycle ----------------------------- */

  start(handlers: TransportHandlers): void {
    this.handlers = handlers;
    handlers.onLink('connecting', null);
    void this.boot();
  }

  private async boot(): Promise<void> {
    try {
      const res = await fetch(URDF_URL);
      if (res.ok) this.kin = new Kinematics(await res.text());
    } catch {
      /* no URDF: joints still move, the tool point is approximate */
    }
    if (this.disposed) return;

    const limits = this.kin ? this.kin.arm.map((j) => [j.lower, j.upper] as [number, number]) : FALLBACK_LIMITS;
    const joints: JointMeta[] = limits.map(([lower, upper], i) => ({
      id: i + 1,
      name: `rev_motor_0${i + 1}`,
      lower,
      upper,
    }));
    joints.push({ id: 8, name: 'rev_motor_08', lower: 0, upper: GEAR_MAX });

    const hello: Hello = {
      type: 'hello',
      version: 1,
      mode: 'sim',
      urdf: URDF_URL,
      joints,
      tools: TOOLS.map(([name, description]) => ({
        name,
        description,
        input_schema: { type: 'object', properties: {} },
      })),
      gestures: GESTURES,
      cameras: ['wrist'],
      table_z: 0,
      events: [],
    };
    this.limits = joints;
    this.handlers?.onMessage(hello);

    this.lastTick = performance.now();
    this.ticker = window.setInterval(() => this.tick(), TICK_MS);
    this.startDemo(1200);
  }

  private limits: JointMeta[] = [];

  dispose(): void {
    this.disposed = true;
    this.epoch++;
    window.clearInterval(this.ticker);
    window.clearTimeout(this.resumeTimer);
  }

  retryNow(): void {
    /* the mock never drops */
  }

  /* ------------------------------ physics ------------------------------ */

  private pose() {
    if (this.kin) return this.kin.fk(this.q);
    return {
      pos: [0.004, 0.348, 0.218] as Vec3,
      approach: [0, 1, 0] as Vec3,
      quat: [0, 0, 0, 1] as [number, number, number, number],
    };
  }

  /** Freezes the arm mid-motion (handy for screenshots): set from the console via `__elrobotMock.paused = true`. */
  paused = false;

  private tick(): void {
    const now = performance.now();
    if (this.paused && this.motion) this.motion.t0 += now - this.lastTick;
    const dt = this.paused ? 0 : Math.min(0.1, (now - this.lastTick) / 1000);
    this.lastTick = now;

    const m = this.motion;
    if (m) {
      const u = Math.min(1, (now - m.t0) / m.dur);
      const f = ease(u) * (m.qs.length - 1);
      const i = Math.min(m.qs.length - 2, Math.floor(f));
      const a = m.qs[i];
      const b = m.qs[i + 1];
      this.qGoal = a.map((v, k) => lerp(v, b[k], f - i));
      if (u >= 1 && this.q.every((v, k) => Math.abs(v - this.qGoal[k]) < 0.012)) {
        this.motion = null;
        m.done();
      }
    }

    // Servos chase the commanded angle with a little lag; a limp arm sags.
    const prev = [...this.q];
    if (this.torqueOn) {
      const k = 1 - Math.exp(-dt / 0.085);
      this.q = this.q.map((v, i) => v + (this.qGoal[i] - v) * k);
    } else {
      const k = 1 - Math.exp(-dt / 0.9);
      this.q = this.q.map((v, i) => v + (REST[i] - v) * k * (i === 0 || i === 4 || i === 6 ? 0 : 1));
      this.qGoal = [...this.q];
    }
    const prevGrip = this.grip;
    const step = 2.4 * dt;
    this.grip += Math.max(-step, Math.min(step, this.gripGoal - this.grip));

    if (dt > 0) {
      for (let i = 0; i < 7; i++) this.vel[i] = lerp(this.vel[i], (this.q[i] - prev[i]) / dt, 0.4);
      this.vel[7] = lerp(this.vel[7], ((prevGrip - this.grip) * GEAR_MAX) / dt, 0.4);
    }

    const pose = this.pose();
    if (this.holding) {
      this.holding.pos = [pose.pos[0], pose.pos[1], pose.pos[2]];
      if (now - this.lastMarkersAt > 120) this.sendMarkers();
    }

    const gearQ = (1 - this.grip) * GEAR_MAX;
    const gearGoal = (1 - this.gripGoal) * GEAR_MAX;
    const joints: JointState[] = this.limits.map((meta, i) => {
      const q = i < 7 ? this.q[i] : gearQ;
      const speed = Math.abs(this.vel[i]);
      let current = IDLE_CURRENT[i] + speed * 62 + Math.random() * 2.5;
      if (i === 7 && this.holding) current += 46;
      if (i === 7 && this.gripError) current = 372 + Math.random() * 12;
      if (!this.torqueOn) current = Math.random() * 1.5;
      this.temps[i] = lerp(this.temps[i], IDLE_TEMP[i] + Math.min(22, current / 5), dt * 0.05);
      return {
        id: meta.id,
        name: meta.name,
        q,
        q_goal: i < 7 ? this.qGoal[i] : gearGoal,
        lower: meta.lower,
        upper: meta.upper,
        tick: Math.round(2048 + (q / (2 * Math.PI)) * 4096),
        current: Math.round(current),
        load: Math.round(Math.sign(this.vel[i] || 1) * Math.min(1000, current * 2.4)),
        temp: Math.round(this.temps[i]),
        volt: Math.round((12.1 - current / 500 + Math.random() * 0.04) * 10) / 10,
        torque: this.torqueOn,
        error: i === 7 ? this.gripError : 0,
      };
    });

    const state: RobotState = {
      type: 'state',
      t: Date.now() / 1000,
      mode: 'sim',
      connected: true,
      calibrated: true,
      torque: this.torqueOn,
      busy: this.busy,
      activity: this.activity,
      source: this.busy ? this.source : null,
      joints,
      gripper: {
        open: this.grip,
        width_mm: Math.round(this.grip * JAW_TRAVEL_MM * 10) / 10,
        holding: this.holding !== null,
      },
      ee: pose,
      cameras: [
        {
          name: 'wrist',
          width: CAM_W,
          height: CAM_H,
          fps: 15 + Math.sin(now / 900) * 0.4,
          age_ms: Math.round(35 + Math.random() * 30),
        },
      ],
    };
    this.handlers?.onMessage(state);
  }

  /* ------------------------- script primitives ------------------------- */

  private emit(ev: Record<string, unknown> & { kind: RobotEvent['kind'] }): void {
    const full = { type: 'event', id: ++this.eventId, t: Date.now() / 1000, ...ev };
    this.handlers?.onMessage(full as unknown as RobotEvent);
  }

  private guard(my: number): void {
    if (my !== this.epoch || this.disposed) throw new Cancelled();
  }

  /** Pending sleeps, so a takeover can wake (and thereby cancel) the old script at once. */
  private wakers = new Set<() => void>();

  private async sleep(my: number, ms: number): Promise<void> {
    await new Promise<void>((resolve) => {
      const wake = () => {
        window.clearTimeout(timer);
        this.wakers.delete(wake);
        resolve();
      };
      const timer = window.setTimeout(wake, ms);
      this.wakers.add(wake);
    });
    this.guard(my);
  }

  /** Run joint-space waypoints; resolves when the arm has arrived. */
  private async follow(my: number, qs: number[][], seconds?: number): Promise<void> {
    this.guard(my);
    if (!this.torqueOn) throw new ToolFailure('torque_off', 'torque is off, the arm is limp');
    const path = [[...this.qGoal], ...qs];
    let travel = 0;
    for (let i = 1; i < path.length; i++) {
      travel += Math.max(...path[i].map((v, k) => Math.abs(v - path[i - 1][k])));
    }
    const dur = (seconds ?? Math.min(3.2, Math.max(0.55, travel / 1.15))) * 1000;
    await new Promise<void>((resolve) => {
      this.motion = { qs: path, t0: performance.now(), dur, done: resolve };
    });
    this.guard(my);
  }

  private async setGripper(my: number, open: number): Promise<void> {
    this.guard(my);
    this.gripGoal = Math.min(1, Math.max(0, open));
    await this.sleep(my, Math.abs(this.gripGoal - this.grip) * 420 + 120);
  }

  private voice(state: VoiceState): void {
    this.emit({ kind: 'voice_state', state });
  }

  private lastMarkersAt = 0;

  private sendMarkers(): void {
    this.lastMarkersAt = performance.now();
    this.emit({ kind: 'markers', markers: toMarkers(this.objects) });
  }

  private clearPlan(callId = ''): void {
    this.emit({ kind: 'plan', call_id: callId, path: [], target: null });
  }

  /** Cartesian waypoints to joint waypoints, plus the tool path they really trace. */
  private solve(waypoints: Waypoint[]): { qs: number[][]; path: Vec3[] } {
    const kin = this.kin;
    if (!kin) return { qs: [[...this.qGoal]], path: [] };
    let q = [...this.qGoal];
    const start = kin.fk(q);
    const all: Waypoint[] = [{ pos: start.pos, approach: start.approach }, ...waypoints];
    const qs: number[][] = [];
    const path: Vec3[] = [start.pos];
    for (let i = 1; i < all.length; i++) {
      const a = all[i - 1];
      const b = all[i];
      const dist = Math.hypot(b.pos[0] - a.pos[0], b.pos[1] - a.pos[1], b.pos[2] - a.pos[2]);
      const steps = Math.max(2, Math.ceil(dist / 0.012));
      for (let s = 1; s <= steps; s++) {
        const t = s / steps;
        const pos: Vec3 = [lerp(a.pos[0], b.pos[0], t), lerp(a.pos[1], b.pos[1], t), lerp(a.pos[2], b.pos[2], t)];
        const dir = unit([
          lerp(a.approach[0], b.approach[0], t),
          lerp(a.approach[1], b.approach[1], t),
          lerp(a.approach[2], b.approach[2], t),
        ]);
        q = kin.ik(pos, dir, q, 0.06);
        qs.push(q);
        path.push(kin.fk(q).pos);
      }
    }
    return { qs, path };
  }

  /** Stream assistant text the way the daemon does: full text so far, then final. */
  private async say(my: number, turn: string, soFar: string, sentence: string, final: boolean): Promise<string> {
    this.voice('speaking');
    const words = sentence.split(' ');
    let text = soFar ? `${soFar} ` : '';
    for (let i = 0; i < words.length; i += 2) {
      text += `${words.slice(i, i + 2).join(' ')} `;
      const done = i + 2 >= words.length;
      this.emit({
        kind: 'chat',
        role: 'assistant',
        text: text.trimEnd(),
        turn_id: turn,
        final: final && done,
        via: 'voice',
      });
      await this.sleep(my, 150);
    }
    await this.sleep(my, Math.min(1400, sentence.length * 22));
    return text.trimEnd();
  }

  private async hear(my: number, text: string, viaVoice = true): Promise<string> {
    const turn = `turn-${++this.turnSeq}`;
    if (viaVoice) {
      this.voice('listening');
      await this.sleep(my, 1500);
      this.voice('transcribing');
      await this.sleep(my, 650);
    }
    this.emit({ kind: 'chat', role: 'user', text, turn_id: turn, final: true, via: viaVoice ? 'voice' : 'text' });
    this.voice('thinking');
    await this.sleep(my, 900);
    return turn;
  }

  /** Wraps a tool body with tool_start / tool_end and the busy flags. */
  private async tool(
    my: number,
    source: Source,
    name: string,
    args: Record<string, unknown>,
    activity: string,
    body: (callId: string) => Promise<string>,
  ): Promise<ToolResult> {
    this.guard(my);
    const callId = `call-${++this.callSeq}`;
    const t0 = performance.now();
    this.emit({ kind: 'tool_start', call_id: callId, tool: name, args, source });
    this.busy = true;
    this.activity = activity;
    this.source = source;
    const end = (ok: boolean, summary: string, error: string | null) => {
      this.busy = false;
      this.activity = 'idle';
      this.emit({
        kind: 'tool_end',
        call_id: callId,
        tool: name,
        ok,
        summary,
        error,
        duration_ms: Math.round(performance.now() - t0),
        source,
      });
    };
    try {
      const summary = await body(callId);
      end(true, summary, null);
      return { ok: true, summary };
    } catch (err) {
      if (err instanceof ToolFailure) {
        end(false, err.message, err.code);
        return { ok: false, summary: err.message, error: err.code };
      }
      end(false, 'stopped before it finished', 'stopped');
      throw err;
    }
  }

  /* ------------------------------- tools ------------------------------- */

  private runTool(my: number, source: Source, name: string, args: Record<string, unknown>): Promise<ToolResult> {
    switch (name) {
      case 'home':
        return this.tool(my, source, name, args, 'homing', async () => {
          await this.follow(my, [HOME]);
          return 'at home pose';
        });
      case 'rest':
        return this.tool(my, source, name, args, 'resting', async () => {
          await this.follow(my, [REST]);
          return 'folded into rest pose';
        });
      case 'gripper': {
        const open = Number(args.open ?? 1);
        return this.tool(my, source, name, args, open > 0.5 ? 'opening gripper' : 'closing gripper', async () => {
          if (open > 0.5 && this.holding) this.release();
          await this.setGripper(my, open);
          return `gripper at ${Math.round(this.gripGoal * JAW_TRAVEL_MM)} mm`;
        });
      }
      case 'move_joints':
        return this.tool(my, source, name, args, 'moving', () => this.moveJoints(my, args));
      case 'gesture': {
        const which = String(args.name ?? 'wave');
        return this.tool(my, source, name, args, which, () => this.gesture(my, which));
      }
      case 'look':
        return this.tool(my, source, name, args, 'looking', () => this.look(my));
      case 'pick':
        return this.tool(my, source, name, args, 'grasping', (callId) =>
          this.pick(my, callId, String(args.object ?? 'red block'), Boolean(args._miss)),
        );
      case 'place':
        return this.tool(my, source, name, args, 'placing', (callId) =>
          this.place(my, callId, String(args.target ?? 'paper cup')),
        );
      default:
        return this.tool(my, source, name, args, name, async () => {
          throw new ToolFailure('unknown_tool', `the demo does not know a tool called "${name}"`);
        });
    }
  }

  private async moveJoints(my: number, args: Record<string, unknown>): Promise<string> {
    const joints = (args.joints ?? {}) as Record<string, number>;
    const relative = Boolean(args.relative);
    const target = [...this.qGoal];
    const moved: string[] = [];
    for (const [key, raw] of Object.entries(joints)) {
      const id = Number(key);
      const value = Number(raw);
      if (!Number.isFinite(value)) continue;
      if (id >= 1 && id <= 7) {
        const meta = this.limits[id - 1];
        const want = relative ? target[id - 1] + value : value;
        target[id - 1] = Math.min(meta.upper, Math.max(meta.lower, want));
        moved.push(`J${id} ${((target[id - 1] * 180) / Math.PI).toFixed(0)}°`);
      } else if (id === 8) {
        const gear = (1 - this.gripGoal) * GEAR_MAX;
        const want = Math.min(GEAR_MAX, Math.max(0, relative ? gear + value : value));
        this.gripGoal = 1 - want / GEAR_MAX;
        moved.push(`grip ${Math.round(this.gripGoal * 100)}%`);
      }
    }
    if (moved.length === 0) throw new ToolFailure('bad_args', 'no valid joints given');
    await this.follow(my, [target], 0.45);
    return `moved ${moved.join(', ')}`;
  }

  private async gesture(my: number, which: string): Promise<string> {
    if (which === 'wave') {
      const up = [0, 0.05, -0.1, 0.05, 0, -1.3, 0];
      const left = [...up];
      const right = [...up];
      left[4] = 0.62;
      right[4] = -0.62;
      await this.follow(my, [up], 1.1);
      this.gripGoal = 1;
      await this.follow(my, [left, right, left, right, up], 2.6);
      await this.follow(my, [HOME], 1.0);
      return 'waved';
    }
    if (which === 'dap') {
      const ready = this.kin ? this.kin.ik([0, 0.355, 0.205], [0, 1, 0], HOME, 0.06) : [0, 0.3, -0.42, 0.26, 0, -0.15, 0];
      const bump = this.kin ? this.kin.ik([0, 0.405, 0.2], [0, 1, 0], ready, 0.06) : [0, 0.59, -0.55, 0, 0, 0, 0];
      this.gripGoal = 0.05;
      await this.follow(my, [ready], 0.9);
      await this.sleep(my, 250);
      await this.follow(my, [bump], 0.28);
      await this.follow(my, [ready], 0.35);
      await this.sleep(my, 200);
      this.gripGoal = 0.85;
      await this.follow(my, [HOME], 0.9);
      return 'dapped you up';
    }
    if (which === 'nod') {
      const down = [...HOME];
      down[5] = 0.5;
      const up = [...HOME];
      up[5] = -0.15;
      await this.follow(my, [down, up, down, HOME], 1.7);
      return 'nodded';
    }
    throw new ToolFailure('unknown_gesture', `no gesture called "${which}"`);
  }

  private async look(my: number): Promise<string> {
    const { qs } = this.solve([{ pos: [0, 0.17, 0.2], approach: unit([0, 0.62, -0.78]) }]);
    await this.follow(my, qs.length ? [qs[qs.length - 1]] : [HOME], 1.3);
    await this.sleep(my, 500);
    const pose = this.pose();
    const items = detect(this.objects, pose.pos, pose.approach);
    this.emit({ kind: 'detections', camera: 'wrist', width: CAM_W, height: CAM_H, items });
    this.sendMarkers();
    await this.sleep(my, 500);
    const seen = this.objects.filter((o) => !o.hidden).map((o) => o.label);
    return `found ${seen.length} things: ${seen.join(', ')}`;
  }

  private find(label: string): WorldObject {
    const key = label.toLowerCase();
    const hit = this.objects.find((o) => !o.hidden && (o.label === key || o.id === key || key.includes(o.id)));
    if (!hit) throw new ToolFailure('not_found', `I can't see a "${label}" on the table`);
    return hit;
  }

  private release(): void {
    const o = this.holding;
    if (!o) return;
    this.holding = null;
    o.pos = [o.pos[0], o.pos[1], o.radius];
    this.sendMarkers();
  }

  private async pick(my: number, callId: string, label: string, miss: boolean): Promise<string> {
    const obj = this.find(label);
    if (this.holding) throw new ToolFailure('holding', `already holding the ${this.holding.label}`);
    // A miss lands a couple of centimetres off, so the fingers close on air.
    const off = miss ? 0.026 : 0;
    const grasp: Vec3 = [obj.pos[0] + off, obj.pos[1] - off * 0.4, obj.pos[2] + 0.004];
    const above: Vec3 = [grasp[0], grasp[1] - 0.012, grasp[2] + 0.062];
    const { qs, path } = this.solve([
      { pos: above, approach: TILTED },
      { pos: grasp, approach: DOWN },
    ]);
    this.emit({ kind: 'detections', camera: 'wrist', width: CAM_W, height: CAM_H, items: [] });
    this.emit({ kind: 'plan', call_id: callId, path, target: { pos: grasp, approach: DOWN } });
    await this.setGripper(my, 1);
    await this.follow(my, qs, 2.4);
    await this.sleep(my, 150);

    if (miss) {
      await this.setGripper(my, 0);
      this.gripError = 32;
      await this.sleep(my, 700);
      this.emit({ kind: 'log', level: 'warn', text: 'gripper closed to 0 mm: nothing between the fingers' });
      this.gripError = 0;
      this.clearPlan(callId);
      await this.setGripper(my, 0.8);
      await this.follow(my, [qs[Math.max(0, qs.length - 6)]], 0.6);
      throw new ToolFailure('grasp_failed', `missed the ${obj.label}, the grasp came up empty`);
    }

    await this.setGripper(my, (obj.radius * 2 * 1000) / JAW_TRAVEL_MM);
    this.holding = obj;
    const lift = this.solve([{ pos: [grasp[0], grasp[1] - 0.02, grasp[2] + 0.075], approach: TILTED }]);
    this.emit({ kind: 'plan', call_id: callId, path: lift.path, target: null });
    await this.follow(my, lift.qs, 0.9);
    this.sendMarkers();
    this.clearPlan(callId);
    return `holding the ${obj.label} (${Math.round(obj.radius * 2000)} mm)`;
  }

  private async place(my: number, callId: string, label: string): Promise<string> {
    const held = this.holding;
    if (!held) throw new ToolFailure('empty', 'nothing in the gripper to place');
    const target = this.find(label);
    const drop: Vec3 = [target.pos[0], target.pos[1], target.pos[2] + 0.062];
    const above: Vec3 = [drop[0], drop[1] - 0.01, drop[2] + 0.035];
    const { qs, path } = this.solve([
      { pos: above, approach: TILTED },
      { pos: drop, approach: unit([0, 0.12, -0.99]) },
    ]);
    this.emit({ kind: 'plan', call_id: callId, path, target: { pos: drop, approach: DOWN } });
    await this.follow(my, qs, 2.2);
    await this.setGripper(my, 1);
    this.holding = null;
    held.pos = [target.pos[0], target.pos[1], target.pos[2]];
    held.hidden = true;
    this.sendMarkers();
    await this.sleep(my, 250);
    const away = this.solve([{ pos: [above[0], above[1] - 0.02, above[2] + 0.03], approach: TILTED }]);
    await this.follow(my, away.qs, 0.7);
    this.clearPlan(callId);
    return `dropped the ${held.label} in the ${target.label}`;
  }

  /* ---------------------------- demo script ---------------------------- */

  private startDemo(delayMs: number): void {
    window.clearTimeout(this.resumeTimer);
    this.resumeTimer = window.setTimeout(() => {
      // A limp arm cannot run the demo; it resumes when torque comes back.
      if (this.torqueOn) void this.runScript((my) => this.demo(my));
    }, delayMs);
  }

  private async runScript(script: (my: number) => Promise<void>): Promise<void> {
    const my = ++this.epoch;
    try {
      await script(my);
    } catch (err) {
      if (!(err instanceof Cancelled)) console.error('mock script failed', err);
    }
  }

  private async demo(my: number): Promise<void> {
    for (;;) {
      this.objects = makeObjects();
      this.holding = null;
      this.emit({ kind: 'markers', markers: [] });
      this.voice('idle');
      await this.sleep(my, 1800);

      // 1. Voice: what's on the table?
      let turn = await this.hear(my, "yo, what's on the table?");
      let text = await this.say(my, turn, '', 'lemme take a look real quick.', false);
      this.voice('acting');
      await this.runTool(my, 'voice', 'look', {});
      await this.say(my, turn, text, 'aight, I see a red block, a tennis ball and a paper cup.', true);
      this.voice('idle');
      await this.sleep(my, 2600);

      // 2. Voice: pick and place, with one honest miss.
      turn = await this.hear(my, 'put the block in the cup');
      text = await this.say(my, turn, '', 'bet.', false);
      this.voice('acting');
      await this.runTool(my, 'voice', 'pick', { object: 'red block', _miss: true });
      text = await this.say(my, turn, text, 'oh shit, I messed up. lemme try again.', false);
      this.voice('acting');
      await this.runTool(my, 'voice', 'pick', { object: 'red block' });
      text = await this.say(my, turn, text, 'got it this time.', false);
      this.voice('acting');
      await this.runTool(my, 'voice', 'place', { target: 'paper cup' });
      await this.runTool(my, 'voice', 'home', {});
      await this.say(my, turn, text, "boom. block's in the cup.", true);
      this.voice('idle');
      await this.sleep(my, 2600);

      // 3. Claude Code drives the arm through the MCP server.
      await this.runTool(my, 'claude-code', 'gesture', { name: 'wave' });
      await this.sleep(my, 500);
      await this.runTool(my, 'claude-code', 'move_joints', { joints: { '1': 0.6, '6': 0.4 }, relative: true });
      await this.sleep(my, 400);
      await this.runTool(my, 'claude-code', 'home', {});
      await this.sleep(my, 2400);

      // 4. Voice: the dap.
      turn = await this.hear(my, 'dap me up');
      text = await this.say(my, turn, '', 'say less.', false);
      this.voice('acting');
      await this.runTool(my, 'voice', 'gesture', { name: 'dap' });
      await this.say(my, turn, text, 'my guy.', true);
      this.voice('idle');
      await this.sleep(my, 3200);
    }
  }

  /** A reply to something the person typed or said in demo mode. */
  private async reply(my: number, turn: string, heard: string): Promise<void> {
    const t = heard.toLowerCase();
    this.voice('thinking');
    await this.sleep(my, 700);
    const act = async (line: string, tool: string, args: Record<string, unknown>, after: string) => {
      const text = await this.say(my, turn, '', line, false);
      this.voice('acting');
      await this.runTool(my, 'voice', tool, args);
      await this.say(my, turn, text, after, true);
    };
    if (/dap|fist|bump/.test(t)) await act('say less.', 'gesture', { name: 'dap' }, 'my guy.');
    else if (/wave|hello|hi\b|hey/.test(t)) await act('yo!', 'gesture', { name: 'wave' }, "what's good?");
    else if (/look|see|table|what/.test(t)) await act('lemme look.', 'look', {}, "that's what I got.");
    else if (/pick|grab|block/.test(t)) {
      const text = await this.say(my, turn, '', 'on it.', false);
      this.voice('acting');
      await this.runTool(my, 'voice', 'look', {});
      const res = await this.runTool(my, 'voice', 'pick', { object: 'red block' });
      await this.say(my, turn, text, res.ok ? 'got it.' : "couldn't get it, my bad.", true);
    } else if (/home|reset/.test(t)) await act('heading home.', 'home', {}, 'done.');
    else if (/rest|sleep|chill/.test(t)) await act('aight, chillin.', 'rest', {}, 'wake me when you need me.');
    else await act("this is demo mode, so I'm just vibing. but watch this.", 'gesture', { name: 'nod' }, 'hook up the real daemon and I got you.');
    this.voice('idle');
  }

  /** Something the person did takes over from the looping demo. */
  private takeOver(): number {
    window.clearTimeout(this.resumeTimer);
    const my = ++this.epoch;
    // Wake whatever the old script was waiting on so it notices and unwinds now.
    const motion = this.motion;
    this.motion = null;
    motion?.done();
    for (const wake of [...this.wakers]) wake();
    this.qGoal = [...this.q];
    this.gripError = 0;
    return my;
  }

  /* -------------------------------- HTTP ------------------------------- */

  async callTool(tool: string, args: Record<string, unknown>): Promise<ToolResult> {
    if (tool === 'stop') {
      await this.stop();
      return { ok: true, summary: 'stopped' };
    }
    if (this.uiBusy) return { ok: false, error: 'busy', summary: 'busy' };
    const my = this.takeOver();
    this.voice('idle');
    this.clearPlan();
    this.uiBusy = true;
    try {
      // Let the cancelled demo step report its tool_end before ours starts.
      await new Promise((r) => window.setTimeout(r, 0));
      return await this.runTool(my, 'ui', tool, args);
    } catch (err) {
      if (err instanceof Cancelled) return { ok: false, error: 'stopped', summary: 'stopped' };
      throw err;
    } finally {
      this.uiBusy = false;
      if (my === this.epoch) this.startDemo(14000);
    }
  }

  async stop(): Promise<void> {
    this.takeOver();
    this.busy = false;
    this.activity = 'idle';
    this.clearPlan();
    this.voice('idle');
    this.emit({ kind: 'log', level: 'warn', text: 'stopped: holding position' });
    this.startDemo(9000);
  }

  async interrupt(): Promise<void> {
    /* stop() already ended the turn */
  }

  async torque(enable: boolean): Promise<void> {
    if (enable === this.torqueOn) return;
    this.takeOver();
    this.busy = false;
    this.activity = 'idle';
    this.clearPlan();
    this.voice('idle');
    this.torqueOn = enable;
    if (enable) this.startDemo(8000);
  }

  async chat(text: string): Promise<ChatResult> {
    this.takeOver();
    const turn = `turn-${++this.turnSeq}`;
    this.clearPlan();
    void this.runScript(async (my) => {
      // Let the script we just cancelled report its tool_end first.
      await this.sleep(my, 0);
      this.emit({ kind: 'chat', role: 'user', text, turn_id: turn, final: true, via: 'text' });
      await this.reply(my, turn, text);
      this.startDemo(12000);
    });
    return { turn_id: turn };
  }

  async utterance(_audio: Blob): Promise<UtteranceResult> {
    const phrases = ['dap me up', "what's on the table?", 'wave at the crowd', 'grab the red block'];
    const text = phrases[this.utteranceSeq++ % phrases.length];
    this.takeOver();
    const turn = `turn-${++this.turnSeq}`;
    this.clearPlan();
    this.voice('transcribing');
    await new Promise((r) => window.setTimeout(r, 600));
    void this.runScript(async (my) => {
      this.emit({ kind: 'chat', role: 'user', text, turn_id: turn, final: true, via: 'voice' });
      await this.reply(my, turn, text);
      this.startDemo(12000);
    });
    return { text, turn_id: turn };
  }

  camera(_name: string): CameraSource {
    return {
      kind: 'canvas',
      width: CAM_W,
      height: CAM_H,
      draw: (ctx) => {
        const pose = this.pose();
        paintCamera(ctx, this.objects, pose.pos, pose.approach, this.grip, 0);
      },
    };
  }
}
