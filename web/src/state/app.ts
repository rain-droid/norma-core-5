import type {
  CameraInfo,
  ControlStatus,
  ChatEvent,
  Detection,
  GripperState,
  Hello,
  JointState,
  Mode,
  PlanTarget,
  RobotEvent,
  RobotState,
  ServerMessage,
  Source,
  Vec3,
  VoiceState,
  WorldMarker,
} from '../api/types.ts';
import { createStore, shallowEqual } from './store.ts';

export type LinkState = 'connecting' | 'open' | 'closed';

export interface StatusSlice {
  mode: Mode;
  connected: boolean;
  calibrated: boolean;
  torque: boolean;
  busy: boolean;
  activity: string;
  source: Source | null;
  holding: boolean;
}

export interface Telemetry {
  joints: JointState[];
  gripper: GripperState;
  cameras: CameraInfo[];
}

export interface ChatItem {
  kind: 'chat';
  key: string;
  id: number;
  t: number;
  role: 'user' | 'assistant';
  /** The part of the turn's text this bubble shows (see `start`). */
  text: string;
  /** Offset of `text` inside the turn's cumulative text. */
  start: number;
  turnId: string;
  final: boolean;
  via: 'voice' | 'text';
}

export interface ToolItem {
  kind: 'tool';
  key: string;
  id: number;
  t: number;
  callId: string;
  tool: string;
  args: Record<string, unknown>;
  source: Source | null;
  status: 'running' | 'ok' | 'failed';
  summary: string;
  error: string | null;
  durationMs: number | null;
}

export interface LogItem {
  kind: 'log';
  key: string;
  id: number;
  t: number;
  level: 'info' | 'warn' | 'error';
  text: string;
}

export type TimelineItem = ChatItem | ToolItem | LogItem;

export interface Plan {
  callId: string;
  path: Vec3[];
  target: PlanTarget | null;
}

export interface DetectionSet {
  camera: string;
  width: number;
  height: number;
  items: Detection[];
  /** Local epoch ms when this set arrived; the overlay fades stale boxes. */
  at: number;
}

export type Meta = Omit<Hello, 'events' | 'type'>;

/** Everything that is derived from the event stream. */
interface Derived {
  timeline: TimelineItem[];
  plan: Plan | null;
  markers: WorldMarker[];
  detections: Record<string, DetectionSet>;
  voiceState: VoiceState;
}

export interface AppState extends Derived {
  transport: 'live' | 'mock';
  link: LinkState;
  /** True once any `hello` has arrived on this page load. */
  everOpened: boolean;
  /** Epoch ms of the next reconnect attempt, while the link is closed. */
  retryAt: number | null;
  meta: Meta | null;
  status: StatusSlice | null;
  /** Joint readouts for the panels, republished at ~10 Hz (the 3D view reads `live`). */
  telemetry: Telemetry | null;
  /** Who holds control of the arm and who waits; changes only when they do (not every tick). */
  control: ControlStatus | null;
  /** Most recent failed tool call seen live; the face winces at it. */
  failure: { id: number; at: number } | null;
}

const EMPTY: Derived = {
  timeline: [],
  plan: null,
  markers: [],
  detections: {},
  voiceState: 'idle',
};

export const app = createStore<AppState>({
  ...EMPTY,
  transport: 'live',
  link: 'connecting',
  everOpened: false,
  retryAt: null,
  meta: null,
  status: null,
  telemetry: null,
  control: null,
  failure: null,
});

/** Identity of a control picture, ignoring the ever-growing `for_s`. */
function controlKey(c: ControlStatus | null | undefined): string {
  if (!c) return '';
  const e = (x: { id: string; name: string; kind: string }) => `${x.id}:${x.name}:${x.kind}`;
  return [c.holder ? e(c.holder) : '-', ...c.queue.map(e)].join('|');
}

/* ------------------------------------------------------------------ */
/* High-rate robot state, kept outside React for the render loop.      */
/* ------------------------------------------------------------------ */

export interface Sample {
  /** When this sample should be shown, on the performance.now() timeline. */
  at: number;
  s: RobotState;
}

export const live: { samples: Sample[]; latest: RobotState | null } = {
  samples: [],
  latest: null,
};

const MAX_SAMPLES = 12;
const TELEMETRY_INTERVAL_MS = 100;
const MAX_TIMELINE = 300;

let clockOffset = Number.NaN;
let lastTelemetryAt = 0;
let lastEventId = -1;

const eventListeners = new Set<(ev: RobotEvent) => void>();

/** Subscribe to events as they happen (never called for the reload backlog). */
export function onLiveEvent(listener: (ev: RobotEvent) => void): () => void {
  eventListeners.add(listener);
  return () => eventListeners.delete(listener);
}

export function ingest(msg: ServerMessage): void {
  if (msg.type === 'state') onState(msg);
  else if (msg.type === 'event') onEvent(msg);
  else if (msg.type === 'hello') onHello(msg);
}

/** Forget everything that came from a daemon (used when switching to the mock). */
export function resetSession(): void {
  live.samples = [];
  live.latest = null;
  clockOffset = Number.NaN;
  lastEventId = -1;
  app.set({ ...EMPTY, meta: null, status: null, telemetry: null, control: null, failure: null });
}

function onHello(msg: Hello): void {
  const { events, type: _type, ...meta } = msg;
  const backlog = Array.isArray(events) ? events : [];
  const newest = backlog.length ? backlog[backlog.length - 1].id : -1;

  // Event ids went backwards: the daemon restarted, so our timeline is from a
  // previous life. Start over from its backlog.
  const restarted = newest < lastEventId;
  let derived: Derived = restarted ? EMPTY : pickDerived(app.get());
  if (restarted) lastEventId = -1;

  for (const ev of backlog) {
    if (ev.id <= lastEventId) continue;
    lastEventId = ev.id;
    derived = reduce(derived, ev);
  }
  // A backlog can end mid-turn; whatever the agent was doing then is over.
  if (backlog.length && !app.get().everOpened) derived = { ...derived, voiceState: 'idle' };

  app.set({ ...derived, meta, link: 'open', everOpened: true, retryAt: null });
}

function onState(s: RobotState): void {
  const now = performance.now();
  const server = s.t * 1000;

  // Map the daemon clock onto ours using the least-delayed sample seen, so
  // network jitter does not turn into motion jitter.
  let at = now;
  if (Number.isFinite(server)) {
    const candidate = now - server;
    if (!Number.isFinite(clockOffset) || Math.abs(candidate - clockOffset) > 1500) clockOffset = candidate;
    else clockOffset = Math.min(candidate, clockOffset + 0.05);
    at = server + clockOffset;
  }
  const last = live.samples[live.samples.length - 1];
  if (last && at <= last.at) at = last.at + 1;

  live.samples.push({ at, s });
  if (live.samples.length > MAX_SAMPLES) live.samples.shift();
  live.latest = s;

  const status: StatusSlice = {
    mode: s.mode,
    connected: s.connected,
    calibrated: s.calibrated,
    torque: s.torque,
    busy: s.busy,
    activity: s.activity,
    source: s.source ?? null,
    holding: Boolean(s.gripper?.holding),
  };
  const prev = app.get().status;
  const patch: Partial<AppState> = {};
  if (!prev || !shallowEqual(prev, status)) patch.status = status;
  if (controlKey(s.control) !== controlKey(app.get().control)) patch.control = s.control ?? null;
  if (now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryAt = now;
    patch.telemetry = { joints: s.joints ?? [], gripper: s.gripper, cameras: s.cameras ?? [] };
  }
  app.set(patch);
}

function onEvent(ev: RobotEvent): void {
  if (typeof ev.id === 'number') {
    if (ev.id <= lastEventId) return;
    lastEventId = ev.id;
  }
  const patch: Partial<AppState> = reduce(pickDerived(app.get()), ev);
  if (ev.kind === 'tool_end' && !ev.ok) patch.failure = { id: ev.id, at: Date.now() };
  app.set(patch);
  eventListeners.forEach((l) => l(ev));
}

function pickDerived(s: AppState): Derived {
  return {
    timeline: s.timeline,
    plan: s.plan,
    markers: s.markers,
    detections: s.detections,
    voiceState: s.voiceState,
  };
}

function reduce(d: Derived, ev: RobotEvent): Derived {
  switch (ev.kind) {
    case 'chat':
      return { ...d, timeline: cap(reduceChat(d.timeline, ev)) };

    case 'tool_start': {
      if (d.timeline.some((it) => it.kind === 'tool' && it.callId === ev.call_id)) return d;
      const item: ToolItem = {
        kind: 'tool',
        key: `tool:${ev.call_id}`,
        id: ev.id,
        t: ev.t,
        callId: ev.call_id,
        tool: ev.tool,
        args: ev.args ?? {},
        source: ev.source ?? null,
        status: 'running',
        summary: '',
        error: null,
        durationMs: null,
      };
      return { ...d, timeline: cap([...d.timeline, item]) };
    }

    case 'tool_end': {
      const done = {
        status: ev.ok ? ('ok' as const) : ('failed' as const),
        summary: ev.summary ?? '',
        error: ev.error ?? null,
        durationMs: ev.duration_ms ?? null,
      };
      const idx = d.timeline.findIndex((it) => it.kind === 'tool' && it.callId === ev.call_id);
      if (idx === -1) {
        // We never saw the start (it fell off the backlog): show the result alone.
        const item: ToolItem = {
          kind: 'tool',
          key: `tool:${ev.call_id}`,
          id: ev.id,
          t: ev.t,
          callId: ev.call_id,
          tool: ev.tool,
          args: {},
          source: ev.source ?? null,
          ...done,
        };
        return { ...d, timeline: cap([...d.timeline, item]) };
      }
      const timeline = d.timeline.slice();
      timeline[idx] = { ...(timeline[idx] as ToolItem), ...done };
      return { ...d, timeline };
    }

    case 'log': {
      // info is noise on the timeline, except for who got or gave up control of the arm
      if (ev.level !== 'warn' && ev.level !== 'error' && ev.topic !== 'control') return d;
      const item: LogItem = { kind: 'log', key: `log:${ev.id}`, id: ev.id, t: ev.t, level: ev.level, text: ev.text };
      return { ...d, timeline: cap([...d.timeline, item]) };
    }

    case 'plan':
      if (!ev.path || ev.path.length === 0) return { ...d, plan: null };
      return { ...d, plan: { callId: ev.call_id, path: ev.path, target: ev.target ?? null } };

    case 'markers':
      return { ...d, markers: ev.markers ?? [] };

    case 'detections': {
      const detections = { ...d.detections };
      if (!ev.items || ev.items.length === 0) delete detections[ev.camera];
      else {
        detections[ev.camera] = {
          camera: ev.camera,
          width: ev.width,
          height: ev.height,
          items: ev.items,
          at: Date.now(),
        };
      }
      return { ...d, detections };
    }

    case 'voice_state':
      return { ...d, voiceState: ev.state };

    default:
      return d;
  }
}

/**
 * Assistant text arrives as "the full text so far" for a turn. To keep the
 * timeline chronological, a bubble stops growing once a tool call has started
 * after it; text that arrives later continues in a new bubble below the tool.
 */
function reduceChat(items: TimelineItem[], ev: ChatEvent): TimelineItem[] {
  const text = ev.text ?? '';
  let idx = -1;
  for (let i = items.length - 1; i >= 0; i--) {
    const it = items[i];
    if (it.kind === 'chat' && it.turnId === ev.turn_id && it.role === ev.role) {
      idx = i;
      break;
    }
  }

  const fresh = (start: number, body: string): ChatItem => ({
    kind: 'chat',
    key: `chat:${ev.turn_id}:${ev.role}:${start}`,
    id: ev.id,
    t: ev.t,
    role: ev.role,
    text: body,
    start,
    turnId: ev.turn_id,
    final: Boolean(ev.final),
    via: ev.via,
  });

  if (idx === -1) return [...items, fresh(0, text)];

  const last = items[idx] as ChatItem;
  const next = items.slice();
  let frozen = false;
  if (ev.role === 'assistant') {
    for (let i = idx + 1; i < items.length; i++) {
      if (items[i].kind === 'tool') {
        frozen = true;
        break;
      }
    }
  }

  if (!frozen) {
    next[idx] = { ...last, text: text.slice(last.start), final: Boolean(ev.final) };
    return next;
  }

  const shown = last.start + last.text.length;
  const rest = text.length > shown ? text.slice(shown) : '';
  if (ev.final) {
    // The turn is over: no earlier bubble of it is still "typing".
    for (let i = 0; i < next.length; i++) {
      const it = next[i];
      if (it.kind === 'chat' && it.turnId === ev.turn_id && it.role === ev.role && !it.final) {
        next[i] = { ...it, final: true };
      }
    }
  }
  if (rest.trim() !== '') next.push(fresh(shown, rest));
  return next;
}

function cap(items: TimelineItem[]): TimelineItem[] {
  return items.length > MAX_TIMELINE ? items.slice(items.length - MAX_TIMELINE) : items;
}
