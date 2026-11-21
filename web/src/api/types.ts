/**
 * Wire types for the robot daemon API (docs/API.md, v1).
 * Lengths are metres, angles radians, everything in the URDF base frame
 * (x = right, y = forward, z = up).
 */

export type Vec3 = [number, number, number];
export type Mode = 'sim' | 'real';
export type Source = 'claude-code' | 'voice' | 'ui';
export type VoiceState = 'idle' | 'listening' | 'transcribing' | 'thinking' | 'speaking' | 'acting';

export interface JointMeta {
  id: number;
  name: string;
  lower: number;
  upper: number;
}

export interface ToolMeta {
  name: string;
  description: string;
  input_schema: unknown;
}

export interface Hello {
  type: 'hello';
  version: number;
  mode: Mode;
  urdf: string;
  joints: JointMeta[];
  tools: ToolMeta[];
  gestures: string[];
  cameras: string[];
  table_z: number;
  events: RobotEvent[];
}

export interface JointState {
  id: number;
  name: string;
  q: number;
  q_goal: number;
  lower: number;
  upper: number;
  tick: number;
  current: number;
  load: number;
  temp: number;
  volt: number;
  torque: boolean;
  error: number;
}

export interface GripperState {
  open: number;
  width_mm: number;
  holding: boolean;
}

export interface EndEffector {
  pos: Vec3;
  quat: [number, number, number, number];
  approach: Vec3;
}

export interface CameraInfo {
  name: string;
  width: number;
  height: number;
  fps: number;
  age_ms: number;
}

export interface RobotState {
  type: 'state';
  t: number;
  mode: Mode;
  connected: boolean;
  calibrated: boolean;
  torque: boolean;
  busy: boolean;
  activity: string;
  source: Source | null;
  joints: JointState[];
  gripper: GripperState;
  ee: EndEffector;
  cameras: CameraInfo[];
}

interface EventBase {
  type: 'event';
  id: number;
  t: number;
}

export interface ToolStartEvent extends EventBase {
  kind: 'tool_start';
  call_id: string;
  tool: string;
  args: Record<string, unknown>;
  source: Source | null;
}

export interface ToolEndEvent extends EventBase {
  kind: 'tool_end';
  call_id: string;
  tool: string;
  ok: boolean;
  summary: string;
  error: string | null;
  duration_ms: number;
  source: Source | null;
}

export interface PlanTarget {
  pos: Vec3;
  approach: Vec3;
}

export interface PlanEvent extends EventBase {
  kind: 'plan';
  call_id: string;
  path: Vec3[];
  target: PlanTarget | null;
}

export interface WorldMarker {
  id: string;
  label: string;
  pos: Vec3;
  color?: string;
  radius?: number;
}

export interface MarkersEvent extends EventBase {
  kind: 'markers';
  markers: WorldMarker[];
}

export interface Detection {
  label: string;
  score: number;
  box: [number, number, number, number];
  point?: [number, number];
  polygon?: [number, number][];
}

export interface DetectionsEvent extends EventBase {
  kind: 'detections';
  camera: string;
  width: number;
  height: number;
  items: Detection[];
}

export interface LogEvent extends EventBase {
  kind: 'log';
  level: 'info' | 'warn' | 'error';
  text: string;
}

export interface ChatEvent extends EventBase {
  kind: 'chat';
  role: 'user' | 'assistant';
  text: string;
  turn_id: string;
  final: boolean;
  via: 'voice' | 'text';
}

export interface SpeechEvent extends EventBase {
  kind: 'speech';
  turn_id: string;
  seq: number;
  text: string;
  url: string;
  duration_ms: number;
}

export interface VoiceStateEvent extends EventBase {
  kind: 'voice_state';
  state: VoiceState;
}

export type RobotEvent =
  | ToolStartEvent
  | ToolEndEvent
  | PlanEvent
  | MarkersEvent
  | DetectionsEvent
  | LogEvent
  | ChatEvent
  | SpeechEvent
  | VoiceStateEvent;

export type ServerMessage = Hello | RobotState | RobotEvent;

export interface ToolResult {
  ok: boolean;
  summary?: string;
  error?: string;
  data?: unknown;
  images?: { mime: string; b64: string }[];
}

export interface ChatResult {
  turn_id: string;
}

export interface UtteranceResult {
  text: string;
  turn_id: string | null;
}
