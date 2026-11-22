import type { ChatResult, ServerMessage, ToolResult, UtteranceResult, VisionFrame } from './types.ts';

export interface TransportHandlers {
  onMessage(msg: ServerMessage): void;
  /** `retryAt` is the epoch ms of the next reconnect attempt, if one is scheduled. */
  onLink(state: 'connecting' | 'open' | 'closed', retryAt: number | null): void;
}

/** Where the camera panel gets its picture from. */
export type CameraSource =
  | { kind: 'mjpeg'; url: string }
  | { kind: 'canvas'; width: number; height: number; draw(ctx: CanvasRenderingContext2D): void };

/**
 * Everything the UI needs from a daemon. `LiveTransport` talks to the real
 * one; `MockTransport` fakes the same contract in the browser.
 */
export interface Transport {
  readonly kind: 'live' | 'mock';
  start(handlers: TransportHandlers): void;
  dispose(): void;
  /** Reconnect now instead of waiting out the backoff. */
  retryNow(): void;

  callTool(tool: string, args: Record<string, unknown>): Promise<ToolResult>;
  stop(): Promise<void>;
  interrupt(): Promise<void>;
  torque(enable: boolean): Promise<void>;
  /** Collaboration mode on or off: softer, slower, backs off when touched. */
  collab(enable: boolean): Promise<void>;
  chat(text: string): Promise<ChatResult>;
  utterance(audio: Blob): Promise<UtteranceResult>;
  camera(name: string): CameraSource;
  /** Live vision layers for a camera's newest frame; null when the daemon has none to give. */
  vision(name: string): Promise<VisionFrame | null>;
}
