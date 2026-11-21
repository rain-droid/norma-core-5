import { getAudioContext, resumeAudio } from './audio.ts';

const MIME_CANDIDATES = ['audio/webm;codecs=opus', 'audio/webm', 'audio/mp4', 'audio/ogg;codecs=opus'];

export interface Recording {
  blob: Blob;
  durationMs: number;
}

/** The microphone: one shared stream, a level meter and a MediaRecorder. */
export class Mic {
  private stream: MediaStream | null = null;
  private analyser: AnalyserNode | null = null;
  private samples: Float32Array<ArrayBuffer> | null = null;
  private recorder: MediaRecorder | null = null;
  private chunks: Blob[] = [];
  private startedAt = 0;
  private opening: Promise<void> | null = null;

  get ready(): boolean {
    return this.stream !== null;
  }

  get recording(): boolean {
    return this.recorder !== null;
  }

  /** Ask for the mic (once). Throws with a readable message if it is refused. */
  open(): Promise<void> {
    if (this.stream) return Promise.resolve();
    this.opening ??= this.acquire().finally(() => {
      this.opening = null;
    });
    return this.opening;
  }

  private async acquire(): Promise<void> {
    if (!navigator.mediaDevices?.getUserMedia) {
      throw new Error('This browser cannot record audio. Open the page on localhost or over https.');
    }
    let stream: MediaStream;
    try {
      stream = await navigator.mediaDevices.getUserMedia({
        audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: true },
      });
    } catch (err) {
      const name = err instanceof DOMException ? err.name : '';
      if (name === 'NotAllowedError' || name === 'SecurityError') {
        throw new Error("Mic is blocked. Allow it in the browser's site settings, then try again.");
      }
      if (name === 'NotFoundError') throw new Error('No microphone found. Plug one in and try again.');
      throw new Error('Could not open the microphone.');
    }
    const ctx = getAudioContext();
    void resumeAudio();
    const source = ctx.createMediaStreamSource(stream);
    const analyser = ctx.createAnalyser();
    analyser.fftSize = 1024;
    analyser.smoothingTimeConstant = 0;
    source.connect(analyser); // not connected to the speakers
    this.stream = stream;
    this.analyser = analyser;
    this.samples = new Float32Array(new ArrayBuffer(analyser.fftSize * 4));
  }

  /** RMS of the last ~20 ms of input, 0..1. */
  level(): number {
    if (!this.analyser || !this.samples) return 0;
    this.analyser.getFloatTimeDomainData(this.samples);
    let sum = 0;
    for (let i = 0; i < this.samples.length; i++) sum += this.samples[i] * this.samples[i];
    return Math.sqrt(sum / this.samples.length);
  }

  start(): void {
    if (!this.stream || this.recorder) return;
    const mimeType = MIME_CANDIDATES.find((m) => MediaRecorder.isTypeSupported(m));
    const recorder = new MediaRecorder(this.stream, mimeType ? { mimeType } : undefined);
    this.chunks = [];
    recorder.ondataavailable = (e) => {
      if (e.data.size > 0) this.chunks.push(e.data);
    };
    recorder.start();
    this.recorder = recorder;
    this.startedAt = performance.now();
  }

  /** Stop and return what was captured, or null if nothing was being recorded. */
  stop(): Promise<Recording | null> {
    const recorder = this.recorder;
    if (!recorder) return Promise.resolve(null);
    this.recorder = null;
    const durationMs = performance.now() - this.startedAt;
    return new Promise((resolve) => {
      recorder.onstop = () => {
        const blob = new Blob(this.chunks, { type: recorder.mimeType || 'audio/webm' });
        this.chunks = [];
        resolve(blob.size > 0 ? { blob, durationMs } : null);
      };
      try {
        recorder.stop();
      } catch {
        resolve(null);
      }
    });
  }

  /** Stop and throw the audio away. */
  cancel(): void {
    void this.stop();
  }

  close(): void {
    this.cancel();
    this.stream?.getTracks().forEach((t) => t.stop());
    this.stream = null;
    this.analyser = null;
  }
}
