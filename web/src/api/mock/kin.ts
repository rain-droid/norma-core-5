/**
 * Tiny kinematics for the in-browser mock only: forward kinematics straight
 * from the URDF text and a damped-least-squares IK, so the fake arm's tool
 * point really follows the `plan` paths the mock emits. The real daemon does
 * its own kinematics; nothing in the live UI depends on this file.
 */
import { Matrix4, Quaternion, Vector3 } from 'three';
import type { Vec3 } from '../types.ts';

interface ChainJoint {
  name: string;
  origin: Matrix4;
  axis: Vector3 | null;
  lower: number;
  upper: number;
}

export interface Pose {
  pos: Vec3;
  approach: Vec3;
  quat: [number, number, number, number];
}

const TIP_LINK = 'Gripper_Base_v1_1';
/** Tool point: this far in front of the gripper base, between the fingertips. */
const TCP_OFFSET = 0.088;

function nums(text: string | undefined, fallback: number[]): number[] {
  if (!text) return fallback;
  const out = text.trim().split(/\s+/).map(Number);
  return out.length === fallback.length && out.every(Number.isFinite) ? out : fallback;
}

function attr(block: string, tag: string, name: string): string | undefined {
  const m = new RegExp(`<${tag}\\b[^>]*\\b${name}="([^"]*)"`).exec(block);
  return m?.[1];
}

export class Kinematics {
  readonly chain: ChainJoint[] = [];
  /** Arm joints (servo 1..7) in chain order. */
  readonly arm: ChainJoint[] = [];
  private readonly tcpLocal = new Vector3();
  private readonly approachLocal = new Vector3();

  constructor(urdf: string) {
    const joints = new Map<string, { block: string; name: string; type: string; parent: string }>();
    const re = /<joint\b[^>]*name="([^"]+)"[^>]*type="([^"]+)"[^>]*>([\s\S]*?)<\/joint>/g;
    for (let m = re.exec(urdf); m; m = re.exec(urdf)) {
      const child = attr(m[3], 'child', 'link');
      const parent = attr(m[3], 'parent', 'link');
      if (child && parent) joints.set(child, { block: m[3], name: m[1], type: m[2], parent });
    }

    // Walk from the gripper base back to the root.
    let link = TIP_LINK;
    const reversed: ChainJoint[] = [];
    for (let j = joints.get(link); j; j = joints.get(link)) {
      const xyz = nums(attr(j.block, 'origin', 'xyz'), [0, 0, 0]);
      const rpy = nums(attr(j.block, 'origin', 'rpy'), [0, 0, 0]);
      const origin = new Matrix4().compose(
        new Vector3(xyz[0], xyz[1], xyz[2]),
        // URDF rpy is fixed-axis roll, pitch, yaw = Rz(yaw) Ry(pitch) Rx(roll).
        new Quaternion()
          .setFromAxisAngle(new Vector3(0, 0, 1), rpy[2])
          .multiply(new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), rpy[1]))
          .multiply(new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), rpy[0])),
        new Vector3(1, 1, 1),
      );
      const moving = j.type === 'revolute' || j.type === 'continuous';
      const a = nums(attr(j.block, 'axis', 'xyz'), [0, 0, 1]);
      reversed.push({
        name: j.name,
        origin,
        axis: moving ? new Vector3(a[0], a[1], a[2]).normalize() : null,
        lower: Number(attr(j.block, 'limit', 'lower') ?? -Math.PI),
        upper: Number(attr(j.block, 'limit', 'upper') ?? Math.PI),
      });
      link = j.parent;
    }
    this.chain = reversed.reverse();
    this.arm = this.chain.filter((j) => j.axis);
    if (this.arm.length === 0) throw new Error('mock kinematics: no arm joints found in URDF');

    // The gripper points along the wrist-roll axis (negated in this URDF).
    const roll = this.arm[this.arm.length - 1].axis!;
    this.approachLocal.copy(roll).multiplyScalar(-1);
    this.tcpLocal.copy(this.approachLocal).multiplyScalar(TCP_OFFSET);
  }

  get dof(): number {
    return this.arm.length;
  }

  clamp(q: number[]): number[] {
    return q.map((v, i) => Math.min(this.arm[i].upper, Math.max(this.arm[i].lower, v)));
  }

  private readonly m = new Matrix4();
  private readonly r = new Matrix4();
  private readonly v = new Vector3();
  private readonly qt = new Quaternion();

  fk(q: readonly number[]): Pose {
    const m = this.m.identity();
    let i = 0;
    for (const j of this.chain) {
      m.multiply(j.origin);
      if (j.axis) m.multiply(this.r.makeRotationAxis(j.axis, q[i++] ?? 0));
    }
    const p = this.v.copy(this.tcpLocal).applyMatrix4(m);
    const pos: Vec3 = [p.x, p.y, p.z];
    const a = this.v.copy(this.approachLocal).transformDirection(m);
    const approach: Vec3 = [a.x, a.y, a.z];
    this.qt.setFromRotationMatrix(m);
    return { pos, approach, quat: [this.qt.x, this.qt.y, this.qt.z, this.qt.w] };
  }

  /**
   * Damped least squares on position + pointing direction. `dirWeight` is in
   * metres per unit of direction error; 0 solves position only.
   */
  ik(target: Vec3, approach: Vec3 | null, seed: readonly number[], dirWeight = 0.12): number[] {
    const n = this.dof;
    const rows = approach ? 6 : 3;
    let q = this.clamp([...seed]);
    const err = (pose: Pose): number[] => {
      const e = [target[0] - pose.pos[0], target[1] - pose.pos[1], target[2] - pose.pos[2]];
      if (approach) {
        for (let k = 0; k < 3; k++) e.push(dirWeight * (approach[k] - pose.approach[k]));
      }
      return e;
    };

    for (let iter = 0; iter < 80; iter++) {
      const pose = this.fk(q);
      const e = err(pose);
      if (Math.hypot(...e) < 4e-4) break;

      // Numeric Jacobian, rows x n.
      const J: number[][] = Array.from({ length: rows }, () => new Array<number>(n).fill(0));
      const h = 1e-4;
      for (let c = 0; c < n; c++) {
        const qh = [...q];
        qh[c] += h;
        const eh = err(this.fk(qh));
        for (let r = 0; r < rows; r++) J[r][c] = (e[r] - eh[r]) / h;
      }

      // dq = Jt (J Jt + lambda^2 I)^-1 e
      const lambda2 = 0.02 * 0.02;
      const A: number[][] = Array.from({ length: rows }, (_, r) =>
        Array.from({ length: rows }, (_, c) => {
          let s = r === c ? lambda2 : 0;
          for (let k = 0; k < n; k++) s += J[r][k] * J[c][k];
          return s;
        }),
      );
      const y = solve(A, e);
      const dq = new Array<number>(n).fill(0);
      for (let c = 0; c < n; c++) for (let r = 0; r < rows; r++) dq[c] += J[r][c] * y[r];

      const biggest = Math.max(...dq.map(Math.abs));
      const scale = biggest > 0.25 ? 0.25 / biggest : 1;
      q = this.clamp(q.map((v, i) => v + dq[i] * scale));
    }
    return q;
  }
}

/** Gaussian elimination with partial pivoting. Mutates neither input. */
function solve(Ain: number[][], bin: number[]): number[] {
  const n = bin.length;
  const A = Ain.map((row, i) => [...row, bin[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    [A[c], A[p]] = [A[p], A[c]];
    const d = A[c][c] || 1e-12;
    for (let r = c + 1; r < n; r++) {
      const f = A[r][c] / d;
      for (let k = c; k <= n; k++) A[r][k] -= f * A[c][k];
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = A[r][n];
    for (let k = r + 1; k < n; k++) s -= A[r][k] * x[k];
    x[r] = s / (A[r][r] || 1e-12);
  }
  return x;
}
