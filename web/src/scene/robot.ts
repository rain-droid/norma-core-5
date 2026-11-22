import {
  Box3,
  Color,
  LoadingManager,
  Mesh,
  MeshStandardMaterial,
  ShaderMaterial,
  Vector3,
  type BufferGeometry,
  type Material,
  type Object3D,
  type Plane,
} from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';
import URDFLoader, { type URDFJoint, type URDFRobot } from 'urdf-loader';
import type { JointMeta } from '../api/types.ts';

const ERROR_COLOR = 0xff2a1f;

/**
 * Colours per arm. Change these to match the real arm's filament.
 * `print` is every printed structural part, `gripper` the gripper body,
 * `jaw` the two fingers, `gear` the gripper's drive gear.
 */
interface Palette {
  print: number;
  servo: number;
  gripper: number;
  jaw: number;
  gear: number;
}

const PALETTES: Record<string, Palette> = {
  elrobot: { print: 0xffbe0a, servo: 0x14161f, gripper: 0xffbe0a, jaw: 0xeef1f8, gear: 0x2b3050 },
  // SO-ARM101: dark STS3215 servos, printed links, orange parallel gripper.
  so101: { print: 0xffbe0a, servo: 0x191b24, gripper: 0xff5b14, jaw: 0xff5b14, gear: 0x23252e },
};

function paletteFor(robotName: string | undefined, urdfUrl: string): Palette {
  if (robotName && PALETTES[robotName]) return PALETTES[robotName];
  const hit = Object.keys(PALETTES).find((k) => urdfUrl.includes(`/${k}/`));
  return PALETTES[hit ?? 'elrobot'];
}

/* ------------------------------------------------------------------ */
/* Gripper mapping                                                     */
/* ------------------------------------------------------------------ */

/** URDF name of the gripper's drive joint on every supported arm; the jaws are `_1` and `_2`. */
export const GRIPPER_JOINT = 'rev_motor_08';

interface GripperDrive {
  /** Drive joint value with the hand fully closed (the zero pose is the open hand). */
  gearClosed: number;
  /** Jaw joints the URDF does not couple to the gear with <mimic>, with their closed value. */
  jaws: { name: string; closed: number }[];
}

const drives = new WeakMap<URDFRobot, GripperDrive | null>();

/** The limit that is further from zero: where the joint ends up when the hand closes. */
function farLimit(j: URDFJoint): number {
  const lo = Number(j.limit?.lower ?? 0);
  const hi = Number(j.limit?.upper ?? 0);
  return Math.abs(lo) > Math.abs(hi) ? lo : hi;
}

function gripperDrive(robot: URDFRobot): GripperDrive | null {
  let d = drives.get(robot);
  if (d !== undefined) return d;
  const gear = robot.joints[GRIPPER_JOINT];
  if (!gear) d = null;
  else {
    const jaws: GripperDrive['jaws'] = [];
    for (const name of [`${GRIPPER_JOINT}_1`, `${GRIPPER_JOINT}_2`]) {
      const j = robot.joints[name];
      // Mimic joints follow the gear on their own (urdf-loader applies multiplier and offset).
      if (j && (j as { type: string }).type !== 'URDFMimicJoint') jaws.push({ name, closed: farLimit(j) });
    }
    d = { gearClosed: farLimit(gear), jaws };
  }
  drives.set(robot, d);
  return d;
}

/**
 * Drives the gripper's URDF joints from `state.gripper.open` (0 = closed,
 * 1 = fully open). Travel comes from the URDF: in both arms' models the zero
 * pose is the open hand, closing turns the gear to its limit, and the two
 * prismatic jaws follow it (through <mimic>, or directly when there is none).
 *
 * If the real arm shows the opposite, use `open` instead of `closed` below.
 */
export function applyGripper(robot: URDFRobot, open: number): void {
  const d = gripperDrive(robot);
  if (!d || !Number.isFinite(open)) return;
  const closed = 1 - Math.min(1, Math.max(0, open));
  robot.setJointValue(GRIPPER_JOINT, closed * d.gearClosed);
  for (const j of d.jaws) robot.setJointValue(j.name, closed * j.closed);
}

/* ------------------------------------------------------------------ */
/* Loading                                                             */
/* ------------------------------------------------------------------ */

export interface RobotModel {
  robot: URDFRobot;
  /** One material per servo id so a faulting servo can turn red. */
  servoMaterials: Map<number, MeshStandardMaterial>;
  servoColor: number;
  /** Every solid material on the arm (for the build-up clipping plane). */
  materials: MeshStandardMaterial[];
  /** Where the first joint's axis stands on the table, base frame (x, y). */
  baseAxis: [number, number];
  /** How far the tool can get from that axis, estimated from the link lengths. */
  reach: number;
}

export interface LoadProgress {
  loaded: number;
  total: number;
}

export interface RobotInfo {
  /** Profile name from the daemon (`so101`, `elrobot`), when it says. */
  name?: string;
  /** Joint names and servo ids from `hello`. */
  joints: JointMeta[];
}

type Part = 'servo' | 'gear' | 'jaw' | 'gripper' | 'dark' | 'print';

/** What a visual is, from its mesh file and the URDF material it was given. */
function classify(file: string, urdfMaterial: string): Part {
  if (/sts?3215/i.test(file)) return 'servo';
  if (file.includes('Gripper_Gear')) return 'gear';
  if (file.includes('Gripper_Jaw')) return 'jaw';
  if (file.includes('Gripper_Base') || urdfMaterial === 'pgripper') return 'gripper';
  // Anything else the URDF paints like a servo (camera, its mount) is black plastic.
  if (urdfMaterial === 'servo') return 'dark';
  return 'print';
}

/** The servo whose body this mesh is: the one that turns the joint hanging off the same link. */
function servoIdFor(mesh: Object3D, file: string, ids: Map<string, number>, gripperId: number | undefined): number {
  const valid = new Set(ids.values());
  let link: Object3D | null = mesh;
  while (link && !(link as { isURDFLink?: boolean }).isURDFLink) link = link.parent;
  for (const child of link?.children ?? []) {
    const j = child as URDFJoint;
    if (!j.isURDFJoint || j.jointType === 'fixed') continue;
    const id = j.name === GRIPPER_JOINT ? gripperId : ids.get(j.name);
    if (id !== undefined) return id;
  }
  // The older arm numbers its servo meshes instead (ST3215_3 is servo 3).
  const n = Number(/ST3215_(\d+)/.exec(file)?.[1]);
  return valid.has(n) ? n : 0;
}

/** Shoulder axis position and an estimate of the reach, read off the zero pose. */
function measure(robot: URDFRobot, armJoints: string[]): { baseAxis: [number, number]; reach: number } {
  robot.updateMatrixWorld(true);
  const inv = robot.matrixWorld.clone().invert();
  const at = (o: Object3D) => o.getWorldPosition(new Vector3()).applyMatrix4(inv);
  const chain = armJoints.map((n) => robot.joints[n]).filter(Boolean);
  if (chain.length < 2) return { baseAxis: [0, 0], reach: 0.4 };
  const pts = chain.map(at);
  // The tool end: the middle of the jaws, or failing that the last link's geometry.
  const tipOwner = robot.joints[`${GRIPPER_JOINT}_1`] ?? chain[chain.length - 1];
  const box = new Box3().setFromObject(tipOwner);
  if (!box.isEmpty()) pts.push(box.getCenter(new Vector3()).applyMatrix4(inv));
  let length = 0;
  for (let i = 2; i < pts.length; i++) length += pts[i].distanceTo(pts[i - 1]);
  // Joint offsets zig-zag a little, so the straight-line reach is a bit less than the sum.
  return { baseAxis: [pts[0].x, pts[0].y], reach: Math.min(0.8, Math.max(0.2, length * 0.9)) };
}

const geometryCache = new Map<string, Promise<BufferGeometry>>();

/** Each STL is fetched, parsed and smoothed once, however many links use it. */
function loadGeometry(stl: STLLoader, path: string): Promise<BufferGeometry> {
  let p = geometryCache.get(path);
  if (!p) {
    p = new Promise<BufferGeometry>((resolve, reject) => {
      stl.load(
        path,
        (raw: BufferGeometry) => {
          // STL has one normal per face; smooth the curved surfaces but keep
          // real edges crisp so the parts catch light like moulded plastic.
          const geometry = toCreasedNormals(raw, (38 * Math.PI) / 180);
          raw.dispose();
          geometry.computeBoundingSphere();
          resolve(geometry);
        },
        undefined,
        (err) => reject(err instanceof Error ? err : new Error(`could not load ${path}`)),
      );
    });
    p.catch(() => geometryCache.delete(path));
    geometryCache.set(path, p);
  }
  return p;
}

/**
 * Loads the URDF and all of its STL meshes once. Resolves when every mesh is
 * in; rejects if the URDF itself or any mesh cannot be fetched.
 */
export function loadRobot(urdfUrl: string, info: RobotInfo, onProgress: (p: LoadProgress) => void): Promise<RobotModel> {
  return new Promise((resolve, reject) => {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    const stl = new STLLoader(manager);
    const palette = paletteFor(info.name, urdfUrl);
    const servoMaterials = new Map<number, MeshStandardMaterial>();
    const shared = {
      // Printed PLA: matte with a soft sheen.
      print: new MeshStandardMaterial({ color: palette.print, roughness: 0.48, metalness: 0.0 }),
      gripper: new MeshStandardMaterial({ color: palette.gripper, roughness: 0.5, metalness: 0.0 }),
      jaw: new MeshStandardMaterial({ color: palette.jaw, roughness: 0.6, metalness: 0.0 }),
      gear: new MeshStandardMaterial({ color: palette.gear, roughness: 0.5, metalness: 0.2 }),
      dark: new MeshStandardMaterial({ color: palette.servo, roughness: 0.55, metalness: 0.1 }),
    };
    const failed: string[] = [];
    let robot: URDFRobot | null = null;
    let settled = false;
    let pending = 0;
    let managerDone = false;

    const finish = () => {
      if (settled || pending > 0 || !managerDone) return;
      settled = true;
      if (!robot) return reject(new Error('The robot model did not load.'));
      if (failed.length) return reject(new Error(`${failed.length} robot mesh files did not load.`));

      const ids = new Map(info.joints.map((j) => [j.name, j.id]));
      const gripperId = ids.get(GRIPPER_JOINT);
      const servoMaterial = (id: number) => {
        let m = servoMaterials.get(id);
        if (!m) {
          m = new MeshStandardMaterial({ color: palette.servo, roughness: 0.42, metalness: 0.15 });
          m.name = `servo-${id}`;
          servoMaterials.set(id, m);
        }
        return m;
      };
      // Paint every visual, including the URDF's own boxes and cylinders.
      robot.traverse((o) => {
        const mesh = o as Mesh;
        if (!mesh.isMesh) return;
        const file = String(mesh.userData.file ?? '');
        const urdfMaterial = mesh.material as Material;
        const part = classify(file, urdfMaterial?.name ?? '');
        mesh.material = part === 'servo' ? servoMaterial(servoIdFor(mesh, file, ids, gripperId)) : shared[part];
        if (!file) urdfMaterial?.dispose();
        mesh.castShadow = true;
        mesh.receiveShadow = true;
      });

      const arm = info.joints.filter((j) => j.name !== GRIPPER_JOINT && j.name in robot!.joints).map((j) => j.name);
      resolve({
        robot,
        servoMaterials,
        servoColor: palette.servo,
        materials: [...Object.values(shared), ...servoMaterials.values()],
        ...measure(robot, arm),
      });
    };

    loader.loadMeshCb = (path, _manager, material, done) => {
      pending++;
      loadGeometry(stl, path)
        .then((geometry) => {
          const mesh = new Mesh(geometry, material);
          mesh.userData.file = path.split('/').pop() ?? '';
          done(mesh);
        })
        .catch((err: unknown) => {
          failed.push(path);
          done(null as unknown as Object3D, err instanceof Error ? err : new Error(`could not load ${path}`));
        })
        .finally(() => {
          pending--;
          finish();
        });
    };

    manager.onProgress = (_url, loaded, total) => onProgress({ loaded, total });
    manager.onLoad = () => {
      managerDone = true;
      finish();
    };

    loader.load(
      urdfUrl,
      (result) => {
        robot = result;
      },
      undefined,
      (err) => {
        if (settled) return;
        settled = true;
        reject(err instanceof Error ? err : new Error(`Could not load ${urdfUrl}`));
      },
    );
  });
}

/* ------------------------------------------------------------------ */
/* Ghost (commanded pose)                                              */
/* ------------------------------------------------------------------ */

/** Rim-lit, see-through material: reads as an outline of where the arm is told to be. */
export function makeGhostMaterial(): ShaderMaterial {
  return new ShaderMaterial({
    transparent: true,
    depthWrite: false,
    uniforms: { uOpacity: { value: 0 }, uColor: { value: new Color(0xffffff) } },
    vertexShader: /* glsl */ `
      varying vec3 vNormal;
      varying vec3 vView;
      void main() {
        vec4 mv = modelViewMatrix * vec4(position, 1.0);
        vNormal = normalize(normalMatrix * normal);
        vView = normalize(-mv.xyz);
        gl_Position = projectionMatrix * mv;
      }
    `,
    fragmentShader: /* glsl */ `
      uniform float uOpacity;
      uniform vec3 uColor;
      varying vec3 vNormal;
      varying vec3 vView;
      void main() {
        float rim = 1.0 - abs(dot(normalize(vNormal), normalize(vView)));
        float a = (0.10 + 0.75 * pow(rim, 2.2)) * uOpacity;
        gl_FragColor = vec4(uColor, a);
      }
    `,
  });
}

/** A second copy of the arm that shares geometry and draws with `material`. */
export function makeGhost(robot: URDFRobot, material: Material): URDFRobot {
  const ghost = robot.clone() as URDFRobot;
  ghost.traverse((o) => {
    const mesh = o as Mesh;
    if (mesh.isMesh) {
      mesh.material = material;
      mesh.castShadow = false;
      mesh.receiveShadow = false;
      mesh.renderOrder = 2;
    }
  });
  return ghost;
}

/** Tints a servo red while its status byte reports a fault. */
export function setServoFault(model: RobotModel, id: number, fault: boolean): void {
  const m = model.servoMaterials.get(id);
  if (!m) return;
  m.color.setHex(fault ? ERROR_COLOR : model.servoColor);
  m.emissive.setHex(fault ? 0x7a0a05 : 0x000000);
}

export function setClipping(model: RobotModel, plane: Plane | null): void {
  for (const m of model.materials) {
    m.clippingPlanes = plane ? [plane] : null;
    m.clipShadows = plane !== null;
    m.needsUpdate = true;
  }
}
