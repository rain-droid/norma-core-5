import {
  Color,
  LoadingManager,
  Mesh,
  MeshStandardMaterial,
  ShaderMaterial,
  type BufferGeometry,
  type Material,
  type Object3D,
  type Plane,
} from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';
import URDFLoader, { type URDFRobot } from 'urdf-loader';

/** Colour of the printed parts. Change this one value to match the real arm's filament. */
export const PRINT_COLOR = 0xffbe0a;
const SERVO_COLOR = 0x14161f;
const FINGER_COLOR = 0xeef1f8;
const ERROR_COLOR = 0xff2a1f;

/* ------------------------------------------------------------------ */
/* Gripper mapping                                                     */
/* ------------------------------------------------------------------ */

const GRIPPER_GEAR_MAX = 2.2028; // rad, rev_motor_08 upper limit
const GRIPPER_JAW_TRAVEL = 0.0255; // m, each prismatic jaw

/**
 * Drives the three URDF joints of servo 8 from `state.gripper.open`
 * (0 = closed, 1 = fully open). In the URDF the zero pose is the open hand:
 * the gear at 0 and both jaws at 0; closing turns the gear and slides the
 * jaws towards each other.
 *
 * If the real arm shows the opposite, flip it here: use `open` instead of
 * `closed` below (or swap the signs of the two jaw lines if only the jaws
 * move the wrong way relative to each other).
 */
export function applyGripper(robot: URDFRobot, open: number): void {
  const closed = 1 - Math.min(1, Math.max(0, open));
  robot.setJointValue('rev_motor_08', closed * GRIPPER_GEAR_MAX); // drive gear
  robot.setJointValue('rev_motor_08_1', -GRIPPER_JAW_TRAVEL * closed); // jaw 02
  robot.setJointValue('rev_motor_08_2', GRIPPER_JAW_TRAVEL * closed); // jaw 01
}

/* ------------------------------------------------------------------ */
/* Loading                                                             */
/* ------------------------------------------------------------------ */

export interface RobotModel {
  robot: URDFRobot;
  /** One material per servo id (1..8) so a faulting servo can turn red. */
  servoMaterials: Map<number, MeshStandardMaterial>;
  /** Every solid material on the arm (for the build-up clipping plane). */
  materials: MeshStandardMaterial[];
}

export interface LoadProgress {
  loaded: number;
  total: number;
}

function partMaterial(file: string, servoMaterials: Map<number, MeshStandardMaterial>, shared: Record<string, MeshStandardMaterial>) {
  const servo = /ST3215_(\d+)/.exec(file);
  if (servo) {
    const id = Number(servo[1]);
    let m = servoMaterials.get(id);
    if (!m) {
      m = new MeshStandardMaterial({ color: SERVO_COLOR, roughness: 0.42, metalness: 0.15 });
      m.name = `servo-${id}`;
      servoMaterials.set(id, m);
    }
    return m;
  }
  if (file.includes('Gripper_Jaw')) return shared.finger;
  if (file.includes('Gripper_Gear')) return shared.gear;
  return shared.print;
}

/**
 * Loads the URDF and all of its STL meshes once. Resolves when every mesh is
 * in; rejects if the URDF itself or any mesh cannot be fetched.
 */
export function loadRobot(urdfUrl: string, onProgress: (p: LoadProgress) => void): Promise<RobotModel> {
  return new Promise((resolve, reject) => {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    const stl = new STLLoader(manager);
    const servoMaterials = new Map<number, MeshStandardMaterial>();
    const shared = {
      // Printed PLA: matte with a soft sheen.
      print: new MeshStandardMaterial({ color: PRINT_COLOR, roughness: 0.48, metalness: 0.0 }),
      finger: new MeshStandardMaterial({ color: FINGER_COLOR, roughness: 0.6, metalness: 0.0 }),
      gear: new MeshStandardMaterial({ color: 0x2b3050, roughness: 0.5, metalness: 0.2 }),
    };
    const failed: string[] = [];
    let robot: URDFRobot | null = null;
    let settled = false;

    loader.loadMeshCb = (path, _manager, _material, done) => {
      stl.load(
        path,
        (raw: BufferGeometry) => {
          // STL has one normal per face; smooth the curved surfaces but keep
          // real edges crisp so the parts catch light like moulded plastic.
          const geometry = toCreasedNormals(raw, (38 * Math.PI) / 180);
          raw.dispose();
          const file = path.split('/').pop() ?? '';
          const mesh = new Mesh(geometry, partMaterial(file, servoMaterials, shared));
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          done(mesh);
        },
        undefined,
        (err) => {
          failed.push(path);
          done(null as unknown as Object3D, err instanceof Error ? err : new Error(`could not load ${path}`));
        },
      );
    };

    manager.onProgress = (_url, loaded, total) => onProgress({ loaded, total });
    manager.onLoad = () => {
      if (settled) return;
      settled = true;
      if (!robot) reject(new Error('The robot model did not load.'));
      else if (failed.length) reject(new Error(`${failed.length} robot mesh files did not load.`));
      else {
        resolve({
          robot,
          servoMaterials,
          materials: [shared.print, shared.finger, shared.gear, ...servoMaterials.values()],
        });
      }
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
  m.color.setHex(fault ? ERROR_COLOR : SERVO_COLOR);
  m.emissive.setHex(fault ? 0x7a0a05 : 0x000000);
}

export function setClipping(model: RobotModel, plane: Plane | null): void {
  for (const m of model.materials) {
    m.clippingPlanes = plane ? [plane] : null;
    m.clipShadows = plane !== null;
    m.needsUpdate = true;
  }
}
