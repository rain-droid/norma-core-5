/**
 * The set the arm stands on: a cutting mat with a printed grid and a reach
 * arc, plus the light rig. Everything here is built in the URDF base frame
 * (Z up, metres) and added to the scene's `world` group.
 */
import {
  CanvasTexture,
  Color,
  DirectionalLight,
  ExtrudeGeometry,
  Group,
  HemisphereLight,
  Mesh,
  MeshBasicMaterial,
  MeshStandardMaterial,
  PlaneGeometry,
  PMREMGenerator,
  ShaderMaterial,
  Shape,
  SRGBColorSpace,
  Vector2,
  type Scene,
  type WebGLRenderer,
} from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

/** How far the tool point can get from the base axis. */
export const REACH_RADIUS = 0.43;

/** Mat extents in the base frame: the robot sits near the back edge, facing +y. */
export const MAT = { minX: -0.56, maxX: 0.56, minY: -0.27, maxY: 0.61, corner: 0.03, thickness: 0.008 };

const MAT_COLOR = 0x2f52ea;
const CHALK = 0xe3ebff;

function roundedRect(w: number, h: number, r: number): Shape {
  const s = new Shape();
  const x = -w / 2;
  const y = -h / 2;
  s.moveTo(x + r, y);
  s.lineTo(x + w - r, y);
  s.quadraticCurveTo(x + w, y, x + w, y + r);
  s.lineTo(x + w, y + h - r);
  s.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  s.lineTo(x + r, y + h);
  s.quadraticCurveTo(x, y + h, x, y + h - r);
  s.lineTo(x, y + r);
  s.quadraticCurveTo(x, y, x + r, y);
  return s;
}

function gridMaterial(center: Vector2, half: Vector2): ShaderMaterial {
  return new ShaderMaterial({
    transparent: true,
    depthWrite: false,
    polygonOffset: true,
    polygonOffsetFactor: -2,
    polygonOffsetUnits: -2,
    uniforms: {
      uCenter: { value: center },
      uHalf: { value: half },
      uCorner: { value: MAT.corner },
      uReach: { value: REACH_RADIUS },
      uColor: { value: new Color(CHALK) },
      uPx: { value: 1 },
    },
    vertexShader: /* glsl */ `
      uniform vec2 uCenter;
      varying vec2 vP;
      void main() {
        vP = position.xy + uCenter;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: /* glsl */ `
      uniform vec2 uCenter;
      uniform vec2 uHalf;
      uniform float uCorner;
      uniform float uReach;
      uniform vec3 uColor;
      uniform float uPx;
      varying vec2 vP;

      // Anti-aliased grid lines of roughly constant pixel width.
      float grid(vec2 p, float cell, float px) {
        vec2 c = p / cell;
        vec2 fw = fwidth(c);
        vec2 g = abs(fract(c - 0.5) - 0.5) / fw;
        float line = 1.0 - clamp(min(g.x, g.y) - (px - 1.0) * 0.5, 0.0, 1.0);
        // Fade out before the lines get denser than the pixels.
        return line * (1.0 - smoothstep(0.18, 0.5, max(fw.x, fw.y)));
      }

      float stroke(float d, float px) {
        float fw = fwidth(d);
        return 1.0 - smoothstep(fw * px * 0.5, fw * (px * 0.5 + 1.0), abs(d));
      }

      float roundBox(vec2 p, vec2 b, float r) {
        vec2 q = abs(p) - b + r;
        return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - r;
      }

      void main() {
        vec2 p = vP;
        float inset = 0.022;
        float sd = roundBox(p - uCenter, uHalf - inset, uCorner);
        float inside = 1.0 - smoothstep(-0.001, 0.001, sd);

        float a = 0.0;
        a = max(a, grid(p, 0.05, 1.0 * uPx) * 0.16 * inside);
        a = max(a, grid(p, 0.10, 1.25 * uPx) * 0.3 * inside);

        // Printed border with centimetre ticks, like a cutting mat.
        a = max(a, stroke(sd, 1.5 * uPx) * 0.5);
        vec2 e = abs(p - uCenter) - (uHalf - inset);
        float along = e.x > e.y ? p.y : p.x;
        float depth = -max(e.x, e.y);
        float cm = abs(fract(along / 0.01 - 0.5) - 0.5) * 0.01;
        float major = step(abs(fract(along / 0.05 - 0.5) - 0.5) * 0.05, 0.001);
        float tick = (1.0 - smoothstep(0.0, fwidth(along) * 1.2 * uPx, cm))
          * step(0.0, depth) * step(depth, mix(0.004, 0.009, major));
        a = max(a, tick * 0.42 * inside * (1.0 - smoothstep(0.1, 0.3, fwidth(along / 0.01))));

        // Reach arc: dashed circle around the base axis.
        float r = length(p);
        float ang = atan(p.y, p.x);
        float dash = step(0.38, fract(ang * uReach / 0.028));
        a = max(a, stroke(r - uReach, 1.6 * uPx) * dash * 0.62 * inside);

        // Base axis mark.
        a = max(a, stroke(r - 0.012, 1.3 * uPx) * 0.5);

        if (a < 0.004) discard;
        gl_FragColor = vec4(uColor, a);
      }
    `,
  });
}

function softShadowTexture(): CanvasTexture {
  const c = document.createElement('canvas');
  c.width = c.height = 256;
  const ctx = c.getContext('2d')!;
  const g = ctx.createRadialGradient(128, 128, 20, 128, 128, 128);
  g.addColorStop(0, 'rgba(6, 12, 70, 0.55)');
  g.addColorStop(0.55, 'rgba(6, 12, 70, 0.22)');
  g.addColorStop(1, 'rgba(6, 12, 70, 0)');
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, 256, 256);
  const t = new CanvasTexture(c);
  t.colorSpace = SRGBColorSpace;
  return t;
}

export interface Stage {
  group: Group;
  setTableZ(z: number): void;
  setPixelRatio(px: number): void;
  dispose(): void;
}

export function buildStage(): Stage {
  const group = new Group();
  const w = MAT.maxX - MAT.minX;
  const h = MAT.maxY - MAT.minY;
  const center = new Vector2((MAT.minX + MAT.maxX) / 2, (MAT.minY + MAT.maxY) / 2);
  const bevel = 0.0016;

  // The mat itself: a thin slab with a soft bevelled edge.
  const slabGeometry = new ExtrudeGeometry(roundedRect(w - bevel * 2, h - bevel * 2, MAT.corner), {
    depth: MAT.thickness,
    bevelEnabled: true,
    bevelThickness: bevel,
    bevelSize: bevel,
    bevelSegments: 3,
    curveSegments: 12,
  });
  const slabMaterial = new MeshStandardMaterial({ color: MAT_COLOR, roughness: 0.86, metalness: 0 });
  const slab = new Mesh(slabGeometry, slabMaterial);
  slab.receiveShadow = true;

  const grid = gridMaterial(center, new Vector2(w / 2, h / 2));
  const gridMesh = new Mesh(new PlaneGeometry(w, h), grid);
  gridMesh.renderOrder = 1;

  // A soft pool of shadow under the mat so it sits on something.
  const poolTexture = softShadowTexture();
  const pool = new Mesh(
    new PlaneGeometry(w * 1.5, h * 1.55),
    new MeshBasicMaterial({ map: poolTexture, transparent: true, depthWrite: false, toneMapped: false }),
  );
  pool.renderOrder = -1;

  group.add(pool, slab, gridMesh);

  const setTableZ = (z: number) => {
    slab.position.set(center.x, center.y, z - MAT.thickness - bevel);
    gridMesh.position.set(center.x, center.y, z + 0.0004);
    pool.position.set(center.x, center.y - 0.02, z - MAT.thickness - 0.03);
  };
  setTableZ(0);

  return {
    group,
    setTableZ,
    setPixelRatio: (px) => {
      grid.uniforms.uPx.value = px;
    },
    dispose: () => {
      slabGeometry.dispose();
      slabMaterial.dispose();
      gridMesh.geometry.dispose();
      grid.dispose();
      pool.geometry.dispose();
      (pool.material as MeshBasicMaterial).dispose();
      poolTexture.dispose();
    },
  };
}

/**
 * Light rig, in three's Y-up world. The robot faces -Z and the default
 * camera stands front-right of it, so the key comes from front-left and
 * high (shadows fall back and to the right, where the camera can see them).
 */
export function buildLights(scene: Scene, renderer: WebGLRenderer): { dispose(): void } {
  const pmrem = new PMREMGenerator(renderer);
  const room = new RoomEnvironment();
  const env = pmrem.fromScene(room, 0.04).texture;
  scene.environment = env;
  scene.environmentIntensity = 0.38;

  // Sky is studio white, the "ground" bounce is the blue of the mat.
  const hemi = new HemisphereLight(0xe8eeff, 0x2a45c8, 0.42);

  const key = new DirectionalLight(0xfff6ea, 3.3);
  key.position.set(-0.75, 1.9, -0.95);
  key.target.position.set(0, 0.08, -0.16);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.radius = 3.2;
  key.shadow.bias = -0.0004;
  key.shadow.normalBias = 0.0022;
  const cam = key.shadow.camera;
  cam.near = 0.4;
  cam.far = 4.5;
  cam.left = -0.8;
  cam.right = 0.8;
  cam.top = 0.8;
  cam.bottom = -0.8;
  cam.updateProjectionMatrix();

  const rim = new DirectionalLight(0xc4d3ff, 1.35);
  rim.position.set(1.3, 1.1, 1.4);
  rim.target.position.set(0, 0.15, -0.1);

  scene.add(hemi, key, key.target, rim, rim.target);

  return {
    dispose: () => {
      scene.remove(hemi, key, key.target, rim, rim.target);
      key.shadow.map?.dispose();
      env.dispose();
      pmrem.dispose();
      room.dispose();
    },
  };
}
