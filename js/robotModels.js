// Robot 3D models built from primitives. Local frame: +X = robot front (intake side),
// +Z = robot right, Y = up, origin on the floor at the robot center.
//
// Each builder returns { root, anim(state, dt), stored[], modules[], extLen }:
//   stored   - FUEL positions shown in the robot (filled in order); entries flagged ext:true sit
//              in the extending part of the hopper and slide out with it
//   modules  - swerve modules { pivot, wheel } animated by robot.js
//   extLen   - how far (m) the hopper extension slides out at full deploy (0 if none)
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { IN, FUEL } from './constants.js';
import { BUMPER_T, BUMPER_Y0, BUMPER_Y1, frameShape, offsetShape, modulePoints } from './robotConfigs.js';
import { ALLIANCE_COLOR_CSS } from './field.js';
import { build8793 as trains8793, conveyorWheels as conveyorWheels8793 } from './mech/r8793.js';
import { FLOOR as FLOOR_2910 } from './mech/r2910.js';
import * as D4 from './mech/r4414.js';
import { NET as NET_1706 } from './mech/r1706.js';
import * as R1690 from './mech/r1690.js';
import { NET as NET_581 } from './mech/r581.js';
import { fuelTexture, squashable } from './fuelSkin.js';
import { ALUMINUM_COLORS } from './aluminumColors.js';
import { chain8793, chain4414, chain1690 } from './echainModel.js';

export const BUMP_Y0 = BUMPER_Y0, BUMP_Y1 = BUMPER_Y1;
const lerp = THREE.MathUtils.lerp;

// ------------------------------------------------------------------ materials
function std(color, rough = 0.55, metal = 0.2, extra = {}) {
  return new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: metal, ...extra });
}
// Polycarbonate. On the lower presets a see-through tint (alpha blended, which also fades its
// reflections); on High and up (setPolyGlass) clear sheet: transmissive, polycarbonate's index
// of refraction, glossy, reflecting the environment at full strength. glass: its color then
// (what it lets through).
const POLY_MATS = [];
let polyGlass = null; // the environment map while clear sheet is on
function polyMat(alpha, glass) {
  const m = new THREE.MeshPhysicalMaterial({ ...alpha, transparent: true, depthWrite: false, side: THREE.DoubleSide });
  m.userData.poly = { alpha, glass };
  POLY_MATS.push(m);
  if (polyGlass) applyPoly(m, polyGlass);
  return m;
}
function applyPoly(m, env) {
  const { alpha, glass, seen = 0 } = m.userData.poly;
  if (env) {
    // (seen: how much of its own tint shows, the rest let through)
    Object.assign(m, { color: new THREE.Color(glass), opacity: 1, transparent: false, depthWrite: true, transmission: 1 - seen, ior: 1.585, thickness: 0.003, roughness: 0.02,
      specularIntensity: 1, clearcoat: 0, envMap: env, envMapIntensity: 0.3 });
  } else {
    Object.assign(m, { color: new THREE.Color(alpha.color), opacity: alpha.opacity, transparent: true, depthWrite: false, transmission: 0, roughness: alpha.roughness,
      clearcoat: 0, envMap: null, envMapIntensity: 1 });
  }
  m.needsUpdate = true;
}
// another model's polycarbonate (the field's CAD panels) in on the same, as a new material to use
// in its place: its own tint below High; from High up sheet tinted glass (what it lets through)
// with seen of the tint showing, so big panels stay visible
export function registerPoly(src, glass = 0xf4f8fc, seen = 0) {
  const alpha = { color: src.color.getHex(), opacity: src.opacity, roughness: src.roughness ?? 0.1 };
  const m = new THREE.MeshPhysicalMaterial({ ...alpha, transparent: true, depthWrite: false, side: src.side ?? THREE.DoubleSide });
  m.userData.poly = { alpha, glass, seen };
  POLY_MATS.push(m);
  if (polyGlass) applyPoly(m, polyGlass);
  return m;
}
// clear sheet on (with the scene's environment map for its reflections) or off
export function setPolyGlass(env) {
  polyGlass = env || null;
  for (const m of POLY_MATS) applyPoly(m, polyGlass);
}

// Aluminum: brushed (a fine grain drawn in the shader, across each face's longest way in its
// plane: the CAD parts have no texture coordinates), metal, and on the presets with reflections
// reflecting the environment as aluminum does (setMetalEnv). Raw for bare or clear-anodized
// aluminum (silver whatever gray the CAD draws it in), dark for dark-anodized (its own color).
const ALU_MATS = [];
let metalEnv = null;
function applyMetal(m) {
  m.envMap = metalEnv;
  m.envMapIntensity = metalEnv ? m.userData.alu.env : 1;
  m.needsUpdate = true;
}
function brushed(m, env) {
  m.userData.alu = { env };
  m.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vAluP;\nvarying vec3 vAluN;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\n  vAluP = position; vAluN = normal;');
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', `#include <common>
varying vec3 vAluP;
varying vec3 vAluN;
float aluHash(float n) { return fract(sin(n) * 43758.5453); }
float aluNoise(float x) { float i = floor(x), f = fract(x); f = f * f * (3.0 - 2.0 * f); return mix(aluHash(i), aluHash(i + 1.0), f); }
// the grain: lines ~3 mm apart (each its own shade, wandering a little), faded out where they're
// finer than the pixels (so it doesn't shimmer far away)
float aluGrain() {
  vec3 n = abs(normalize(vAluN));
  vec2 uv = (n.y > n.x && n.y > n.z) ? vAluP.xz : (n.x > n.z ? vAluP.zy : vAluP.xy);
  float line = uv.y * 320.0 + aluNoise(uv.x * 5.0) * 4.0;
  float g = 0.65 * aluNoise(line) + 0.35 * aluNoise(line * 2.7 + 17.0);
  g = mix(g, aluNoise(uv.x * 30.0 + floor(line) * 7.13), 0.25);
  float fw = fwidth(line);
  return mix(0.5, g, clamp(1.5 - fw, 0.0, 1.0));
}`)
      .replace('#include <color_fragment>', '#include <color_fragment>\n  float aluG = aluGrain();\n  diffuseColor.rgb *= 0.93 + 0.14 * aluG;')
      .replace('#include <roughnessmap_fragment>', '#include <roughnessmap_fragment>\n  roughnessFactor = clamp(roughnessFactor + (aluG - 0.5) * 0.2, 0.05, 1.0);');
  };
  m.customProgramCacheKey = () => 'alu';
  ALU_MATS.push(m);
  applyMetal(m);
  return m;
}
// powder-coated aluminum (the field's HUB frames): its paint's color under a glossy coat, no
// grain, reflecting the environment through the coat on the presets with reflections
export function powderCoat(color) {
  const m = new THREE.MeshPhysicalMaterial({ color, roughness: 0.5, metalness: 0, clearcoat: 0.6, clearcoatRoughness: 0.28 });
  m.userData.alu = { env: 0.45 };
  ALU_MATS.push(m);
  applyMetal(m);
  return m;
}
// the environment to reflect (High and up), or none
export function setMetalEnv(env) {
  metalEnv = env || null;
  for (const m of ALU_MATS) applyMetal(m);
}
const ALU_CACHE = new Map();
function aluMat(hex, finish) {
  const k = finish === 'raw' ? 'raw' : finish + hex;
  if (!ALU_CACHE.has(k)) {
    ALU_CACHE.set(k, finish === 'raw'
      ? brushed(new THREE.MeshStandardMaterial({ color: 0xc3c7cc, roughness: 0.36, metalness: 0.9 }), 0.6)
      : brushed(new THREE.MeshStandardMaterial({ color: hex, roughness: 0.42, metalness: 0.7 }), 0.35));
  }
  return ALU_CACHE.get(k);
}
// An aluminum part's finish by its CAD color, as tools/aluminum-colors.py judges a color: colored
// (chroma 0.15 and up: anodized or powder-coated, like 8793's orange tubes) and near-black (a black
// coat, like 8793's rollers) are left as they are (null); light grays and the slate 67, 72, 77 (a
// CAD library's aluminum) are raw; other grays dark-anodized.
function aluFinish(c) {
  const r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
  const chroma = (Math.max(r, g, b) - Math.min(r, g, b)) / 255, light = Math.max(r, g, b) / 255;
  if (chroma >= 0.15 || light < 0.1) return null;
  return light >= 0.45 || (r === 67 && g === 72 && b === 77) ? 'raw' : 'dark';
}
// A CAD part's material as drawn. tools/extract-parts.mjs names a material by the team's bill of
// materials when it knows the parts' materials ("alu": aluminum parts, "not-alu": parts of another
// material, so a 3D print or cable carrier in the plates' color stays plastic; a recipe's
// "bomKinds" names some of those finer, drawn as that material in the part's color: see
// KIND_LOOK); the rest (parts
// the BOM doesn't place, and GLBs made without it) go by color: js/aluminumColors.js lists which
// of a team's CAD colors are mostly aluminum.
function cadMaterial(team, m) {
  if (KIND_LOOK[m.name] && m.color) return kindMat(m.name, m.color.getHex());
  if (m.name === 'not-alu') return m;
  if (m.name === 'alu') { const f = m.color && aluFinish(m.color.getHex()); return f ? aluMat(m.color.getHex(), f) : m; }
  return aluminumFor(team, m) ?? m;
}
// which of a team's CAD colors are aluminum (js/aluminumColors.js), for a CAD part's material
function aluminumFor(team, m) {
  const list = ALUMINUM_COLORS[team];
  if (!list || !m.color) return null;
  const c = m.color.getHex(), r = (c >> 16) & 255, g = (c >> 8) & 255, b = c & 255;
  const hit = list.find((a) => Math.abs(a.rgb[0] - r) <= 2 && Math.abs(a.rgb[1] - g) <= 2 && Math.abs(a.rgb[2] - b) <= 2);
  return hit && hit.finish !== 'colored' ? aluMat(c, hit.finish) : null;
}
// The materials a recipe's "bomKinds" names by the team's bill of materials (604's so far), each
// drawn in its part's CAD color (as the recipe repaints it): coroplast (corrugated polypropylene
// sheet, satin; its flutes are in the CAD), plywood (birch: matte, with a grain drawn in the
// shader), print (PETG and TPU prints: plastic), rubber (belts, silicone tubing, wheels) and steel
// (gears, sprockets, chain; also brass and bronze).
const KIND_LOOK = {
  coroplast: { roughness: 0.62, metalness: 0 },
  plywood: { roughness: 0.82, metalness: 0, grain: true },
  print: { roughness: 0.5, metalness: 0 },
  rubber: { roughness: 0.88, metalness: 0 },
  steel: { roughness: 0.42, metalness: 0.75 },
};
const KIND_CACHE = new Map();
function kindMat(kind, hex) {
  const k = kind + hex;
  if (!KIND_CACHE.has(k)) {
    const m = new THREE.MeshStandardMaterial({ color: hex, roughness: KIND_LOOK[kind].roughness, metalness: KIND_LOOK[kind].metalness });
    if (KIND_LOOK[kind].grain) woodGrain(m);
    KIND_CACHE.set(k, m);
  }
  return KIND_CACHE.get(k);
}
// birch plywood's face grain: long wavering bands (a few mm apart) along one way of each face, in
// the part's own meters (the GLB's quantized positions times the mesh's scale), lighter and darker
// by a few percent, faded out where finer than the pixels
function woodGrain(m) {
  m.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', '#include <common>\nvarying vec3 vWoodP;\nvarying vec3 vWoodN;')
      .replace('#include <begin_vertex>', '#include <begin_vertex>\n  vWoodP = position * vec3(length(modelMatrix[0].xyz), length(modelMatrix[1].xyz), length(modelMatrix[2].xyz)); vWoodN = normal;');
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', `#include <common>
varying vec3 vWoodP;
varying vec3 vWoodN;
float woodHash(float n) { return fract(sin(n) * 43758.5453); }
float woodNoise(float x) { float i = floor(x), f = fract(x); f = f * f * (3.0 - 2.0 * f); return mix(woodHash(i), woodHash(i + 1.0), f); }
float woodGrain() {
  vec3 n = abs(normalize(vWoodN));
  vec2 uv = (n.y > n.x && n.y > n.z) ? vWoodP.xz : (n.x > n.z ? vWoodP.zy : vWoodP.xy);
  float line = uv.y * 260.0 + woodNoise(uv.x * 9.0) * 6.0 + woodNoise(uv.x * 37.0) * 1.5;
  float g = smoothstep(0.35, 0.9, woodNoise(line)) * 0.7 + woodNoise(line * 0.23 + 5.0) * 0.3;
  return mix(0.4, g, clamp(1.5 - fwidth(line), 0.0, 1.0));
}`)
      .replace('#include <color_fragment>', '#include <color_fragment>\n  diffuseColor.rgb *= 1.06 - 0.2 * woodGrain();');
  };
  m.customProgramCacheKey = () => 'wood';
}

const M = {
  alu: brushed(std(0xb4b9c1, 0.32, 0.85), 0.6),
  aluDark: brushed(std(0x4a4f58, 0.4, 0.75), 0.35),
  anodBlack: std(0x1d1f24, 0.45, 0.55),
  black: std(0x131417, 0.7, 0.15),
  tread: std(0x0e0e10, 0.95, 0.0),
  hubGrey: std(0x6b7078, 0.4, 0.7),
  kraken: std(0x22252b, 0.35, 0.6),
  krakenRing: std(0xc9ccd2, 0.3, 0.9),
  // polycarbonate: lightly tinted so you can tell it's there (clear, see polyMat, on High and up)
  poly: polyMat({ color: 0xa9c4dd, opacity: 0.26, roughness: 0.05 }, 0xf4f8fc),
  polyTint: polyMat({ color: 0x8aa3bb, opacity: 0.32, roughness: 0.08 }, 0xe2ebf3),
  compliant: std(0x2b2b2e, 0.9, 0.0),
  green: std(0x8ccf6a, 0.6, 0.1), // 2910's drum wheels
  orangeWheel: std(0xe0782a, 0.7, 0.05),
  blueWheel: std(0x2d6fd6, 0.7, 0.05),
  copper: std(0xb87333, 0.35, 0.9),
  battery: std(0x1a1a1a, 0.8, 0.05),
  red: std(0xc62828, 0.5, 0.1),
  rioGrey: std(0x9ea3aa, 0.4, 0.5),
  radioWhite: std(0xe8e8e8, 0.6, 0.0),
  lens: std(0x1e2b3c, 0.1, 0.8, { emissive: 0x0b1a33 }),
  led: std(0x39ff6a, 0.3, 0, { emissive: 0x39ff6a, emissiveIntensity: 1.1 }),
  polyBelt: std(0x3a3d42, 0.8, 0.0),
};

function mesh(geo, mat, parent, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, mat);
  m.position.set(x, y, z);
  m.castShadow = true;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}
const bx = (w, h, d, mat, parent, x, y, z) => mesh(new THREE.BoxGeometry(w, h, d), mat, parent, x, y, z);
const rbx = (w, h, d, r, mat, parent, x, y, z) => mesh(new RoundedBoxGeometry(w, h, d, 2, r), mat, parent, x, y, z);
// cylinder with its axis along local Z
function cylZ(r, len, mat, parent, x, y, z, segs = 20) {
  const m = mesh(new THREE.CylinderGeometry(r, r, len, segs), mat, parent, x, y, z);
  m.rotation.x = Math.PI / 2;
  return m;
}
function cylX(r, len, mat, parent, x, y, z, segs = 20) {
  const m = mesh(new THREE.CylinderGeometry(r, r, len, segs), mat, parent, x, y, z);
  m.rotation.z = Math.PI / 2;
  return m;
}
const cylY = (r, len, mat, parent, x, y, z, segs = 20) => mesh(new THREE.CylinderGeometry(r, r, len, segs), mat, parent, x, y, z);

// ------------------------------------------------------------------ parts library
// 2x1 aluminum tube between two points in the XZ plane at height y (runs along X or Z)
function tube(parent, x0, z0, x1, z1, y, mat = M.alu, w = 1 * IN, h = 2 * IN) {
  const len = Math.hypot(x1 - x0, z1 - z0);
  const m = rbx(len, h, w, 0.003, mat, parent, (x0 + x1) / 2, y, (z0 + z1) / 2);
  m.rotation.y = -Math.atan2(z1 - z0, x1 - x0);
  return m;
}

// flat plate with round lightening pockets, lying in the XY plane (thickness along Z)
function pocketPlate(parent, w, h, t, mat, holes = [], x = 0, y = 0, z = 0) {
  const s = new THREE.Shape();
  const r = Math.min(0.012, w / 6, h / 6);
  s.moveTo(-w / 2 + r, -h / 2);
  s.lineTo(w / 2 - r, -h / 2); s.quadraticCurveTo(w / 2, -h / 2, w / 2, -h / 2 + r);
  s.lineTo(w / 2, h / 2 - r); s.quadraticCurveTo(w / 2, h / 2, w / 2 - r, h / 2);
  s.lineTo(-w / 2 + r, h / 2); s.quadraticCurveTo(-w / 2, h / 2, -w / 2, h / 2 - r);
  s.lineTo(-w / 2, -h / 2 + r); s.quadraticCurveTo(-w / 2, -h / 2, -w / 2 + r, -h / 2);
  for (const [hx, hy, hr] of holes) { const p = new THREE.Path(); p.absarc(hx, hy, hr, 0, Math.PI * 2, true); s.holes.push(p); }
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false, curveSegments: 12 });
  g.translate(0, 0, -t / 2);
  return mesh(g, mat, parent, x, y, z);
}

// Kraken X60 (or X44 when small) motor, axis along Y by default
function kraken(parent, x, y, z, small = false, axis = 'y') {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  parent.add(g);
  const r = small ? 0.026 : 0.03, len = small ? 0.055 : 0.07;
  cylY(r, len, M.kraken, g, 0, 0, 0, 18);
  cylY(r * 1.02, 0.008, M.krakenRing, g, 0, len / 2 - 0.004, 0, 18);
  bx(0.014, len * 0.8, 0.02, M.kraken, g, r, 0, 0); // controller bump
  if (axis === 'x') g.rotation.z = Math.PI / 2;
  if (axis === 'z') g.rotation.x = Math.PI / 2;
  return g;
}

// MK5n-style swerve module: square top plate, two motors on top, 4in wheel underneath
function swerveModule(parent, x, z) {
  const g = new THREE.Group();
  g.position.set(x, 0, z);
  parent.add(g);
  rbx(0.118, 0.008, 0.118, 0.004, M.anodBlack, g, 0, 0.158, 0);
  cylY(0.056, 0.035, M.aluDark, g, 0, 0.135, 0, 28); // steering bearing housing
  kraken(g, -0.028, 0.2, 0.028);
  kraken(g, 0.028, 0.2, -0.028);
  bx(0.028, 0.012, 0.028, M.red, g, 0.035, 0.168, 0.035); // CANcoder
  return moduleWheel(g);
}

// a module's steering pivot (the fork, forkH tall round forkY) and its 4in wheel
function moduleWheel(g, forkY = 0.09, forkH = 0.075) {
  const pivot = new THREE.Group();
  g.add(pivot);
  // fork
  for (const s of [-1, 1]) bx(0.05, forkH, 0.008, M.aluDark, pivot, 0, forkY, s * 0.03);
  const wheel = new THREE.Group();
  wheel.position.set(0, 0.0508, 0);
  pivot.add(wheel);
  // robot.js spins the wheel with wheel.rotation.y, so the axle is this group's local Y
  const spin = new THREE.Group();
  spin.rotation.x = Math.PI / 2;
  const tire = new THREE.Mesh(new THREE.CylinderGeometry(0.0508, 0.0508, 0.038, 24), M.tread);
  tire.castShadow = true;
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.04, 16), M.hubGrey);
  spin.add(tire, hub);
  // tread nubs so the spin is visible
  for (let i = 0; i < 6; i++) {
    const n = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.04, 0.006), M.hubGrey);
    const a = (i * Math.PI) / 3;
    n.position.set(Math.cos(a) * 0.02, 0, Math.sin(a) * 0.02);
    spin.add(n);
  }
  wheel.add(spin);
  return { pivot, wheel: spin };
}

// the convex hull of [u, v] points (counterclockwise), and a polygon cut back to u + v <= k
function hull2(pts) {
  const p = [...pts].sort((a, b) => a[0] - b[0] || a[1] - b[1]), cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const half = (q) => { const h = []; for (const x of q) { while (h.length > 1 && cr(h[h.length - 2], h[h.length - 1], x) <= 0) h.pop(); h.push(x); } h.pop(); return h; };
  return [...half(p), ...half([...p].reverse())];
}
function cutDiag(pts, k) {
  const out = [], f = (q) => q[0] + q[1] - k;
  pts.forEach((a, i) => {
    const b = pts[(i + 1) % pts.length], fa = f(a), fb = f(b);
    if (fa <= 0) out.push(a);
    if (fa * fb < 0) { const t = fa / (fa - fb); out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]); }
  });
  return out;
}

// An inverted swerve module, made like WCP's X2i (4414's custom ones): its plate over the wheel,
// out to the frame's edges (ex, ez from the steering axis) and bolted on the frame there, the drive
// and steering motors (Kraken X60s) hanging under it beside the wheel, inboard (their bottoms 1.6in
// off the carpet), the gears between it and a top plate over them. Nothing of it is over 0.15 m, so
// none of it comes up through a hopper's floor. (sx, sz): its corner; at a chamfered corner (cut: the
// chamfer's line, u + v = cut, u and v out toward the corner along x and z) its plates are cut along
// the diagonal, so the corner's bumper runs straight across it.
function invertedModule(parent, x, z, sx, sz, ex, ez, cut = Infinity) {
  const g = new THREE.Group();
  g.position.set(x, 0, z);
  parent.add(g);
  const at = ([u, v]) => [sx * u, sz * v];
  // the drive motor inboard along x, the steering motor inboard along z, clear of the wheel's swing
  const mot = [[-0.088, 0], [0, -0.088]], rm = 0.04;
  let pts = [[ex - 0.002, ez - 0.002], [ex - 0.002, -0.05], [-0.05, ez - 0.002]];
  for (const [mu, mv] of mot) for (let i = 0; i < 20; i++) { const a = (i / 20) * Math.PI * 2; pts.push([mu + rm * Math.cos(a), mv + rm * Math.sin(a)]); }
  pts = hull2(pts);
  if (cut < Infinity) pts = cutDiag(pts, cut - 0.002);
  const outline = pts.map(at);
  // pockets: a triangle between the motors, and a round one in the top plate over the gears
  const tri = [[-0.05, -0.012], [-0.012, -0.05], [-0.06, -0.06]].map(at);
  const pocket = arcPts(...at([-0.03, -0.03]), 0.012, 0, Math.PI * 2, 16);
  flatPart(outline, [tri], 0.006, M.anodBlack, g, 0.118); // the plate, 0.112..0.118
  flatPart(outline, [tri, pocket], 0.004, M.anodBlack, g, 0.146); // the top plate
  // standoffs between them, round the outline
  for (let i = 0; i < pts.length; i += Math.max(1, Math.floor(pts.length / 6))) {
    const [u, v] = pts[i], l = Math.hypot(u, v) || 1, [px, pz] = at([u - (0.008 * u) / l, v - (0.008 * v) / l]);
    cylY(0.004, 0.024, M.alu, g, px, 0.13, pz, 8);
  }
  // the gears: the steering's big one round the axis, the motors' pinions
  spurGear(0.05, 60, 0.008, M.aluDark, g, 6).position.y = 0.13;
  for (const m of mot) { const [px, pz] = at(m); spurGear(0.022, 18, 0.008, M.aluDark, g).position.set(px, 0.13, pz); }
  bx(0.026, 0.008, 0.026, M.red, g, 0, 0.15, 0); // CANcoder, on the top plate over the axis
  cylY(0.048, 0.008, M.aluDark, g, 0, 0.108, 0, 28); // the steering bearing, under the plate
  // the motors, upside down under the plate (output up into the gears)
  for (const m of mot) { const [px, pz] = at(m); kraken(g, px, 0.112 - 0.036, pz); }
  // the fork from the bearing down to the axle, the wheel under the plate
  return moduleWheel(g, 0.078, 0.05);
}

function bumperTexture(number, alliance) {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 128;
  const g = c.getContext('2d');
  g.clearRect(0, 0, c.width, c.height);
  g.fillStyle = '#ffffff';
  g.font = '900 104px "Arial Black", Arial, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(String(number), 256, 68);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// One continuous, round-edged bumper around the whole frame perimeter (chamfers included),
// with the team number on every straight side
function addBumpers(root, cfg, alliance, chamferBack = 0) {
  const L = cfg.frame.length, W = cfg.frame.width, T = BUMPER_T;
  const h = BUMP_Y1 - BUMP_Y0;
  const bevel = Math.min(0.02, h / 2 - 0.004);
  const outline = frameShape(cfg);
  // shape space: (u, v) = (x, -z); after rotateX(-90deg) shape v -> world -z, depth -> +y
  const P = outline.map(([x, z]) => new THREE.Vector2(x, -z));
  let area = 0;
  for (let i = 0; i < P.length; i++) { const a = P[i], b = P[(i + 1) % P.length]; area += a.x * b.y - b.x * a.y; }
  if (area < 0) P.reverse();
  const n = P.length;
  const rOut = T - bevel; // the bevel adds the rest of the thickness
  const shape = new THREE.Shape();
  for (let i = 0; i < n; i++) {
    const p = P[i], a = P[(i - 1 + n) % n], b = P[(i + 1) % n];
    const d0 = new THREE.Vector2().subVectors(p, a).normalize(), d1 = new THREE.Vector2().subVectors(b, p).normalize();
    let a0 = Math.atan2(-d0.x, d0.y), a1 = Math.atan2(-d1.x, d1.y); // outward normals (CCW polygon)
    if (a1 < a0) a1 += Math.PI * 2;
    const sx = p.x + rOut * Math.cos(a0), sy = p.y + rOut * Math.sin(a0);
    if (i === 0) shape.moveTo(sx, sy); else shape.lineTo(sx, sy);
    shape.absarc(p.x, p.y, rOut, a0, a1, false);
  }
  shape.closePath();
  const hole = new THREE.Path();
  const inset = 0.004;
  const HP = P.map((p) => p.clone().multiplyScalar(1 - inset / Math.max(L, W)));
  hole.moveTo(HP[0].x, HP[0].y);
  for (let i = HP.length - 1; i >= 1; i--) hole.lineTo(HP[i].x, HP[i].y);
  hole.closePath();
  shape.holes.push(hole);
  const geo = new THREE.ExtrudeGeometry(shape, {
    depth: h - 2 * bevel, bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 4, curveSegments: 10,
  });
  geo.rotateX(-Math.PI / 2);
  geo.translate(0, BUMP_Y0 + bevel, 0);
  geo.computeVertexNormals();
  const fabric = std(ALLIANCE_COLOR_CSS[alliance], 0.88, 0.0);
  const ring = mesh(geo, fabric, root, 0, 0, 0);
  ring.name = 'bumper';
  // team numbers on each straight side
  const tex = bumperTexture(cfg.team, alliance);
  const numMat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2 });
  const yc = (BUMP_Y0 + BUMP_Y1) / 2;
  const ch = cfg.frame.chamfer || 0, rd = cfg.frame.round;
  const sides = rd ? [
    // round: the flat front, and short labels on the curve at the back and the widest points
    { x: L / 2 + T + 0.002, z: 0, ry: Math.PI / 2, len: rd.front },
    { x: -L / 2 - T - 0.002, z: 0, ry: -Math.PI / 2, len: 0.3 },
    { x: rd.cx, z: W / 2 + T + 0.002, ry: 0, len: 0.3 },
    { x: rd.cx, z: -W / 2 - T - 0.002, ry: Math.PI, len: 0.3 },
  ] : [
    { x: L / 2 + T + 0.002, z: 0, ry: Math.PI / 2, len: W },
    { x: -L / 2 - T - 0.002, z: 0, ry: -Math.PI / 2, len: W - 2 * ch },
    { x: ch / 2, z: W / 2 + T + 0.002, ry: 0, len: L - ch }, // (the straight run, L / 2 back to the cut)
    { x: ch / 2, z: -W / 2 - T - 0.002, ry: Math.PI, len: L - ch },
  ];
  for (const sd of sides) {
    const w = Math.min(0.42, sd.len * 0.8), hh = w / 4;
    const pl = new THREE.Mesh(new THREE.PlaneGeometry(w, Math.min(hh, h * 0.85)), numMat);
    pl.position.set(sd.x, yc, sd.z);
    pl.rotation.y = sd.ry;
    root.add(pl);
  }
  return ring;
}

// frame tubes, bellypan and swerve modules (frame: false when the team's CAD brings its own frame)
// (inverted modules, cfg.drive.inverted: made like WCP's X2i, the motors under their plates; the
// bellypan and the cross tubes clear of them, and at a chamfered corner the module's plate is the
// corner: no tube along the cut)
function addDrivebase(root, cfg, chamferBack = 0, frame = true) {
  const L = cfg.frame.length, W = cfg.frame.width, ch = cfg.frame.chamfer || 0, inv = !!cfg.drive?.inverted;
  const pts = modulePoints(cfg);
  const modules = pts.map(([x, z]) => {
    if (!inv) return swerveModule(root, x, z);
    const sx = Math.sign(x), sz = Math.sign(z), ex = L / 2 - Math.abs(x), ez = W / 2 - Math.abs(z);
    return invertedModule(root, x, z, sx, sz, ex, ez, sx < 0 && ch ? ex + ez - ch : Infinity);
  });
  if (!frame) return modules;
  const frameMat = std(cfg.colors.frame, 0.4, 0.65);
  const y = 0.085;
  const outline = frameShape(cfg);
  for (let i = 0; i < outline.length; i++) {
    const [x0, z0] = outline[i], [x1, z1] = outline[(i + 1) % outline.length];
    if (inv && x0 !== x1 && z0 !== z1) continue;
    // pull each tube 0.5in inside the perimeter
    const cx = (x0 + x1) / 2, cz = (z0 + z1) / 2, k = 0.0127 / Math.max(1e-6, Math.hypot(cx, cz));
    tube(root, x0 - x0 * k * 0.9, z0 - z0 * k * 0.9, x1 - x1 * k * 0.9, z1 - z1 * k * 0.9, y, frameMat);
  }
  // bellypan (the frame's own shape, or notched round inverted modules) and cross tubes
  // (inverted: along the notches' inner edges, clear of the motors)
  const notch = 0.135, at = (sx, sz) => pts.find(([x, z]) => Math.sign(x) === sx && Math.sign(z) === sz).map((v) => Math.abs(v) - notch);
  let panPts = offsetShape(outline, -0.02);
  if (inv) {
    const a = L / 2 - 0.02, b = W / 2 - 0.02;
    panPts = [[1, 1], [-1, 1], [-1, -1], [1, -1]].flatMap(([sx, sz]) => {
      const [nx, nz] = at(sx, sz), p = [[sx * a, sz * nz], [sx * nx, sz * nz], [sx * nx, sz * b]];
      return sx * sz > 0 ? p : p.reverse();
    });
  }
  const pan = new THREE.Shape(panPts.map(([x, z]) => new THREE.Vector2(x, -z)));
  const panGeo = new THREE.ExtrudeGeometry(pan, { depth: 0.004, bevelEnabled: false, curveSegments: 4 });
  panGeo.rotateX(-Math.PI / 2);
  mesh(panGeo, M.aluDark, root, 0, 0.055, 0);
  const inside = (x) => { // the frame's half-width at x
    let hw = 0;
    for (let i = 0; i < outline.length; i++) {
      const [x0, z0] = outline[i], [x1, z1] = outline[(i + 1) % outline.length];
      if ((x0 - x) * (x1 - x) <= 0 && x0 !== x1) hw = Math.max(hw, Math.abs(z0 + ((z1 - z0) * (x - x0)) / (x1 - x0)));
    }
    return hw || W / 2;
  };
  const xcs = inv ? [-1, 1].map((sx) => sx * (at(sx, 1)[0] - 0.0127)) : [-L / 2 + 0.16, L / 2 - 0.16];
  for (const xc of xcs) { const hw = inside(xc) - 0.03; tube(root, xc, -hw, xc, hw, y, frameMat); }
  return modules;
}

// battery, roboRIO, PDH, radio on the bellypan
function addElectronics(parent, x, z, rot = 0) {
  const g = new THREE.Group();
  g.position.set(x, 0.06, z);
  g.rotation.y = rot;
  parent.add(g);
  rbx(0.18, 0.17, 0.077, 0.006, M.battery, g, 0, 0.085, 0);
  cylY(0.006, 0.02, M.red, g, 0.05, 0.18, 0.02, 8);
  cylY(0.006, 0.02, M.black, g, -0.05, 0.18, 0.02, 8);
  rbx(0.14, 0.03, 0.1, 0.005, M.rioGrey, g, 0, 0.015, 0.1); // roboRIO
  rbx(0.14, 0.04, 0.09, 0.005, M.anodBlack, g, 0, 0.02, -0.1); // PDH
  bx(0.02, 0.006, 0.06, M.red, g, 0, 0.042, -0.1);
  rbx(0.1, 0.03, 0.06, 0.008, M.radioWhite, g, 0.12, 0.015, 0.1); // radio
  return g;
}

// a Limelight, its lens facing +x (turned ry about y, then tilted tilt rad up); ll4: a Limelight 4's
// face (one lens off to one side, the finned heat sink, a green status bar), not the two LEDs
function limelight(parent, x, y, z, ry = 0, { tilt = 0, ll4 = false } = {}) {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  g.rotation.set(0, ry, tilt, 'YXZ');
  parent.add(g);
  rbx(0.03, 0.06, 0.1, 0.006, M.anodBlack, g, 0, 0, 0);
  const lens = mesh(new THREE.CircleGeometry(ll4 ? 0.011 : 0.014, 16), M.lens, g, 0.0155, 0, ll4 ? -0.012 : 0);
  lens.rotation.y = Math.PI / 2;
  if (ll4) {
    for (const fz of [0.044, 0.037, 0.03, 0.004, -0.028, -0.035, -0.042]) bx(0.004, 0.05, 0.003, M.anodBlack, g, 0.0165, 0, fz);
    bx(0.002, 0.022, 0.005, M.led, g, 0.0158, 0, 0.016);
  } else for (const s of [-1, 1]) { const l = mesh(new THREE.CircleGeometry(0.006, 10), M.led, g, 0.0155, 0, s * 0.03); l.rotation.y = Math.PI / 2; }
  return g;
}

// polycarbonate panel with aluminum angle along its top edge; lies in the XY plane
function polyWall(parent, w, h, x, y, z, ry = 0, mat = M.poly, edgeMat = M.alu) {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  g.rotation.y = ry;
  parent.add(g);
  const p = mesh(new THREE.PlaneGeometry(w, h), mat, g, 0, 0, 0);
  p.castShadow = false;
  rbx(w, 0.016, 0.016, 0.002, edgeMat, g, 0, h / 2, 0);
  rbx(0.016, h, 0.012, 0.002, edgeMat, g, -w / 2, 0, 0);
  rbx(0.016, h, 0.012, 0.002, edgeMat, g, w / 2, 0, 0);
  return g;
}

// Over-bumper intake arm from its pivot to the roller: the roller reaches the intake's rated
// reach past the bumper (where the physics grabs FUEL) and sits at FUEL height when deployed.
function intakeArm(cfg, px, py, rollerR) {
  const tx = cfg.frame.length / 2 + BUMPER_T + cfg.intake.reach - rollerR;
  const ty = Math.max(rollerR + 0.012, 0.05);
  return { armLen: Math.hypot(tx - px, py - ty), deploy: -Math.atan2(py - ty, tx - px) };
}

// stowed intakes stand nearly upright just in front of the hopper
const STOWED = 1.4;

// Parts exported from a team's own CAD (cad/robots/*.glb, see cad/README.md), loaded in the
// browser only; the drawn part stays until (and unless) the CAD arrives.
function cadPart(parent, file, onLoad) {
  if (typeof window === 'undefined' || window.__gfxNoCad) return; // (the Lowest graphics: drawn models only)
  const team = file.match(/(\d+)-/)?.[1];
  import('three/addons/loaders/GLTFLoader.js')
    .then(({ GLTFLoader }) => new GLTFLoader().loadAsync('cad/' + file))
    .then((gltf) => {
      gltf.scene.traverse((o) => {
        if (!o.isMesh) return;
        o.castShadow = true;
        o.receiveShadow = true;
        // polycarbonate (see-through in the CAD, or marked "poly" by tools/extract-parts.mjs)
        const m = o.material;
        if (m.transparent || m.name === 'poly') { o.material = M.poly; o.castShadow = false; }
        else o.material = cadMaterial(team, m);
      });
      parent.add(gltf.scene);
      onLoad(gltf.scene);
    })
    .catch((err) => console.warn('robot CAD part failed to load:', file, err));
}

// A CAD export sometimes merges a part into the wrong moving group (a hopper bracket into the
// intake arm's mesh). Moves the triangles of `scene` (a loaded cadPart, in `parent`'s frame) for
// which test(center, min, max) of the triangle's corners holds over to `target` (copy: keeps
// them in place too); `at` is where `parent`'s frame sits in `target`'s frame at rest.
function moveCadTris(scene, parent, test, target, at = [0, 0, 0], copy = false) {
  const holder = new THREE.Group();
  holder.position.set(...at);
  target.add(holder);
  parent.updateMatrixWorld(true);
  const inv = new THREE.Matrix4().copy(parent.matrixWorld).invert();
  const meshes = [];
  scene.traverse((o) => { if (o.isMesh) meshes.push(o); });
  const v = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
  const c = new THREE.Vector3(), lo = new THREE.Vector3(), hi = new THREE.Vector3();
  for (const o of meshes) {
    const rel = new THREE.Matrix4().multiplyMatrices(inv, o.matrixWorld);
    const g = o.geometry, pos = g.attributes.position;
    const idx = g.index ? Array.from(g.index.array) : Array.from({ length: pos.count }, (_, i) => i);
    const stay = [], go = [];
    for (let i = 0; i < idx.length; i += 3) {
      for (let j = 0; j < 3; j++) v[j].fromBufferAttribute(pos, idx[i + j]).applyMatrix4(rel);
      c.copy(v[0]).add(v[1]).add(v[2]).multiplyScalar(1 / 3);
      lo.copy(v[0]).min(v[1]).min(v[2]);
      hi.copy(v[0]).max(v[1]).max(v[2]);
      const tri = [idx[i], idx[i + 1], idx[i + 2]];
      if (test(c, lo, hi)) { go.push(...tri); if (copy) stay.push(...tri); } else stay.push(...tri);
    }
    if (!go.length) continue;
    const moved = new THREE.Mesh(g.clone(), o.material);
    moved.geometry.setIndex(go);
    if (!copy) g.setIndex(stay);
    moved.matrixAutoUpdate = false;
    moved.matrix.copy(rel);
    moved.castShadow = o.castShadow;
    moved.receiveShadow = o.receiveShadow;
    holder.add(moved);
  }
}

// 2910's slap-down intake from their Onshape CAD (Re•Blitz top level assembly, "Pivoting Intake
// Assembly"): pivot position in the robot frame, and how far it swings from stowed (as exported)
// until its 2in roller is down at FUEL height, ~7.8in past the bumper (cfg.intake.arm)
const intake2910 = (cfg) => { const a = cfg.intake.arm; return { pivot: [a.x, a.y], swing: (a.deployDeg - a.stowDeg) * Math.PI / 180 }; };

// row of shooter wheels on a shaft along Z
function wheelStack(parent, len, r, n, wheelMat, x, y, z, width = 0.02) {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  parent.add(g);
  cylZ(0.006, len, M.alu, g, 0, 0, 0, 8);
  for (let i = 0; i < n; i++) {
    const zz = -len / 2 + (i + 0.5) * (len / n);
    cylZ(r, width, wheelMat, g, 0, 0, zz, 20);
  }
  return g;
}

// A wheel drawn about its own axis (local Z) at radius r, width w: 'mecanum' (a hub and rollers
// set at angle deg round the rim), 'omni' (two staggered rows of rollers across the rim) or a
// compliant wheel (a soft tire on a hub)
function rolledWheel(kind, r, w, deg = 45) {
  const g = new THREE.Group();
  if (kind === 'compliant') {
    cylZ(r, w, M.compliant, g, 0, 0, 0, 24);
    cylZ(r * 0.45, w + 0.002, M.hubGrey, g, 0, 0, 0, 12);
    return g;
  }
  const mecanum = kind === 'mecanum';
  cylZ(r * (mecanum ? 0.62 : 0.55), w * (mecanum ? 0.9 : 0.8), M.hubGrey, g, 0, 0, 0, 16);
  const rows = mecanum ? [0] : [-w / 4, w / 4];
  const n = mecanum ? 10 : Math.max(8, Math.round((2 * Math.PI * r) / 0.022));
  const rr = mecanum ? r * 0.15 : Math.min(0.009, w / 4);
  const len = mecanum ? w * 1.15 : ((2 * Math.PI * r) / n) * 0.85;
  const geo = new THREE.CapsuleGeometry(rr, Math.max(0.001, len - 2 * rr), 3, 8);
  rows.forEach((z, k) => {
    for (let i = 0; i < n; i++) {
      const a = ((i + (k ? 0.5 : 0)) / n) * Math.PI * 2;
      const roller = new THREE.Mesh(geo, M.black);
      roller.castShadow = true;
      roller.position.set(Math.cos(a) * (r - rr), Math.sin(a) * (r - rr), z);
      // a roller's axis: along the rim (omni), or turned deg from the wheel's axis toward the rim
      // (mecanum); the capsule's own axis is its Y
      const tangent = new THREE.Vector3(-Math.sin(a), Math.cos(a), 0);
      const dir = mecanum ? new THREE.Vector3(0, 0, 1).multiplyScalar(Math.cos((deg * Math.PI) / 180)).addScaledVector(tangent, Math.sin((deg * Math.PI) / 180)) : tangent;
      roller.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
      g.add(roller);
    }
  });
  return g;
}

function storedGrid(xs, zs, ys, ext = false) {
  const out = [];
  for (const y of ys) for (const x of xs) for (const z of zs) { const v = new THREE.Vector3(x, y, z); v.ext = ext; out.push(v); }
  return out;
}

// ============================================================ 2910 Re•Blitz
// Dumper: huge one-piece hopper over most of the robot, a 4-wide drum shooter with an
// adjustable hood across the back (fixed to the chassis), slap-down intake at the front.
// The hopper front wall slides forward with the intake to open up more room.
function build2910(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, H = cfg.height;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg);
  // the battery lies flat on the bellypan at the front, under the hopper's floor (their CAD)
  rbx(0.167, 0.076, 0.18, 0.006, M.battery, root, 0.222, 0.078, 0);
  const plate = std(0x8f949c, 0.35, 0.85);
  const purple = std(cfg.colors.accent, 0.45, 0.35);
  const green = std(cfg.colors.trim, 0.5, 0.2);
  const extLen = cfg.storage.extLen;

  // ---- shooter tower at the back: pocketed billet side plates, drum + hood. It fires out the
  // back, away from the intake: FUEL comes up the front of the drum from the indexer, rides over
  // the top under the hood and leaves at the back.
  const tower = new THREE.Group();
  tower.position.set(-L / 2 + 0.1, 0, 0);
  root.add(tower);
  const holes = [[-0.04, -0.12, 0.025], [0.04, -0.12, 0.025], [-0.04, 0.0, 0.02], [0.05, 0.02, 0.018], [0, 0.12, 0.018]];
  const drumY = 0.37, hoodR = 0.17; // hood clears a FUEL (5.9in) squeezed on the 4in drum
  for (const s of [-1, 1]) {
    pocketPlate(tower, 0.19, H - 0.14, 0.0127, plate, holes, 0, 0.14 + (H - 0.14) / 2, s * (W / 2 - 0.03));
    kraken(tower, -0.02, drumY, s * (W / 2 + 0.01), false, 'z');
  }
  const drum = wheelStack(tower, W - 0.09, 0.0508, 4, M.green, -0.02, drumY, 0, (W - 0.12) / 4);
  const hood = new THREE.Group();
  hood.position.set(-0.02, drumY, 0);
  tower.add(hood);
  // the hood only covers the exit side: a polycarbonate arc on ribs from just past the top of the
  // drum (70deg) back to where FUEL leaves (139deg), so the drum stays in view
  const arc0 = 1.22, arcLen = 1.21;
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(hoodR, hoodR, W - 0.1, 20, 1, true, arc0 + Math.PI / 2, arcLen), M.polyTint);
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (let i = 0; i < 5; i++) {
    const rib = new THREE.Mesh(new THREE.TorusGeometry(hoodR + 0.004, 0.004, 4, 24, arcLen), purple);
    rib.rotation.z = arc0;
    rib.position.z = -W / 2 + 0.06 + i * ((W - 0.12) / 4);
    hood.add(rib);
  }
  const hoodRollers = [];
  for (const a of [1.45, 1.95]) hoodRollers.push(cylZ(0.016, W - 0.12, M.alu, hood, Math.cos(a) * (hoodR - 0.02), Math.sin(a) * (hoodR - 0.02), 0, 12));
  // compliant indexer wheels feeding the drum + inclined powered floor
  const indexer = wheelStack(tower, W - 0.1, 0.03, 8, M.compliant, 0.085, 0.25, 0, 0.022);
  const feeder = new THREE.Group();
  feeder.position.set(0.06, 0.1, 0);
  feeder.rotation.z = 0.26;
  root.add(feeder);
  const feedRollers = [];
  for (let i = 0; i < 9; i++) feedRollers.push(cylZ(0.016, W - 0.1, M.alu, feeder, 0.32 - i * 0.05, 0, 0, 10));
  mesh(new THREE.PlaneGeometry(0.46, W - 0.1), M.polyBelt, feeder, 0.1, -0.017, 0).rotation.x = -Math.PI / 2;

  // ---- one-piece hopper (tinted polycarbonate on an aluminum frame)
  const hopper = new THREE.Group();
  root.add(hopper);
  const hY = 0.17, hH = H - hY - 0.01;
  const hBack = -L / 2 + 0.2, hFront = L / 2 - 0.02;
  const hMid = (hBack + hFront) / 2, hLen = hFront - hBack;
  for (const s of [-1, 1]) polyWall(hopper, hLen, hH, hMid, hY + hH / 2, s * (W / 2 - 0.012), 0, M.polyTint, green);
  // expanding front section: telescoping side panels + front wall ride out over the intake as it
  // deploys (the stowed intake stands just behind the front wall)
  const front = new THREE.Group();
  root.add(front);
  const rails = [];
  for (const s of [-1, 1]) {
    polyWall(front, extLen + 0.04, hH * 0.85, hFront - extLen / 2 + 0.02, hY + hH * 0.85 / 2, s * (W / 2 - 0.028), 0, M.polyTint, M.alu);
    rails.push(tube(root, hFront - 0.28, s * (W / 2 - 0.05), hFront - 0.02, s * (W / 2 - 0.05), hY + hH - 0.02, M.aluDark, 0.012, 0.012)); // slide rail
  }
  // front wall stops short of the floor: the intake feeds FUEL in through the slot under it
  const slot = 0.2, fwH = hH * 0.85 - slot;
  polyWall(front, W - 0.06, fwH, hFront + 0.02, hY + slot + fwH / 2, 0, Math.PI / 2, M.polyTint, M.alu);
  // clear lids: one over the fixed hopper, and one on the expanding section, nested just under it
  const lid = (parent, x0, x1, y, w) => {
    const m = mesh(new THREE.PlaneGeometry(x1 - x0, w), M.polyTint, parent, (x0 + x1) / 2, y, 0);
    m.rotation.x = -Math.PI / 2;
    m.castShadow = false;
    for (const s of [-1, 1]) rbx(x1 - x0, 0.012, 0.016, 0.002, M.alu, parent, (x0 + x1) / 2, y, s * w / 2);
    rbx(0.016, 0.012, w, 0.002, M.alu, parent, x1, y, 0);
    return m;
  };
  lid(hopper, hBack, hFront, hY + hH, W - 0.03);
  lid(front, hFront - extLen + 0.02, hFront + 0.02, hY + hH * 0.85, W - 0.06);
  // their "R2 Hopper" assembly: side, top and front panels that sit over the shooter when stowed
  // and slide out along the slotted rails as the intake deploys; replaces the drawn hopper
  const drawnFront = [...front.children];
  cadPart(front, 'robots/2910-hopper.glb', () => {
    hopper.visible = false;
    for (const c of [...drawnFront, ...rails]) c.visible = false;
  });
  limelight(root, hFront - 0.3, H + 0.01, W / 2 - 0.03);
  limelight(root, -L / 2 + 0.02, H - 0.03, -W / 2 + 0.12, Math.PI);

  // ---- slap-down intake: pivots at the front of the frame, arms fold up over the hopper
  const intake = new THREE.Group();
  intake.position.set(L / 2 - 0.035, 0.26, 0);
  root.add(intake);
  const { armLen, deploy } = intakeArm(cfg, L / 2 - 0.035, 0.26, 0.03);
  for (const s of [-1, 1]) {
    pocketPlate(intake, armLen + 0.04, 0.055, 0.0095, plate, [[-0.08, 0, 0.012], [0.04, 0, 0.012]], armLen / 2, 0, s * (cfg.intake.width / 2 + 0.012));
    kraken(intake, 0.02, 0, s * (cfg.intake.width / 2 + 0.04), true, 'z');
  }
  const intakeRollers = [
    wheelStack(intake, cfg.intake.width, 0.03, 8, M.orangeWheel, armLen, 0, 0, 0.03),
    wheelStack(intake, cfg.intake.width, 0.022, 6, M.compliant, armLen - 0.09, -0.035, 0, 0.025),
  ];
  rbx(0.05, 0.05, cfg.intake.width + 0.04, 0.004, purple, intake, armLen - 0.02, 0.05, 0);
  const cadIntake = new THREE.Group();
  const INTAKE_2910 = intake2910(cfg);
  cadIntake.position.set(INTAKE_2910.pivot[0], INTAKE_2910.pivot[1], 0);
  root.add(cadIntake);
  cadPart(cadIntake, 'robots/2910-intake.glb', () => { intake.visible = false; });
  // their "Shooter & Feeder" assembly: powered floor, the roller ramp that indexes FUEL up under
  // the rollered hood, and the drum at the back; it replaces the drawn tower and floor
  cadPart(root, 'robots/2910-shooter.glb', () => { tower.visible = false; feeder.visible = false; });
  // the hopper's floor (their CAD's, as js/mech/r2910.js has it; the exported shooter leaves its
  // sleeves out): ten powered rollers, 1.25in silicone sleeves sloping down to the back, then bare
  // aluminum, then two up the back wall
  const silicone = std(0x3d4148, 0.85, 0.0);
  const floorRollers = FLOOR_2910.map(([x, y, d, grip]) => cylZ(d / 2, 0.559, grip === 'aluminum' ? M.alu : silicone, root, x, y, 0, 14));
  // their hood (its two rollers on arms that pivot on the drum shaft): it swings back over the
  // drum for a flatter shot (cfg.shooter.hood)
  const hoodCfg = cfg.shooter.hood;
  const cadHood = new THREE.Group();
  cadHood.position.set(hoodCfg.x, hoodCfg.y, 0);
  root.add(cadHood);
  cadPart(cadHood, 'robots/2910-hood.glb', () => {});

  // ---- FUEL inside: main hopper, then the extension, then the tower
  const stored = storedGrid([-0.13, 0.02, 0.17, 0.3], [-0.225, -0.075, 0.075, 0.225], [0.2, 0.34, 0.47]);
  stored.push(...storedGrid([0.28, 0.43], [-0.225, -0.075, 0.075, 0.225], [0.22, 0.36], true).map((v) => { v.x -= extLen; return v; }));
  for (const z of [-0.19, -0.063, 0.063, 0.19]) stored.push(new THREE.Vector3(-0.2, 0.3, z));

  const anim = (st, dt) => {
    // intake: 0 = stowed (arms up in front of the hopper), 1 = deployed over the bumper
    intake.rotation.z = lerp(STOWED, deploy, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.z += st.intakeSpeed * dt * 40;
    cadIntake.rotation.z = lerp(0, INTAKE_2910.swing, st.intakeDeploy);
    front.position.x = st.hopperDeploy * extLen; // the hopper expands with the intake
    drum.rotation.z -= st.flywheel * dt * 6;
    for (const r of hoodRollers) r.rotation.y += st.flywheel * dt * 8;
    indexer.rotation.z -= st.feeding * dt * 25;
    for (const r of feedRollers) r.rotation.y -= st.feeding * dt * 30;
    for (const r of floorRollers) r.rotation.y += (st.floorSpin ?? st.feeding * 30) * dt; // (their tops run back)
    hood.rotation.z = -(st.hoodDeg - 58) * Math.PI / 180 * 0.4; // a higher shot tips the exit up
    cadHood.rotation.z = ((hoodCfg.cadDeg - st.hoodDeg) * Math.PI) / 180;
  };
  return { root, anim, stored, modules, extLen };
}

// ============================================================ 4414 RIPCURRENT
// Modeled on 4414's tech binder CAD renders (2026.team4414.com; their CAD isn't public):
//  - chamfer-backed hopper of smoked polycarbonate with the teal truss along the back, and a
//    pocketed top plate that runs from the back truss to the turret bearing
//  - the intake box (also the hopper's extension) slides out on racks at the start of the
//    match: a front panel with an under-roller, a hinged ramp of passive rollers up to the
//    bumper, and a star roller that drives FUEL over it
//  - the Dye Rotor: a pocketed platter that spins, carrying FUEL round (the Dolphin Fin ramp on
//    its outside sweeps the load), into a fixed hook of passive rollers that steers it to the
//    feeder at the center column; the feeder wheels lift it up the ramp into the turret
//  - a netted top that stays folded down in the starting configuration and springs up once
//    the hopper is out (it squashes under the TRENCH arm)

// carbon / SRPP weave
function carbonTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#7d8289';
  g.fillRect(0, 0, 64, 64);
  for (let y = 0; y < 64; y += 4) {
    for (let x = 0; x < 64; x += 4) {
      g.fillStyle = ((x + y) / 4) % 2 ? '#8f949b' : '#6c7178';
      g.fillRect(x, y, 4, 4);
    }
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(12, 12);
  return t;
}

// knotless netting, 2in square mesh of thin cord: one tile = 0.1 m (two cells)
function fineNetTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  g.clearRect(0, 0, 128, 128);
  g.strokeStyle = 'rgba(255,255,255,0.95)';
  g.lineWidth = 2.2;
  for (const p of [1, 65]) {
    g.beginPath(); g.moveTo(p, 0); g.lineTo(p, 128); g.stroke();
    g.beginPath(); g.moveTo(0, p); g.lineTo(128, p); g.stroke();
  }
  g.fillStyle = 'rgba(255,255,255,1)';
  for (const x of [1, 65]) for (const y of [1, 65]) g.fillRect(x - 2.5, y - 2.5, 5, 5); // knots
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(10, 10);
  t.anisotropy = 8;
  return t;
}

// strip with triangular cutouts (the teal truss), len along x, w along y, t thick (centered on z): a
// Warren truss, its chords m wide, its triangular pockets' corners rounded (r, as the renders'
// machined pockets); mesh.userData.nodes: where its webs meet the chords ([x, y], for rivets)
function trussStrip(len, w, t, mat, parent, { m = 0.006, r = 0.004, pitch = 0.9 } = {}) {
  const s = new THREE.Shape();
  s.moveTo(0, 0); s.lineTo(len, 0); s.lineTo(len, w); s.lineTo(0, w); s.closePath();
  const n = Math.max(1, Math.floor((len - m) / (w * pitch)));
  const p = (len - m) / n, nodes = [[m / 2, w - m / 2], [m / 2, m / 2]];
  for (let i = 0; i < n; i++) {
    const x0 = m + i * p, x1 = x0 + p - m;
    if (x1 - x0 < 0.01) continue;
    const tri = i % 2 ? [[x0, w - m], [x1, w - m], [(x0 + x1) / 2, m]] : [[x0, m], [x1, m], [(x0 + x1) / 2, w - m]];
    s.holes.push(new THREE.Path(roundPts(tri, r).map(([x, y]) => new THREE.Vector2(x, y))));
    nodes.push([(x0 + x1) / 2, i % 2 ? m / 2 : w - m / 2], [x1 + m / 2, i % 2 ? w - m / 2 : m / 2]);
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false, curveSegments: 4 });
  g.translate(0, 0, -t / 2);
  const mm = mesh(g, mat, parent);
  mm.userData.nodes = nodes;
  return mm;
}

// button-head bolts, all in one instanced mesh: add(position, outward normal)
function boltSet(parent, mat, r = 0.004) {
  const pts = [];
  return {
    add(p, n) { pts.push([p.clone(), n.clone()]); },
    done() {
      const geo = new THREE.CylinderGeometry(r, r * 1.1, 0.003, 8);
      geo.translate(0, 0.0015, 0);
      const im = new THREE.InstancedMesh(geo, mat, Math.max(1, pts.length));
      const up = new THREE.Vector3(0, 1, 0), q = new THREE.Quaternion(), m4 = new THREE.Matrix4(), one = new THREE.Vector3(1, 1, 1);
      pts.forEach(([p, n], i) => { q.setFromUnitVectors(up, n); m4.compose(p, q, one); im.setMatrixAt(i, m4); });
      im.count = pts.length;
      parent.add(im);
      return im;
    },
  };
}

// flat part from an outline in the XZ plane (robot frame, points [x, z]), top face at y
function flatPart(outline, holes, t, mat, parent, y) {
  const s = new THREE.Shape(outline.map(([x, z]) => new THREE.Vector2(x, -z)));
  for (const h of holes) s.holes.push(new THREE.Path(h.map(([x, z]) => new THREE.Vector2(x, -z))));
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false, curveSegments: 24 });
  g.rotateX(-Math.PI / 2);
  return mesh(g, mat, parent, 0, y - t, 0);
}

const arcPts = (cx, cz, r, a0, a1, n) => Array.from({ length: n + 1 }, (_, i) => { const a = a0 + ((a1 - a0) * i) / n; return [cx + r * Math.cos(a), cz - r * Math.sin(a)]; });

// a polygon ([x, z] or [x, y] points) with its corners rounded off: radius r (at most nearly half
// of either side), n segments round each (a machined pocket's corners)
function roundPts(pts, r, n = 3) {
  const out = [], m = pts.length;
  for (let i = 0; i < m; i++) {
    const P = pts[i], A = pts[(i + m - 1) % m], B = pts[(i + 1) % m];
    const la = Math.hypot(A[0] - P[0], A[1] - P[1]) || 1, lb = Math.hypot(B[0] - P[0], B[1] - P[1]) || 1;
    const rr = Math.min(r, la * 0.45, lb * 0.45);
    const a = [P[0] + ((A[0] - P[0]) * rr) / la, P[1] + ((A[1] - P[1]) * rr) / la], b = [P[0] + ((B[0] - P[0]) * rr) / lb, P[1] + ((B[1] - P[1]) * rr) / lb];
    for (let k = 0; k <= n; k++) { const t = k / n, u = 1 - t; out.push([u * u * a[0] + 2 * u * t * P[0] + t * t * b[0], u * u * a[1] + 2 * u * t * P[1] + t * t * b[1]]); }
  }
  return out;
}

// spur gear lying flat (axis along y), top at y = 0
function spurGear(r, teeth, t, mat, parent, holes = 0) {
  const s = new THREE.Shape();
  const td = r * 0.06;
  for (let i = 0; i < teeth; i++) {
    const a = (i / teeth) * Math.PI * 2, da = (Math.PI * 2) / teeth;
    const pts = [[r - td, a], [r + td, a + da * 0.2], [r + td, a + da * 0.45], [r - td, a + da * 0.65]];
    pts.forEach(([rr, aa], k) => { const x = rr * Math.cos(aa), y = rr * Math.sin(aa); if (i === 0 && k === 0) s.moveTo(x, y); else s.lineTo(x, y); });
  }
  s.closePath();
  for (let i = 0; i < holes; i++) {
    const a0 = (i / holes) * Math.PI * 2 + 0.12, a1 = ((i + 1) / holes) * Math.PI * 2 - 0.12;
    const h = new THREE.Path();
    h.absarc(0, 0, r * 0.78, a0, a1, false);
    h.absarc(0, 0, r * 0.3, a1, a0, true);
    s.holes.push(h);
  }
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false, curveSegments: 6 });
  g.rotateX(-Math.PI / 2);
  g.translate(0, -t, 0);
  return mesh(g, mat, parent);
}

// thin band following a curve of [x, z] points at height y0..y1 (vertical ribbon)
function ribbon(points, y0, y1, mat, parent) {
  const pos = [], idx = [];
  points.forEach(([x, z], i) => {
    pos.push(x, y0, z, x, y1, z);
    if (i) { const k = 2 * i; idx.push(k - 2, k, k + 1, k - 2, k + 1, k - 1); }
  });
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return mesh(g, mat, parent);
}

// flat curved strip following [x, z] points, width w, at height y
function flatRibbon(points, w, y, mat, parent) {
  const pos = [], idx = [];
  points.forEach(([x, z], i) => {
    const [ax, az] = points[Math.max(0, i - 1)], [bx2, bz] = points[Math.min(points.length - 1, i + 1)];
    let tx = bx2 - ax, tz = bz - az;
    const l = Math.hypot(tx, tz) || 1;
    tx /= l; tz /= l;
    pos.push(x - tz * w / 2, y, z + tx * w / 2, x + tz * w / 2, y, z - tx * w / 2);
    if (i) { const k = 2 * i; idx.push(k - 2, k, k + 1, k - 2, k + 1, k - 1); }
  });
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return mesh(g, mat, parent);
}

// cylinder between two points
function rod(parent, a, b, r, mat, segs = 8) {
  const d = new THREE.Vector3().subVectors(b, a);
  const m = mesh(new THREE.CylinderGeometry(r, r, d.length(), segs), mat, parent);
  m.position.copy(a).add(b).multiplyScalar(0.5);
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d.normalize());
  return m;
}

// A net stretched over an open hopper top: pinned along its edges (and round a hole, e.g. a
// turret), draped over whatever FUEL pokes up above the pins, taut everywhere else. An extending
// hopper can carry its front edge out: the part past xFixed stretches to follow it.
//   x0..x1: back and (stowed) front edge; x1Out: the front edge with the hopper out; hw: half width;
//   y: height of the pinned edges (yFront: the front edge's, for a net stretched diagonally down
//   to an intake). drape(load, ex, at): ex is how far the front has moved out; at { y, yFront, xBack }
//   moves the pinned heights (a lid that lifts).
// (inside(x, z): the hopper's outline, for one that isn't a rectangle: the net stops at it; hole.y:
// the height it's tied on at round the hole, if not the edge's; slack { sag, sagOut }: a slack net,
// not a taut one: with nothing under it, it hangs sag m below its ties at its middle (sagOut with
// the front all the way out, where it's taken up some of its slack), and it drapes down between
// the FUEL it lies on instead of spanning them)
function hopperNet(parent, { x0, x1, x1Out = x1, xFixed = x1, hw, y, yFront = y, hole = null, inside = null, NX = 36, NZ = 28, color = 0x15171a, material = null, slack = null }) {
  const mat = material || new THREE.MeshStandardMaterial({ map: fineNetTexture(), transparent: true, depthWrite: false, side: THREE.DoubleSide, roughness: 0.9, color });
  const nv = (NX + 1) * (NZ + 1);
  const base = new Float32Array(nv), pin = new Uint8Array(nv), hNow = new Float32Array(nv), hMin = new Float32Array(nv);
  const pos = new Float32Array(nv * 3), uv = new Float32Array(nv * 2), idx = [];
  const inHole = (x, z, pad) => hole && Math.hypot(x - hole.x, z - hole.z) < hole.r + pad;
  for (let i = 0; i <= NX; i++) {
    for (let j = 0; j <= NZ; j++) {
      const k = i * (NZ + 1) + j, x = x0 + ((x1Out - x0) * i) / NX, z = -hw + (2 * hw * j) / NZ;
      base[k] = x;
      pin[k] = i === 0 || i === NX || j === 0 || j === NZ || inHole(x, z, 0.01) || (inside && !inside(x, z)) ? 1 : 0;
      hNow[k] = y;
      pos.set([x, y, z], k * 3);
      uv.set([x, z], k * 2);
    }
  }
  for (let i = 0; i < NX; i++) {
    for (let j = 0; j < NZ; j++) {
      const k = i * (NZ + 1) + j;
      const cx = x0 + ((x1Out - x0) * (i + 0.5)) / NX, cz = -hw + (2 * hw * (j + 0.5)) / NZ;
      if (inHole(cx, cz, 0) || (inside && !inside(cx, cz))) continue;
      idx.push(k, k + 1, k + NZ + 2, k, k + NZ + 2, k + NZ + 1);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uv, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  const m = mesh(geo, mat, parent);
  m.castShadow = false;
  m.frustumCulled = false;
  m.userData.net = true;
  const R = FUEL.radius;
  const hPin = new Float32Array(nv);
  // (a slack net: how a sheet pinned at its edges hangs under its own weight, each point dropped
  // a step below its neighbors' average, to the depth at its middle for a drop of 1; and it starts
  // hanging so)
  const hang = new Float32Array(nv);
  let hangMid = 1;
  if (slack) {
    for (let it = 0; it < 400; it++) {
      for (let i = 1; i < NX; i++) {
        for (let j = 1; j < NZ; j++) {
          const k = i * (NZ + 1) + j;
          if (pin[k]) continue;
          hang[k] += 1.9 * ((hang[k - 1] + hang[k + 1] + hang[k - NZ - 1] + hang[k + NZ + 1]) / 4 + 1 - hang[k]);
        }
      }
    }
    hangMid = Math.max(...hang);
    for (let k = 0; k < nv; k++) hNow[k] = y - (slack.sag * hang[k]) / hangMid;
  }
  const drape = (load, ex = 0, at = null) => {
    const kx = x1Out > xFixed ? (x1 + ex - xFixed) / (x1Out - xFixed) : 1;
    const yb = at?.y ?? y, yf = at?.yFront ?? yFront, xf = x1 + ex;
    // (at.xBack: its back edge held there instead, the cover gathered up toward the front)
    const xb = at?.xBack !== undefined && at.xBack > x0 ? Math.min(at.xBack, xf - 0.02) : x0;
    for (let k = 0; k < nv; k++) {
      let x = base[k] <= xFixed ? base[k] : xFixed + (base[k] - xFixed) * kx;
      if (xb > x0) x = xb + ((x - x0) * (xf - xb)) / (xf - x0);
      pos[k * 3] = x;
      hPin[k] = yb + (yf - yb) * Math.min(1, Math.max(0, (x - x0) / Math.max(0.01, xf - x0)));
      if (hole && hole.y !== undefined && inHole(x, pos[k * 3 + 2], 0.01)) hPin[k] = hole.y;
      hMin[k] = slack ? -Infinity : hPin[k]; // (a slack one hangs below its ties)
    }
    // (slack: how far it hangs now, and so each point's drop below its neighbors)
    const sag = slack ? slack.sag + ((slack.sagOut ?? slack.sag) - slack.sag) * Math.min(1, Math.max(0, x1Out > x1 ? ex / (x1Out - x1) : 0)) : 0;
    const drop = sag / hangMid;
    // held up by the FUEL poking above the pins (or above where a slack one hangs)
    for (const e of load || []) {
      const p = e.p;
      if (p.y + R < Math.min(yb, yf) - sag) continue;
      for (let k = 0; k < nv; k++) {
        if (pin[k]) continue;
        const dx = pos[k * 3] - p.x, dz = pos[k * 3 + 2] - p.z, d2 = dx * dx + dz * dz;
        if (d2 < R * R) hMin[k] = Math.max(hMin[k], p.y + Math.sqrt(R * R - d2) + 0.004);
      }
    }
    // a taut sheet: each point sits at its neighbors' average unless the FUEL holds it up (a slack
    // one: drop below it)
    for (let it = 0; it < 10; it++) {
      for (let i = 1; i < NX; i++) {
        for (let j = 1; j < NZ; j++) {
          const k = i * (NZ + 1) + j;
          if (pin[k]) continue;
          hNow[k] = Math.max(hMin[k], (hNow[k - 1] + hNow[k + 1] + hNow[k - NZ - 1] + hNow[k + NZ + 1]) / 4 - drop);
        }
      }
    }
    for (let k = 0; k < nv; k++) pos[k * 3 + 1] = pin[k] ? hPin[k] : hNow[k];
    geo.attributes.position.needsUpdate = true;
    geo.computeVertexNormals();
  };
  return { mesh: m, drape };
}

function build4414(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width;
  const bay = cfg.bay, rs = bay.rotor, col = bay.column;
  const extLen = cfg.storage.extLen;
  const wallTop = bay.wallTop; // top of the hopper walls and the top plate
  const hY = 0.15;             // walls start just under the bumper top
  // the walls' faces where the physical model's are (js/mech/r4414.js BAY: the bumper backing, on the
  // frame perimeter), so the FUEL against them are drawn against them
  const { x0: back, hw: wallZ, chamfer } = D4.BAY;
  const R_FUEL = FUEL.radius;
  addBumpers(root, cfg, alliance, chamfer);
  const modules = addDrivebase(root, cfg, chamfer);
  addElectronics(root, -0.06, W / 2 - 0.2, Math.PI / 2); // (between the right side's modules, clear of their motors)

  const teal = std(cfg.colors.accent, 0.4, 0.45);
  const blackAl = std(0x2a2e35, 0.42, 0.55);
  const darkPlate = std(0x50565f, 0.45, 0.5);
  const greyAl = std(0x5c6169, 0.38, 0.7);
  const plastic = std(0x3a3e45, 0.7, 0.05);
  const carbon = new THREE.MeshStandardMaterial({ map: carbonTexture(), color: 0xffffff, roughness: 0.4, metalness: 0.3 });
  // (the box's carbon fiber: dark, a satin weave rather than a mirror)
  const carbonDark = new THREE.MeshStandardMaterial({ map: carbonTexture(), color: 0x5e636b, roughness: 0.5, metalness: 0.1 });
  const smoke = polyMat({ color: 0x2c3036, opacity: 0.58, roughness: 0.2 }, 0x3a4048);
  const clearSmoke = polyMat({ color: 0x59606a, opacity: 0.22, roughness: 0.15 }, 0x9aa2ac);
  const bolts = boltSet(root, std(0xc4c8ce, 0.3, 0.9));
  const up = new THREE.Vector3(0, 1, 0);

  // ---- hopper walls: smoked polycarbonate, chamfered back corners
  const cz = wallZ - chamfer, cx = back + chamfer;
  const wallPts = [[L / 2 - 0.005, -wallZ], [cx, -wallZ], [back, -cz], [back, cz], [cx, wallZ], [L / 2 - 0.005, wallZ]];
  const wallSegs = [];
  for (let i = 0; i < wallPts.length - 1; i++) {
    const [x0, z0] = wallPts[i], [x1, z1] = wallPts[i + 1];
    const len = Math.hypot(x1 - x0, z1 - z0), ry = -Math.atan2(z1 - z0, x1 - x0);
    const p = mesh(new THREE.PlaneGeometry(len, wallTop - hY), smoke, root, (x0 + x1) / 2, (hY + wallTop) / 2, (z0 + z1) / 2);
    p.rotation.y = ry;
    p.castShadow = false;
    // outward normal (the polygon runs clockwise seen from above)
    const n = new THREE.Vector3(-(z1 - z0) / len, 0, (x1 - x0) / len);
    if (n.x * (x0 + x1) + n.z * (z0 + z1) < 0) n.negate();
    wallSegs.push({ x0, z0, x1, z1, len, ry, n });
    // rivets along the top and bottom edges
    for (const y of [wallTop - 0.012, hY + 0.012]) {
      const k = Math.max(2, Math.round(len / 0.07));
      for (let j = 0; j <= k; j++) {
        const t = (j + 0.5) / (k + 1);
        bolts.add(new THREE.Vector3(x0 + (x1 - x0) * t + n.x * 0.001, y, z0 + (z1 - z0) * t + n.z * 0.001), n);
      }
    }
    // thin bent lip along the top edge
    const lip = bx(len, 0.004, 0.018, blackAl, root, (x0 + x1) / 2 - n.x * 0.009, wallTop - 0.002, (z0 + z1) / 2 - n.z * 0.009);
    lip.rotation.y = ry;
  }
  // The teal truss (the binder's renders): a bent angle along the top of the back and chamfer walls,
  // its web down the wall's face and its flange over the top, each a Warren truss of round-cornered
  // triangular pockets riveted at its nodes; down each back corner a bent angle round the corner, a
  // flange on each wall's face with the same truss and rivets down its outer edge; and over the top
  // of each corner a gusset joining the flanges, fanned with pockets
  for (const sgm of wallSegs.slice(1, 4)) {
    const { x0, z0, x1, z1, len, ry, n } = sgm;
    const g = new THREE.Group();
    g.position.set(x0, 0, z0);
    g.rotation.y = ry;
    root.add(g);
    // which way is out in the group's frame
    const zs = new THREE.Vector3(0, 0, 1).applyAxisAngle(up, ry).dot(n) > 0 ? 1 : -1;
    const web = trussStrip(len, 0.05, 0.004, teal, g, { r: 0.005 });
    web.position.set(0, wallTop - 0.05, zs * 0.004);
    for (const [u, v] of web.userData.nodes) {
      if (u < 0.03 || u > len - 0.03) continue;
      bolts.add(new THREE.Vector3(x0 + ((x1 - x0) * u) / len + n.x * 0.0065, wallTop - 0.05 + v, z0 + ((z1 - z0) * u) / len + n.z * 0.0065), n);
    }
    const fl = trussStrip(len, 0.042, 0.004, teal, g, { r: 0.005 });
    fl.rotation.x = zs > 0 ? -Math.PI / 2 : Math.PI / 2;
    fl.position.set(0, wallTop + 0.002, zs * 0.004);
  }
  for (const i of [1, 2, 3, 4]) {
    const [x, z] = wallPts[i];
    const sa = wallSegs[i - 1], sb = wallSegs[i];
    // the corner's angle: a flange down each wall's face, from the corner along the wall
    for (const [sg, away] of [[sa, [sa.x0 - sa.x1, sa.z0 - sa.z1]], [sb, [sb.x1 - sb.x0, sb.z1 - sb.z0]]]) {
      const ux = away[0] / sg.len, uz = away[1] / sg.len, H = wallTop - hY + 0.01, fw = 0.042;
      const g = new THREE.Group();
      g.position.set(x + sg.n.x * 0.006, hY - 0.01, z + sg.n.z * 0.006);
      g.rotation.y = Math.atan2(-uz, ux);
      root.add(g);
      const c = trussStrip(H, fw, 0.004, teal, g, { r: 0.005, pitch: 1.1 });
      c.rotation.z = Math.PI / 2;
      c.position.x = fw;
      for (const [u, v] of c.userData.nodes) {
        if (v < fw / 2 || u < 0.02 || u > H - 0.06) continue; // (the outer edge's; the top's under the gusset)
        const d = fw - v;
        bolts.add(new THREE.Vector3(g.position.x + ux * d + sg.n.x * 0.0025, g.position.y + u, g.position.z + uz * d + sg.n.z * 0.0025), sg.n);
      }
    }
    // the gusset over the corner, on the flanges: from the corner along both walls, in to a point
    {
      const P = new THREE.Vector2(x, z), a1 = new THREE.Vector2(sa.x0 - sa.x1, sa.z0 - sa.z1).normalize(), a2 = new THREE.Vector2(sb.x1 - sb.x0, sb.z1 - sb.z0).normalize();
      const n1 = new THREE.Vector2(sa.n.x, sa.n.z), n2 = new THREE.Vector2(sb.n.x, sb.n.z), bi = n1.clone().add(n2).normalize().negate();
      const at = (v) => [v.x, v.y];
      const L = 0.1, D = 0.04;
      const out = [
        at(P.clone().addScaledVector(n1.clone().add(n2).normalize(), 0.006)),
        at(P.clone().addScaledVector(a2, L)), at(P.clone().addScaledVector(a2, L).addScaledVector(n2, -D)),
        at(P.clone().addScaledVector(bi, 0.07)),
        at(P.clone().addScaledVector(a1, L).addScaledVector(n1, -D)), at(P.clone().addScaledVector(a1, L)),
      ];
      // its pockets: a fan out from near the corner
      const O = P.clone().addScaledVector(bi, 0.016), dirs = [];
      const d1 = a1.clone().addScaledVector(n1, -0.25).normalize(), d2 = a2.clone().addScaledVector(n2, -0.25).normalize();
      for (let k = 0; k <= 3; k++) dirs.push(d1.clone().lerp(d2, k / 3).normalize());
      const holes = [];
      for (let k = 0; k < 3; k++) {
        const q = [at(O.clone().addScaledVector(dirs[k], 0.012)), at(O.clone().addScaledVector(dirs[k], k === 0 ? 0.075 : 0.05)), at(O.clone().addScaledVector(dirs[k + 1], k === 2 ? 0.075 : 0.05)), at(O.clone().addScaledVector(dirs[k + 1], 0.012))];
        holes.push(roundPts(q, 0.004));
      }
      flatPart(roundPts(out, 0.006), holes, 0.004, teal, root, wallTop + 0.008);
      for (const v of [P.clone().addScaledVector(a1, L - 0.01).addScaledVector(n1, -0.01), P.clone().addScaledVector(a2, L - 0.01).addScaledVector(n2, -0.01), P.clone().addScaledVector(bi, 0.055)]) bolts.add(new THREE.Vector3(v.x, wallTop + 0.008, v.y), up);
    }
  }

  // ---- top plate: pocketed, from the back truss forward to the turret bearing (a keyhole)
  const tx = cfg.shooter.turretPos.x, tz = cfg.shooter.turretPos.z;
  const xs = -0.165, ro = 0.2, riT = 0.158;
  // the shroud the shooter hangs in, down inside the ring (the physical model's: the load can't come
  // up round the turret)
  const [shY, shR] = D4.BAY.shroud;
  // (drawn up to just under the energy chain, which runs round the shooter just under the plate)
  const shTop = D4.BAY.top - 0.06;
  mesh(new THREE.CylinderGeometry(shR, shR, shTop - shY, 40, 1, true), clearSmoke, root, tx, (shTop + shY) / 2, tz).castShadow = false;
  {
    // The top plate (the binder's renders: Turret, Turret PT): the back strip a Warren truss between
    // chords lined with small slots, the ring round the turret's bearing with a band of slots, the
    // strip and ring joined by concave fillets, and behind the turret, where the two overlap, a web
    // of pockets between ribs running out from the bearing (under it the energy chain's compliant
    // wheel). The plate's outline: the chamfered back, the strip's front edge, the fillets, the ring.
    const rf = 0.035, dxc = xs + rf - tx, zc = Math.sqrt((ro + rf) ** 2 - dxc ** 2);
    const a1 = Math.atan2(zc, dxc), b1 = Math.atan2(zc, -dxc);
    const outline = [
      [back, -cz], [cx, -wallZ], [xs, -wallZ],
      ...arcPts(xs + rf, -zc, rf, Math.PI, 2 * Math.PI - b1, 10),
      ...arcPts(tx, tz, ro, a1, -a1, 48),
      ...arcPts(xs + rf, zc, rf, b1, Math.PI, 10),
      [xs, wallZ], [cx, wallZ], [back, cz],
    ];
    const holes = [arcPts(tx, tz, riT, 0, Math.PI * 2, 48)];
    const inChamfer = (x, z) => (x - back) + (wallZ - Math.abs(z)) < chamfer + 0.02;
    const inside = (pts) => !pts.some(([x, z]) => inChamfer(x, z));
    // the ring's band of slots, the bolts between them
    for (let k = 0; k < 24; k++) {
      const a = (k / 24) * Math.PI * 2, da = 4.2 / 57.3;
      const pts = [...arcPts(tx, tz, 0.176, a - da, a + da, 3), ...arcPts(tx, tz, 0.19, a + da, a - da, 3)];
      if (pts.every(([x]) => x > xs - 0.01) || pts.every(([x, z]) => Math.hypot(x - tx, z - tz) < 0.2)) holes.push(roundPts(pts, 0.003, 2));
      const ab = a + Math.PI / 24;
      bolts.add(new THREE.Vector3(tx + Math.cos(ab) * 0.183, wallTop, tz - Math.sin(ab) * 0.183), up);
    }
    // the strip's truss either side of the turret, its chords 20 mm
    const px0 = back + 0.02, px1 = xs - 0.02, zIn = zc + 0.01, pw = 0.075;
    for (const sd of [-1, 1]) {
      const zA = zIn, zB = wallZ - 0.025, n = Math.max(1, Math.round((zB - zA) / pw)), p = (zB - zA) / n;
      for (let i = 0; i < n; i++) {
        const z0 = zA + i * p + 0.004, z1 = z0 + p - 0.008, zm = (z0 + z1) / 2;
        const tri = i % 2 ? [[px0, z0], [px0, z1], [px1, zm]] : [[px1, z0], [px1, z1], [px0, zm]];
        const pts = tri.map(([x, z]) => [x, sd * z]);
        if (inside(pts)) holes.push(roundPts(pts, 0.006));
        else {
          // (by the chamfer: a smaller pocket clear of it)
          const sh = pts.map(([x, z]) => [x + (px1 - x) * 0.35, z]);
          if (inside(sh)) holes.push(roundPts(sh, 0.005));
        }
      }
      // the chords' slots: along the back edge and the strip's front edge
      for (let z = zIn + 0.01; z < wallZ - 0.02; z += 0.03) {
        for (const x of [back + 0.009, xs - 0.009]) {
          const pts = [[x - 0.002, sd * (z - 0.006)], [x + 0.002, sd * (z - 0.006)], [x + 0.002, sd * (z + 0.006)], [x - 0.002, sd * (z + 0.006)]];
          if (inside(pts)) holes.push(roundPts(pts, 0.0019, 2));
        }
      }
    }
    for (let z = -zIn + 0.015; z < zIn - 0.01; z += 0.03) {
      const x = back + 0.009, pts = [[x - 0.002, z - 0.006], [x + 0.002, z - 0.006], [x + 0.002, z + 0.006], [x - 0.002, z + 0.006]];
      holes.push(roundPts(pts, 0.0019, 2));
    }
    // behind the turret, between the ring and the back chord: the web, two rows of pockets between
    // ribs running out from the bearing
    const rib = 0.008, xb = back + 0.02;
    const rows = [[ro + 0.012, ro + 0.045], [ro + 0.045 + rib, 0.4]];
    for (const [r0, r1] of rows) {
      for (let k = -4; k < 4; k++) {
        const aa = Math.PI + (k * 13) / 57.3 + 0.5 * rib / r0, ab = Math.PI + ((k + 1) * 13) / 57.3 - 0.5 * rib / r0;
        const pts = [];
        for (const a of [aa, (aa + ab) / 2, ab]) pts.push([tx + r0 * Math.cos(a), tz - r0 * Math.sin(a)]);
        for (const a of [ab, (aa + ab) / 2, aa]) {
          // (out to r1, or to the back chord)
          const c = Math.cos(a), r = c < 0 ? Math.min(r1, (xb - tx) / c) : r1;
          pts.push([tx + r * c, tz - r * Math.sin(a)]);
        }
        const ok = pts.every(([x, z]) => Math.abs(z) < zIn - rib && x >= xb - 1e-6) && Math.hypot(pts[3][0] - pts[2][0], pts[3][1] - pts[2][1]) > 0.012 && inside(pts);
        if (ok && Math.hypot(pts[0][0] - pts[5][0], pts[0][1] - pts[5][1]) > 0.012) holes.push(roundPts(pts, 0.005));
      }
    }
    flatPart(outline, holes, 0.005, blackAl, root, wallTop);
  }

  // ---- intake box: slides out on its racks (at the start of the match) and stays out
  const ext = new THREE.Group();
  root.add(ext);
  const xF = L / 2 + BUMPER_T + cfg.intake.reach - extLen; // front face, stowed
  // the box's side panels and racks run outboard of the hopper walls and the pinion strips
  const sideZ = D4.BOX.side + 0.002, rackZ = W / 2 + 0.012, boxBack = xF - 0.33, bw = sideZ + 0.002;
  const chinY = D4.BOX.chin, I4 = D4.INTAKE;
  {
    // front panel (carbon fiber, the renders), with a bent chin at the bottom over the roller
    mesh(new THREE.BoxGeometry(0.004, wallTop - chinY, 2 * bw), carbonDark, ext, xF - 0.006, (wallTop + chinY) / 2, 0);
    const chin = bx(0.004, 0.05, 2 * bw - 0.02, carbonDark, ext, xF - 0.022, chinY - 0.021, 0);
    chin.rotation.z = -0.72;
    bx(0.006, 0.005, 2 * bw - 0.01, blackAl, ext, xF - 0.006, chinY, 0); // bend line
    for (let j = 0; j < 14; j++) for (const y of [wallTop - 0.012, chinY + 0.014]) bolts.add(new THREE.Vector3(xF - 0.003, y, -bw + 0.03 + j * (2 * bw - 0.06) / 13), new THREE.Vector3(1, 0, 0));
    // side panels (smoked polycarbonate): full height at the front, stepping down in a curve toward
    // the back, where the box slides in under the robot's top (the renders); cut away at the
    // bottom to clear the bumper
    const sp = new THREE.Shape(), low = wallTop - 0.075;
    sp.moveTo(0, 0.17); sp.lineTo(0.12, 0.17);
    sp.quadraticCurveTo(0.2, 0.17, 0.235, 0.06);
    sp.lineTo(0.33, 0.035); sp.lineTo(0.33, wallTop); sp.lineTo(0.2, wallTop);
    sp.bezierCurveTo(0.16, wallTop, 0.165, low, 0.12, low); sp.lineTo(0, low); sp.closePath();
    const spGeo = new THREE.ExtrudeGeometry(sp, { depth: 0.004, bevelEnabled: false });
    for (const s of [-1, 1]) {
      mesh(spGeo, smoke, ext, boxBack, 0, s * sideZ - 0.002).castShadow = false;
      // a bent black angle along the front corner, the front panel and side panel bolted to it
      bx(0.003, wallTop - 0.07, 0.025, blackAl, ext, xF + 0.001, (wallTop + 0.07) / 2, s * (bw - 0.012));
      bx(0.025, wallTop - 0.07, 0.003, blackAl, ext, xF - 0.012, (wallTop + 0.07) / 2, s * (bw + 0.001));
      for (let j = 0; j < 7; j++) {
        bolts.add(new THREE.Vector3(xF + 0.003, 0.09 + j * 0.045, s * (bw - 0.012)), new THREE.Vector3(1, 0, 0));
        bolts.add(new THREE.Vector3(xF - 0.012, 0.09 + j * 0.045, s * (bw + 0.003)), new THREE.Vector3(0, 0, s));
      }
      // rack along the bottom of the box (it runs back through the pinion plate)
      bx(0.52, 0.012, 0.01, blackAl, ext, xF - 0.3, 0.182, s * rackZ);
      const teeth = new THREE.InstancedMesh(new THREE.BoxGeometry(0.006, 0.008, 0.012), blackAl, 60);
      const m4 = new THREE.Matrix4();
      for (let j = 0; j < 60; j++) { m4.makeTranslation(xF - 0.555 + j * 0.0085, 0.192, s * rackZ); teeth.setMatrixAt(j, m4); }
      ext.add(teeth);
      // the kicker's drive (the renders): a Kraken X60 inside each side panel over it, its pulley
      // and the belt down to the kicker's big pulley outside the panel
      const mx = xF + I4.primary.dx - 0.04, my = I4.primary.y + 0.13, px = xF + I4.primary.dx, py = I4.primary.y, bz = s * (sideZ + 0.012);
      kraken(ext, mx, my, s * (sideZ - 0.045), false, 'z');
      cylZ(0.016, 0.014, blackAl, ext, mx, my, bz, 16);
      cylZ(0.036, 0.014, blackAl, ext, px, py, bz, 28);
      cylZ(0.012, 0.018, M.alu, ext, px, py, bz + s * 0.004, 12);
      const bl = Math.hypot(px - mx, py - my), ba = Math.atan2(py - my, px - mx), nx = -Math.sin(ba), ny = Math.cos(ba);
      for (const [r0, r1, k] of [[0.016, 0.036, 1], [0.016, 0.036, -1]]) {
        const b = bx(bl, 0.004, 0.012, M.polyBelt, ext, (mx + px) / 2 + (k * nx * (r0 + r1)) / 2, (my + py) / 2 + (k * ny * (r0 + r1)) / 2, bz);
        b.rotation.z = ba + k * Math.asin((r1 - r0) / bl);
      }
      // the black roller sitting on the bumper's top, flush with its face, belted down to the kick
      // wheel just out from the corner of the bumper and the carpet (the team), a black plate along
      // the belt with a pulley at each end, the X44 driving them
      const tx = xF + I4.bar.dx, ty = I4.bar.y, kx = xF + I4.kick.dx, ky = I4.kick.y;
      const ll = Math.hypot(tx - kx, ty - ky), la = Math.atan2(ty - ky, tx - kx);
      bx(ll, 0.022, 0.006, blackAl, ext, (kx + tx) / 2, (ky + ty) / 2, s * (sideZ + 0.024)).rotation.z = la;
      for (const k of [1, -1]) bx(ll, 0.003, 0.01, M.polyBelt, ext, (kx + tx) / 2 - k * Math.sin(la) * 0.013, (ky + ty) / 2 + k * Math.cos(la) * 0.013, bz).rotation.z = la;
      for (const [x, y] of [[tx, ty], [kx, ky]]) {
        cylZ(0.013, 0.012, blackAl, ext, x, y, bz, 16);
        cylZ(0.006, 0.016, M.alu, ext, x, y, s * (sideZ + 0.03), 10);
      }
      kraken(ext, tx + 0.02, ty + 0.09, s * (sideZ - 0.04), true, 'z');
    }
  }
  // the primary 3in polycarbonate roller under the chin, on big printed flanged hubs, the black
  // roller on the bumper and the kick wheel (where the physical model has them: js/mech/r4414.js)
  const intakeRoller = new THREE.Group();
  intakeRoller.position.set(xF + I4.primary.dx, I4.primary.y, 0);
  ext.add(intakeRoller);
  // the kicker (the primary 3in roller, low in front of the bumper's face): a clear 1/8in-wall
  // polycarbonate tube, the hex shaft and the printed hubs inside it showing through
  const clearTube = polyMat({ color: 0xe9eef3, opacity: 0.5, roughness: 0.12 }, 0xc9d0d8);
  cylZ(I4.primary.r, 2 * sideZ - 0.03, clearTube, intakeRoller, 0, 0, 0, 28).castShadow = false;
  cylZ(0.0064, 2 * sideZ - 0.01, M.alu, intakeRoller, 0, 0, 0, 6); // 1/2in hex shaft
  for (const s of [-1, 1]) for (const zz of [0.02, 0.25]) cylZ(I4.primary.r - 0.003, 0.014, plastic, intakeRoller, 0, 0, s * (sideZ - zz), 6);
  // the small roller sitting on top of the bumper, flush with its face (the renders: dark, in short
  // segments on its shaft)
  const bottomBar = new THREE.Group();
  bottomBar.position.set(xF + I4.bar.dx, I4.bar.y, 0);
  ext.add(bottomBar);
  cylZ(0.005, 2 * sideZ - 0.01, M.alu, bottomBar, 0, 0, 0, 6);
  for (let i = 0; i < 24; i++) cylZ(I4.bar.r, (2 * sideZ - 0.05) / 24 - 0.004, std(0x2a2d33, 0.7, 0.05), bottomBar, 0, 0, -sideZ + 0.025 + (i + 0.5) * (2 * sideZ - 0.05) / 24, 14);
  // the kick wheel, a powered breaker bar just out from the corner of the bumper's face and the
  // carpet (dark compliant segments, as the black roller's)
  const kickBar = new THREE.Group();
  kickBar.position.set(xF + I4.kick.dx, I4.kick.y, 0);
  ext.add(kickBar);
  cylZ(0.005, 2 * sideZ - 0.01, M.alu, kickBar, 0, 0, 0, 6);
  for (let i = 0; i < 24; i++) cylZ(I4.kick.r, (2 * sideZ - 0.05) / 24 - 0.004, std(0x2a2d33, 0.7, 0.05), kickBar, 0, 0, -sideZ + 0.025 + (i + 0.5) * (2 * sideZ - 0.05) / 24, 14);
  // the two guard tubes over the primary roller, free on bearings (the isolated intake render)
  for (const g of I4.guards) {
    const tube = new THREE.Group();
    tube.position.set(xF + g.dx, g.y, 0);
    ext.add(tube);
    cylZ(I4.guardR, 2 * sideZ - 0.03, std(0xb9bec6, 0.45, 0.3), tube, 0, 0, 0, 16);
    for (const s of [-1, 1]) cylZ(I4.guardR * 0.7, 0.008, M.alu, tube, 0, 0, s * (sideZ - 0.01), 12);
  }
  // (no floor: the box's bottom is open, the renders)
  // (a star roller over the bumper: not in their binder, so not drawn)
  const star = new THREE.Group();
  star.position.set(L / 2, 0.345, 0);
  star.visible = false;
  root.add(star);
  cylZ(0.006, 2 * wallZ - 0.01, M.alu, star, 0, 0, 0, 8);
  for (let i = 0; i < 22; i++) {
    const z = -wallZ + 0.04 + i * (2 * wallZ - 0.08) / 21;
    const w = cylZ(0.018, 0.012, M.compliant, star, 0, 0, z, 6);
    w.rotation.y = i * 0.5;
  }
  // pinion strips (SRPP) on the outside of the side walls: the rack's pinion, driven by a Kraken
  // X44 inside the wall
  const stripX0 = 0.085, stripX1 = 0.195;
  for (const s of [-1, 1]) {
    const pp = new THREE.Shape();
    pp.moveTo(0, 0); pp.lineTo(stripX1 - stripX0, 0); pp.lineTo(stripX1 - stripX0, wallTop - hY);
    pp.lineTo(0, wallTop - hY); pp.lineTo(0, 0.04); pp.quadraticCurveTo(0, 0, 0.04, 0);
    const h = new THREE.Path(); h.absarc(0.055, 0.058, 0.01, 0, Math.PI * 2, true); pp.holes.push(h);
    mesh(new THREE.ExtrudeGeometry(pp, { depth: 0.005, bevelEnabled: false }), carbon, root, stripX0, hY, s * (W / 2 + 0.0035) - 0.0025);
    for (let j = 0; j < 8; j++) for (const x of [stripX0 + 0.012, stripX1 - 0.012]) bolts.add(new THREE.Vector3(x, hY + 0.05 + j * 0.03, s * (W / 2 + 0.006)), new THREE.Vector3(0, 0, s));
    const pin = spurGear(0.018, 12, 0.01, M.copper, root);
    pin.rotation.x = Math.PI / 2;
    pin.position.set(stripX0 + 0.055, 0.206, s * (rackZ + 0.005));
    kraken(root, stripX0 + 0.055, 0.206, s * (W / 2 - 0.04), true, 'z');
  }

  // ---- Dye Rotor
  const rotor = new THREE.Group();
  rotor.position.set(rs.x, rs.y, rs.z);
  root.add(rotor);
  // the floor under the rotor: pocketed spokes, a center hole round the hub, skinned with thin
  // smoked poly (fixed: the hook turns over it)
  {
    const s = new THREE.Shape();
    s.absarc(0, 0, rs.r, 0, Math.PI * 2, false);
    const hub = new THREE.Path(); hub.absarc(0, 0, 0.078, 0, Math.PI * 2, true); s.holes.push(hub);
    const n = 12;
    for (let i = 0; i < n; i++) {
      for (const [r0, r1] of [[0.095, 0.17], [0.183, rs.r - 0.03]]) {
        const gap = 0.008;
        const a0 = (i / n) * Math.PI * 2 + gap / r0 + (r0 > 0.15 ? Math.PI / n : 0), a1 = a0 + (Math.PI * 2) / n - 2 * gap / r0;
        const h = new THREE.Path();
        h.absarc(0, 0, r1, a0 + gap / r1 - gap / r0, a1 - gap / r1 + gap / r0, false);
        h.absarc(0, 0, r0, a1, a0, true);
        s.holes.push(h);
      }
    }
    const g = new THREE.ExtrudeGeometry(s, { depth: 0.006, bevelEnabled: false, curveSegments: 10 });
    g.rotateX(-Math.PI / 2);
    g.translate(0, -0.006, 0);
    mesh(g, darkPlate, rotor);
    const skin = mesh(new THREE.RingGeometry(0.08, rs.r - 0.004, 64, 1), clearSmoke, rotor, 0, 0.0015, 0);
    skin.rotation.x = -Math.PI / 2;
    skin.castShadow = false;
    // rim band and bolts
    mesh(new THREE.CylinderGeometry(rs.r, rs.r, 0.012, 72, 1, true), blackAl, rotor, 0, 0.0, 0);
    const rb = boltSet(rotor, std(0x9ea3aa, 0.35, 0.8), 0.0045);
    for (let i = 0; i < 36; i++) { const a = (i / 36) * Math.PI * 2; rb.add(new THREE.Vector3(Math.cos(a) * (rs.r - 0.015), 0.002, Math.sin(a) * (rs.r - 0.015)), up); }
    for (let i = 0; i < 12; i++) { const a = (i / 12) * Math.PI * 2; rb.add(new THREE.Vector3(Math.cos(a) * 0.088, 0.002, Math.sin(a) * 0.088), up); }
    rb.done();
    // the rotor's big 10DP gear under the center (36.4:1 from the motor)
    spurGear(0.12, 72, 0.008, blackAl, rotor, 8).position.y = -0.008;
  }
  // base plate and the fixed ring wall round the rotor, on standoffs: the bowl's wall (js/mech/r4414.js
  // STADIUM, the physical model's), standing up from the floor to the stadium's edge, lower toward
  // the front; the hook's tip runs flush against it
  flatPart(arcPts(rs.x, rs.z, rs.r + 0.03, 0, Math.PI * 2, 48), [], 0.005, darkPlate, root, rs.y - 0.03);
  {
    const S4 = D4.STADIUM, pos = [], idx = [];
    for (let i = 0; i <= 96; i++) {
      const a = (i / 96) * Math.PI * 2, x = D4.AXIS[0] + S4.r * Math.cos(a), z = D4.AXIS[1] + S4.r * Math.sin(a);
      pos.push(x, D4.FLOOR - 0.006, z, x, D4.stadiumY(x + 1e-4 * Math.cos(a), z + 1e-4 * Math.sin(a)), z);
      if (i) { const k = 2 * i; idx.push(k - 2, k, k + 1, k - 2, k + 1, k - 1); }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    mesh(g, smoke, root).castShadow = false;
  }
  for (let i = 0; i < 16; i++) {
    const a = (i / 16) * Math.PI * 2;
    cylY(0.004, 0.06, blackAl, root, rs.x + Math.cos(a) * (rs.r + 0.02), rs.y - 0.01, rs.z + Math.sin(a) * (rs.r + 0.02), 6);
  }
  // rotor motors (Kraken X60s) standing round the rotor's side under the stadium, belted to the gear
  // train (between the swerve modules' plates and motors, and clear of the battery)
  for (const a of [-1.31, -1.52, -1.73]) {
    const x = rs.x + 0.34 * Math.cos(a), z = rs.z + 0.34 * Math.sin(a);
    kraken(root, x, 0.13, z);
    bx(0.03, 0.006, 0.02, M.led, root, x + 0.02, 0.166, z);
  }
  // printed "stadium" pieces that funnel FUEL down to the bowl's wall: the physical model's own
  // slope (D4.stadiumY), just under it, so the FUEL drawn on them sit on what's drawn (a point
  // inside the rim takes the rim's height, so the edge meets the wall's top)
  {
    const NX = 64, NZ = 76, x0 = back, x1 = bay.x1, z0 = -wallZ, z1 = wallZ;
    const hAt = (x, z) => {
      const dx = x - D4.AXIS[0], dz = z - D4.AXIS[1], d = Math.hypot(dx, dz), k = d < D4.STADIUM.r + 1e-4 ? (D4.STADIUM.r + 1e-4) / (d || 1) : 1;
      return D4.stadiumY(D4.AXIS[0] + dx * k, D4.AXIS[1] + dz * k) - 0.002;
    };
    const inBowl = (x, z) => Math.hypot(x - D4.AXIS[0], z - D4.AXIS[1]) < D4.STADIUM.r;
    const inside = (x, z) => (x - x0) - Math.abs(z) + wallZ - chamfer >= -0.005;
    const pos = [], idx = [], map = new Map();
    const vid = (i, j) => {
      const key = i * 1000 + j;
      if (map.has(key)) return map.get(key);
      const x = x0 + ((x1 - x0) * i) / NX, z = z0 + ((z1 - z0) * j) / NZ;
      pos.push(x, Math.max(rs.y - 0.005, hAt(x, z)), z);
      map.set(key, pos.length / 3 - 1);
      return pos.length / 3 - 1;
    };
    for (let i = 0; i < NX; i++) {
      for (let j = 0; j < NZ; j++) {
        const xa = x0 + ((x1 - x0) * (i + 0.5)) / NX, za = z0 + ((z1 - z0) * (j + 0.5)) / NZ;
        if (inBowl(xa, za) || !inside(xa, za)) continue;
        const a = vid(i, j), b = vid(i + 1, j), c = vid(i + 1, j + 1), d = vid(i, j + 1);
        idx.push(a, d, c, a, c, b);
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    mesh(g, std(0x40454d, 0.75, 0.05), root);
  }
  // center column: a cage of four legs under the turret bearing, wrapped low down in the
  // "shrink wrap" sheet so FUEL slides off it onto the rotor
  const colTop = wallTop - 0.012;
  flatPart(arcPts(col.x, col.z, col.r + 0.02, 0, Math.PI * 2, 40), [arcPts(col.x, col.z, col.r - 0.005, 0, Math.PI * 2, 40)], 0.008, blackAl, root, colTop);
  for (let i = 0; i < 4; i++) {
    const a = Math.PI / 4 + (i * Math.PI) / 2;
    const top = new THREE.Vector3(col.x + Math.cos(a) * (col.r + 0.005), colTop - 0.008, col.z + Math.sin(a) * (col.r + 0.005));
    const bot = new THREE.Vector3(col.x + Math.cos(a) * 0.062, rs.y - 0.03, col.z + Math.sin(a) * 0.062);
    const leg = rod(root, top, bot, 0.012, blackAl, 6);
    leg.scale.set(1, 1, 0.5);
  }
  // the Dye Rotor's moving part (js/mech/r4414.js, the physical model's own geometry): its hook,
  // its face a row of little rollers on bearings, backed by the Dolphin Fin, curving in to the hub's
  // side; across
  // the way in from its end the stack (the TTB wheels, the 3in omni over them), the 1.25in omnis
  // across the entrance, the channel in through the hub's side, all turning together over the fixed
  // floor
  const rotorParts = new THREE.Group();
  rotorParts.position.set(D4.AXIS[0], 0, D4.AXIS[1]);
  root.add(rotorParts);
  const H4 = D4.HOOK, F4 = D4.FEED;
  const hookPts = [];
  for (let i = 0; i <= 24; i++) { const { r, a } = D4.hookPlate(i / 24); hookPts.push([r * Math.cos(a), r * Math.sin(a)]); }
  for (const y of [H4.y0, H4.y1]) flatRibbon(hookPts, 0.02, y, blackAl, rotorParts);
  const rollerMat = std(0x3b3f45, 0.5, 0.1);
  const hookRollers = D4.hookRollerAt().map(([x, y, z]) => cylY(H4.rollers.r, H4.y1 - H4.y0 - 0.006, rollerMat, rotorParts, x - D4.AXIS[0], y, z - D4.AXIS[1], 10));
  {
    // the Dolphin Fin: a ramp from the hook's top edge down behind it to the floor
    const pos = [], idx = [];
    for (let i = 0; i <= 24; i++) {
      const { r, a } = D4.hookPlate(i / 24), b = a - H4.finBack / r;
      pos.push(r * Math.cos(a), H4.y1, r * Math.sin(a), r * Math.cos(b), D4.FLOOR + 0.004, r * Math.sin(b));
      if (i) { const k = 2 * i; idx.push(k - 2, k, k + 1, k - 2, k + 1, k - 1); }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    mesh(g, std(0x7a818b, 0.45, 0.3, { side: THREE.DoubleSide }), rotorParts);
  }
  // the stack: the TTB wheels (orange-tan urethane on copper hubs) and the 3in omni over them
  const ttbSpin = [];
  const omni3 = (parent, y) => {
    cylY(F4.ttb.r, F4.ttb.wid, std(0x2a2d33, 0.6, 0.1), parent, 0, y, 0, 24);
    for (let k = 0; k < 10; k++) { const a = (k * Math.PI) / 5; cylY(0.007, 0.016, std(0x8d939b, 0.5, 0.2), parent, Math.cos(a) * F4.ttb.r, y, Math.sin(a) * F4.ttb.r, 6).rotation.set(0, -a, Math.PI / 2); }
  };
  {
    const tw = FUEL.radius + F4.ttb.r - F4.ttb.squeeze;
    const [x, , z] = D4.chanL(F4.ttb.u, 0, tw);
    const axle = new THREE.Group();
    axle.position.set(x, 0, z);
    rotorParts.add(axle);
    ttbSpin.push(axle);
    for (const y of F4.ttb.ys) cylY(F4.ttb.r, F4.ttb.wid, std(0xc9874a, 0.6, 0.05), axle, 0, y, 0, 24);
    cylY(0.006, F4.ttb.omniY - D4.FLOOR, M.alu, axle, 0, (F4.ttb.omniY + D4.FLOOR) / 2, 0, 8);
    omni3(axle, F4.ttb.omniY);
  }
  // the 1.25in omnis on their shaft across the bend, and the channel's walls in the hub
  const bendAxle = new THREE.Group();
  {
    const [x, , z] = D4.chanL(D4.BEND.u, 0, 0);
    bendAxle.position.set(x, D4.BEND.y, z);
    bendAxle.rotation.y = -D4.CH; // (its local x across the channel)
    rotorParts.add(bendAxle);
    const spin = new THREE.Group();
    bendAxle.add(spin);
    cylZ(0.004, 2 * F4.w, M.alu, spin, 0, 0, 0, 6);
    // (a row of small omni wheels, each with its rollers round its rim)
    for (let i = 0; i < 10; i++) {
      const z = -F4.w + (i + 0.5) * (2 * F4.w) / 10;
      cylZ(F4.omni.r * 0.7, 0.012, std(0x2a2d33, 0.6, 0.1), spin, 0, 0, z, 10);
      for (let k = 0; k < 6; k++) { const a = (k * Math.PI) / 3; cylZ(0.0045, 0.01, std(0x8d939b, 0.5, 0.2), spin, Math.cos(a) * F4.omni.r * 0.75, Math.sin(a) * F4.omni.r * 0.75, z, 6).rotation.z = a; }
    }
    bendAxle.userData.spin = spin;
  }
  // (the leading one cut where the stack's wheels reach through it, as js/mech/r4414.js bore())
  const st4 = D4.stackAt(), cutU = F4.ttb.u - Math.sqrt(F4.ttb.r ** 2 - (st4.w - F4.w) ** 2) - F4.ttb.window.gap, wy4 = F4.ttb.window.y1;
  for (const [w, u0, u1, y0] of [[F4.w, -D4.BEND.u, cutU, D4.FLOOR], [F4.w, cutU, D4.COLUMN.r, wy4], [-F4.w, -D4.BEND.u, D4.COLUMN.r, D4.FLOOR]]) {
    const [x0, , z0] = D4.chanL(u0, 0, w), [x1, , z1] = D4.chanL(u1, 0, w);
    const len = Math.hypot(x1 - x0, z1 - z0);
    const plate = bx(len, D4.BORE.top - y0, 0.004, darkPlate, rotorParts, (x0 + x1) / 2, (D4.BORE.top + y0) / 2, (z0 + z1) / 2);
    plate.rotation.y = -Math.atan2(z1 - z0, x1 - x0);
  }
  // the hub's skin (the Shrink Wrap), slotted where the channel comes in (and closed over it above
  // the FUEL), with the window the stack's wheels poke out through beside the slot (as
  // js/mech/r4414.js column())
  {
    const C = D4.COLUMN, slot = Math.asin(Math.min(1, (F4.w + 0.005) / C.r)), win = D4.stackAt().win, wy = F4.ttb.window.y1;
    // (an arc from a0 to a1, rad round from +x toward +z as the physics has them; three's cylinder
    // runs its theta from +z toward +x)
    const arc = (a0, a1, y0, y1) => {
      const g = new THREE.CylinderGeometry(C.r, C.r, y1 - y0, Math.max(4, Math.ceil((a1 - a0) * 8)), 1, true, Math.PI / 2 - a1, a1 - a0);
      mesh(g, clearSmoke, rotorParts, 0, (y0 + y1) / 2, 0).castShadow = false;
    };
    arc(D4.CH + win, D4.CH + Math.PI * 2 - slot, D4.FLOOR, C.top);
    arc(D4.CH + slot, D4.CH + win, wy, C.top);
    arc(D4.CH - slot, D4.CH + slot, D4.YB + FUEL.radius + 0.01, C.top);
    // (the window's frame: a thin dark lip round its edges)
    const [x0, z0] = [C.r * Math.cos(D4.CH + win), C.r * Math.sin(D4.CH + win)];
    bx(0.006, wy - D4.FLOOR, 0.006, darkPlate, rotorParts, x0, (wy + D4.FLOOR) / 2, z0);
    const lip = new THREE.CylinderGeometry(C.r + 0.002, C.r + 0.002, 0.006, 8, 1, true, Math.PI / 2 - (D4.CH + win), win - slot);
    mesh(lip, darkPlate, rotorParts, 0, wy, 0);
  }

  // ---- turret: bearing ring in the top plate, the shooter in it where the physical model has it
  // (js/mech/r4414.js SHOOTER, FLY: its x along the shot): the 3in flywheel (four stealth wheels,
  // copper disks at its ends, the binder) just ahead of the axis, the fixed backing low behind it,
  // and the hood, coaxial with it (the team), its two plates swinging up over it with the hood
  // roller at their apex, the wheel opposite the flywheel (three sushi wheels on one shaft), belted
  // up the plates from stubs geared off the flywheel's shaft (the renders); the Limelight sunk in
  // just ahead of the flywheel; the four X44s down the column
  const turret = new THREE.Group();
  turret.position.set(tx, wallTop, tz);
  root.add(turret);
  {
    // The turret's top plate (the binder's Turret renders): round, its opening a rounded square the
    // shooter's side plates come up through (tighter corners at the front, where they reach out to
    // the flywheel), notched at the front for the Limelight sunk in just ahead of the flywheel; a
    // row of short slots round its rim and long ones inside it where the plate is wide (the sides
    // and back), bolts between the rim's slots
    const R0 = 0.198, x0 = -0.125, x1 = 0.165, hz = 0.12, rb = 0.05, rf = 0.03, nz = 0.056, xn = 0.178;
    const open = [
      ...arcPts(x1 - rf, hz - rf, rf, -Math.PI / 2, 0, 6), [x1, nz], [xn, nz], [xn, -nz], [x1, -nz],
      ...arcPts(x1 - rf, -(hz - rf), rf, 0, Math.PI / 2, 6),
      ...arcPts(x0 + rb, -(hz - rb), rb, Math.PI / 2, Math.PI, 8),
      ...arcPts(x0 + rb, hz - rb, rb, Math.PI, 1.5 * Math.PI, 8),
    ];
    // (how far a point is outside the opening: the rounded square's distance, the notch's box)
    const sdOpen = (x, z) => {
      const r = x > 0 ? rf : rb, cx = (x0 + x1) / 2, qx = Math.abs(x - cx) - ((x1 - x0) / 2 - r), qz = Math.abs(z) - (hz - r);
      const box = Math.hypot(Math.max(qx, 0), Math.max(qz, 0)) + Math.min(Math.max(qx, qz), 0) - r;
      const notch = Math.max(Math.abs(x - (x1 + xn) / 2) - (xn - x1) / 2, Math.abs(z) - nz);
      return Math.min(box, notch);
    };
    const clear = (pts, m) => pts.every(([x, z]) => sdOpen(x, z) > m);
    const holes = [open];
    const tb = boltSet(turret, std(0xc4c8ce, 0.3, 0.9));
    for (let k = 0; k < 36; k++) {
      const a = (k / 36) * Math.PI * 2, da = 3 / 57.3;
      const sl = [...arcPts(0, 0, 0.181, a - da, a + da, 3), ...arcPts(0, 0, 0.191, a + da, a - da, 3)];
      if (clear(sl, 0.007)) holes.push(roundPts(sl, 0.0045, 2));
      const ab = a + Math.PI / 36, bp = [Math.cos(ab) * 0.186, -Math.sin(ab) * 0.186];
      if (k % 2 === 0 && clear([bp], 0.01)) tb.add(new THREE.Vector3(bp[0], 0.008, bp[1]), up);
    }
    for (let k = 0; k < 12; k++) {
      const a = (k / 12) * Math.PI * 2 + Math.PI / 12, da = 12 / 57.3;
      const sl = [...arcPts(0, 0, 0.152, a - da, a + da, 6), ...arcPts(0, 0, 0.166, a + da, a - da, 6)];
      if (clear(sl, 0.01)) holes.push(roundPts(sl, 0.006, 2));
    }
    tb.done();
    const ring = flatPart(arcPts(0, 0, R0, 0, Math.PI * 2, 64), holes, 0.006, darkPlate, turret, 0.008);
    ring.castShadow = true;
    // the column under the shooter, down inside the hub's skin to the bore's top (it turns with
    // the turret, not the rotor): a ring up to the sushi feeder, then open at the front where the
    // feeder's wheels reach in (js/mech/r4414.js turret())
    const S4f = D4.SHOOTER.feeder, cr = D4.BORE.r + 0.006, f0 = S4f.ys[0] - S4f.r - 0.003, c1 = D4.COLUMN.top + 0.04;
    const cm = std(0x23262b, 0.5, 0.4, { side: THREE.DoubleSide });
    mesh(new THREE.CylinderGeometry(cr, cr, f0 - D4.BORE.top, 32, 1, true), cm, turret, 0, (f0 + D4.BORE.top) / 2 - wallTop, 0);
    // (three's cylinder: theta from +z toward +x; the window centered on +x, the shot's way)
    mesh(new THREE.CylinderGeometry(cr, cr, c1 - f0, 32, 1, true, Math.PI / 2 + 0.55, 2 * Math.PI - 1.1), cm, turret, 0, (c1 + f0) / 2 - wallTop, 0);
  }
  const S4 = D4.SHOOTER, F4s = D4.FLY, ax = S4.x, ay = F4s.y - wallTop, pz = S4.plateZ;
  const at = (r, deg) => [ax + r * Math.cos((deg * Math.PI) / 180), ay + r * Math.sin((deg * Math.PI) / 180)];
  const body = new THREE.Group();
  turret.add(body);
  // side plates (pocketed) carrying the flywheel's axle, down to the hub's top (the column goes on
  // down inside it)
  const pb = D4.COLUMN.top + 0.01 - wallTop;
  for (const sz of [-1, 1]) {
    const sp = new THREE.Shape();
    sp.moveTo(-0.11, pb); sp.lineTo(ax + 0.06, pb); sp.lineTo(ax + 0.06, ay + 0.02);
    sp.quadraticCurveTo(ax + 0.06, ay + 0.06, ax, ay + 0.06); sp.lineTo(-0.11, ay + 0.03); sp.closePath();
    for (const [hx, hy] of [[-0.07, -0.125], [0.0, -0.13], [-0.065, -0.06]]) { const h = new THREE.Path(); h.absarc(hx, hy, 0.016, 0, Math.PI * 2, true); sp.holes.push(h); }
    const g = new THREE.ExtrudeGeometry(sp, { depth: 0.005, bevelEnabled: false });
    mesh(g, blackAl, body, 0, 0, sz * (pz + 0.03) - 0.0025);
  }
  // the flywheel: four 3in stealth wheels on a hex shaft, copper disks at its ends, the gear that
  // drives it outside a side plate
  const fly = new THREE.Group();
  fly.position.set(ax, ay, 0);
  body.add(fly);
  const ww = F4s.w / 4;
  for (let i = 0; i < 4; i++) {
    cylZ(F4s.r, ww - 0.003, M.compliant, fly, 0, 0, -F4s.w / 2 + (i + 0.5) * ww, 24);
    for (let k = 0; k < 6; k++) { const a = (k * Math.PI) / 3; bx(0.004, F4s.r * 0.8, ww - 0.002, std(0x1b1d21, 0.6, 0.1), fly, 0, 0, -F4s.w / 2 + (i + 0.5) * ww).rotation.z = a; }
  }
  for (const sz of [-1, 1]) for (const k of [0, 1]) cylZ(F4s.r - 0.002, 0.006, M.copper, fly, 0, 0, sz * (F4s.w / 2 + 0.004 + k * 0.007), 24);
  cylZ(0.0064, 2 * (pz + 0.04), M.alu, fly, 0, 0, 0, 6);
  { const gr = spurGear(0.03, 30, 0.01, M.copper, body); gr.rotation.x = Math.PI / 2; gr.position.set(ax, ay, pz + 0.042); }
  // the hood roller's drive (Shooter PT): a ~30T 20DP gear on the flywheel's shaft at each end,
  // inside the side plates, turning with it (dz: where these gears and the belts run, between the
  // hood's plates and the side plates)
  const G30 = 0.75 * IN, G20 = 0.5 * IN, dz = pz + 0.0155;
  for (const sz of [-1, 1]) { const gr = spurGear(G30, 30, 0.005, M.copper, fly, 3); gr.rotation.x = Math.PI / 2; gr.position.set(0, 0, sz > 0 ? dz + 0.005 : -dz); }
  // the fixed backing low behind the flywheel (its printed arc)
  {
    const g = new THREE.CylinderGeometry(S4.fixedR + 0.003, S4.fixedR + 0.003, 2 * pz, 16, 1, true, 0, ((S4.fixedA[1] - S4.fixedA[0]) * Math.PI) / 180);
    const b = mesh(g, std(0x2c2f35, 0.6, 0.1, { side: THREE.DoubleSide }), body, ax, ay, 0);
    // (three's cylinder: angle from +z toward +x about y; turned to lie round the axle in x-y, its
    // sweep from fixedA[0])
    b.rotation.x = Math.PI / 2;
    b.rotation.y = Math.PI / 2 + (S4.fixedA[0] * Math.PI) / 180;
  }
  // the sushi feeder: three 1in wheels midway up the column's front side, in its window (the
  // bottom two a softer compound, the top one harder: lighter here), each shaft's gear down the
  // column's side from the flywheel's
  const fx = -D4.BORE.r + 2 * FUEL.radius - 0.5 * IN + S4.feeder.r;
  S4.feeder.ys.forEach((y, i) => {
    cylZ(S4.feeder.r, S4.feeder.w, i < 2 ? M.compliant : std(0x6b7380, 0.55, 0.1), body, fx, y - wallTop, 0, 14);
    cylZ(0.004, S4.feeder.w + 0.03, M.alu, body, fx, y - wallTop, 0, 6);
    const g = spurGear(0.011, 14, 0.004, M.copper, body); g.rotation.x = Math.PI / 2; g.position.set(fx, y - wallTop, S4.feeder.w / 2 + 0.012);
  });
  // the hood: built at its stow angle, turning about the flywheel's axle
  const hood = new THREE.Group();
  hood.position.set(ax, ay, 0);
  body.add(hood);
  const h0 = S4.hoodStow, h1 = h0 + S4.hoodSpan, rIn = S4.hoodR, rOut = S4.hoodR + 0.045;
  // the hood roller's place: at the hood's tip (where it lets FUEL go), 1in into the FUEL
  const rr = S4.rollerR, raD = h0 + S4.roller.at, ra = (raD * Math.PI) / 180;
  // the stub each side: carried on the hood's plate, its 20T meshed with the flywheel's ~30T (so it
  // rolls round that gear as the hood swings), its GT2 pulley under the belt up the plate's edge
  // to the roller's (sa: the stub's angle on the hood, from the renders: ahead of the flywheel's top
  // when the hood is up)
  const sd = G30 + G20, sa = ((raD - 80) * Math.PI) / 180, sx = sd * Math.cos(sa), sy = sd * Math.sin(sa), rp = 0.0115;
  for (const sz of [-1, 1]) {
    // a plate each side, a pocketed sector (the renders' truss): from round the axle out along the
    // belt's edge to the roller at its apex, round the arc behind (the sector gear's, doubled 1/8in
    // 7075: the binder) and back in; the hood's backing between them
    const ap = new THREE.Shape(), n = 16, a0 = raD - 3, a1 = h1 + 8;
    ap.moveTo(0.026 * Math.cos(((a0 - 90) * Math.PI) / 180), 0.026 * Math.sin(((a0 - 90) * Math.PI) / 180));
    ap.lineTo(sx + 0.016 * Math.cos(sa - Math.PI / 2), sy + 0.016 * Math.sin(sa - Math.PI / 2));
    for (let i = 0; i <= n; i++) { const a = ((a0 + ((a1 - a0) * i) / n) * Math.PI) / 180; ap.lineTo(rOut * Math.cos(a), rOut * Math.sin(a)); }
    ap.lineTo(0.026 * Math.cos(((a1 + 90) * Math.PI) / 180), 0.026 * Math.sin(((a1 + 90) * Math.PI) / 180));
    ap.absarc(0, 0, 0.026, ((a1 + 90) * Math.PI) / 180, ((a0 + 270) * Math.PI) / 180, false);
    for (const [hr, hd, hs] of [[0.5, 22, 0.014], [0.72, 14, 0.012], [0.72, 32, 0.012], [0.5, 46, 0.012], [0.82, 50, 0.009]]) { const a = ((raD + hd) * Math.PI) / 180, rm = hr * rOut; const h = new THREE.Path(); h.absarc(rm * Math.cos(a), rm * Math.sin(a), hs, 0, Math.PI * 2, true); ap.holes.push(h); }
    mesh(new THREE.ExtrudeGeometry(ap, { depth: 0.005, bevelEnabled: false }), blackAl, hood, 0, 0, sz * (pz + 0.012) - 0.0025);
  }
  {
    // (the printed backing, from just short of the roller back over the flywheel)
    const b0 = S4.roller.at + S4.roller.gap, g = new THREE.CylinderGeometry(rIn + 0.002, rIn + 0.002, 2 * pz, 20, 1, true, 0, ((S4.hoodSpan - b0) * Math.PI) / 180);
    const b = mesh(g, std(0x2c2f35, 0.6, 0.1, { side: THREE.DoubleSide }), hood, 0, 0, 0);
    b.rotation.x = Math.PI / 2;
    b.rotation.y = Math.PI / 2 + ((h0 + b0) * Math.PI) / 180;
  }
  // the hood roller: three 1in sushi wheels side by side on its hex shaft across the plates, the
  // wheel opposite the flywheel (its face runs with the FUEL, at half the flywheel's surface speed)
  const hoodRoller = new THREE.Group();
  hoodRoller.position.set(rr * Math.cos(ra), rr * Math.sin(ra), 0);
  hood.add(hoodRoller);
  for (let i = 0; i < S4.roller.n; i++) {
    const w = cylZ(S4.roller.r, S4.roller.seg - 0.002, M.compliant, hoodRoller, 0, 0, (i - (S4.roller.n - 1) / 2) * S4.roller.seg, 16);
    w.rotation.y = i * 0.7; // (the segments' facets out of step: they read as separate wheels)
  }
  cylZ(0.0064, 2 * (dz + 0.008), M.alu, hoodRoller, 0, 0, 0, 6);
  for (const sz of [-1, 1]) bx(0.003, S4.roller.r * 1.6, 0.002, std(0x1b1d21, 0.6, 0.1), hoodRoller, 0, 0, sz * (S4.roller.seg / 2)); // (the joints)
  // the stubs (they turn against the flywheel: one mesh), the belts up the plates (two strands each,
  // over the GT2 pulleys)
  const stubs = [];
  for (const sz of [-1, 1]) {
    const st = new THREE.Group();
    st.position.set(sx, sy, 0);
    hood.add(st);
    stubs.push(st);
    { const gr = spurGear(G20, 20, 0.005, M.copper, st, 0); gr.rotation.x = Math.PI / 2; gr.position.set(0, 0, sz > 0 ? dz + 0.005 : -dz); }
    const zb = sz * (dz + 0.0085);
    for (const p of [st, hoodRoller]) cylZ(rp, 0.006, M.alu, p, 0, 0, zb, 14);
    const a = new THREE.Vector3(sx, sy, zb), b = new THREE.Vector3(rr * Math.cos(ra), rr * Math.sin(ra), zb);
    const d = new THREE.Vector3().subVectors(b, a).normalize(), nrm = new THREE.Vector3(-d.y, d.x, 0).multiplyScalar(rp);
    for (const k of [-1, 1]) rod(hood, a.clone().addScaledVector(nrm, k), b.clone().addScaledVector(nrm, k), 0.003, M.polyBelt, 6);
  }
  // the four X44s, 1:1 to the flywheel, down the column's side, their gears up to its gear
  for (let i = 0; i < 4; i++) kraken(body, -0.03 - (i % 2) * 0.055, ay - 0.06 - Math.floor(i / 2) * 0.065, pz + 0.07, true, 'z');
  { const gr = spurGear(0.024, 24, 0.01, M.copper, body); gr.rotation.x = Math.PI / 2; gr.position.set(-0.03, ay - 0.06, pz + 0.042); }
  // the Limelight (one lens, the finned face: a Limelight 4, by the renders): just ahead of the
  // flywheel, centered across the shooter, sunk into the notch in the turret's top plate, its top
  // about flush with it, looking out along the shot and a little up (ShooterV10, V11, Turret PT)
  limelight(body, ax + F4s.r + 0.03, -0.014, 0, 0, { tilt: 0.75, ll4: true });
  // the energy chain from the left of the robot round the tensioner, back round the compliant wheel
  // and on round the guides at the shooter's corners (js/echainModel.js; the links where js/echain.js
  // puts them for the turret's yaw)
  const echain = chain4414(root, turret, wallTop, M);

  // ---- net over the top: pinned to the wall tops, the top plate's front edge and the turret
  // ring. It drapes over the load and stretches up where the FUEL bulges it (the hopper physics
  // lets the load bulge it as far as bay.dome allows)
  const net = hopperNet(root, {
    x0: bay.dome.x0 - 0.02, x1: xF - 0.006, x1Out: xF - 0.006 + extLen, xFixed: L / 2,
    hw: wallZ, y: wallTop + 0.004, hole: { x: tx, z: tz, r: bay.dome.rHole - 0.005 },
  });
  bolts.done();

  const anim = (st, dt) => {
    // the intake box slides out on its racks (and stays out)
    const ex = st.hopperDeploy * extLen;
    ext.position.x = ex;
    intakeRoller.rotation.z -= st.intakeSpeed * dt * 45; // pulls FUEL in up the ramp
    bottomBar.rotation.z -= st.intakeSpeed * dt * 90;
    kickBar.rotation.z -= st.intakeSpeed * dt * 90;
    star.rotation.z += st.intakeSpeed * dt * 35;
    // the Dye Rotor's hook and feeder turn over the fixed floor (clockwise from above: its angle
    // from the physical model), the TTB wheels spin while it feeds
    rotorParts.rotation.y = -(st.rotorTurn ?? 0);
    for (const r of hookRollers) r.rotation.y += st.feeding * dt * 25;
    bendAxle.userData.spin.rotation.z += st.feeding * dt * 144; // (2.4 times the stack's speed)
    for (const w of ttbSpin) w.rotation.y += st.feeding * dt * 60; // (the stack turns about +y: its face toward the channel runs in)
    turret.rotation.y = st.turretYaw;
    echain.update(st.turretYaw);
    fly.rotation.z -= st.flywheel * dt * 10;
    // (the stubs and the roller against it, 1.5 times as fast: the ~30:20 mesh, 1:1 belts)
    hoodRoller.rotation.z += st.flywheel * dt * 15;
    for (const s of stubs) s.rotation.z += st.flywheel * dt * 15;
    hood.rotation.z = st.hoodTurn ?? 0; // (its turn from stow, the physical model's)
    net.drape(st.load, ex);
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 8793 Hopperless
// From 8793's Onshape CAD ("8793-2026-A-0000 Robot"), split by tools/extract-parts.mjs
// (cad/robots/8793.json): the body (drivetrain, Conveyor V2's omni wheels over a polycarbonate
// ramp, and the fixed turret tower), Intake V3's mount and its arm (exported down on the carpet;
// origin on the pivot, it folds up and back by intake.fold.stowDeg to stow), and the turret (200T
// turntable with the hooded 4in flywheel shooter; origin on the turntable axis). Holds only the
// FUEL in its path. Until the CAD loads (and headless), a drawn stand-in shows.
function build8793(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, fold = cfg.intake.fold;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const orange = std(cfg.colors.accent, 0.45, 0.35);
  const plate = std(0x2a2c31, 0.5, 0.6);
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  addElectronics(drawn, -0.12, -W / 2 + 0.2, Math.PI / 2);

  // electronics deck
  rbx(L - 0.12, 0.008, W - 0.12, 0.003, std(0x1b1c20, 0.6, 0.4), drawn, -0.02, 0.172, 0);
  for (const s of [-1, 1]) rbx(L * 0.45, 0.01, 0.16, 0.003, orange, drawn, -0.14, 0.18, s * (W / 2 - 0.14));

  // conveyor: funnel of compliant wheels from the intake back to the turret
  const conv = new THREE.Group();
  conv.position.set(0.1, 0.19, 0);
  drawn.add(conv);
  const convWheels = [];
  for (let row = 0; row < 3; row++) {
    const w = W - 0.18 - row * 0.1;
    const n = 6 - row;
    const stack = wheelStack(conv, w, 0.035, n, M.compliant, 0.12 - row * 0.1, 0.02 + row * 0.03, 0, 0.03);
    convWheels.push(stack);
  }
  for (const s of [-1, 1]) pocketPlate(conv, 0.36, 0.12, 0.006, orange, [[-0.1, 0, 0.025], [0.02, 0, 0.025], [0.13, 0, 0.02]], 0.02, 0.05, s * (W / 2 - 0.08));
  kraken(conv, 0.02, 0.05, W / 2 - 0.05, true, 'z');

  // turret: big bearing ring at the back with the hooded shooter
  const turret = new THREE.Group();
  turret.position.set(cfg.shooter.turretPos.x, 0.25, cfg.shooter.turretPos.z);
  root.add(turret);
  mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.02, 48), std(0x2a2d33, 0.45, 0.5), turret, 0, 0, 0);
  mesh(new THREE.TorusGeometry(0.215, 0.009, 8, 60), M.alu, turret, 0, 0.012, 0).rotation.x = Math.PI / 2;
  const body = new THREE.Group();
  turret.add(body);
  for (const s of [-1, 1]) pocketPlate(body, 0.3, 0.2, 0.008, orange, [[-0.09, 0.02, 0.03], [0.02, -0.03, 0.03], [0.1, 0.04, 0.025]], 0, 0.11, s * 0.1);
  const fly = wheelStack(body, 0.16, 0.05, 2, M.orangeWheel, 0.02, 0.1, 0, 0.05);
  const hood = new THREE.Group();
  hood.position.set(0.02, 0.1, 0);
  body.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.19, 20, 1, true, 0, Math.PI * 0.6), std(0x1b1d22, 0.45, 0.5, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (let i = 0; i < 5; i++) cylZ(0.022, 0.03, M.compliant, hood, -Math.cos(0.5 + i * 0.3) * 0.12, Math.sin(0.5 + i * 0.3) * 0.12, 0, 10);
  for (const s of [-1, 1]) kraken(body, 0.02, 0.1, s * 0.13, false, 'z');
  for (let i = 0; i < 7; i++) mesh(new THREE.SphereGeometry(0.007, 8, 6), M.led, body, 0.09 + i * 0.012, 0.16, 0.108);
  limelight(body, -0.15, 0.2, 0, Math.PI);

  // Intake V3: multi-roller over-bumper intake
  const intake = new THREE.Group();
  intake.position.set(L / 2 - 0.03, 0.28, 0);
  root.add(intake);
  const { armLen, deploy } = intakeArm(cfg, L / 2 - 0.03, 0.28, 0.024);
  for (const s of [-1, 1]) pocketPlate(intake, armLen + 0.04, 0.07, 0.008, plate, [[-0.1, 0, 0.018], [0.02, 0, 0.018], [0.12, 0, 0.015]], armLen / 2, -0.01, s * (cfg.intake.width / 2 + 0.02));
  const intakeRollers = [];
  for (let i = 0; i < 3; i++) intakeRollers.push(wheelStack(intake, cfg.intake.width, 0.024, 1, M.black, armLen - i * 0.075, -0.02 + i * 0.03, 0, cfg.intake.width - 0.02));
  wheelStack(intake, cfg.intake.width - 0.06, 0.035, 7, M.compliant, armLen - 0.19, 0.03, 0, 0.025);
  kraken(intake, 0.03, 0, cfg.intake.width / 2 + 0.05, true, 'z');

  // the team's CAD
  const cadTurret = new THREE.Group();
  cadTurret.position.set(cfg.shooter.turretPos.x, 0, cfg.shooter.turretPos.z);
  root.add(cadTurret);
  const cadArm = new THREE.Group();
  cadArm.position.set(fold.pivot[0], fold.pivot[1], 0);
  root.add(cadArm);
  // The wheels in the ball path, drawn from the physical model (js/mech/r8793.js): the mecanums
  // and the compliant wheel on the conveyor's shafts, the vertical rollers (a 4in omni on the left,
  // 4in 45A compliants on the right) and the neck's (a 4in omni and a 4in compliant). The CAD's
  // are out of date (2in compliant wheels where the robot has mecanums, a 5in and a 3in omni at
  // the neck, omnis on both vertical rollers), so they're cut out of it as it loads.
  const pathWheels = conveyorWheels8793(trains8793()).filter((wh) => !/shaft$/.test(wh.name));
  const spinning = [];
  const wheelsUp = new THREE.Group();
  root.add(wheelsUp);
  for (const wh of pathWheels) {
    const kind = wh.kind === 'plain' ? 'compliant' : wh.kind;
    const g = rolledWheel(kind, wh.radius, wh.width, wh.mecanum);
    const holder = new THREE.Group();
    holder.position.set(...wh.center);
    holder.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(...wh.axis));
    holder.add(g);
    wheelsUp.add(holder);
    spinning.push({ g, k: wh.shaft.k * (wh.shaft.train ? 1 : 0) });
  }
  // what to cut from the CAD: anything inside a path wheel's cylinder (the hex shafts and hubs
  // stay); the neck's CAD wheels were bigger (5in on the left)
  const cutters = pathWheels.map((wh) => ({
    c: new THREE.Vector3(...wh.center), a: new THREE.Vector3(...wh.axis),
    r: (/^neck/.test(wh.name) ? Math.max(wh.radius, 2.5 * IN) : wh.radius) + 0.006, hw: wh.width / 2 + 0.004,
  }));
  const inWheel = (p, cut) => {
    const d = p.clone().sub(cut.c), along = d.dot(cut.a), rad = d.addScaledVector(cut.a, -along).length();
    return Math.abs(along) < cut.hw && rad > 0.012 && rad < cut.r;
  };
  const scrap = new THREE.Group();
  scrap.visible = false;
  root.add(scrap);
  const cutCad = (scene, parent, list) => moveCadTris(scene, parent, (c) => list.some((cut) => inWheel(c, cut)), scrap);
  // (the pivot shaft's wheels are in the intake's CAD, about its own origin: the arm turns about them)
  const pivotCuts = cutters.filter((_, i) => /^pivot/.test(pathWheels[i].name)).map((cut) => ({ ...cut, c: cut.c.clone().sub(new THREE.Vector3(fold.pivot[0], fold.pivot[1], 0)) }));
  cadPart(root, 'robots/8793-body.glb', (sc) => { drawn.visible = false; cutCad(sc, root, cutters); });
  cadPart(root, 'robots/8793-mount.glb', (sc) => cutCad(sc, root, cutters));
  cadPart(cadArm, 'robots/8793-intake.glb', (sc) => { intake.visible = false; cutCad(sc, cadArm, pivotCuts); });
  cadPart(cadTurret, 'robots/8793-turret.glb', () => { turret.visible = false; });
  // the energy chain's track (fixed on the tower: tools/split-8793-track.py took it out of the
  // turret's file) and the chain in it (js/echainModel.js)
  const cadTrack = new THREE.Group();
  cadTrack.position.set(cfg.shooter.turretPos.x, 0, cfg.shooter.turretPos.z);
  root.add(cadTrack);
  cadPart(cadTrack, 'robots/8793-track.glb', () => {});
  const echain = chain8793(root, [cfg.shooter.turretPos.x, cfg.shooter.turretPos.z]);

  // FUEL in the ball path (intake -> conveyor -> turret)
  const stored = [];
  const path = [[0.33, 0.13], [0.26, 0.16], [0.19, 0.2], [0.12, 0.24], [0.05, 0.28], [-0.02, 0.32]];
  for (let i = path.length - 1; i >= 0; i--) for (const z of [-0.08, 0.08]) stored.push(new THREE.Vector3(path[i][0] - 0.1, path[i][1] + 0.06, z));

  const anim = (st, dt) => {
    intake.rotation.z = lerp(STOWED, deploy, st.intakeDeploy);
    for (const r of intakeRollers) r.rotation.z += st.intakeSpeed * dt * 40;
    for (const w of convWheels) w.rotation.z += (st.intakeSpeed + st.feeding) * dt * 25;
    // (the path's wheels at a tenth of their real speed, the X60 at ~550 rad/s, so you can see them turn)
    for (const w of spinning) w.g.rotation.z += -st.intakeSpeed * 55 * w.k * dt;
    turret.rotation.y = st.turretYaw;
    // (where its pivot's motors and the FUEL under it have the arm: js/mech/phys8793.js)
    cadArm.rotation.z = st.armTurn ?? (1 - st.intakeDeploy) * fold.stowDeg * Math.PI / 180;
    cadTurret.rotation.y = st.turretYaw;
    echain.update(st.turretYaw);
    fly.rotation.z -= st.flywheel * dt * 8;
    hood.rotation.z = (st.hoodDeg - 62) * Math.PI / 180 * 0.8;
  };
  return { root, anim, stored, modules, extLen: 0 };
}

// ============================================================ 971 Mixtape
// From 971's public Onshape CAD ("2026 971 Robot Mixtape Public Release"), split into parts by
// tools/extract-parts.mjs (cad/robots/971.json): the body (drivetrain, roller floor, separator,
// kicker, ramps, turret platform, fixed hopper walls), the sliding front of the hopper, the 4-bar
// ground intake (stowed folded up inside the hopper, as exported) and a turret (drawn twice). A
// net covers the open top of the hopper.
const INTAKE_971 = { pivot: [0.217, 0.143], swing: -2.227 }; // beater from folded up to on the carpet
function build971(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const blackAl = std(0x2a2e35, 0.42, 0.55);
  const top = bay.top, hw = bay.hw + 0.005;
  // drawn stand-ins until the CAD arrives
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  for (const s of [-1, 1]) polyWall(drawn, 0.27, top - 0.05, 0.15, 0.05 + (top - 0.05) / 2, s * hw);
  polyWall(drawn, 2 * hw, top - 0.17, 0.03, 0.17 + (top - 0.17) / 2, 0, Math.PI / 2);
  const floorRollers = new THREE.Group();
  drawn.add(floorRollers);
  for (let x = 0.04; x < 0.3; x += 0.055) cylZ(0.016, 2 * hw - 0.02, M.alu, floorRollers, x, bay.floor.a + bay.floor.b * x - 0.018, 0, 10);
  rbx(0.32, 0.008, W - 0.02, 0.003, M.alu, drawn, -0.15, 0.28, 0); // turret platform
  // the hopper's sliding front: telescoping side plates and the front wall
  const slide = new THREE.Group();
  root.add(slide);
  const slideDrawn = new THREE.Group();
  slide.add(slideDrawn);
  const xFront = 0.606; // front wall, hopper out (as exported)
  for (const s of [-1, 1]) polyWall(slideDrawn, 0.47, top - 0.12, xFront - 0.235, 0.12 + (top - 0.12) / 2, s * (hw + 0.012));
  polyWall(slideDrawn, 2 * hw + 0.03, top - 0.12, xFront, 0.12 + (top - 0.12) / 2, 0, Math.PI / 2);
  // 4-bar ground intake
  const intake = new THREE.Group();
  intake.position.set(INTAKE_971.pivot[0], INTAKE_971.pivot[1], 0);
  root.add(intake);
  const intakeDrawn = new THREE.Group();
  intake.add(intakeDrawn);
  const beater = cylZ(0.028, cfg.intake.width, std(0x2b2b2e, 0.8, 0.1), intakeDrawn, -0.145, 0.27, 0, 14);
  for (const s of [-1, 1]) rod(intakeDrawn, new THREE.Vector3(0, 0, s * (cfg.intake.width / 2 + 0.01)), new THREE.Vector3(-0.145, 0.27, s * (cfg.intake.width / 2 + 0.01)), 0.012, blackAl);
  // two turrets
  const turrets = sh.turrets.map((t) => {
    const g = new THREE.Group();
    g.position.set(t.x, 0.327, t.z); // the bearings on the platform (the CAD floor is 0.043 m under its origin)
    root.add(g);
    const d = new THREE.Group();
    g.add(d);
    mesh(new THREE.TorusGeometry(0.13, 0.012, 8, 40), M.alu, d, 0, 0.005, 0).rotation.x = Math.PI / 2;
    for (const s of [-1, 1]) pocketPlate(d, 0.24, 0.2, 0.006, blackAl, [[-0.05, 0, 0.03], [0.05, 0.02, 0.03]], -0.02, 0.11, s * 0.075);
    wheelStack(d, 0.1, 0.05, 2, M.black, -0.06, 0.15, 0, 0.045);
    cadPart(g, 'robots/971-turret.glb', () => { d.visible = false; });
    return g;
  });
  cadPart(root, 'robots/971-body.glb', () => { drawn.visible = false; });
  cadPart(slide, 'robots/971-hopper.glb', () => { slideDrawn.visible = false; });
  cadPart(intake, 'robots/971-intake.glb', () => { intakeDrawn.visible = false; });
  // net over the open top, from the back wall to the sliding front wall
  const net = hopperNet(root, { x0: 0.035, x1: xFront - extLen - 0.01, x1Out: xFront - 0.01, xFixed: 0.27, hw, y: top + 0.004 });

  const anim = (st, dt) => {
    const ex = st.hopperDeploy * extLen;
    slide.position.x = ex - extLen; // exported with the hopper out
    intake.rotation.z = st.intakeDeploy * INTAKE_971.swing;
    beater.rotation.y += st.intakeSpeed * dt * 40;
    for (const r of floorRollers.children) r.rotation.y -= st.feeding * dt * 30;
    // the CAD's shooter fires out its back (over the flywheel, away from the hood): at yaw pi
    turrets.forEach((g, i) => { g.rotation.y = (st.turretYaws ? st.turretYaws[i] : st.turretYaw) - Math.PI; });
    net.drape(st.load, ex);
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 1678 Limestone
// From 1678's public Onshape CAD ("1678-26c-0000 CAD Release"), split by tools/extract-parts.mjs
// (cad/robots/1678.json): the body (drivetrain with its polycarbonate walls on the bumper mounts,
// roller floor, ball tunnel and drum shooter), the slapdown intake (exported folded up), the
// polycarbonate side plates of the horizontal extension (they slide out with the intake), and the
// climber with the corrugated lid, which lifts to make the hopper taller (its side skirts ride over
// the walls). A net stretches diagonally from the lid's front edge down to the extension's front.
// folded up inside the hopper (as exported) -> down 130deg (their code: stowed 130, deployed 0),
// where its crash bar stands square under the extension's front panel and its plates follow
// the panel's chamfer
// pivot: their code and AdvantageScope model (the extracted part's origin is 5.5 cm behind it)
const INTAKE_1678 = { pivot: [0.309, 0.169], origin: [0.254, 0.194], swing: -130 * Math.PI / 180 };
function build1678(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const blackAl = std(0x23262b, 0.42, 0.55);
  const lime = std(cfg.colors.accent, 0.5, 0.2);
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  const wallTop = 0.495, hw = W / 2 - 0.01;
  for (const s of [-1, 1]) polyWall(drawn, L - 0.02, wallTop - 0.15, 0, 0.15 + (wallTop - 0.15) / 2, s * hw);
  // ball tunnel and drum across the back
  for (const s of [-1, 1]) pocketPlate(drawn, 0.22, 0.42, 0.006, blackAl, [[0, 0.05, 0.05], [0, -0.1, 0.04]], -0.22, 0.3, s * 0.318);
  const drum = wheelStack(drawn, 0.6, 0.045, 1, M.black, -0.288, 0.477, 0, 0.58);
  for (let i = 0; i < 3; i++) cylZ(0.022, 0.6, M.black, drawn, -0.09 - i * 0.012, 0.43 + i * 0.035, 0, 12);
  for (let x = -0.05; x < 0.32; x += 0.06) cylZ(0.013, 0.58, M.black, drawn, x, bay.floor.a + bay.floor.b * x - 0.02, 0, 10);
  // horizontal extension: polycarbonate side plates that slide out with the intake
  const slide = new THREE.Group();
  root.add(slide);
  // lid on the climber
  const lift = new THREE.Group();
  root.add(lift);
  const lidDrawn = rbx(0.38, 0.01, 2 * hw, 0.003, std(0xe8e8e2, 0.8, 0), lift, 0.15, 0.54, 0);
  for (const s of [-1, 1]) rbx(0.02, 0.45, 0.02, 0.003, M.alu, lift, -0.02, 0.32, s * (hw - 0.015));
  // slapdown intake
  const intake = new THREE.Group();
  intake.position.set(INTAKE_1678.pivot[0], INTAKE_1678.pivot[1], 0);
  root.add(intake);
  const intakePart = new THREE.Group();
  intakePart.position.set(INTAKE_1678.origin[0] - INTAKE_1678.pivot[0], INTAKE_1678.origin[1] - INTAKE_1678.pivot[1], 0);
  intake.add(intakePart);
  const intakeDrawn = new THREE.Group();
  intakePart.add(intakeDrawn);
  const roller = cylZ(0.028, cfg.intake.width, lime, intakeDrawn, -0.156, 0.207, 0, 14);
  for (const s of [-1, 1]) rod(intakeDrawn, new THREE.Vector3(0, 0, s * 0.33), new THREE.Vector3(-0.156, 0.207, s * 0.33), 0.012, blackAl);
  cadPart(root, 'robots/1678-body.glb', () => { drawn.visible = false; });
  cadPart(slide, 'robots/1678-slide.glb', () => {});
  cadPart(lift, 'robots/1678-lift.glb', (s) => {
    lidDrawn.visible = false;
    // the climber's gearbox (motor, gears and their plate, down at the frame) came out merged
    // with the parts that rise: it stays put. The climber tubes telescope: the outer ones stay,
    // the inner ones (and the hooks, pulleys and lid) rise.
    moveCadTris(s, lift, (c, lo, hi) => hi.x < 0.025 && lo.x > -0.05 && lo.y > 0.04 && hi.y < 0.5 && hi.y - lo.y > 0.1, root, [0, 0, 0], true);
    moveCadTris(s, lift, (c, lo, hi) => hi.y < 0.2, root);
  });
  cadPart(intakePart, 'robots/1678-intake.glb', (s) => {
    intakeDrawn.visible = false;
    // a stray 5 cm block at the robot's origin (on the carpet) came out with the intake
    moveCadTris(s, intakePart, (c) => c.y + INTAKE_1678.origin[1] < 0.03 && Math.abs(c.x + INTAKE_1678.origin[0]) < 0.04, new THREE.Group());
    // the deploy gearbox (Kraken, gear plates, the stage on the old origin) came out with the
    // intake: it's mounted on the frame below the pivot, so it stays put
    const [ox, oy] = INTAKE_1678.origin;
    moveCadTris(s, intakePart, (c) => {
      const x = c.x + ox, y = c.y + oy;
      return y < 0.148 || (y < 0.205 && x < 0.293) || Math.hypot(x - ox, y - oy) < 0.03;
    }, root, [ox, oy, 0]);
  });
  // a net stretched diagonally from the front edge of the lid down to the front of the
  // extension (it rides out with it, and up with the lid)
  const lidX1 = bay.slope.x, H = bay.lift.h, lidY0 = 0.54;
  const net = hopperNet(root, { x0: lidX1, x1: lidX1 + 0.01, x1Out: lidX1 + extLen, xFixed: lidX1, hw, y: lidY0, yFront: bay.slope.y, NX: 12, NZ: 24 });

  const anim = (st, dt) => {
    slide.position.x = st.hopperDeploy * extLen;
    intake.rotation.z = st.armTurn ?? st.intakeDeploy * INTAKE_1678.swing;
    roller.rotation.y += st.intakeSpeed * dt * 40;
    drum.rotation.z += st.flywheel * dt * 8;
    // the lid rides up on the climber tubes (and all the way up to reach the rung for a climb)
    const up = st.climbState && st.climbState !== 'none' ? 0.26 : (st.lift ?? 0) * H;
    lift.position.y = up;
    const ex = st.hopperDeploy * extLen;
    net.mesh.visible = ex > 0.02;
    net.drape(st.load, Math.max(0, ex - 0.01), { y: lidY0 + up - 0.01 });
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 1690 Kepler
// 1690's swerve modules, from their CAD. Inverted: the X60 and X44 hang under the module's plate,
// their shafts up into the gear housing over it (in 1690-body with the plates). What steers is the
// fork (1690-fork) and the 4in wheel in it (1690-wheel), both exported from the back right module,
// origin on the axle's middle, on the steering axis; at each corner's drive.moduleAt (modulePoints).
// The CAD has the wheels' axles fore and aft (rolling sideways), and the front forks are the back
// ones turned half round (their holes), so pivot.rotation.y = pi / 2 is the CAD's pose at every
// corner. Returns the { pivot, wheel } robot.js steers and spins (the wheel about its local y).
function modules1690(root, cfg) {
  return modulePoints(cfg).map(([x, z]) => {
    const g = new THREE.Group();
    g.position.set(x, 0, z);
    root.add(g);
    const pivot = new THREE.Group();
    g.add(pivot);
    const psi = x > 0 ? Math.PI / 2 : -Math.PI / 2, axle = 0.0508;
    const fork = new THREE.Group();
    fork.position.y = axle;
    fork.rotation.y = psi;
    pivot.add(fork);
    // (the wheel turns about pivot's z, as moduleWheel's; the CAD in it turned back to the fork's pose)
    const spin = new THREE.Group();
    spin.position.y = axle;
    spin.rotation.x = Math.PI / 2;
    pivot.add(spin);
    const cad = new THREE.Group();
    cad.quaternion.setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0)).multiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(0, psi, 0)));
    spin.add(cad);
    // (drawn until the CAD loads)
    const wDrawn = new THREE.Group();
    spin.add(wDrawn);
    mesh(new THREE.CylinderGeometry(0.0508, 0.0508, 0.04, 24), M.tread, wDrawn);
    for (let i = 0; i < 6; i++) bx(0.008, 0.042, 0.006, M.hubGrey, wDrawn, Math.cos((i * Math.PI) / 3) * 0.02, 0, Math.sin((i * Math.PI) / 3) * 0.02);
    const fDrawn = new THREE.Group();
    fork.add(fDrawn);
    for (const s of [-1, 1]) bx(0.008, 0.05, 0.05, M.aluDark, fDrawn, s * 0.03, 0.02, 0);
    cadPart(fork, 'robots/1690-fork.glb', () => { fDrawn.visible = false; });
    cadPart(cad, 'robots/1690-wheel.glb', () => { wDrawn.visible = false; });
    return { pivot, wheel: spin };
  });
}

// From 1690's public X_T release (their CAD release thread's Drive folder), imported into Onshape and
// split by tools/extract-parts.mjs (cad/robots/1690.json): the body (the chassis and what's fixed of
// its inverted swerve modules (modules1690 steers their forks and wheels), the roller floor's
// plates, the belts' housing and backing plate, the kickers' frames, the turret's fixed deck and the
// energy chain's tray and spring, the electronics box), the hopper's polycarbonate side and back
// panels (1690-panels: clear; their aluminum corner brackets stay in the body) and its polycarbonate
// lattice top plate (1690-top, bowed by the load), the turret (its base and plates, the two X60s), the
// hood (its lattice arms and printed "hollow hood" and its actuator riding on them: the X44, its
// VersaPlanetary and bevel pair; turning about the 4in wheel's axle), the actuator's two 12T pinions
// (1690-pinions: they walk the racks on the turret's side plates as the hood turns), the intake's
// carriage (its polycarbonate lattice U-frame, clear; the black side bars, the rollers' motor and
// belts) and the kicker bar's arms. The
// X_T has no mates: the carriage slides out and down 12 deg (js/mech/r1690.js INTAKE.slide), the
// kicker bar falls on its pin, the hood turns about the wheel's axle. What turns is drawn here, at the
// CAD's axes, so it spins about them: the floor's rollers and the two over its back, the intake's
// rollers, the belts' pulleys, the vertical kicker, the bottom and top
// kickers' wheels, the 4in shooter wheel and the hood's wheels. The energy chain round the turret
// is js/echainModel.js (the X_T has its tray, spring, guide and swivel, not its links). Until the CAD loads
// (and at the Lowest graphics), a drawn stand-in.
function build1690(cfg, alliance) {
  const root = new THREE.Group();
  addBumpers(root, cfg, alliance);
  const modules = modules1690(root, cfg);
  const D = R1690, S = D.SHOOTER, F = S.feeder, I = D.INTAKE, [tx, tz] = D.TURRET.at;
  const green = std(0x62c94a, 0.7, 0.05), silicone = std(0x2a2c30, 0.75, 0.05), stripe = std(0x6a6f78, 0.6, 0.2);
  const beltMat = std(0x111214, 0.85, 0.05), tread = std(0x1f2022, 0.85, 0.05), lattice = std(cfg.colors.frame, 0.45, 0.55);
  // a roller along z at (x, y): a group on its axle with stripes along it, so it's seen to turn
  const rollerZ = (parent, x, y, z, r, len, mat) => {
    const g = new THREE.Group(); g.position.set(x, y, z); parent.add(g);
    cylZ(r, len, mat, g, 0, 0, 0, 16);
    for (let k = 0; k < 3; k++) { const a = (k * 2 * Math.PI) / 3; bx(0.005, 0.005, len - 0.01, stripe, g, r * Math.cos(a), r * Math.sin(a), 0).rotation.z = a; }
    return g;
  };
  // a wheel on a fore-aft (x) axle: spokes, so it's seen to turn
  const wheelX = (parent, x, y, z, r, w, mat) => {
    const g = new THREE.Group(); g.position.set(x, y, z); parent.add(g);
    cylX(r, w, mat, g, 0, 0, 0, 16);
    cylX(0.006, w + 0.03, M.alu, g, 0, 0, 0, 6);
    for (let k = 0; k < 6; k++) bx(w + 0.004, r * 1.6, 0.006, std(0x2d6f2a, 0.8, 0.05), g, 0, 0, 0).rotation.x = (k * Math.PI) / 6;
    return g;
  };

  // ---- the drawn stand-in (hidden when the body's CAD loads)
  const drawn = new THREE.Group();
  root.add(drawn);
  // (the frame, and drive.inverted's drawn modules' plates and motors at the CAD's axes)
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.removeFromParent());
  for (const z of [D.WALL.left, D.WALL.right]) polyWall(drawn, 0.62, D.WALL.top - 0.1, 0, 0.1 + (D.WALL.top - 0.1) / 2, z, 0, M.poly, lattice);
  bx(0.6, 0.02, 0.76, lattice, drawn, 0, D.WALL.top - 0.01, 0);
  bx(0.06, 0.18, 0.66, M.poly, drawn, D.BACK.x, 0.23, 0);
  addElectronics(drawn, -0.14, 0.19, Math.PI / 2);

  // ---- what turns, at the CAD's axes
  const floorRollers = D.FLOOR.at.map(([x, y], i) => rollerZ(root, x, y, (D.FLOOR.z0[i] + D.FLOOR.z1[i]) / 2, D.FLOOR.r, D.FLOOR.z1[i] - D.FLOOR.z0[i], silicone));
  const topRollers = D.TOP.at.map(([x, y]) => rollerZ(root, x, y, (D.TOP.z[0] + D.TOP.z[1]) / 2, D.TOP.r, D.TOP.z[1] - D.TOP.z[0], silicone));
  // (the belts: their loops round the pulleys on fore-aft axles; the pulleys turn)
  const pulleys = [];
  for (const run of [D.BELT.bottom, D.BELT.top]) {
    const r = D.BELT.r, w = D.BELT.x[1] - D.BELT.x[0], cx = (D.BELT.x[0] + D.BELT.x[1]) / 2, up = run === D.BELT.bottom ? -1 : 1;
    const yc = run.y + up * r, len = run.z[1] - run.z[0];
    for (const dy of [-r + 0.002, r - 0.002]) bx(w, 0.004, len, beltMat, root, cx, yc + dy, (run.z[0] + run.z[1]) / 2);
    for (const z of run.z) {
      const g = new THREE.Group(); g.position.set(cx, yc, z); root.add(g);
      cylX(r, w, beltMat, g, 0, 0, 0, 18); cylX(r * 0.6, w + 0.006, M.alu, g, 0, 0, 0, 12);
      for (let k = 0; k < 3; k++) bx(w + 0.008, 0.004, r * 1.1, stripe, g, 0, 0, 0).rotation.x = (k * Math.PI) / 3;
      pulleys.push(g);
    }
  }
  const vkick = new THREE.Group();
  vkick.position.set(D.VKICK.at[0], (D.VKICK.y[0] + D.VKICK.y[1]) / 2, D.VKICK.at[1]); root.add(vkick);
  cylY(D.VKICK.r, D.VKICK.y[1] - D.VKICK.y[0], M.alu, vkick, 0, 0, 0, 14);
  for (let k = 0; k < 3; k++) { const a = (k * 2 * Math.PI) / 3; bx(0.004, D.VKICK.y[1] - D.VKICK.y[0] - 0.01, 0.004, stripe, vkick, D.VKICK.r * Math.cos(a), 0, D.VKICK.r * Math.sin(a)); }
  const feedWheels = D.FEED.wheels.map(([z, y]) => wheelX(root, D.FEED.x, y, z, D.FEED.r, D.FEED.w, green));
  const topKick = D.FEED.top.map(([z, y]) => wheelX(root, D.FEED.x, y, z, D.FEED.topR, D.FEED.w, green));

  // ---- the turret: its CAD in the turret's frame (exported at YAW0), the 4in wheel, the hood
  const turretY = D.TURRET.deck;
  const turret = new THREE.Group();
  turret.position.set(tx, 0, tz);
  root.add(turret);
  const turretCad = new THREE.Group();
  turretCad.rotation.y = -D.YAW0;
  turret.add(turretCad);
  const tDrawn = new THREE.Group();
  turret.add(tDrawn);
  mesh(new THREE.TorusGeometry(D.TURRET.bore.r + 0.01, 0.008, 8, 40), M.alu, tDrawn, 0, turretY, 0).rotation.x = Math.PI / 2;
  for (const s of [-1, 1]) bx(0.3, 0.16, 0.006, lattice, tDrawn, -0.02, turretY + 0.1, s * 0.09);
  // (the 4in wheel on its axle, its two 42t gears, turning about the turret's z)
  const wheel = new THREE.Group();
  wheel.position.set(S.axle[0], S.axle[1], 0); turret.add(wheel);
  cylZ(S.r, S.w, tread, wheel, 0, 0, 0, 28);
  cylZ(S.r * 0.75, S.w + 0.004, M.alu, wheel, 0, 0, 0, 20);
  for (let k = 0; k < 3; k++) bx(0.006, S.r * 1.3, S.w + 0.006, M.aluDark, wheel, 0, 0, 0).rotation.z = (k * Math.PI) / 3;
  for (const s of [-1, 1]) { const gr = spurGear(0.0279, 42, 0.008, M.alu, wheel, 4); gr.rotation.x = Math.PI / 2; gr.position.z = s * (S.w / 2 + 0.012) + (s > 0 ? 0.008 : 0); }
  // (the hood: its CAD about the axle as exported at HOOD0, its feeder wheels on its arms)
  const hood = new THREE.Group();
  hood.position.set(S.axle[0], S.axle[1], 0); turret.add(hood);
  const hoodCad = new THREE.Group();
  hoodCad.rotation.y = -D.YAW0;
  hood.add(hoodCad);
  // (the hood's actuator rides on its arm: its X44, VersaPlanetary and bevel pair are in the hood's
  // CAD; the cross shaft's two 12T pinions (1690-pinions, origin on their axis, the 2in roller's) walk
  // the racks on the turret's side plates as the hood turns, turning on its arm the hood's turn times
  // the racks' pitch radius over theirs: SHOOTER.hoodRatio's 19.333 after the 4:1)
  const pinions = new THREE.Group();
  pinions.position.set(-0.2552 + 0.0701, 0.4235 - 0.464, -0.2412 + 0.1587);
  hoodCad.add(pinions);
  const pinionAxis = new THREE.Vector3(Math.sin(D.YAW0), 0, Math.cos(D.YAW0));
  const hoodArm = new THREE.Group();
  hoodArm.rotation.z = (D.HOOD0 * Math.PI) / 180;
  hood.add(hoodArm);
  const hDrawn = new THREE.Group();
  hoodArm.add(hDrawn);
  for (const s of [-1, 1]) bx(F.R + 0.03, 0.03, 0.006, lattice, hDrawn, F.R / 2, 0, s * 0.06);
  const feeder = [0, F.lower].map((da) => {
    const a = (da * Math.PI) / 180, g = new THREE.Group();
    g.position.set(F.R * Math.cos(a), F.R * Math.sin(a), 0); hoodArm.add(g);
    cylZ(F.r, F.w, da ? std(0x8a8f96, 0.5, 0.4) : green, g, 0, 0, 0, 16);
    for (let k = 0; k < 3; k++) bx(0.004, F.r * 1.6, F.w + 0.004, std(0x2d6f2a, 0.8, 0.05), g, 0, 0, 0).rotation.z = (k * Math.PI) / 3;
    return g;
  });
  limelight(turret, -0.08, turretY + 0.13, -0.1, Math.PI, { tilt: 0.3 });
  // the energy chain (js/echain.js SWIVEL1690: either way round the turret from its swivel)
  const echain = chain1690(root, turret, turretY, M);

  // ---- the intake: the carriage (slides out), its rollers, the kicker bar on its pin
  const carriage = new THREE.Group();
  root.add(carriage);
  const cDrawn = new THREE.Group();
  carriage.add(cDrawn);
  const FR = I.frame, x0 = FR.front - I.slide[0] - 0.33;
  for (const s of [-1, 1]) bx(0.33, 0.24, 0.006, lattice, cDrawn, x0 + 0.165 + 0.0, FR.top - I.slide[1] - 0.12, s * FR.sides);
  bx(0.006, 0.26, 2 * FR.sides, lattice, cDrawn, FR.front - I.slide[0], FR.top - I.slide[1] - 0.13, 0);
  const intakeRollers = I.rollers.map(([x, y, r]) => rollerZ(carriage, x, y, 0, r, I.width, silicone));
  const K = I.kick;
  const kick = new THREE.Group();
  kick.position.set(K.pivot[0], K.pivot[1], 0); root.add(kick);
  const kDrawn = new THREE.Group();
  kick.add(kDrawn);
  for (const s of [-1, 1]) rod(kDrawn, new THREE.Vector3(0, 0, s * 0.338), new THREE.Vector3(K.tube[0] - K.pivot[0], K.tube[1] - K.pivot[1], s * 0.338), 0.008, std(0x17181b, 0.5, 0.3));
  rollerZ(kick, K.tube[0] - K.pivot[0], K.tube[1] - K.pivot[1], 0, K.tube[2], K.width, M.alu);

  // ---- the hopper's polycarbonate: its side and back panels (1690-panels) and the lattice top
  // plate (1690-top, 2 mm), which the load bows up (js/mech/r1690.js PLATE: mp.bow at its free front
  // edge's middle, bowShape over it): its triangles moved into the plate's group unquantized, so
  // their corners can follow the bow
  cadPart(root, 'robots/1690-panels.glb', () => {});
  const plate = new THREE.Group();
  root.add(plate);
  const plateParts = [];
  let plateBow = 0;
  cadPart(plate, 'robots/1690-top.glb', (sc) => {
    plate.updateMatrixWorld(true);
    const inv = new THREE.Matrix4().copy(plate.matrixWorld).invert();
    const meshes = [];
    sc.traverse((o) => { if (o.isMesh) meshes.push(o); });
    for (const o of meshes) {
      const g = o.geometry.clone(), src = g.attributes.position, a = new Float32Array(src.count * 3);
      for (let i = 0; i < src.count; i++) a.set([src.getX(i), src.getY(i), src.getZ(i)], i * 3);
      g.setAttribute('position', new THREE.BufferAttribute(a, 3));
      g.deleteAttribute('normal');
      g.applyMatrix4(new THREE.Matrix4().multiplyMatrices(inv, o.matrixWorld));
      g.computeVertexNormals();
      o.geometry = g;
      o.position.set(0, 0, 0); o.quaternion.identity(); o.scale.set(1, 1, 1);
      plate.add(o);
      plateParts.push({ g, base: a.slice() });
    }
    plate.remove(sc);
    plateBow = -1;
  });
  const bowPlate = (bow) => {
    if (Math.abs(bow - plateBow) < 0.0005) return;
    plateBow = bow;
    for (const { g, base } of plateParts) {
      const a = g.attributes.position.array;
      for (let i = 0; i < a.length; i += 3) a[i + 1] = base[i + 1] + bow * D.bowShape(base[i], base[i + 2]);
      g.attributes.position.needsUpdate = true;
      g.computeVertexNormals();
    }
  };
  // ---- the net out over the U-frame (their CAD has none: js/mech/r1690.js NET), from the top
  // plate's front edge to the U-frame's top edges
  const net = hopperNet(root, { x0: D.HALF_L, x1: D.HALF_L, x1Out: D.NET.x1, xFixed: D.HALF_L, hw: D.NET.hw, y: D.NET.y + 0.004, yFront: FR.top + 0.004, NX: 16, NZ: 30, slack: D.NET.hang });

  // ---- the CAD
  cadPart(root, 'robots/1690-body.glb', () => { drawn.visible = false; });
  cadPart(turretCad, 'robots/1690-turret.glb', () => { tDrawn.visible = false; });
  cadPart(hoodCad, 'robots/1690-hood.glb', () => { hDrawn.visible = false; hoodArm.remove(hDrawn); });
  cadPart(pinions, 'robots/1690-pinions.glb', () => {});
  cadPart(carriage, 'robots/1690-carriage.glb', () => { cDrawn.visible = false; });
  cadPart(kick, 'robots/1690-kick.glb', () => { kDrawn.visible = false; });

  const anim = (st, dt) => {
    const mp = st.mech;
    // (the carriage: mp.ext of the way out; the kicker bar's fall)
    const ext = mp ? mp.ext : st.intakeDeploy;
    carriage.position.set(I.slide[0] * ext, I.slide[1] * ext, 0);
    kick.rotation.z = mp ? mp.kick : (R1690.kickAt(ext) * Math.PI) / 180;
    // (each part turns about its own axle at its shaft's speed: the rollers' shafts at -1 times the
    // intake's train; the floor's about +z, the rollers over it the other way; the belts' pulleys about
    // -x (their runs toward the FUEL go left); the vertical kicker about -y; the kickers' wheels about
    // their fore-aft axles; the shooter wheel about the turret's -z and the feeder wheels about +z)
    for (const r of intakeRollers) r.rotation.z -= (mp ? mp.intakeRollers : st.intakeSpeed * 300) * dt;
    floorRollers.forEach((r, i) => { r.rotation.z += (mp ? mp.floor * D.FLOOR.k[i] : st.feeding * 30) * dt; });
    topRollers.forEach((r, i) => { r.rotation.z += (mp ? mp.floor * D.TOP.k[i] : -st.feeding * 30) * dt; });
    // (the bottom belt's pulleys are under its run, the top belt's over its: they turn opposite ways)
    pulleys.forEach((p, i) => { p.rotation.x += (i < 2 ? -1 : 1) * (mp ? mp.belts[i >> 1] / D.BELT.pitch : st.feeding * 60) * dt; });
    vkick.rotation.y -= (mp ? mp.vkick : st.feeding * 60) * dt;
    feedWheels.forEach((w) => { w.rotation.x -= (mp ? mp.feed : st.feeding * 60) * dt; });
    topKick.forEach((w, i) => { w.rotation.x += (i ? 1 : -1) * (mp ? mp.feed : st.feeding * 60) * dt; });
    turret.rotation.y = st.turretYaw;
    echain.update(st.turretYaw);
    wheel.rotation.z -= (mp ? mp.wheel : st.flywheel * 9) * dt;
    const h = mp ? mp.hood : ((st.hoodDeg + 90) * Math.PI) / 180;
    hood.rotation.z = h - (D.HOOD0 * Math.PI) / 180;
    pinions.quaternion.setFromAxisAngle(pinionAxis, hood.rotation.z * (S.hoodRatio / 4));
    for (const w of feeder) w.rotation.z += (mp ? mp.feeder : st.flywheel * 6) * dt;
    const bow = mp?.bow ?? 0;
    bowPlate(bow);
    net.drape(st.load, Math.max(0, ext * I.slide[0] - 0.01), { y: D.NET.y + 0.004 + (2 / Math.PI) * bow });
  };
  return { root, anim, stored: [], modules, extLen: cfg.storage.extLen };
}

// ============================================================ 4946 Moto Moto
// 4946's Onshape document is view-only (no export), so this is drawn from Onshape's scaled renders
// of it and their engineering report: a round robot (half-circle drivetrain, 30in front edge) with
// clear hopper walls on its bumpers, a Dye Rotor tray whose spinner carries FUEL round to the
// center, the turret on the center of rotation (a tall column with a flared shroud and the shooter
// in a bearing ring on top), a gantry of perforated tube over the front with the beacon, and an
// intake box that slides out the front. A net stretches diagonally from the top of the hopper's
// front wall down to the front of the intake box.
function build4946(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen, ch = bay.chamfer;
  addBumpers(root, cfg, alliance, ch);
  const modules = addDrivebase(root, cfg, ch);
  // the modules sit under the Dye Rotor tray: nothing of them shows above it
  root.updateMatrixWorld(true);
  for (const md of modules) {
    md.pivot.parent.traverse((o) => {
      if (o.isMesh && new THREE.Box3().setFromObject(o).max.y > cfg.bay.rotor.y - 0.012) o.visible = false;
    });
  }
  const grey = std(0x9aa0a8, 0.4, 0.7);
  const dark = std(0x24262b, 0.5, 0.45);
  const trayMat = std(0xa9bccb, 0.5, 0.4);
  const green = std(0x5fb94a, 0.75, 0.05);
  const bolts = boltSet(root, std(0xc4c8ce, 0.3, 0.9));
  const top = bay.top, y0 = 0.17;
  const t = sh.turretPos;

  // ---- clear walls on the bumpers, following the frame round the back
  const out = 0.025;
  // on the bumpers, 1.125in out from the frame (a 35in hopper), cut at the front of the hopper
  const pts = offsetShape(frameShape(cfg), 0.029);
  const xFront = bay.x1 + 0.005;
  pts[0][0] = pts[pts.length - 1][0] = xFront;
  for (let i = 0; i < pts.length; i++) {
    const [xa, za] = pts[i], [xb, zb] = pts[(i + 1) % pts.length];
    const len = Math.hypot(xb - xa, zb - za);
    const g = new THREE.Group();
    g.position.set((xa + xb) / 2, 0, (za + zb) / 2);
    g.rotation.y = -Math.atan2(zb - za, xb - xa);
    root.add(g);
    const pane = mesh(new THREE.PlaneGeometry(len, top - y0), M.poly, g, 0, (y0 + top) / 2, 0);
    pane.castShadow = false;
    rbx(len, 0.014, 0.014, 0.002, grey, g, 0, top, 0);
    // posts at the ends of the straight walls and every few segments round the curve
    if (len > 0.15 || i % 6 === 0) rbx(0.014, top - y0, 0.014, 0.002, grey, g, len / 2, (y0 + top) / 2, 0);
  }

  // ---- Dye Rotor tray: a round plate under the load, with the spinner's vanes on it
  const rs = bay.rotor;
  mesh(new THREE.CylinderGeometry(rs.r + 0.01, rs.r + 0.01, 0.012, 48), trayMat, root, rs.x, rs.y - 0.006, rs.z);
  for (let i = 0; i < 16; i++) {
    const a = (i / 16) * Math.PI * 2;
    bolts.add(new THREE.Vector3(rs.x + Math.cos(a) * (rs.r - 0.02), rs.y + 0.001, rs.z - Math.sin(a) * (rs.r - 0.02)), new THREE.Vector3(0, 1, 0));
  }
  // the tray's ring of interlocking printed walls (their CAD render): its inner face slopes
  // steeply down onto the platter's edge from the hopper walls; flat on top out to the walls
  {
    const f = bay.funnel, NX = 120, NZ = 140, xa = -0.5, xb = xFront + 0.01, za = -bay.hw - 0.03, zb = bay.hw + 0.03;
    const r1 = rs.r + 0.008; // its lip on the platter
    const hAt = (x, z) => rs.y + Math.min(f.cap, (Math.hypot(x - rs.x, z - rs.z) - rs.r) * f.slope);
    const inside = (x, z) => {
      if (x > xFront) return false;
      let c = false;
      for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
        const [xi, zi] = pts[i], [xj, zj] = pts[j];
        if ((zi > z) !== (zj > z) && x < ((xj - xi) * (z - zi)) / (zj - zi) + xi) c = !c;
      }
      return c;
    };
    // the nearest point on the walls (with the front cut at xFront)
    const edge = (x, z) => {
      let best = [x, z], bd = Infinity;
      const segs = pts.map((q, i) => [q, pts[(i + 1) % pts.length]]);
      for (const [[ax, az], [bx2, bz]] of segs) {
        const dx = bx2 - ax, dz = bz - az, l2 = dx * dx + dz * dz || 1;
        const u = Math.max(0, Math.min(1, ((x - ax) * dx + (z - az) * dz) / l2));
        let px = ax + u * dx, pz = az + u * dz;
        px = Math.min(px, xFront);
        const d = Math.hypot(px - x, pz - z);
        if (d < bd) { bd = d; best = [px, pz]; }
      }
      return best;
    };
    const pos = [], idx = [], map = new Map();
    const vid = (i, j) => {
      const k = i * 10000 + j;
      if (!map.has(k)) {
        let x = xa + ((xb - xa) * i) / NX, z = za + ((zb - za) * j) / NZ;
        if (!inside(x, z)) [x, z] = edge(x, z); // a clean edge along the walls
        const d = Math.hypot(x - rs.x, z - rs.z);
        if (d < r1) { x = rs.x + ((x - rs.x) * r1) / d; z = rs.z + ((z - rs.z) * r1) / d; } // and on the lip
        pos.push(x, hAt(x, z) + 0.002, z);
        map.set(k, pos.length / 3 - 1);
      }
      return map.get(k);
    };
    for (let i = 0; i < NX; i++) {
      for (let j = 0; j < NZ; j++) {
        const xs = [i, i + 1].map((q) => xa + ((xb - xa) * q) / NX), zs = [j, j + 1].map((q) => za + ((zb - za) * q) / NZ);
        const corners = [[xs[0], zs[0]], [xs[1], zs[0]], [xs[1], zs[1]], [xs[0], zs[1]]];
        if (!corners.some(([x, z]) => inside(x, z))) continue;
        if (corners.every(([x, z]) => Math.hypot(x - rs.x, z - rs.z) < r1)) continue;
        const a = vid(i, j), b = vid(i + 1, j), c = vid(i + 1, j + 1), d = vid(i, j + 1);
        idx.push(a, d, c, a, c, b);
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    const ringMat = std(0x1e2024, 0.7, 0.08, { side: THREE.DoubleSide });
    mesh(g, ringMat, root);
    // the lip round the platter, the rim along the top of the slope, and the seams between the
    // printed pieces
    mesh(new THREE.TorusGeometry(r1, 0.006, 6, 72), ringMat, root, rs.x, rs.y + 0.006, rs.z).rotation.x = Math.PI / 2;
    const rTop = rs.r + f.cap / f.slope;
    const seamMat = std(0x0b0c0e, 0.9, 0);
    for (let i = 0; i < 12; i++) {
      const a = (i / 12) * Math.PI * 2, c = Math.cos(a), s2 = -Math.sin(a);
      if (rs.x + c * rTop > xFront) continue;
      const seam = bx((rTop - r1) * Math.hypot(1, f.slope), 0.003, 0.004, seamMat, root, rs.x + c * (r1 + rTop) / 2, rs.y + f.cap / 2 + 0.004, rs.z + s2 * (r1 + rTop) / 2);
      seam.rotation.set(0, a, Math.atan(f.slope));
    }
  }

  // the Dye Rotor (their engineering report): everything here turns together round the column
  const rotor = new THREE.Group();
  rotor.position.set(rs.x, rs.y, rs.z);
  root.add(rotor);
  const plateMat = std(0xb7c3cf, 0.45, 0.6);
  {
    // the spoked platter: rim, 16 spokes, hub (the big pulley the drivetrain's gearbox belts to)
    const rim = new THREE.Shape();
    rim.absarc(0, 0, rs.r - 0.005, 0, Math.PI * 2, false);
    const inner = new THREE.Path(); inner.absarc(0, 0, rs.r - 0.035, 0, Math.PI * 2, true); rim.holes.push(inner);
    const rg = new THREE.ExtrudeGeometry(rim, { depth: 0.006, bevelEnabled: false, curveSegments: 48 });
    rg.rotateX(-Math.PI / 2);
    mesh(rg, plateMat, rotor, 0, -0.004, 0);
    for (let i = 0; i < 16; i++) {
      const a = (i / 16) * Math.PI * 2, rm = (rs.r - 0.03 + 0.17) / 2;
      bx(rs.r - 0.2, 0.006, 0.018, plateMat, rotor, Math.cos(a) * rm, -0.001, -Math.sin(a) * rm).rotation.y = a;
    }
    mesh(new THREE.CylinderGeometry(0.17, 0.17, 0.008, 40), plateMat, rotor, 0, -0.001, 0);
    for (let i = 0; i < 24; i++) {
      const a = (i / 24) * Math.PI * 2;
      bolts.add(new THREE.Vector3(rs.x + Math.cos(a) * (rs.r - 0.02), rs.y + 0.003, rs.z - Math.sin(a) * (rs.r - 0.02)), new THREE.Vector3(0, 1, 0));
    }
  }
  // the arm: a curved black wall from the column's foot out to the tray wall, its tip leading
  // (rotor.sweep), the same curve the physics pushes FUEL with
  const armPts = [];
  for (let i = 0; i <= 24; i++) {
    const d = rs.finR0 + ((rs.r - 0.012 - rs.finR0) * i) / 24, a = (rs.sweep ?? 0) * (d - rs.finR0) / (rs.r - rs.finR0);
    armPts.push([d * Math.cos(a), -d * Math.sin(a)]);
  }
  const armMat = std(0x1d1f23, 0.55, 0.3, { side: THREE.DoubleSide });
  ribbon(armPts, 0.004, 0.16, armMat, rotor);
  flatRibbon(armPts, 0.035, 0.16, armMat, rotor); // its top flange
  // standoffs along the back of the arm
  for (let i = 3; i < armPts.length; i += 5) {
    const [x, z] = armPts[i], a = Math.atan2(-z, x);
    cylY(0.006, 0.15, M.alu, rotor, x + 0.03 * Math.sin(a), 0.08, z + 0.03 * Math.cos(a), 6);
  }
  // the powered vertical roller (red) out in front of the arm's inner end (their CAD render): the
  // arm's face leads with its curve, FUEL it gathers slides in along it to the mouth in the column
  // between the arm and the roller, and the roller drives it in; its motor rides above it
  // (a FUEL's width, and a little, between the arm's face and the roller)
  const rr = 0.25, ra = (rs.sweep ?? 0) * (rr - rs.finR0) / (rs.r - rs.finR0) + (2 * FUEL.radius + 0.032 + 0.025) / rr;
  const tipX = rr * Math.cos(ra), tipZ = -rr * Math.sin(ra);
  const tipRoller = new THREE.Group();
  tipRoller.position.set(tipX, 0.07, tipZ);
  rotor.add(tipRoller);
  mesh(new THREE.CylinderGeometry(0.032, 0.032, 0.11, 20), std(0xc8442a, 0.5, 0.2), tipRoller, 0, 0, 0);
  for (const y of [-0.045, 0.045]) mesh(new THREE.TorusGeometry(0.032, 0.004, 6, 20), std(0x6a1d10, 0.6, 0.1), tipRoller, 0, y, 0).rotation.x = Math.PI / 2;
  // held off the column's turning sleeve by pocketed aluminum plates above and below it (their
  // photos), with a standoff at the sleeve
  const r0p = 0.168, plateLen = rr + 0.035 - r0p, rm = (r0p + rr + 0.035) / 2;
  const plateAl = std(0xa7adb5, 0.35, 0.8);
  for (const y of [0.132, 0.012]) {
    pocketPlate(rotor, plateLen, 0.06, 0.008, plateAl, [[-plateLen / 4, 0, 0.012], [plateLen / 6, 0, 0.012]], rm * Math.cos(ra), y, -rm * Math.sin(ra)).rotation.set(-Math.PI / 2, ra, 0, 'YXZ');
  }
  for (const s of [-1, 1]) {
    const a2 = ra + (s * 0.022) / r0p;
    cylY(0.005, 0.12, M.alu, rotor, (r0p + 0.01) * Math.cos(a2), 0.072, -(r0p + 0.01) * Math.sin(a2), 6);
  }
  kraken(rotor, tipX, 0.19, tipZ, true, 'y');
  // the blue scoop at the arm's root: a curved fin that lifts FUEL into the column's mouth
  const scoop = [];
  for (let i = 0; i <= 12; i++) {
    const u = i / 12, d = rs.finR0 + 0.06 - 0.05 * u * u, a = 0.25 + 0.45 * u;
    scoop.push([d * Math.cos(a), -d * Math.sin(a)]);
  }
  ribbon(scoop, 0.004, 0.13, std(0x2638d8, 0.45, 0.2, { side: THREE.DoubleSide }), rotor);
  // a sleeve round the column's foot with the mouth cut in it, ahead of the arm
  const mouth = 0.8; // rad
  const sleeve = mesh(new THREE.CylinderGeometry(0.165, 0.175, 0.17, 40, 1, true, Math.PI / 2 + 0.62 + mouth / 2, Math.PI * 2 - mouth), std(0x1b1d21, 0.5, 0.35, { side: THREE.DoubleSide }), rotor, 0, 0.085, 0);
  sleeve.castShadow = true;
  mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.17, 24), std(0x08090a, 0.9, 0), rotor, 0, 0.085, 0); // dark inside the mouth

  // ---- the turret's column and, on top, the turret itself
  cylY(0.15, 0.27, dark, root, t.x, 0.2 + 0.135, t.z, 40);
  for (let i = 0; i < 8; i++) {
    const a = (i / 8) * Math.PI * 2;
    const slot = bx(0.012, 0.05, 0.02, std(0x0e0f11, 0.8, 0), root, t.x + Math.cos(a) * 0.152, 0.4, t.z - Math.sin(a) * 0.152);
    slot.rotation.y = a;
  }
  const turret = new THREE.Group();
  turret.position.set(t.x, 0.47, t.z);
  root.add(turret);
  const shroud = mesh(new THREE.CylinderGeometry(0.175, 0.15, 0.2, 40, 1, true), std(0x2c2f35, 0.5, 0.4, { side: THREE.DoubleSide }), turret, 0, 0.1, 0);
  shroud.castShadow = true;
  for (let i = 0; i < 10; i++) {
    const a = (i / 10) * Math.PI * 2;
    const w = bx(0.004, 0.03, 0.022, std(0x0e0f11, 0.8, 0), turret, Math.cos(a) * 0.165, 0.12, -Math.sin(a) * 0.165);
    w.rotation.y = a;
  }
  mesh(new THREE.TorusGeometry(0.19, 0.012, 8, 48), grey, turret, 0, 0.24, 0).rotation.x = Math.PI / 2;
  mesh(new THREE.RingGeometry(0.15, 0.2, 40), std(0x6b7078, 0.4, 0.6, { side: THREE.DoubleSide }), turret, 0, 0.245, 0).rotation.x = -Math.PI / 2;
  // shooter in the ring: the flywheel across it, the hood on the exit side (+x, the shot direction)
  // 3in Stealth wheels (their engineering report), geared up 4:3 from two Kraken X60s
  const fly = wheelStack(turret, 0.22, 0.0381, 6, std(0x5d6168, 0.6, 0.15), -0.02, 0.2, 0, 0.025);
  const hood = new THREE.Group();
  hood.position.set(-0.02, 0.2, 0);
  turret.add(hood);
  const hoodShell = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.22, 20, 1, true, 0.1, 1.6), std(0x1b1d22, 0.45, 0.5, { side: THREE.DoubleSide }));
  hoodShell.rotation.x = Math.PI / 2;
  hood.add(hoodShell);
  for (const s of [-1, 1]) kraken(turret, -0.08, 0.13, s * 0.12, false, 'z');

  // ---- gantry over the front of the hopper: perforated tube, beacon on top
  const gx = xFront - 0.06, gz = cfg.frame.round.front / 2 + 0.01, gy = 0.74;
  const tubeMat = std(0xb8bdc4, 0.35, 0.85);
  for (const sz of [-1, 1]) rbx(0.025, gy - 0.12, 0.025, 0.003, tubeMat, root, gx, 0.12 + (gy - 0.12) / 2, sz * gz);
  rbx(0.025, 0.025, 2 * gz + 0.025, 0.003, tubeMat, root, gx, gy, 0);
  for (let z = -gz + 0.04; z < gz; z += 0.035) bolts.add(new THREE.Vector3(gx + 0.0126, gy, z), new THREE.Vector3(1, 0, 0));
  cylY(0.02, 0.04, std(0xff8a1f, 0.4, 0.1, { emissive: 0x6a2c00 }), root, gx, gy + 0.03, -gz + 0.03, 14);
  limelight(root, gx, gy - 0.08, 0, 0);

  // ---- intake box: slides out the front
  const box = new THREE.Group();
  root.add(box);
  const iw = cfg.intake.width / 2 + 0.012;
  const xF = L / 2 + BUMPER_T + cfg.intake.reach - extLen; // front of the box, stowed
  const bx0 = xF - 0.26; // back of its side plates
  const plateShape = new THREE.Shape();
  plateShape.moveTo(bx0, 0.05); plateShape.lineTo(xF, 0.05); plateShape.lineTo(xF, 0.44); plateShape.lineTo(xF - 0.07, bay.slope.y + 0.02); plateShape.lineTo(bx0, top - 0.03); plateShape.closePath();
  const hole = new THREE.Path();
  hole.moveTo(bx0 + 0.05, 0.2); hole.lineTo(xF - 0.06, 0.2); hole.lineTo(xF - 0.06, 0.38); hole.lineTo(bx0 + 0.05, top - 0.1); hole.closePath();
  plateShape.holes.push(hole);
  for (const sz of [-1, 1]) {
    const g = new THREE.ExtrudeGeometry(plateShape, { depth: 0.006, bevelEnabled: false });
    g.translate(0, 0, sz * iw - 0.003);
    mesh(g, grey, box);
  }
  // spun about their axles (z): the two over the FUEL turn with their undersides moving in,
  // the low one it rides over turns the other way
  const star = wheelStack(box, 2 * iw - 0.02, 0.032, 10, green, xF - 0.03, 0.26, 0, 0.022);
  const upper = cylZ(0.022, 2 * iw - 0.02, M.black, box, xF - 0.02, 0.19, 0, 12);
  const under = cylZ(0.02, 2 * iw - 0.02, M.alu, box, xF - 0.1, 0.08, 0, 12);
  rbx(0.03, 0.03, 2 * iw, 0.004, tubeMat, box, xF - 0.015, 0.44, 0); // top crossbar
  kraken(box, xF - 0.12, 0.3, iw + 0.04, true, 'z');
  bolts.done();

  // a net over the hopper's top, from the walls to a ring round the column, under the turret
  // (fixed: four posts up from a flange on the column's top)
  const ringR = 0.2, ringY = top - 0.03;
  mesh(new THREE.RingGeometry(0.14, ringR + 0.012, 40), std(0x2c2f35, 0.5, 0.4, { side: THREE.DoubleSide }), root, t.x, 0.472, t.z).rotation.x = -Math.PI / 2;
  for (let i = 0; i < 4; i++) {
    const a = Math.PI / 4 + (i * Math.PI) / 2;
    cylY(0.006, ringY - 0.47, grey, root, t.x + Math.cos(a) * ringR, (0.47 + ringY) / 2, t.z - Math.sin(a) * ringR, 6);
  }
  mesh(new THREE.TorusGeometry(ringR, 0.006, 6, 40), grey, root, t.x, ringY, t.z).rotation.x = Math.PI / 2;
  const inWalls = (x, z) => {
    if (x > xFront) return false;
    let c = false;
    for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      const [xi, zi] = pts[i], [xj, zj] = pts[j];
      if ((zi > z) !== (zj > z) && x < ((xj - xi) * (z - zi)) / (zj - zi) + xi) c = !c;
    }
    return c;
  };
  const xs = pts.map((q) => q[0]), zs = pts.map((q) => Math.abs(q[1]));
  const topNet = hopperNet(root, { x0: Math.min(...xs) - 0.01, x1: xFront, hw: Math.max(...zs) + 0.01, y: top, hole: { x: t.x, z: t.z, r: ringR, y: ringY }, inside: inWalls, NX: 40, NZ: 44 });
  // the net from the top of the front wall down to the intake box's top crossbar
  const net = hopperNet(root, { x0: xFront, x1: xF - 0.02, x1Out: xF - 0.02 + extLen, xFixed: xFront, hw: iw - 0.01, y: top, yFront: 0.45, NX: 14, NZ: 24 });

  const anim = (st, dt) => {
    const ex = st.hopperDeploy * extLen;
    box.position.x = ex;
    const spin = st.intakeSpeed * dt * 40;
    star.rotation.z -= spin; // a group: its axle is its z
    upper.rotation.y -= spin; // cylZ meshes are tipped onto z, so their own y is the axle
    under.rotation.y += spin;
    rotor.rotation.y = st.rotorAngle ?? 0;
    tipRoller.rotation.y += (st.feeding || st.intakeSpeed ? 25 : 0) * dt;
    turret.rotation.y = st.turretYaw;
    fly.rotation.z -= st.flywheel * dt * 9;
    hood.rotation.z = (st.hoodDeg - 60) * Math.PI / 180 * 0.4;
    net.drape(st.load, ex);
    topNet.drape(st.load, 0);
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 3928 (Team Neutrino)
// From 3928's public Onshape CAD ("26"), split by tools/extract-parts.mjs (cad/robots/3928.json):
// the body (drivetrain, hopper walls, spindexer ring and corner wedges, the tower), the
// spindexer's spoked wheel and cone (it turns), the intake box (exported slid out) and the turret
// (the shooter on its bearing plate with the curved guard over its front). Drawn stand-ins show
// until the CAD loads, and in headless runs.
function build3928(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, rs = bay.rotor, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const orange = std(cfg.colors.accent, 0.5, 0.2), dark = std(cfg.colors.frame, 0.45, 0.5);
  const sheet = std(0xc4c8cd, 0.45, 0.6, { side: THREE.DoubleSide });
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  // tall sheet walls on the sides and back, the tower in the back corner
  for (const s of [-1, 1]) bx(L - 0.02, bay.top - 0.1, 0.004, sheet, drawn, 0, 0.1 + (bay.top - 0.1) / 2, s * (W / 2 - 0.005));
  bx(0.004, bay.top - 0.1, W - 0.02, sheet, drawn, -L / 2 + 0.005, 0.1 + (bay.top - 0.1) / 2, 0);
  const tw = bay.obstacles[1].box;
  for (const [x, z] of [[tw[0] + 0.03, tw[4] + 0.02], [tw[1] - 0.02, tw[4] + 0.02], [tw[0] + 0.03, tw[5] - 0.02], [tw[1] - 0.02, tw[5] - 0.02]]) bx(0.025, 0.52, 0.025, dark, drawn, x, 0.3, z);
  mesh(new THREE.TorusGeometry(0.2, 0.012, 8, 40), orange, drawn, sh.turretPos.x, 0.545, sh.turretPos.z).rotation.x = Math.PI / 2;
  // the spindexer: spoked wheel and cone
  const spin = new THREE.Group();
  spin.position.set(rs.x, 0, rs.z);
  root.add(spin);
  const spinDrawn = new THREE.Group();
  spin.add(spinDrawn);
  mesh(new THREE.TorusGeometry(0.13, 0.01, 6, 36), dark, spinDrawn, 0, rs.y, 0).rotation.x = Math.PI / 2;
  for (let i = 0; i < rs.fins; i++) { const a = (i / rs.fins) * 2 * Math.PI; bx(0.13, 0.01, 0.02, dark, spinDrawn, Math.cos(a) * 0.065, rs.y, -Math.sin(a) * 0.065).rotation.y = a; }
  mesh(new THREE.ConeGeometry(0.055, 0.18, 16), dark, spinDrawn, 0, rs.y + 0.09, 0);
  // the intake box (slides out with the hopper extension)
  const box = new THREE.Group();
  root.add(box);
  const boxDrawn = new THREE.Group();
  box.add(boxDrawn);
  for (const s of [-1, 1]) bx(0.5, 0.55, 0.004, sheet, boxDrawn, L / 2 + extLen - 0.25, 0.4, s * (W / 2 - 0.01));
  const rollers = [cylZ(0.025, cfg.intake.width, orange, boxDrawn, L / 2 + extLen + 0.09, 0.16, 0, 14), cylZ(0.019, cfg.intake.width, orange, boxDrawn, L / 2 + extLen + 0.07, 0.225, 0, 12)];
  // turret on top of the tower
  const turret = new THREE.Group();
  turret.position.set(sh.turretPos.x, 0, sh.turretPos.z);
  root.add(turret);
  const tDrawn = new THREE.Group();
  turret.add(tDrawn);
  for (const s of [-1, 1]) bx(0.28, 0.14, 0.008, orange, tDrawn, 0, 0.64, s * 0.1);
  const fly = wheelStack(tDrawn, 0.18, 0.05, 2, std(0xd8c9a0, 0.5, 0.3), 0.1, 0.645, 0, 0.04);
  // net over the top (their CAD doesn't model it): pinned to the wall tops, the intake box's top
  // crossbar and round the turret
  const net = hopperNet(root, {
    x0: bay.x0, x1: bay.x1, x1Out: bay.x1 + extLen, xFixed: bay.x1, hw: bay.hw, y: bay.top + 0.004, yFront: 0.685,
    hole: { x: sh.turretPos.x, z: sh.turretPos.z, r: bay.dome.rHole - 0.005 },
  });
  cadPart(root, 'robots/3928-body.glb', () => { drawn.visible = false; });
  cadPart(spin, 'robots/3928-spin.glb', () => { spinDrawn.visible = false; });
  cadPart(box, 'robots/3928-intake.glb', () => { boxDrawn.visible = false; });
  cadPart(turret, 'robots/3928-turret.glb', () => { tDrawn.visible = false; });

  const anim = (st, dt) => {
    box.position.x = (st.hopperDeploy - 1) * extLen; // exported slid out
    net.drape(st.load, st.hopperDeploy * extLen);
    for (const r of rollers) r.rotation.y += st.intakeSpeed * dt * 40;
    spin.rotation.y = st.rotorAngle ?? 0;
    turret.rotation.y = st.turretYaw; // the CAD's shooter faces forward
    fly.rotation.z -= st.flywheel * dt * 8;
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 341 Miss Daisy XXIV
// From 341's public Onshape CAD ("Miss Daisy XXIV"), split by tools/extract-parts.mjs
// (cad/robots/341.json): the body (drivetrain, the serializer tray with its three omni wheels,
// the table and uptake, the L1 climber), the "brontosaurus" slapdown (exported down; origin on its
// pivot) and the shooter on its turret (origin on the 92T turret gear's axis).
const TURRET_341_YAW = -150 * Math.PI / 180; // where the CAD's shooter points as exported
function build341(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter, fold = cfg.intake.fold;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const blue = std(cfg.colors.accent, 0.5, 0.2), yellow = std(cfg.colors.trim, 0.5, 0.15);
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  addElectronics(drawn, -0.2, 0.15, Math.PI / 2);
  rbx(bay.x1 - bay.x0, 0.004, 2 * bay.hw, 0.002, M.alu, drawn, (bay.x0 + bay.x1) / 2, bay.floor.a - 0.002, 0); // the tray
  const omnis = bay.obstacles.map((o) => cylY(o.wheel, 0.035, M.black, drawn, o.x, 0.25, o.z, 20));
  rbx(0.62, 0.004, 0.62, 0.002, M.alu, drawn, -0.03, 0.376, 0); // the table
  for (const [x, z] of [[-0.31, -0.3], [-0.31, 0.3], [0.16, -0.3], [0.16, 0.3]]) bx(0.025, 0.36, 0.025, M.alu, drawn, x, 0.19, z);
  // the slapdown
  const arm = new THREE.Group();
  arm.position.set(fold.pivot[0], fold.pivot[1], 0);
  root.add(arm);
  const armDrawn = new THREE.Group();
  arm.add(armDrawn);
  for (const s of [-1, 1]) {
    const shp = new THREE.Shape(fold.hull.map(([x, y]) => new THREE.Vector2(x, y)));
    const g = new THREE.ExtrudeGeometry(shp, { depth: 0.006, bevelEnabled: false });
    mesh(g, M.anodBlack, armDrawn, 0, 0, s * (cfg.intake.width / 2 + 0.01) - 0.003);
  }
  const rollers = [[0.19, -0.29], [0.325, -0.18], [0.29, -0.06], [0.2, -0.02], [0.09, 0.0]].map(([x, y], i) => cylZ(i < 2 ? 0.018 : 0.025, cfg.intake.width, i < 2 ? M.black : M.anodBlack, armDrawn, x, y, 0, 14));
  // the turret
  const turret = new THREE.Group();
  turret.position.set(sh.turretPos.x, 0, sh.turretPos.z);
  root.add(turret);
  const tDrawn = new THREE.Group();
  turret.add(tDrawn);
  for (const s of [-1, 1]) bx(0.4, 0.15, 0.006, M.alu, tDrawn, 0, 0.46, s * 0.1);
  const fly = wheelStack(tDrawn, 0.18, 0.05, 2, blue, 0.14, 0.47, 0, 0.05);
  cylZ(0.05, 0.12, std(0x1e9a8c, 0.6, 0.1), tDrawn, -0.14, 0.47, 0, 20);
  for (let i = 0; i < 3; i++) cylZ(0.016, 0.2, yellow, tDrawn, 0.08 + i * 0.05, 0.53, 0, 10);
  const cadTurret = new THREE.Group();
  turret.add(cadTurret);
  cadTurret.rotation.y = -TURRET_341_YAW;
  cadTurret.position.set(0, 0, 0);
  cadPart(root, 'robots/341-body.glb', () => { drawn.visible = false; });
  cadPart(arm, 'robots/341-intake.glb', () => { armDrawn.visible = false; });
  cadPart(cadTurret, 'robots/341-turret.glb', () => { tDrawn.visible = false; });

  const anim = (st, dt) => {
    arm.rotation.z = (1 - st.intakeDeploy) * fold.stowDeg * Math.PI / 180;
    for (const r of rollers) r.rotation.z -= st.intakeSpeed * dt * 40;
    for (const w of omnis) w.rotation.y += (st.feeding + 0.3 * st.intakeSpeed) * dt * 20;
    turret.rotation.y = st.turretYaw;
    fly.rotation.z -= st.flywheel * dt * 8;
  };
  return { root, anim, stored: [], modules, extLen: 0 };
}

// ============================================================ 4930 Electric Mayhem, Floyd 2
// From 4930's public Onshape CAD ("Floyd 2: Whole Robot Assembly"), split by
// tools/extract-parts.mjs (cad/robots/4930.json): the body (drivetrain, the belt floor, the updexer
// and the three shooters on one long flywheel shaft) and the intake (exported down; origin on its
// pivot). Its CAD isn't centered on its origin, so the recipe moves it to the frame's middle.
function build4930(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter, fold = cfg.intake.fold;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const green = std(cfg.colors.accent, 0.55, 0.1), pink = std(cfg.colors.trim, 0.5, 0.15);
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  // belt floor down to the updexer, side plates, three hooded shooters on one shaft
  const f = bay.floor, fx0 = 0.04, fx1 = 0.3, fy = (x) => clamp01(f.a + f.b * x, f.lo, f.hi);
  const belt = rbx(Math.hypot(fx1 - fx0, fy(fx1) - fy(fx0)), 0.01, 2 * bay.hw - 0.04, 0.003, M.polyBelt, drawn, (fx0 + fx1) / 2, (fy(fx0) + fy(fx1)) / 2 - 0.08, 0);
  belt.rotation.z = Math.atan2(fy(fx1) - fy(fx0), fx1 - fx0);
  for (const z of [-0.31, -0.105, 0.105, 0.31]) bx(0.4, 0.5, 0.006, z === -0.31 || z === 0.31 ? M.alu : pink, drawn, -0.12, 0.3, z);
  const shaft = cylZ(0.006, 0.72, M.alu, drawn, -0.113, 0.57, 0, 8);
  const flys = sh.lanes.map((z) => cylZ(0.05, 0.04, std(0xd8c9a0, 0.5, 0.3), drawn, -0.113, 0.57, z, 20));
  for (const z of sh.lanes) { const hood = mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.1, 16, 1, true, 0, Math.PI * 0.9), green, drawn, -0.113, 0.57, z); hood.rotation.x = Math.PI / 2; hood.rotation.y = Math.PI; }
  // the intake
  const arm = new THREE.Group();
  arm.position.set(fold.pivot[0], fold.pivot[1], 0);
  root.add(arm);
  const armDrawn = new THREE.Group();
  arm.add(armDrawn);
  for (const s of [-1, 1]) {
    const shp = new THREE.Shape(fold.hull.map(([x, y]) => new THREE.Vector2(x, y)));
    const p = mesh(new THREE.ShapeGeometry(shp), M.poly, armDrawn, 0, 0, s * (cfg.intake.width / 2 + 0.01));
    p.castShadow = false;
  }
  const rollers = [[0.36, -0.12], [0.3, -0.03]].map(([x, y]) => cylZ(0.024, cfg.intake.width, green, armDrawn, x, y, 0, 14));
  // the net from the front of the shooters' side plates down to the intake (their CAD doesn't
  // model it); it rides out and down with the intake
  const extLen = cfg.storage.extLen;
  const net = hopperNet(root, { x0: bay.slope.x, x1: bay.x1, x1Out: bay.x1 + extLen, xFixed: bay.slope.x, hw: bay.hw, y: bay.top, yFront: bay.slope.y, NX: 14, NZ: 24 });
  cadPart(root, 'robots/4930-body.glb', () => { drawn.visible = false; });
  cadPart(arm, 'robots/4930-intake.glb', () => { armDrawn.visible = false; });

  const anim = (st, dt) => {
    arm.rotation.z = (1 - st.intakeDeploy) * fold.stowDeg * Math.PI / 180;
    for (const r of rollers) r.rotation.z -= st.intakeSpeed * dt * 40;
    for (const w of flys) w.rotation.z += st.flywheel * dt * 8;
    shaft.rotation.z = flys[0].rotation.z;
    net.drape(st.load, st.hopperDeploy * extLen);
  };
  return { root, anim, stored: [], modules, extLen };
}
const clamp01 = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

// ============================================================ 1706 Ratchet Rockers, Mirage
// From 1706's public Onshape CAD ("Mirage Public Release"), split by tools/extract-parts.mjs
// (cad/robots/1706.json): the body (drivetrain, spindexer floor, the outer hopper walls, the
// turret rings), the intake's fixed frame plates, the hopper box that slides out, the intake arm
// (exported folded up inside the hopper; origin on its pivot), the two spindexer discs and the two
// shooters (origins on their turret axes). The team's front is the turret end; this model's +x is
// the intake end, like every other robot here.
// The arm turns on the hex shaft through its plates (0.262, 0.168), not on the frame's cross tube
// where the export puts its origin (0.149, 0.127); its mate's 113.4deg takes its 1in wheels down to
// the bumper's face, 12in past the frame (the extension limit). Physical (js/mech/phys1706.js), the
// arm, the box, the spindexers and the turrets go where its mechanisms put them.
const INTAKE_1706 = { pivot: [0.262, 0.168], origin: [0.149, 0.127], swing: -113.4 * Math.PI / 180 };
// where the CAD's shooters fire as exported (the right one, the left: their shots' headings, out over
// their 3in accelerator wheels; tools/path-mesh-1706.py)
const TURRET_1706_YAW = [160.2 * Math.PI / 180, 152.0 * Math.PI / 180];
function build1706(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const blue = std(cfg.colors.accent, 0.5, 0.2);
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  for (const s of [-1, 1]) polyWall(drawn, bay.x1 - bay.x0, bay.top - 0.22, (bay.x0 + bay.x1) / 2, 0.22 + (bay.top - 0.22) / 2, s * (bay.hw + 0.004));
  // the hopper box's front wall (slides out)
  const box = new THREE.Group();
  root.add(box);
  const boxDrawn = new THREE.Group();
  box.add(boxDrawn);
  polyWall(boxDrawn, 2 * bay.hw, bay.top - 0.22, bay.x1, 0.22 + (bay.top - 0.22) / 2, 0, Math.PI / 2);
  // the spindexers
  const spins = bay.rotors.map((r) => {
    const g = new THREE.Group();
    g.position.set(r.x, 0, r.z);
    root.add(g);
    const d = mesh(new THREE.CylinderGeometry(r.r, r.r, 0.004, 32), M.anodBlack, g, 0, r.y, 0);
    cylY(0.05, 0.1, blue, g, 0, r.y + 0.1, 0, 16);
    return { g, d, r };
  });
  // the intake arm, folded up inside the hopper
  const arm = new THREE.Group();
  arm.position.set(INTAKE_1706.pivot[0], INTAKE_1706.pivot[1], 0);
  root.add(arm);
  const armDrawn = new THREE.Group();
  arm.add(armDrawn);
  for (const s of [-1, 1]) rod(armDrawn, new THREE.Vector3(0, 0, s * 0.33), new THREE.Vector3(0, 0.32, s * 0.33), 0.012, M.alu);
  const rollers = [cylZ(0.025, 0.66, M.alu, armDrawn, 0.0, 0.32, 0, 14), cylZ(0.025, 0.66, M.alu, armDrawn, -0.08, 0.25, 0, 14)];
  // two turrets
  const turrets = sh.turrets.map((t, i) => {
    const g = new THREE.Group();
    g.position.set(t.x, 0, t.z);
    root.add(g);
    const d = new THREE.Group();
    g.add(d);
    mesh(new THREE.TorusGeometry(0.11, 0.01, 8, 36), blue, d, 0, 0.35, 0).rotation.x = Math.PI / 2;
    for (const s of [-1, 1]) pocketPlate(d, 0.24, 0.15, 0.006, M.alu, [[-0.05, 0, 0.03], [0.05, 0.02, 0.03]], 0, 0.43, s * 0.06);
    wheelStack(d, 0.1, 0.063, 1, M.black, 0.1, 0.45, 0, 0.05);
    const cad = new THREE.Group();
    cad.rotation.y = -TURRET_1706_YAW[i];
    g.add(cad);
    cadPart(cad, `robots/1706-turret-${i ? 'b' : 'a'}.glb`, () => { d.visible = false; });
    return g;
  });
  // the net over the open top (their CAD doesn't model it; js/mech/r1706.js NET): slack cord tied
  // along the walls' tops, the back panel's top tube and the hopper box's top edges, its front
  // following the box out; empty, it hangs (onto the folded intake's top, 6.5 cm under the walls,
  // with the box in; less with the box out, which takes up some of its slack), and it drapes over
  // the load as the load lifts it
  const net = hopperNet(root, { x0: NET_1706.x0, x1: NET_1706.x1, x1Out: NET_1706.x1 + extLen, xFixed: NET_1706.x0, hw: NET_1706.hw, y: NET_1706.y + 0.004, NX: 30, NZ: 36, slack: NET_1706.hang });
  cadPart(root, 'robots/1706-body.glb', () => { drawn.visible = false; });
  cadPart(root, 'robots/1706-mount.glb', () => {});
  cadPart(box, 'robots/1706-hopper.glb', () => { boxDrawn.visible = false; });
  const armCad = new THREE.Group();
  armCad.position.set(INTAKE_1706.origin[0] - INTAKE_1706.pivot[0], INTAKE_1706.origin[1] - INTAKE_1706.pivot[1], 0);
  arm.add(armCad);
  cadPart(armCad, 'robots/1706-intake.glb', (s) => {
    armDrawn.visible = false;
    // the hopper box's top corner gussets came out merged into the arm's end blocks
    moveCadTris(s, armCad, (c) => c.y + INTAKE_1706.origin[1] > 0.53, box, [...INTAKE_1706.origin, 0]);
  });
  spins.forEach((s, i) => cadPart(s.g, `robots/1706-spin-${i ? 'b' : 'a'}.glb`, () => { s.d.visible = false; }));

  const anim = (st, dt) => {
    box.position.x = st.hopperDeploy * extLen; // exported in
    net.drape(st.load, st.hopperDeploy * extLen);
    arm.rotation.z = st.armTurn ?? st.intakeDeploy * INTAKE_1706.swing;
    for (const r of rollers) r.rotation.z -= st.intakeSpeed * dt * 40;
    spins.forEach((s, i) => { s.g.rotation.y = st.mech ? st.mech.spin[i] : (st.rotorAngle ?? 0) * (s.r.dir ?? 1); });
    turrets.forEach((g, i) => { g.rotation.y = st.turretYaws ? st.turretYaws[i] : st.turretYaw; });
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 581 Blazing Bulldogs, Rubble
// From 581's public Onshape CAD ("2026 Dumper Champs Bot", exported with the intake out), split
// by tools/extract-parts.mjs (cad/robots/581.json): the body (drivetrain, shooter tower, hopper
// walls), the intake's fixed deploy gearboxes (mount), the intake that slides out (its plates,
// rollers, motors, side panels, the hopper's front plate and the titanium racks), its kicker
// plates, the roller floor and the hood. The joints are the CAD's own mates: the slider (0.306 m,
// tilted 5.5deg), the kicker's free pivot, the floor's pivot at the tower and the hood's 35deg on
// the drum's shaft.
const S581 = {
  slide: [-0.3047, 0.0294], // full retract, from out (as exported): back along the racks and up
  kicker: [0.5062, 0.1578], kickerLen: 0.144, kickerRest: 20 * Math.PI / 180, // the STEP's pose hangs 20deg back
  floor: [-0.0675, 0.108], floorUp: 7.1 * Math.PI / 180, // raised by the deploy coming in (the CAD's two poses)
  hood: [-0.2858, 0.4762], hoodTravel: 35 * Math.PI / 180,
};
function build581(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, W = cfg.frame.width, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const red = std(cfg.colors.accent, 0.5, 0.2);
  // drawn stand-ins until the CAD loads
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  for (const s of [-1, 1]) polyWall(drawn, bay.x1 - bay.x0, bay.top - 0.2, (bay.x0 + bay.x1) / 2, 0.2 + (bay.top - 0.2) / 2, s * (bay.hw + 0.004));
  bx(0.12, bay.top - 0.1, 2 * bay.hw, M.anodBlack, drawn, -L / 2 + 0.07, 0.1 + (bay.top - 0.1) / 2, 0);
  // the intake slides out along the racks, carrying the front of the hopper
  const slide = new THREE.Group();
  root.add(slide);
  const slideDrawn = new THREE.Group();
  slide.add(slideDrawn);
  cylZ(0.038, cfg.intake.width, red, slideDrawn, 0.582, 0.176, 0, 16);
  cylZ(0.025, cfg.intake.width, red, slideDrawn, 0.533, 0.233, 0, 14);
  polyWall(slideDrawn, 2 * bay.hw, 0.24, 0.645, 0.37, 0, Math.PI / 2);
  // its kicker plates hang from a pivot on the intake plates and swing up over the bumper
  const kicker = new THREE.Group();
  kicker.position.set(...S581.kicker, 0);
  slide.add(kicker);
  const kickerCad = new THREE.Group();
  kicker.add(kickerCad);
  // the roller floor, on its pivot at the tower
  const floor = new THREE.Group();
  floor.position.set(...S581.floor, 0);
  root.add(floor);
  // the hood, on the drum's shaft
  const hood = new THREE.Group();
  hood.position.set(...S581.hood, 0);
  root.add(hood);
  cadPart(root, 'robots/581-body.glb', () => { drawn.visible = false; });
  cadPart(root, 'robots/581-mount.glb', () => {});
  cadPart(slide, 'robots/581-intake.glb', () => { slideDrawn.visible = false; });
  cadPart(kickerCad, 'robots/581-kicker.glb', () => {});
  cadPart(floor, 'robots/581-floor.glb', () => {});
  cadPart(hood, 'robots/581-hood.glb', () => {});
  // the net (js/mech/r581.js NET): only over the hopper's front, between the polycarbonate top's
  // front edge and the top of the intake's front plate, across between the intake's side panels;
  // slack bungee, its front riding out and in with the intake. Empty it hangs (as much as the load
  // can lift it), and it drapes over the load as the load lifts it
  const N = NET_581, net = hopperNet(root, { x0: N.topX, x1: N.frontX + S581.slide[0], x1Out: N.frontX, xFixed: N.topX, hw: N.hw, y: N.topY + 0.004, yFront: N.frontY + 0.004, NX: 16, NZ: 32, slack: { sag: N.bulge } });
  const xb = L / 2 + BUMPER_T, zc = BUMPER_Y1 + 0.006;
  const anim = (st, dt) => {
    // the slide: wherever the intake is, or further out while the load holds the hopper open
    const s = Math.max(st.intakeDeploy, st.hopperDeploy);
    slide.position.set((1 - s) * S581.slide[0], (1 - s) * S581.slide[1], 0);
    // the kicker hangs straight down; coming in, the bumper pushes it forward and up: as little as
    // it takes for its bottom edge to clear the bumper's face or its top
    const px = S581.kicker[0] + slide.position.x, py = S581.kicker[1] + slide.position.y, K = S581.kickerLen;
    let th = 0;
    if (px < xb) {
      const a1 = Math.asin(clamp01((xb - px) / K, 0, 1)), a2 = Math.acos(clamp01((py - zc) / K, -1, 1));
      th = Math.min(a1, a2);
    }
    kicker.rotation.z = th;
    kickerCad.rotation.z = S581.kickerRest;
    // pushing the deploy in raises the floor's front: 2in followers on its front corners ride ramps
    // on the intake's side plates (js/mech/r581.js FLOOR.cam), which lifts the load toward the
    // shooter, their compaction while shooting
    floor.rotation.z = st.floorTurn ?? clamp01((0.35 - s) / 0.35, 0, 1) * S581.floorUp;
    // the hood opens up (flatter shots) from its stop at the steepest one
    hood.rotation.z = st.hoodTurn ?? clamp01((sh.hoodMax - st.hoodDeg) / (sh.hoodMax - sh.hoodMin), 0, 1) * S581.hoodTravel;
    net.drape(st.load, -s * S581.slide[0], { yFront: N.frontY + 0.004 + (1 - s) * S581.slide[1] });
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 1114 Simbotics, Simbot Tim
// From 1114's public Onshape CAD ("S26-A000", exported with the box out), split by
// tools/extract-parts.mjs (cad/robots/1114.json): the body (drivetrain, belt floor, the tower with
// its ball elevator and flywheel), the box's fixed rack gearboxes (mount), the hopper box that slides
// out on the racks with the intake rollers on its front, the kicker bar on its arms, and the hood.
// The joints are the CAD's own mates: the box's slide (0.298 m, rising 8.7deg toward the robot),
// the kicker's pivot (0 to 90deg) and the hood's 30deg on the flywheel shaft.
const S1114 = {
  slide: [-0.2945, 0.0453], // full retract, from out (as exported): back along the racks and up
  kicker: [0.4513, 0.196], kickerLen: 0.168, kickerMax: Math.PI / 2,
  hood: [-0.0176, 0.5715], hoodTravel: 30, // (deg)
};
const BLANKET_X0 = -0.02; // the blanket's back edge (box out), just in front of the hood
function build1114(cfg, alliance) {
  const root = new THREE.Group();
  const L = cfg.frame.length, bay = cfg.bay, sh = cfg.shooter;
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const red = std(cfg.colors.accent, 0.5, 0.2);
  // drawn stand-ins until the CAD loads
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  bx(0.25, 0.62, 0.6, M.alu, drawn, -0.15, 0.4, 0);
  // the hopper box: slides out on the racks with the intake on its front
  const slide = new THREE.Group();
  root.add(slide);
  const slideDrawn = new THREE.Group();
  slide.add(slideDrawn);
  for (const s of [-1, 1]) polyWall(slideDrawn, 0.62, 0.43, 0.29, 0.485, s * 0.37);
  polyWall(slideDrawn, 0.74, 0.43, 0.6, 0.485, 0, Math.PI / 2);
  cylZ(0.025, cfg.intake.width, red, slideDrawn, 0.545, 0.165, 0, 14);
  // its kicker bar, on arms that swing up over the bumper as the box comes in
  const kicker = new THREE.Group();
  kicker.position.set(...S1114.kicker, 0);
  slide.add(kicker);
  // the hood, on the flywheel's shaft
  const hood = new THREE.Group();
  hood.position.set(...S1114.hood, 0);
  root.add(hood);
  cadPart(root, 'robots/1114-body.glb', () => { drawn.visible = false; });
  cadPart(root, 'robots/1114-pulleys.glb', () => {});
  cadPart(root, 'robots/1114-mount.glb', () => {});
  cadPart(slide, 'robots/1114-slide.glb', () => { slideDrawn.visible = false; });
  cadPart(kicker, 'robots/1114-kicker.glb', () => {});
  cadPart(hood, 'robots/1114-hood.glb', () => {});
  // the "magic blanket" over the box's top (their reveal; after 1124's 2008 claw): red plush, tied
  // round the edges, sagging over the load and riding up on it
  const plush = new THREE.MeshStandardMaterial({ color: 0xb3171d, roughness: 1, side: THREE.DoubleSide });
  const blanket = hopperNet(slide, { x0: BLANKET_X0, x1: 0.606, hw: 0.372, y: 0.704, NX: 26, NZ: 22, material: plush });
  blanket.mesh.castShadow = true;
  const xb = L / 2 + BUMPER_T, zc = BUMPER_Y1 + 0.006;
  const shifted = [];
  // the rollers turning (a dark seam along each one's grip tape, so the CAD's static rollers are
  // seen to turn): the intake roller on the box, the floor's nine, the elevator's two columns, the
  // flywheel and the hood's three (positions from js/mech/r1114.js; sign: +1 turns it as it
  // feeds)
  const seamMat = std(0x111214, 0.9, 0.0);
  const spinner = (parent, x, y, r, len, sign) => {
    const g = new THREE.Group();
    g.position.set(x, y, 0);
    parent.add(g);
    bx(0.006, 0.004, len, seamMat, g, 0, r + 0.0015, 0);
    bx(0.006, 0.004, len, seamMat, g, 0, -r - 0.0015, 0);
    g.userData.sign = sign;
    return g;
  };
  const intakeSpin = [spinner(slide, 0.5457 - S1114.slide[0] * 0, 0.1663, 0.0262, 0.63, -1)];
  const floorSpin = [[0.2857, 0.1668], [0.2276, 0.1457], [0.1696, 0.1246], [0.1116, 0.1034], [0.0536, 0.0823], [-0.0326, 0.0719], [-0.1144, 0.1007], [-0.1751, 0.1626], [-0.2022, 0.2451]].map(([x, y]) => spinner(root, x, y, 0.0167, 0.39, 1));
  const elevSpin = [...[0.3224, 0.3816, 0.4408, 0.5001].map((y) => spinner(root, -0.2096, y, 0.0167, 0.39, 1)), ...[0.3224, 0.3816, 0.4408, 0.5001].map((y) => spinner(root, -0.0343, y, 0.0167, 0.39, -1))];
  const flySpin = spinner(root, -0.0176, 0.5715, 0.0402, 0.4, -1);
  const hoodSpin = [[-0.2024, 0.6137], [-0.1841, 0.6621], [-0.1534, 0.7037]].map(([x, y]) => spinner(hood, x - S1114.hood[0], y - S1114.hood[1], 0.0198, 0.4, 1));
  const anim = (st, dt) => {
    // (physical: the floor's rollers at st.floorSpin rad/s; the rest at a visible pace)
    for (const g of intakeSpin) g.rotation.z += g.userData.sign * st.intakeSpeed * dt * 40;
    for (const g of floorSpin) g.rotation.z += g.userData.sign * (st.floorSpin ?? st.feeding * 30) * dt * 0.25;
    for (const g of elevSpin) g.rotation.z += g.userData.sign * st.feeding * dt * 60;
    flySpin.rotation.z -= Math.min(60, (st.flywheel || 0) / 0.0402) * dt * 0.4;
    for (const g of hoodSpin) g.rotation.z += Math.min(60, (st.flywheel || 0) / 0.0198) * dt * 0.3;
    const s = Math.max(st.intakeDeploy, st.hopperDeploy);
    slide.position.set((1 - s) * S1114.slide[0], (1 - s) * S1114.slide[1], 0);
    // the kicker hangs straight down; coming in, the bumper swings it forward and up (to its stop)
    const px = S1114.kicker[0] + slide.position.x, py = S1114.kicker[1] + slide.position.y, K = S1114.kickerLen;
    let th = 0;
    if (px < xb) th = Math.min(Math.asin(clamp01((xb - px) / K, 0, 1)), Math.acos(clamp01((py - zc) / K, -1, 1)));
    kicker.rotation.z = Math.min(th, S1114.kickerMax);
    // the hood: drawn set for a flat shot (sh.hood.cadDeg), swinging up and back over the
    // flywheel for steeper ones
    hood.rotation.z = clamp01(st.hoodDeg - sh.hood.cadDeg, -5, S1114.hoodTravel) * Math.PI / 180;
    // the blanket rides with the box: the load in the box's frame
    const load = st.load || [];
    for (let i = 0; i < load.length; i++) {
      const e = shifted[i] || (shifted[i] = { p: new THREE.Vector3() });
      e.p.set(load[i].p.x - slide.position.x, load[i].p.y - slide.position.y, load[i].p.z);
    }
    shifted.length = load.length;
    // its back edge stays in front of the hood: coming in, the box doesn't pull it back over the
    // shooter, it gathers up toward the front
    blanket.drape(shifted, 0, { xBack: BLANKET_X0 - slide.position.x });
  };
  return { root, anim, stored: [], modules, extLen };
}

// ============================================================ 604 Quixilver, Toploader
// From 604's public Onshape CAD ("Toploader Assembly", a STEP translation), split by
// tools/extract-parts.mjs (cad/robots/604.json) along the assembly's own mates: the body (the
// chassis, the plywood DPC bucket and the rotor's drive, the intake's gearboxes, the superstructure
// and its coroplast walls), its motors (flat shaded), the intake arms on their pivot (exported at
// their 0 deg, nearly down), the sliding hopper box (exported out, 0.2921 m), the DPC rotor with its
// hook, ramp, omni wheels and tube (origin on its axis, 1in behind the center), the turret on the
// same axis (as exported it fires to the robot's right) and the hood on the main roller's axle.
// The game draws the bumpers.
const S604 = {
  pivot: [0.3048, 0.2064], hopperOut: 0.2921, axis: [-0.0254, 0],
  turretCad: -Math.PI / 2, // where the CAD's turret fires as exported (the robot's right)
  hood: [-0.0007, 0.5969, 0.0744], // the hood's pivot (the main roller's axle) from the turret's axis, as exported
  start: 98, deploy: -5, // their wrist angles (the CAD's pose is their 0)
};
function build604(cfg, alliance) {
  const root = new THREE.Group();
  const extLen = cfg.storage.extLen;
  addBumpers(root, cfg, alliance);
  const modules = addDrivebase(root, cfg, 0, false);
  const gold = std(cfg.colors.accent, 0.55, 0.1), coro = std(cfg.colors.trim, 0.8, 0.05), ply = std(0xd9c08a, 0.85, 0.0);
  // drawn stand-ins until the CAD loads
  const drawn = new THREE.Group();
  root.add(drawn);
  addDrivebase(drawn, cfg, 0, true).forEach((md) => md.pivot.parent.removeFromParent());
  cylY(0.31, 0.012, ply, drawn, S604.axis[0], 0.148, 0, 40); // the bucket's floor
  for (const s of [-1, 1]) bx(0.49, 0.43, 0.004, coro, drawn, -0.103, 0.476, s * 0.323); // the back walls
  bx(0.004, 0.43, 0.23, coro, drawn, -0.345, 0.476, 0);
  // the intake arms on their pivot
  const arm = new THREE.Group();
  arm.position.set(...S604.pivot, 0);
  root.add(arm);
  const armDrawn = new THREE.Group();
  arm.add(armDrawn);
  for (const s of [-1, 1]) bx(0.34, 0.18, 0.006, M.polyTint, armDrawn, 0.135, 0.013, s * 0.327);
  const rollers = [[0.278, -0.054], [0.2505, 0.0317]].map(([x, y]) => cylZ(0.027, cfg.intake.width, M.black, armDrawn, x, y, 0, 16));
  // the hopper box, out over the intake
  const hopper = new THREE.Group();
  root.add(hopper);
  const hopperDrawn = new THREE.Group();
  hopper.add(hopperDrawn);
  for (const s of [-1, 1]) bx(0.68, 0.43, 0.004, coro, hopperDrawn, 0.288, 0.476, s * 0.34);
  bx(0.004, 0.43, 0.69, coro, hopperDrawn, 0.633, 0.476, 0);
  // the DPC rotor, on its axis
  const rotor = new THREE.Group();
  rotor.position.set(S604.axis[0], 0, S604.axis[1]);
  root.add(rotor);
  const rotorDrawn = new THREE.Group();
  rotor.add(rotorDrawn);
  cylY(0.08, 0.17, coro, rotorDrawn, 0, 0.437, 0, 24); // the tube
  bx(0.2, 0.056, 0.012, M.alu, rotorDrawn, -0.15, 0.228, 0.06); // the hook
  // the turret on the same axis, and the hood on the main roller's axle
  const turret = new THREE.Group();
  turret.position.set(S604.axis[0], 0, S604.axis[1]);
  root.add(turret);
  const cadTurret = new THREE.Group();
  cadTurret.rotation.y = -S604.turretCad;
  turret.add(cadTurret);
  const turretDrawn = new THREE.Group();
  cadTurret.add(turretDrawn);
  cylY(0.17, 0.006, M.alu, turretDrawn, 0, 0.689, 0, 40);
  for (const s of [-1, 1]) bx(0.22, 0.16, 0.005, M.alu, turretDrawn, s * 0.107, 0.606, 0);
  const hood = new THREE.Group();
  hood.position.set(...S604.hood);
  cadTurret.add(hood);
  const hoodDrawn = new THREE.Group();
  hood.add(hoodDrawn);
  cylX(0.027, 0.114, gold, hoodDrawn, 0, 0, 0, 16);
  cadPart(root, 'robots/604-body.glb', () => { drawn.visible = false; });
  cadPart(root, 'robots/604-motors.glb', () => {});
  cadPart(arm, 'robots/604-intake.glb', () => { armDrawn.visible = false; });
  cadPart(hopper, 'robots/604-hopper.glb', () => { hopperDrawn.visible = false; });
  cadPart(rotor, 'robots/604-rotor.glb', () => { rotorDrawn.visible = false; });
  cadPart(cadTurret, 'robots/604-turret.glb', () => { turretDrawn.visible = false; });
  cadPart(hood, 'robots/604-hood.glb', () => { hoodDrawn.visible = false; });
  let spin = 0;
  const anim = (st, dt) => {
    // the arms: their wrist angle (the physical model's turn from the CAD's pose), or between boot
    // (98) and deployed (-5)
    arm.rotation.z = st.armTurn ?? (S604.start + (S604.deploy - S604.start) * st.intakeDeploy) * Math.PI / 180;
    for (const r of rollers) r.rotation.z -= st.intakeSpeed * dt * 40;
    // the hopper box: as far out as the arms push it
    hopper.position.x = (st.hopperDeploy ?? st.intakeDeploy) * extLen - S604.hopperOut;
    // the rotor turns clockwise seen from above; the turret and its hood
    spin = st.rotorTurn ?? spin + st.feeding * 15 * dt;
    rotor.rotation.y = -spin;
    turret.rotation.y = st.turretYaw;
    // (the hood's turn about the main roller's axle, raising the shot: about the turret's own z,
    // which is the CAD's -x as exported)
    hood.rotation.x = -(st.hoodTurn ?? 0);
  };
  return { root, anim, stored: [], modules, extLen };
}

export function buildRobotModel(cfg, alliance) {
  let m;
  if (cfg.key === '2910') m = build2910(cfg, alliance);
  else if (cfg.key === '4414') m = build4414(cfg, alliance);
  else if (cfg.key === '971') m = build971(cfg, alliance);
  else if (cfg.key === '1678') m = build1678(cfg, alliance);
  else if (cfg.key === '1690') m = build1690(cfg, alliance);
  else if (cfg.key === '4946') m = build4946(cfg, alliance);
  else if (cfg.key === '3928') m = build3928(cfg, alliance);
  else if (cfg.key === '341') m = build341(cfg, alliance);
  else if (cfg.key === '4930') m = build4930(cfg, alliance);
  else if (cfg.key === '1706') m = build1706(cfg, alliance);
  else if (cfg.key === '581') m = build581(cfg, alliance);
  else if (cfg.key === '1114') m = build1114(cfg, alliance);
  else if (cfg.key === '604') m = build604(cfg, alliance);
  else m = build8793(cfg, alliance);
  // stored FUEL visual (instanced)
  const r = FUEL.radius;
  const geo = new THREE.SphereGeometry(r, 32, 22);
  // squeezed FUEL flattens where it's pressed (Hopper.touchPatches; fuelSkin.js squashable)
  const MAX = 260; // room for whatever the hopper holds
  const mat = new THREE.MeshStandardMaterial({ map: fuelTexture(), roughness: 0.72 });
  const touch = squashable(geo, mat, MAX, r);
  const inst = new THREE.InstancedMesh(geo, mat, MAX);
  inst.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  inst.frustumCulled = false;
  inst.castShadow = true;
  inst.count = 0;
  m.root.add(inst);
  m.storedMesh = inst;
  m.storedTouch = touch;
  m.root.traverse((o) => {
    if (o.isMesh && o.material !== M.poly && o.material !== M.polyTint) o.castShadow = true;
  });
  return m;
}

export function addClimberVisual(model, cfg) {
  const g = new THREE.Group();
  const L = cfg.frame.length;
  g.position.set(-L / 2 + 0.06, 0, 0);
  const mat = std(0x9aa0a8, 0.35, 0.85);
  for (const s of [-1, 1]) bx(0.04, 0.5, 0.04, mat, g, 0, 0.3, s * 0.2);
  const hook = new THREE.Group();
  hook.position.set(0, 0.55, 0);
  g.add(hook);
  bx(0.04, 0.04, 0.5, mat, hook, 0, 0, 0);
  for (const s of [-1, 1]) bx(0.08, 0.02, 0.03, mat, hook, 0.04, 0.02, s * 0.2);
  model.root.add(g);
  model.climber = { group: g, hook };
}
