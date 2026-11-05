// The venue round the field, as at a district event or a regional in a gym: bleachers with a
// crowd in team shirts, the scoring table and the field-reset crew, each alliance's drive teams
// behind their driver stations, referees at the field's edges, lighting trusses over the field,
// pipe-and-drape with team banners and screens with the live score and match time. Drawn only:
// nothing here has a collider or touches the simulation. All of it is made here, in code (no
// images, no logos).
//
// Cheap to draw: what doesn't move is merged into one mesh per material, the people are two
// instanced meshes (seated, standing) whose idle sway and cheering are done in the vertex shader
// (no per-frame CPU work but a few uniforms), nothing casts a shadow, and the venue is left out of
// the bloom's extra pass (js/graphics.js). How much is built follows the graphics preset
// (PRESETS[k].venue): 0 a few plain blocks for depth, 1 a small venue, 2 a full gym, 3 a fuller
// crowd.
//
// Kept out of play and out of the cameras' way: the stands start 2.5 m past the side of the field,
// the trusses are 7 m up and outside the field's edges (so the Overhead view sees past them), the
// drive teams stand behind the glass, and the people close to the camera (your own drive team, at
// the Driver Station view) aren't drawn.
import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { HALF_L, HALF_W, FIELD_L, DRIVER_STATIONS, BLUE, RED } from './constants.js';

// a small seeded random (the same venue every time)
function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// team shirt colors (bright, saturated: a crowd at an event is a patchwork of them)
const SHIRTS = [0x1d4ed8, 0xdc2626, 0x16a34a, 0xf59e0b, 0x7c3aed, 0x0891b2, 0xea580c, 0xdb2777, 0x111827,
  0xf3f4f6, 0x65a30d, 0x0f766e, 0xfacc15, 0x9f1239, 0x1e3a8a, 0x6b7280, 0x4338ca, 0xb91c1c];

// the seat rows: rise and depth per row (m), seat height above a row's floor
const ROW_RISE = 0.4, ROW_D = 0.8, SEAT_H = 0.42;
// how far the stands start from the side of the field
const STAND_GAP = 2.5;
// lighting trusses: height, and their distance out from the field's sides
const TRUSS_Y = 7.2, TRUSS_OUT = 1.2;

const LEVELS = [
  null,
  { rows: 6, standX: 9, occ: 0.5, far: false, walls: false, backScreen: false },
  { rows: 8, standX: 12.5, occ: 0.62, far: true, walls: true, backScreen: true },
  { rows: 8, standX: 12.5, occ: 0.88, far: true, walls: true, backScreen: true },
];

// ------------------------------------------------------------------ geometry helpers
// collects geometry by material and merges each material's into one mesh
class Builder {
  constructor() { this.parts = new Map(); }
  add(key, geo) {
    if (!this.parts.has(key)) this.parts.set(key, []);
    this.parts.get(key).push(geo);
    return geo;
  }
  box(key, w, h, d, x, y, z, ry = 0) {
    const g = new THREE.BoxGeometry(w, h, d);
    if (ry) g.rotateY(ry);
    g.translate(x, y, z);
    return this.add(key, g);
  }
  // a thin square bar from a to b
  bar(key, a, b, t) {
    const dir = new THREE.Vector3().subVectors(b, a), len = dir.length();
    const g = new THREE.BoxGeometry(t, len, t);
    g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize()));
    g.translate((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2);
    return this.add(key, g);
  }
  meshes(mats, group) {
    for (const [key, geos] of this.parts) {
      const geo = mergeGeometries(geos, false);
      for (const g of geos) g.dispose();
      const m = new THREE.Mesh(geo, mats[key]);
      m.matrixAutoUpdate = false;
      m.updateMatrix();
      m.name = 'venue-' + key;
      group.add(m);
    }
    this.parts.clear();
  }
}

// ------------------------------------------------------------------ people
// Low-poly people, one geometry per pose. The color attribute isn't a color: r is the part (0
// shirt, 1 skin, 2 pants, 3 arm, 4 hair), g the arms' resting angle (radians, about the shoulder's
// x axis: negative swings them forward), b the shoulder's height. Facing +z, feet (standing) or
// seat (seated) at y = 0.
function personGeometry(seated) {
  const geos = [];
  const part = (g, p, armA, shY) => {
    g = g.toNonIndexed();
    const n = g.attributes.position.count, c = new Float32Array(3 * n);
    for (let i = 0; i < n; i++) { c[3 * i] = p; c[3 * i + 1] = armA; c[3 * i + 2] = shY; }
    g.setAttribute('color', new THREE.BufferAttribute(c, 3));
    g.deleteAttribute('uv');
    geos.push(g);
  };
  const B = (w, h, d, x, y, z) => new THREE.BoxGeometry(w, h, d).translate(x, y, z);
  const hip = seated ? 0 : 0.86;          // seat / hip height
  const shY = hip + 0.58, armA = seated ? -0.75 : -0.06;
  if (seated) {
    part(B(0.36, 0.14, 0.46, 0, 0.07, 0.15), 2, 0, 0);                       // thighs
    for (const s of [-1, 1]) part(B(0.12, 0.44, 0.13, s * 0.1, -0.18, 0.36), 2, 0, 0); // shins
  } else {
    for (const s of [-1, 1]) part(B(0.14, hip, 0.16, s * 0.1, hip / 2, 0), 2, 0, 0);    // legs
  }
  part(B(0.4, 0.6, 0.23, 0, hip + 0.3, 0), 0, 0, 0);                         // torso
  part(B(0.08, 0.08, 0.08, 0, hip + 0.64, 0), 1, 0, 0);                      // neck
  part(B(0.19, 0.23, 0.21, 0, hip + 0.79, 0.005), 1, 0, 0);                  // head
  part(B(0.2, 0.07, 0.22, 0, hip + 0.925, -0.005), 4, 0, 0);                 // hair
  for (const s of [-1, 1]) {
    part(B(0.09, 0.5, 0.1, s * 0.245, shY - 0.25, 0), 3, armA, shY);         // arms
    part(B(0.08, 0.1, 0.09, s * 0.245, shY - 0.55, 0), 1.5, armA, shY);      // hands (skin, move with the arm)
  }
  const g = mergeGeometries(geos, false);
  for (const x of geos) x.dispose();
  return g;
}

// the people's material: Lambert, the colors and the motion added in its shaders
function peopleMaterial(uniforms) {
  const m = new THREE.MeshLambertMaterial({ vertexColors: true });
  m.onBeforeCompile = (sh) => {
    Object.assign(sh.uniforms, uniforms);
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', `#include <common>
attribute vec4 aInfo;  // seed, which alliance they're for (-1 blue .. 1 red), how much they cheer, referee stripes
attribute vec3 aShirt;
uniform float uTime, uCheerB, uCheerR, uLive;
varying float vStripe, vLx;`)
      .replace('#include <color_vertex>', `
float part = color.r, seed = aInfo.x, ph = seed * 6.2831853;
bool isArm = part > 1.25 && part < 3.5 && part != 2.0;
// colors: the shirt (per person), skin, pants and hair from the seed
vec3 skin = mix(vec3(0.62, 0.40, 0.29), vec3(0.13, 0.07, 0.045), fract(seed * 7.13));
vec3 pants = fract(seed * 3.71) > 0.72 ? vec3(0.33, 0.27, 0.17) : mix(vec3(0.02, 0.025, 0.04), vec3(0.04, 0.07, 0.14), fract(seed * 5.3));
vec3 hair = mix(vec3(0.01, 0.008, 0.006), vec3(0.20, 0.11, 0.05), pow(fract(seed * 11.3), 2.0));
vColor = part < 0.5 ? aShirt : part < 1.75 ? skin : part < 2.5 ? pants : part < 3.5 ? aShirt : hair;
vStripe = aInfo.w * ((part < 0.5 || (part > 2.5 && part < 3.5)) ? 1.0 : 0.0);
vLx = position.x;
// cheering: for the alliance that scored (more so its fans), some more than others
float ch = aInfo.z * clamp(uCheerB * (0.6 - 0.4 * aInfo.y) + uCheerR * (0.6 + 0.4 * aInfo.y), 0.0, 1.0);
ch *= 0.35 + 0.65 * fract(seed * 5.31);
float hop = ch * 0.09 * max(0.0, sin(uTime * 10.0 + ph * 4.0));
float armA = color.g - ch * (1.9 + 0.7 * fract(seed * 3.3)) + 0.18 * ch * sin(uTime * 13.0 + ph * 2.0);
float ca = cos(armA), sa = sin(armA);
float sway = (0.02 + 0.02 * uLive) * aInfo.z * sin(uTime * (0.5 + 0.6 * fract(seed * 9.7)) + ph);`)
      .replace('#include <beginnormal_vertex>', `#include <beginnormal_vertex>
if (isArm) objectNormal = vec3(objectNormal.x, objectNormal.y * ca - objectNormal.z * sa, objectNormal.y * sa + objectNormal.z * ca);`)
      .replace('#include <begin_vertex>', `#include <begin_vertex>
if (isArm) { vec3 q = transformed - vec3(0.0, color.b, 0.0); transformed = vec3(q.x, q.y * ca - q.z * sa + color.b, q.y * sa + q.z * ca); }
transformed.x += sway * max(transformed.y, 0.0);
transformed.y += hop + (part > 0.75 && part < 1.25 || part > 3.5 ? 0.008 * sin(uTime * 1.7 + ph * 3.0) : 0.0);`);
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', '#include <common>\nvarying float vStripe, vLx;')
      .replace('#include <color_fragment>', `#include <color_fragment>
if (vStripe > 0.5) diffuseColor.rgb = fract(vLx * 11.0) < 0.5 ? vec3(0.02) : vec3(0.75);`);
  };
  m.customProgramCacheKey = () => 'venue-people';
  return m;
}

// ------------------------------------------------------------------ banners and the screen
// generic team banners: a color, a stripe or two, a simple emblem and a team-style number
function bannerAtlas(r) {
  const cols = 8, rows = 2, cw = 128, chh = 256;
  const c = document.createElement('canvas');
  c.width = cols * cw; c.height = rows * chh;
  const g = c.getContext('2d');
  const hex = (n) => '#' + n.toString(16).padStart(6, '0');
  for (let i = 0; i < cols * rows; i++) {
    const x = (i % cols) * cw, y = Math.floor(i / cols) * chh;
    const bg = SHIRTS[Math.floor(r() * SHIRTS.length)];
    let fg = SHIRTS[Math.floor(r() * SHIRTS.length)];
    if (fg === bg) fg = bg === 0xf3f4f6 ? 0x111827 : 0xf3f4f6;
    g.fillStyle = hex(bg); g.fillRect(x, y, cw, chh);
    g.fillStyle = hex(fg);
    // stripes
    if (r() < 0.6) { g.fillRect(x, y + 18, cw, 8); g.fillRect(x, y + chh - 26, cw, 8); }
    if (r() < 0.4) { g.save(); g.beginPath(); g.rect(x, y, cw, chh); g.clip(); g.translate(x, y + chh * 0.62); g.rotate(-0.35); g.fillRect(-20, 0, cw + 60, 14); g.restore(); }
    // emblem: a circle, a triangle, a cog or a star
    const ex = x + cw / 2, ey = y + 78, R = 34, kind = Math.floor(r() * 4);
    g.beginPath();
    if (kind === 0) g.arc(ex, ey, R, 0, Math.PI * 2);
    else if (kind === 1) { g.moveTo(ex, ey - R); g.lineTo(ex + R, ey + R * 0.8); g.lineTo(ex - R, ey + R * 0.8); }
    else {
      const n = kind === 2 ? 16 : 10;
      for (let k = 0; k < n; k++) {
        const a = (k / n) * Math.PI * 2, rr = kind === 2 ? (k % 2 ? R * 0.78 : R) : (k % 2 ? R * 0.45 : R);
        g.lineTo(ex + rr * Math.sin(a), ey - rr * Math.cos(a));
      }
    }
    g.closePath(); g.fill();
    if (kind === 2) { g.fillStyle = hex(bg); g.beginPath(); g.arc(ex, ey, R * 0.35, 0, Math.PI * 2); g.fill(); g.fillStyle = hex(fg); }
    // the number
    g.font = 'bold 44px Arial, Helvetica, sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    g.fillText(String(Math.floor(100 + r() * 9800)), ex, y + 170);
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return { tex: t, cell: (i) => ({ u0: (i % cols) / cols, v0: 1 - (Math.floor(i / cols) + 1) / rows, du: 1 / cols, dv: 1 / rows }) };
}

// a plane w x h at (x, y, z) facing the yaw ry (0: +z), its uvs a cell of an atlas
function quad(w, h, x, y, z, ry, cell) {
  const g = new THREE.PlaneGeometry(w, h);
  if (cell) {
    const uv = g.attributes.uv;
    for (let i = 0; i < uv.count; i++) uv.setXY(i, cell.u0 + uv.getX(i) * cell.du, cell.v0 + uv.getY(i) * cell.dv);
  }
  if (ry) g.rotateY(ry);
  return g.translate(x, y, z);
}

// ------------------------------------------------------------------ the venue
export class Venue {
  constructor(scene) {
    this.scene = scene;
    this.root = new THREE.Group();
    this.root.name = 'venue';
    scene.add(this.root);
    this.level = -1;
    this.uniforms = { uTime: { value: 0 }, uCheerB: { value: 0 }, uCheerR: { value: 0 }, uLive: { value: 0 } };
    this.body = null;    // what the bloom's black pass leaves out (js/graphics.js)
    this.standing = [];  // the standing people: position, for hiding the ones at the camera
    this.screenT = 0;
    this.screenKey = '';
    this.t = 0;
  }

  setLevel(level) {
    if (level === this.level) return;
    this.level = level;
    this._clear();
    this.body = new THREE.Group();
    this.root.add(this.body);
    if (level <= 0) this._buildMinimal();
    else this._build(LEVELS[Math.min(3, level)]);
  }

  _clear() {
    const seen = new Set();
    this.root.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      for (const m of o.material ? [].concat(o.material) : []) {
        if (seen.has(m)) continue;
        seen.add(m);
        if (m.map) m.map.dispose();
        m.dispose();
      }
    });
    this.root.clear();
    this.standingMesh = null;
    this.standing = [];
    this.screen = null;
  }

  // Lowest: a few plain blocks of stands and the scoring table, for depth
  _buildMinimal() {
    const b = new Builder();
    for (const s of [1, -1]) for (let i = 0; i < 4; i++) b.box('stand', FIELD_L + 8, 0.6, 1.2, 0, 0.3 + i * 0.6, s * (HALF_W + 3.2 + i * 1.2));
    b.box('table', 6.06, 0.9, 0.72, 0, 0.45, 5.86);
    b.meshes({
      stand: new THREE.MeshLambertMaterial({ color: 0x1a1d24 }),
      table: new THREE.MeshLambertMaterial({ color: 0x33363d }),
    }, this.body);
  }

  _build(L) {
    const r = rng(2026);
    const b = new Builder();
    const lens = new Builder();   // what glows: the trusses' lights (outside this.body: bloom sees them)
    const banners = bannerAtlas(r);
    let bannerN = 0;
    const banner = (w, h, x, y, z, ry) => b.add('banner', quad(w, h, x, y, z, ry, banners.cell(bannerN++ % 16)));
    const seated = [], standing = [];
    const shirt = () => SHIRTS[Math.floor(r() * SHIRTS.length)];
    // a person: where, facing (yaw, 0 = +z), shirt, who they're for, how much they cheer, stripes
    const person = (list, x, y, z, yaw, o = {}) => list.push({ x, y, z, yaw, shirt: o.shirt ?? shirt(), lean: o.lean ?? (r() * 2 - 1), cheer: o.cheer ?? 1, stripe: o.stripe ?? 0, s: 0.93 + r() * 0.13 });

    // ---- the stands: rows of seats, solid underneath, a rail along the front, aisles between sections
    const stands = (side, x0, x1, rows, occ) => {
      const zF = side * (HALF_W + STAND_GAP + (side > 0 ? 0.9 : 0)); // (the far side leaves room for the scoring table)
      const len = x1 - x0, xc = (x0 + x1) / 2;
      for (let i = 0; i < rows; i++) {
        const h = i * ROW_RISE, zc = zF + side * (i + 0.5) * ROW_D;
        if (h > 0) b.box('stand', len, h, ROW_D, xc, h / 2, zc);
        // the seat: a bench at the back of the row
        b.box('seat', len, 0.07, 0.32, xc, h + SEAT_H - 0.035, zF + side * ((i + 1) * ROW_D - 0.2));
        b.box('stand', len, SEAT_H - 0.07, 0.06, xc, h + (SEAT_H - 0.07) / 2, zF + side * ((i + 1) * ROW_D - 0.34));
        // the people
        const nSeat = Math.floor(len / 0.52);
        for (let k = 0; k < nSeat; k++) {
          const x = x0 + (k + 0.5) * (len / nSeat);
          if (Math.abs(((x - x0) % 6.4) - 3.2) < 0.45) continue; // aisle
          if (r() > occ) continue;
          person(seated, x + (r() - 0.5) * 0.08, h + SEAT_H, zF + side * ((i + 1) * ROW_D - 0.24), side > 0 ? Math.PI : 0,
            { lean: Math.max(-1, Math.min(1, (r() * 2 - 1) * 0.8 + x / 14)) });
        }
      }
      // aisle rails and the front rail
      for (let x = x0 + 3.2; x < x1 - 0.5; x += 6.4) {
        const p = (i) => new THREE.Vector3(x, i * ROW_RISE + 0.9, zF + side * i * ROW_D);
        b.bar('metal', p(0), p(rows), 0.04);
        for (let i = 0; i <= rows; i += 2) b.bar('metal', new THREE.Vector3(x, i * ROW_RISE, zF + side * i * ROW_D), p(i), 0.04);
      }
      b.box('metal', len, 0.05, 0.05, xc, 1.0, zF - side * 0.05);
      for (let x = x0; x <= x1 + 0.01; x += len / Math.round(len / 2)) b.box('metal', 0.05, 1.0, 0.05, x, 0.5, zF - side * 0.05);
      return zF + side * rows * ROW_D;
    };
    const back = stands(-1, -L.standX, L.standX, L.rows, L.occ);
    if (L.far) {
      for (const s of [-1, 1]) stands(1, s > 0 ? 4 : -12.5, s > 0 ? 12.5 : -4, 5, L.occ * 0.85);
    }

    // ---- the gym's walls (inward-facing only: a camera outside sees through)
    if (L.walls) {
      const H = 11, zB = back - 1.2, zN = HALF_W + STAND_GAP + 0.9 + 5 * ROW_D + 1.2, xE = 21;
      b.add('wall', quad(2 * xE, H, 0, H / 2, zB, 0));
      b.add('wall', quad(2 * xE, H, 0, H / 2, zN, Math.PI));
      for (const s of [-1, 1]) b.add('wall', quad(zN - zB, H, s * xE, H / 2, (zN + zB) / 2, -s * Math.PI / 2));
      // a painted band round the gym
      b.add('accent', quad(2 * xE, 0.5, 0, 3.6, zB + 0.01, 0));
      b.add('accent', quad(2 * xE, 0.5, 0, 3.6, zN - 0.01, Math.PI));
      for (const s of [-1, 1]) b.add('accent', quad(zN - zB, 0.5, s * (xE - 0.01), 3.6, (zN + zB) / 2, -s * Math.PI / 2));
      // banners high on the back wall, over the stands
      for (let i = 0; i < 12; i++) banner(1.3, 2.6, -14.3 + i * 2.6, 6.2, zB + 0.02, 0);
      for (const s of [-1, 1]) for (let i = 0; i < 4; i++) banner(1.3, 2.6, s * (6 + i * 2.6), 6.2, zN - 0.02, Math.PI);
    }

    // ---- pipe and drape: 8 ft black drape on uprights, behind the driver stations and along the
    // scoring table's side, with team banners on it
    const DRAPE_H = 2.4;
    const drape = (ax, az, bx, bz) => {
      const len = Math.hypot(bx - ax, bz - az), n = Math.max(2, Math.round(len / 0.15));
      const g = new THREE.PlaneGeometry(len, DRAPE_H - 0.1, n, 1), p = g.attributes.position;
      for (let i = 0; i < p.count; i++) p.setZ(i, 0.045 * Math.sin((p.getX(i) / 0.3) * Math.PI));
      g.computeVertexNormals();
      const ry = Math.atan2(-(bz - az), bx - ax);
      g.rotateY(ry);
      g.translate((ax + bx) / 2, (DRAPE_H - 0.1) / 2 + 0.02, (az + bz) / 2);
      b.add('drape', g);
      const k = Math.round(len / 3);
      for (let i = 0; i <= k; i++) {
        const t = i / k, x = ax + (bx - ax) * t, z = az + (bz - az) * t;
        b.box('metal', 0.04, DRAPE_H, 0.04, x, DRAPE_H / 2, z);
        b.box('dark', 0.45, 0.02, 0.45, x, 0.01, z);
      }
      b.bar('metal', new THREE.Vector3(ax, DRAPE_H, az), new THREE.Vector3(bx, DRAPE_H, bz), 0.03);
    };
    const xD = HALF_L + 3.0, zDn = HALF_W + STAND_GAP - 0.4, zDf = HALF_W + 3.0;
    for (const s of [-1, 1]) {
      drape(s * xD, -zDn, s * xD, zDf);
      for (let i = 0; i < 4; i++) banner(0.9, 1.6, s * (xD - 0.07), 1.3, -4.5 + i * 3, -s * Math.PI / 2);
    }
    const xF = L.far ? 3.9 : xD;
    drape(-xF, zDf, xF, zDf);
    for (let x = -xF + 1.2; x < xF - 0.8; x += L.far ? 2.2 : 3.3) if (Math.abs(x) > 1.0) banner(0.9, 1.6, x, 1.3, zDf - 0.07, Math.PI);

    // ---- the scoring table, its volunteers and the field-reset crew's area
    {
      // (where the field's CAD has its scoring table, 240 x 29 x 36 in: drawn a little inside it,
      // so the CAD's covers it where it's loaded)
      const z = 5.86, top = 0.914;
      b.box('skirt', 6.06, top - 0.01, 0.72, 0, (top - 0.01) / 2, z);
      for (let i = 0; i < 5; i++) {
        const x = -2.4 + i * 1.2;
        b.box('dark', 0.34, 0.02, 0.24, x, top + 0.01, z + 0.12);                 // laptop
        b.add('dark', new THREE.BoxGeometry(0.34, 0.22, 0.01).rotateX(-0.25).translate(x, top + 0.12, z - 0.01));
        b.add('lcd', new THREE.PlaneGeometry(0.31, 0.19).translate(0, 0, 0.006).rotateX(-0.25).translate(x, top + 0.12, z - 0.01)); // its screen
        b.box('dark', 0.4, 0.6, 0.4, x, 0.3, z + 0.9);                            // stool
        person(seated, x, 0.62, z + 0.8, Math.PI, { shirt: i === 2 ? 0x1d4ed8 : 0x6b7280, cheer: 0 });
      }
      // field reset: FUEL carts and the crew at the corners
      for (const s of [-1, 1]) {
        const x = s * (HALF_L - 1.4);
        b.box('dark', 1.0, 0.6, 0.6, x, 0.45, HALF_W + 1.9);
        b.box('fuelBin', 0.9, 0.08, 0.5, x, 0.79, HALF_W + 1.9);
        for (let k = 0; k < 4; k++) b.box('dark', 0.06, 0.15, 0.06, x + (k % 2 - 0.5) * 0.8, 0.075, HALF_W + 1.9 + (Math.floor(k / 2) - 0.5) * 0.45);
        // (far enough back that the Broadcast camera sees the field's edge over them)
        for (let k = 0; k < 2; k++) person(standing, x + s * (0.9 + k * 0.6), 0, HALF_W + 2.3 + k * 0.3, Math.PI + (r() - 0.5), { shirt: 0xeab308, cheer: 0.2 });
      }
      // the field supervisor and the head referee by the table
      person(standing, -3.6, 0, HALF_W + 2.0, Math.PI, { shirt: 0x111827, cheer: 0 });
      person(standing, 3.6, 0, HALF_W + 2.0, Math.PI, { stripe: 1, cheer: 0 });
    }
    // referees on the far side of the field
    for (const s of [-1, 1]) person(standing, s * 3.0, 0, -(HALF_W + 1.0), s * 0.3, { stripe: 1, cheer: 0 });

    // ---- the drive teams: driver, operator and coach behind each driver station
    for (const a of [BLUE, RED]) {
      const sx = a === BLUE ? -1 : 1;
      for (const ds of DRIVER_STATIONS) {
        const zc = -sx * (HALF_W - ds.fy);
        const team = shirt(), yaw = sx < 0 ? Math.PI / 2 : -Math.PI / 2, lean = a === BLUE ? -1 : 1;
        for (const dz of [-0.38, 0.38]) person(standing, sx * (HALF_L + 0.62), 0, zc + dz, yaw, { shirt: team, lean, cheer: 0.6 });
        person(standing, sx * (HALF_L + 1.05), 0, zc + 0.1, yaw, { shirt: team, lean, cheer: 0.6 });
      }
    }

    // ---- lighting trusses: two along the field's sides, two across behind the driver stations,
    // on four towers; lights on the long ones aimed at the field
    {
      const zT = HALF_W + TRUSS_OUT, xT = HALF_L + 2.2, W = 0.3;
      const truss = (a, b2) => {
        // a 30 cm box truss from a to b2 (along x or z): four chords and zig-zag webbing
        const dir = new THREE.Vector3().subVectors(b2, a), len = dir.length();
        dir.normalize();
        const side = Math.abs(dir.y) > 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(-dir.z, 0, dir.x).normalize();
        const up = new THREE.Vector3().crossVectors(side, dir).normalize().multiplyScalar(W / 2);
        side.multiplyScalar(W / 2);
        const corners = [side.clone().add(up), side.clone().sub(up), side.clone().negate().sub(up), side.clone().negate().add(up)];
        for (const c of corners) b.bar('metal', a.clone().add(c), b2.clone().add(c), 0.035);
        const n = Math.round(len / 0.5);
        for (let i = 0; i < n; i++) {
          const p0 = a.clone().addScaledVector(dir, (len * i) / n), p1 = a.clone().addScaledVector(dir, (len * (i + 1)) / n);
          for (let f = 0; f < 4; f++) {
            const c0 = corners[f], c1 = corners[(f + 1) % 4];
            b.bar('metal', p0.clone().add(i % 2 ? c0 : c1), p1.clone().add(i % 2 ? c1 : c0), 0.018);
          }
        }
      };
      for (const s of [-1, 1]) {
        truss(new THREE.Vector3(-xT - 0.15, TRUSS_Y, s * zT), new THREE.Vector3(xT + 0.15, TRUSS_Y, s * zT));
        truss(new THREE.Vector3(s * xT, TRUSS_Y, -zT + 0.15), new THREE.Vector3(s * xT, TRUSS_Y, zT - 0.15));
        for (const t of [-1, 1]) truss(new THREE.Vector3(s * xT, 0, t * zT), new THREE.Vector3(s * xT, TRUSS_Y - 0.15, t * zT));
        for (const t of [-1, 1]) b.box('dark', 0.7, 0.03, 0.7, s * xT, 0.015, t * zT);
      }
      // the lights: a can hung under the truss, aimed down and in at the field
      const body = new THREE.CylinderGeometry(0.13, 0.16, 0.34, 10, 1);
      for (const s of [-1, 1]) {
        for (let x = -HALF_L + 0.6; x <= HALF_L - 0.5; x += 2.2) {
          const m = new THREE.Matrix4().compose(new THREE.Vector3(x, TRUSS_Y - 0.42, s * zT),
            new THREE.Quaternion().setFromEuler(new THREE.Euler(s * 0.55, 0, 0)), new THREE.Vector3(1, 1, 1));
          b.add('dark', body.clone().applyMatrix4(m));
          b.add('dark', new THREE.BoxGeometry(0.04, 0.3, 0.04).translate(x, TRUSS_Y - 0.27, s * zT));
          lens.add('lens', new THREE.CircleGeometry(0.135, 12).rotateX(Math.PI / 2).translate(0, -0.171, 0).applyMatrix4(m));
        }
      }
      body.dispose();
    }

    // ---- the screens: over each end's drape (facing the field), and high on the back wall
    const screens = [];
    for (const s of [-1, 1]) {
      const x = s * (xD + 0.2), y = 4.7, w = 4.8, h = 2.7;
      b.box('dark', 0.25, h + 0.24, w + 0.24, x + s * 0.13, y, 0);
      screens.push(quad(w, h, x, y, 0, -s * Math.PI / 2));
      for (const t of [-1, 1]) b.box('metal', 0.08, y - h / 2, 0.08, x + s * 0.2, (y - h / 2) / 2, t * (w / 2 - 0.3));
    }
    if (L.backScreen) {
      const zB = back - 1.2, y = 7.0, w = 6.4, h = 3.6;
      b.box('dark', w + 0.3, h + 0.3, 0.2, 0, y, zB + 0.1);
      screens.push(quad(w, h, 0, y, zB + 0.21, 0));
    }
    const sc = document.createElement('canvas');
    sc.width = 512; sc.height = 288;
    const stex = new THREE.CanvasTexture(sc);
    stex.colorSpace = THREE.SRGBColorSpace;
    stex.anisotropy = 4;
    const screenMat = new THREE.MeshBasicMaterial({ map: stex, toneMapped: false });
    const sm = new THREE.Mesh(mergeGeometries(screens, false), screenMat);
    for (const g of screens) g.dispose();
    sm.matrixAutoUpdate = false;
    sm.name = 'venue-screens';
    this.body.add(sm);
    this.screen = { canvas: sc, g: sc.getContext('2d'), tex: stex };
    this.screenKey = '';
    this._drawScreen(null);

    // ---- materials, and the merged meshes
    const lam = (color, o = {}) => new THREE.MeshLambertMaterial({ color, ...o });
    b.meshes({
      stand: lam(0x262a33),
      seat: lam(0x4a5568),
      metal: lam(0x9aa1ab),
      dark: lam(0x17181c),
      wall: lam(0x2a2d33),
      accent: lam(0x5b2333),
      drape: lam(0x101115, { side: THREE.DoubleSide }),
      banner: lam(0xffffff, { map: banners.tex }),
      skirt: lam(0x8a8f99),
      lcd: new THREE.MeshBasicMaterial({ color: 0x8fb4e8 }),
      fuelBin: lam(0xe0b81c),
    }, this.body);
    lens.meshes({ lens: new THREE.MeshStandardMaterial({ color: 0x000000, emissive: 0xfff4e0, emissiveIntensity: 1.6 }) }, this.root);

    // ---- the people: two instanced meshes, one material
    const mat = peopleMaterial(this.uniforms);
    const crowd = (list, seatedPose) => {
      if (!list.length) return null;
      const geo = personGeometry(seatedPose);
      const n = list.length, info = new Float32Array(4 * n), col = new Float32Array(3 * n);
      const mesh = new THREE.InstancedMesh(geo, mat, n);
      const m = new THREE.Matrix4(), q = new THREE.Quaternion(), c = new THREE.Color();
      list.forEach((p, i) => {
        m.compose(new THREE.Vector3(p.x, p.y, p.z), q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), p.yaw), new THREE.Vector3(p.s, p.s, p.s));
        mesh.setMatrixAt(i, m);
        info.set([r(), p.lean, p.cheer, p.stripe], 4 * i);
        c.setHex(p.shirt);
        col.set([c.r, c.g, c.b], 3 * i);
      });
      geo.setAttribute('aInfo', new THREE.InstancedBufferAttribute(info, 4));
      geo.setAttribute('aShirt', new THREE.InstancedBufferAttribute(col, 3));
      mesh.computeBoundingSphere();
      mesh.matrixAutoUpdate = false;
      mesh.name = seatedPose ? 'venue-crowd' : 'venue-people';
      this.body.add(mesh);
      return mesh;
    };
    crowd(seated, true);
    this.standingMesh = crowd(standing, false);
    this.standing = standing.map((p, i) => {
      const mm = new THREE.Matrix4();
      this.standingMesh.getMatrixAt(i, mm);
      return { pos: new THREE.Vector3(p.x, 1.25, p.z), m: mm, hidden: false };
    });
    this.count = { seated: seated.length, standing: standing.length };
  }

  // a volley went in (CheerWatch in js/sound.js): the crowd cheers, its fans more
  cheer(c) {
    const k = Math.min(1, 0.55 + c.burst / 30), u = this.uniforms;
    if (c.blue > 0) u.uCheerB.value = Math.max(u.uCheerB.value, k * Math.min(1, 0.4 + c.blue / c.burst));
    if (c.red > 0) u.uCheerR.value = Math.max(u.uCheerR.value, k * Math.min(1, 0.4 + c.red / c.burst));
  }

  // every frame: the time, the cheering dying down, your own drive team hidden from the Driver
  // Station camera, and the screens a few times a second
  // cut: the alliance wall's fade (js/wallCutaway.js: { fade 0..1, side }): the drive teams behind
  // that wall go with it, so they don't hide your robot and the FUEL either
  update(dt, game, camera, live, cut = null) {
    if (this.level <= 0) return;
    const u = this.uniforms;
    this.t += dt;
    u.uTime.value = this.t % 1000;
    u.uCheerB.value *= Math.exp(-dt / 1.6);
    u.uCheerR.value *= Math.exp(-dt / 1.6);
    u.uLive.value += ((live ? 1 : 0) - u.uLive.value) * Math.min(1, dt);
    // (match end: the winners' side cheers)
    const ph = game ? game.match.phase : null;
    if (ph === 'done' && this._phase && this._phase !== 'done') {
      const m = game.match, d = m.total(BLUE) - m.total(RED);
      if (d >= 0) u.uCheerB.value = 1;
      if (d <= 0) u.uCheerR.value = 1;
    }
    this._phase = ph;
    // the people right at the camera (your drive team, at the Driver Station view) aren't drawn,
    // nor those behind an alliance wall while it's faded out
    if (this.standingMesh) {
      let dirty = false;
      const cutting = cut && cut.fade > 0.3;
      this.standing.forEach((p, i) => {
        const hide = p.pos.distanceToSquared(camera.position) < 1.1 * 1.1
          || (cutting && p.pos.x * cut.side > HALF_L - 0.3 && Math.abs(p.pos.z) < HALF_W + 0.5);
        if (hide === p.hidden) return;
        p.hidden = hide;
        this.standingMesh.setMatrixAt(i, hide ? ZERO : p.m);
        dirty = true;
      });
      if (dirty) this.standingMesh.instanceMatrix.needsUpdate = true;
    }
    this.screenT -= dt;
    if (this.screen && this.screenT <= 0) {
      this.screenT = 0.25;
      this._drawScreen(game);
    }
  }

  // the scoreboard: both alliances' scores and teams, the match time and phase
  _drawScreen(game) {
    const m = game ? game.match : null;
    const teams = (a) => (game ? game.robots.filter((r) => r.alliance === a).map((r) => r.cfg.team).filter(Boolean).join('  ') : '');
    const key = m ? [m.total(BLUE), m.total(RED), m.clockText(), m.phaseLabel(), teams(BLUE), teams(RED)].join('|') : 'idle';
    if (key === this.screenKey) return;
    this.screenKey = key;
    const { g, canvas: c, tex } = this.screen, W = c.width, H = c.height;
    g.fillStyle = '#05070c'; g.fillRect(0, 0, W, H);
    g.textAlign = 'center'; g.textBaseline = 'middle';
    if (!m) {
      g.fillStyle = '#e5e7eb'; g.font = 'bold 44px Arial, Helvetica, sans-serif';
      g.fillText('QUALIFICATIONS', W / 2, H / 2 - 30);
      g.fillStyle = '#9ca3af'; g.font = '28px Arial, Helvetica, sans-serif';
      g.fillText('next match on deck', W / 2, H / 2 + 26);
    } else {
      const [sb, sr, clock, phase, tb, tr] = key.split('|');
      const panel = (x, color, score, tm) => {
        g.fillStyle = color; g.fillRect(x, 70, 180, 190);
        g.fillStyle = '#ffffff'; g.font = 'bold 96px Arial, Helvetica, sans-serif';
        g.fillText(score, x + 90, 158);
        g.font = 'bold 22px Arial, Helvetica, sans-serif';
        g.fillText(tm, x + 90, 235);
      };
      panel(12, '#1d4ed8', sb, tb);
      panel(W - 192, '#c81e1e', sr, tr);
      g.fillStyle = '#ffffff'; g.font = 'bold 64px Arial, Helvetica, sans-serif';
      g.fillText(clock, W / 2, 150);
      g.fillStyle = '#fbbf24'; g.font = 'bold 22px Arial, Helvetica, sans-serif';
      g.fillText(phase.slice(0, 13), W / 2, 215);
      g.fillStyle = '#e5e7eb'; g.font = 'bold 30px Arial, Helvetica, sans-serif';
      g.fillText('MATCH', W / 2, 36);
    }
    tex.needsUpdate = true;
  }
}
const ZERO = new THREE.Matrix4().makeScale(0, 0, 0);
