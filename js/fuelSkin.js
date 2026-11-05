import * as THREE from 'three';
import { DAMAGE } from './fuelDamage.js';

const BROKEN = DAMAGE.broken;

// FUEL's skin, wrapped round the sphere (longitude across, latitude down): the lemon yellow of
// the official ball (AndyMark's photos: about sRGB 233, 206, 1), its molding's mid seam, a black
// print on one side and a date code on the other (the real ball's are the FIRST and AndyMark
// logos and a three-digit Julian date above the seam), so its spin shows from any side
let skin = null;
export function fuelTexture() {
  if (skin) return skin;
  const W = 512, H = 256, c = document.createElement('canvas');
  c.width = W; c.height = H;
  const g = c.getContext('2d');
  g.fillStyle = '#e9ce02';
  g.fillRect(0, 0, W, H);
  // the seam round the middle
  g.fillStyle = '#c4a900';
  g.fillRect(0, H / 2 - 1.5, W, 3);
  g.fillStyle = '#161616';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  // the print: two lines, the upper bolder (stretched a little as it nears the pole)
  g.font = 'italic 900 30px Arial, sans-serif';
  g.save(); g.translate(W * 0.25, H * 0.39); g.scale(1.25, 1); g.fillText('FUEL', 0, 0); g.restore();
  g.font = '700 18px Arial, sans-serif';
  g.save(); g.translate(W * 0.25, H * 0.6); g.scale(1.1, 1); g.fillText('REBUILT 2026', 0, 0); g.restore();
  // the date code, small, just above the seam on the far side
  g.font = '700 16px Arial, sans-serif';
  g.fillText('026', W * 0.75, H * 0.44);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return (skin = t);
}

// The skin's material: real FUEL are molded foam with a smooth, vivid yellow skin that's only
// slightly reflective (a thin, soft clearcoat: a dull sheen of the arena's lights, no mirror
// highlights); the open-cell foam shows only where the skin is torn (squashable's damage)
export function fuelMaterial() {
  const m = new THREE.MeshPhysicalMaterial({ map: fuelTexture(), roughness: 0.55, metalness: 0, clearcoat: 0.25, clearcoatRoughness: 0.4 });
  // (the shadow map from its front faces: from its back faces (three.js's default), the carpet
  // under it, looked up a normal bias above itself, is inside the ball and in front of its far
  // side, so the spot it touches came out lit inside a ring of shadow)
  m.shadowSide = THREE.FrontSide;
  return m;
}

// Squeezed FUEL flattens where it's pressed: up to four contact patches per instance (touch0..3:
// the patch's normal, in the instance's own frame, and its distance from the center). Foam gives
// around a contact too, not just on it, so the sphere eases into each flat patch over a band that
// widens the harder it's pressed (a smooth minimum of the sphere and the contact plane). Adds the
// attributes to geo (n instances, each with no patch: a plane beyond the ball) and the squash to
// mat; returns { touch: the four attributes, dmg }.
// damage: also each instance's wear (dmg: [damage + 4 x seed, x, y, z], js/fuelDamage.js wearSlot): patches of torn skin
// where a noise over the ball (in its own frame, so they turn with it) is over a level that falls
// as the damage grows, on the side it was worn, each with a torn edge (a lip, the skin's thickness, and a pale rim of
// stretched skin); inside, the foam: grayer, duller yellow, rough, no clearcoat, pitted with open
// cells. Past DAMAGE.broken a chunk is torn out: a cap of the ball pushed in to a ragged bowl.
export function squashable(geo, mat, n, r, damage = false) {
  const touch = [0, 1, 2, 3].map((j) => {
    const a = new THREE.InstancedBufferAttribute(new Float32Array(4 * n), 4);
    for (let i = 0; i < n; i++) { a.array[4 * i + 1] = 1; a.array[4 * i + 3] = 1; }
    a.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('touch' + j, a);
    return a;
  });
  let dmg = null;
  if (damage) {
    dmg = new THREE.InstancedBufferAttribute(new Float32Array(4 * n), 4);
    dmg.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('fdmg', dmg);
  }
  const R = r.toFixed(4), B = BROKEN.toFixed(2);
  const crater = `float fk = floor(fdmg.x / 4.0), fd = fdmg.x - 4.0 * fk;
vFObj = position / ${R}; vFDmg = vec2(fd, (fk + 0.5) / 256.0); vFW = fdmg.yzw; vFC = vec4(0.0, 1.0, 0.0, 2.0);
if (fd >= ${B}) {
  // the torn-out chunk: a cap round where it was worn (or, worn all over, a direction of its own),
  // wider the worse it is
  vec3 cd = length(fdmg.yzw) > 0.2 ? normalize(fdmg.yzw)
    : normalize(vec3(fract(vFDmg.y * 113.7), fract(vFDmg.y * 57.3 + 0.31), fract(vFDmg.y * 211.9 + 0.67)) - 0.5 + 1e-3);
  float kb = clamp(fd / ${B} - 1.0, 0.0, 1.0);
  float ca = mix(0.78, 0.5, kb);
  vFC = vec4(cd, ca);
  float u = clamp((dot(normalize(position), cd) - ca) / (1.0 - ca), 0.0, 1.0);
  float A = mix(0.3, 0.45, kb);
  sqP *= 1.0 - A * sqrt(u);
  if (u > 0.0) {
    // (the bowl's own normal: r(t) = 1 - A sqrt(u) round cd, n ~ r o - dr/dt t^)
    vec3 on = normalize(position);
    float ct = dot(on, cd), st = sqrt(max(1.0 - ct * ct, 1e-6));
    float drdt = A * 0.5 * st / ((1.0 - ca) * sqrt(max(u, 0.02)));
    objectNormal = normalize((1.0 - A * sqrt(u)) * on - drdt * (ct * on - cd) / st);
  }
}`;
  mat.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', `#include <common>
attribute vec4 touch0; attribute vec4 touch1; attribute vec4 touch2; attribute vec4 touch3;
${damage ? 'attribute vec4 fdmg; varying vec3 vFObj; varying vec2 vFDmg; varying vec3 vFW; varying vec4 vFC;' : ''}
void squash(vec4 c, inout vec3 p, inout vec3 nrm) {
  float k = 0.01 + 1.2 * max(0.0, ${R} - c.w); // blend band: wider for a deeper squash
  float h = dot(p, c.xyz);
  float q = max(k - abs(h - c.w), 0.0) / k;
  float hs = min(h, c.w) - q * q * k * 0.25;
  if (hs < h) {
    p -= c.xyz * (h - hs);
    nrm = normalize(mix(c.xyz, nrm, clamp(0.5 + 0.5 * (c.w - h) / k, 0.0, 1.0)));
  }
}`)
      .replace('#include <beginnormal_vertex>', `#include <beginnormal_vertex>
vec3 sqP = position;
${damage ? crater : ''}
squash(touch0, sqP, objectNormal); squash(touch1, sqP, objectNormal);
squash(touch2, sqP, objectNormal); squash(touch3, sqP, objectNormal);`)
      .replace('#include <begin_vertex>', 'vec3 transformed = sqP;');
    if (!damage) return;
    sh.fragmentShader = sh.fragmentShader
      .replace('#include <common>', `#include <common>
varying vec3 vFObj; varying vec2 vFDmg; varying vec3 vFW; varying vec4 vFC;
float fHash(vec3 p) { p = fract(p * 0.3183099 + 0.1); p *= 17.0; return fract(p.x * p.y * p.z * (p.x + p.y + p.z)); }
float fNoise(vec3 x) {
  vec3 i = floor(x), f = fract(x); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(mix(fHash(i), fHash(i + vec3(1, 0, 0)), f.x), mix(fHash(i + vec3(0, 1, 0)), fHash(i + vec3(1, 1, 0)), f.x), f.y),
             mix(mix(fHash(i + vec3(0, 0, 1)), fHash(i + vec3(1, 0, 1)), f.x), mix(fHash(i + vec3(0, 1, 1)), fHash(i + vec3(1, 1, 1)), f.x), f.y), f.z);
}
// open-cell foam: distances to the nearest two cell centers (F1, F2) of a jittered 3D grid, and
// the unit vectors from those centers to x (the gradients of F1 and F2)
vec2 fCell(vec3 x, out vec3 g1, out vec3 g2) {
  vec3 i = floor(x), f = fract(x), r1 = vec3(1.0), r2 = vec3(1.0);
  float d1 = 8.0, d2 = 8.0;
  for (int z = -1; z <= 1; z++) for (int y = -1; y <= 1; y++) for (int w = -1; w <= 1; w++) {
    vec3 g = vec3(float(w), float(y), float(z));
    vec3 r = f - g - vec3(fHash(i + g), fHash(i + g + 19.1), fHash(i + g + 47.3)) * 0.85;
    float d = dot(r, r);
    if (d < d1) { d2 = d1; r2 = r1; d1 = d; r1 = r; } else if (d < d2) { d2 = d; r2 = r; }
  }
  g1 = normalize(r1); g2 = normalize(r2);
  return sqrt(vec2(d1, d2));
}
// a pore's depth (0 on the struts between cells, 1 at a cell's middle: rounded bowls) at x * k,
// and its gradient in x (analytic, so the bump is smooth per pixel, not per 2x2 quad)
float fPore(vec3 x, float k, out vec3 grad) {
  vec3 g1, g2;
  vec2 c = fCell(x * k, g1, g2);
  float s = clamp((c.y - c.x) / 0.5, 0.0, 1.0), t = s * s * (3.0 - 2.0 * s);
  float dt = s > 0.0 && s < 1.0 ? 6.0 * s * (1.0 - s) / 0.5 : 0.0;
  grad = (2.0 - 2.0 * t) * dt * (g2 - g1) * k;
  return t * (2.0 - t);
}
// a bump on the view-space normal N from the height's screen derivatives (m per pixel)
vec3 fBumpD(vec3 N, float hx, float hy) {
  vec3 p = -vViewPosition, sx = dFdx(p), sy = dFdy(p);
  vec3 r1 = cross(sy, N), r2 = cross(N, sx);
  float det = dot(sx, r1);
  return normalize(abs(det) * N - sign(det) * (hx * r1 + hy * r2));
}`)
      .replace('#include <map_fragment>', `#include <map_fragment>
float fDm = 0.0, fHgt = 0.0, fCr = 0.0; vec3 fGrd = vec3(0.0);
if (vFDmg.x > 0.001) {
  // (where: a broad noise; the ragged edge: a finer one on top)
  vec3 o = normalize(vFObj), q = o * 2.2 + vFDmg.y * vec3(71.3, 37.9, 53.1);
  float nz = 0.72 * fNoise(q) + 0.2 * fNoise(q * 3.1 + 5.1) + 0.08 * fNoise(q * 9.0 + 9.7);
  float thr = 0.84 - 0.36 * sqrt(min(vFDmg.x, 1.0)) - 0.06 * max(vFDmg.x - 1.0, 0.0);
  // only on the side it was worn: as much higher away from there as the wear is in one place (a
  // FUEL knocked about from all sides wears all over)
  float wc = min(length(vFW), 1.0);
  if (wc > 0.0) thr += wc * (0.42 * (0.5 - 0.5 * dot(o, vFW / max(length(vFW), 1e-4))) - 0.06);
  float aa = fwidth(nz) + 0.002, e = nz - thr;
  fDm = smoothstep(-aa, aa, e);
  // the torn-out chunk (broken): its ragged rim
  if (vFC.w < 1.5) fCr = smoothstep(-0.015, 0.015, dot(o, vFC.xyz) - vFC.w + 0.07 * (nz - 0.5));
  float foam = max(fDm, fCr);
  // torn foam: a ragged surface (lumps ~6 mm and ~2 mm) of fine open cells (pores ~0.6 mm and
  // ~0.3 mm, bowls between thin struts: bumped, and darkened in their depths); the skin's
  // thickness at the patch's edge (the foam ~1.5 mm under it). Each scale fades out before its
  // cells get smaller than ~3 pixels (no grain or sparkle from afar): the pores only show close up.
  float px = length(fwidth(o));
  float c0 = clamp(2.0 - px * 60.0, 0.0, 1.0), c0b = clamp(2.0 - px * 200.0, 0.0, 1.0);
  float c1 = clamp(2.0 - px * 330.0, 0.0, 1.0), c2 = clamp(2.0 - px * 720.0, 0.0, 1.0);
  vec3 fo = o + vFDmg.y * vec3(3.7, 1.3, 2.9);
  float lump = (fNoise(fo * 11.0) - 0.5) * c0 + 0.45 * (fNoise(fo * 36.0 + 3.1) - 0.5) * c0b;
  float pore = 0.0, pore2 = 0.0;
  vec3 gp = vec3(0.0), gp2 = vec3(0.0);
  if (foam > 0.0 && c1 > 0.0) {
    pore = fPore(fo, 120.0, gp) * c1;
    if (c2 > 0.0) pore2 = fPore(fo + 0.13, 250.0, gp2) * c2;
  }
  // (pores too small to draw still darken the foam by their average)
  float cav = clamp(0.5 * mix(0.55, pore, c1) + 0.25 * mix(0.55, pore2, c2) - 0.4 * lump, 0.0, 1.0);
  fHgt = foam * (-0.0015 + 0.001 * lump);
  fGrd = -foam * (0.00025 * c1 * gp + 0.0001 * c2 * gp2);
  vec3 foamCol = vec3(0.43, 0.36, 0.08) * (1.05 - 0.45 * cav) * (1.0 - 0.3 * fCr);
  // (a pale rim of stretched skin round each tear)
  float rim = smoothstep(-0.015, -aa, e) * (1.0 - fDm);
  diffuseColor.rgb = mix(diffuseColor.rgb, diffuseColor.rgb * vec3(1.0, 0.97, 0.8), 0.6 * rim);
  diffuseColor.rgb = mix(diffuseColor.rgb, foamCol, foam);
  fDm = foam;
}`)
      .replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
roughnessFactor = mix(roughnessFactor, 0.93, fDm);`)
      .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
if (vFDmg.x > 0.001) {
  vec3 fo3 = normalize(vFObj);
  normal = fBumpD(normal, dFdx(fHgt) + dot(fGrd, dFdx(fo3)), dFdy(fHgt) + dot(fGrd, dFdy(fo3)));
}`)
      .replace('#include <lights_physical_fragment>', `#include <lights_physical_fragment>
#ifdef USE_CLEARCOAT
material.clearcoat *= 1.0 - fDm;
#endif`);
  };
  mat.customProgramCacheKey = () => 'fuel' + (damage ? 'd' : '');
  return { touch, dmg };
}

// Contact shadows: under each FUEL near the carpet a soft dark spot, darkest where it touches,
// fading and spreading as it rises (so it's grounded with no shadow map too, on Lowest). One
// instanced quad for all of them: place(instance matrices) puts one under each drawn FUEL less
// than LIFT above the carpet and draws only those.
export function contactShadows(n, r) {
  const LIFT = 0.4;
  const geo = new THREE.PlaneGeometry(2, 2).rotateX(-Math.PI / 2);
  const fade = new THREE.InstancedBufferAttribute(new Float32Array(n), 1);
  fade.setUsage(THREE.DynamicDrawUsage);
  geo.setAttribute('fade', fade);
  const mat = new THREE.ShaderMaterial({
    transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
    vertexShader: `attribute float fade; varying vec2 vUv; varying float vFade;
void main() { vUv = position.xz; vFade = fade; gl_Position = projectionMatrix * modelViewMatrix * instanceMatrix * vec4(position, 1.0); }`,
    fragmentShader: `varying vec2 vUv; varying float vFade;
void main() {
  float d2 = dot(vUv, vUv);
  float a = vFade * (0.55 * exp(-d2 * 9.0) + 0.3 * exp(-d2 * 3.0)) * (1.0 - smoothstep(0.7, 1.0, d2));
  gl_FragColor = vec4(0.0, 0.0, 0.0, a);
}`,
  });
  const mesh = new THREE.InstancedMesh(geo, mat, n);
  mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  mesh.frustumCulled = false;
  mesh.renderOrder = -1; // (before the other see-through things: it lies on the carpet)
  mesh.userData.noMerge = true;
  mesh.count = 0;
  const out = mesh.instanceMatrix.array, f = fade.array;
  mesh.place = (inst) => {
    let k = 0;
    for (let i = 0; i < n; i++) {
      const o = 16 * i;
      if (inst[o] === 0 && inst[o + 1] === 0 && inst[o + 2] === 0) continue; // hidden
      const h = inst[o + 13] - r;
      if (h > LIFT || h < -0.05) continue;
      const u = Math.max(0, h) / LIFT, s = r * (1.05 + 1.6 * u), p = 16 * k;
      out.fill(0, p, p + 16);
      out[p] = s; out[p + 5] = 1; out[p + 10] = s; out[p + 15] = 1;
      out[p + 12] = inst[o + 12]; out[p + 13] = 0.002; out[p + 14] = inst[o + 14];
      f[k++] = (1 - u) * (1 - u);
    }
    mesh.count = k;
    mesh.instanceMatrix.needsUpdate = true;
    fade.needsUpdate = true;
  };
  return mesh;
}
