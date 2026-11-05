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

// The skin's material: real FUEL are molded foam with a smooth, glossy, vivid yellow skin (a
// clearcoat over the color, the arena's lights and reflections in it); the open-cell foam shows
// only where the skin is torn (squashable's damage)
export function fuelMaterial() {
  const m = new THREE.MeshPhysicalMaterial({ map: fuelTexture(), roughness: 0.42, metalness: 0, clearcoat: 1, clearcoatRoughness: 0.08 });
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
// damage: also each instance's wear (dmg: [damage, seed], js/fuelDamage.js): patches of torn skin
// where a noise over the ball (in its own frame, so they turn with it) is over a level that falls
// as the damage grows, each with a torn edge (a lip, the skin's thickness, and a pale rim of
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
    dmg = new THREE.InstancedBufferAttribute(new Float32Array(2 * n), 2);
    dmg.setUsage(THREE.DynamicDrawUsage);
    geo.setAttribute('fdmg', dmg);
  }
  const R = r.toFixed(4), B = BROKEN.toFixed(2);
  const crater = `vFObj = position / ${R}; vFDmg = fdmg; vFC = vec4(0.0, 1.0, 0.0, 2.0);
if (fdmg.x >= ${B}) {
  // the torn-out chunk: a cap round a direction of its own, wider the worse it is
  vec3 cd = normalize(vec3(fract(fdmg.y * 113.7), fract(fdmg.y * 57.3 + 0.31), fract(fdmg.y * 211.9 + 0.67)) - 0.5 + 1e-3);
  float kb = clamp(fdmg.x / ${B} - 1.0, 0.0, 1.0);
  float ca = mix(0.78, 0.5, kb);
  vFC = vec4(cd, ca);
  float u = clamp((dot(normalize(position), cd) - ca) / (1.0 - ca), 0.0, 1.0);
  sqP *= 1.0 - mix(0.3, 0.45, kb) * sqrt(u);
}`;
  mat.onBeforeCompile = (sh) => {
    sh.vertexShader = sh.vertexShader
      .replace('#include <common>', `#include <common>
attribute vec4 touch0; attribute vec4 touch1; attribute vec4 touch2; attribute vec4 touch3;
${damage ? 'attribute vec2 fdmg; varying vec3 vFObj; varying vec2 vFDmg; varying vec4 vFC;' : ''}
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
varying vec3 vFObj; varying vec2 vFDmg; varying vec4 vFC;
float fHash(vec3 p) { p = fract(p * 0.3183099 + 0.1); p *= 17.0; return fract(p.x * p.y * p.z * (p.x + p.y + p.z)); }
float fNoise(vec3 x) {
  vec3 i = floor(x), f = fract(x); f = f * f * (3.0 - 2.0 * f);
  return mix(mix(mix(fHash(i), fHash(i + vec3(1, 0, 0)), f.x), mix(fHash(i + vec3(0, 1, 0)), fHash(i + vec3(1, 1, 0)), f.x), f.y),
             mix(mix(fHash(i + vec3(0, 0, 1)), fHash(i + vec3(1, 0, 1)), f.x), mix(fHash(i + vec3(0, 1, 1)), fHash(i + vec3(1, 1, 1)), f.x), f.y), f.z);
}
// a bump of height h (m) on the view-space normal N (screen-space derivatives)
vec3 fBump(vec3 N, float h) {
  vec3 p = -vViewPosition, sx = dFdx(p), sy = dFdy(p);
  vec3 r1 = cross(sy, N), r2 = cross(N, sx);
  float det = dot(sx, r1);
  return normalize(abs(det) * N - sign(det) * (dFdx(h) * r1 + dFdy(h) * r2));
}`)
      .replace('#include <map_fragment>', `#include <map_fragment>
float fDm = 0.0, fHgt = 0.0, fCr = 0.0;
if (vFDmg.x > 0.001) {
  // (where: a broad noise; the ragged edge: a finer one on top)
  vec3 o = normalize(vFObj), q = o * 2.2 + vFDmg.y * vec3(71.3, 37.9, 53.1);
  float nz = 0.72 * fNoise(q) + 0.2 * fNoise(q * 3.1 + 5.1) + 0.08 * fNoise(q * 9.0 + 9.7);
  float thr = 0.84 - 0.36 * sqrt(min(vFDmg.x, 1.0)) - 0.06 * max(vFDmg.x - 1.0, 0.0);
  float aa = fwidth(nz) + 0.002, e = nz - thr;
  fDm = smoothstep(-aa, aa, e);
  // the torn-out chunk (broken): its ragged rim
  if (vFC.w < 1.5) fCr = smoothstep(-0.015, 0.015, dot(o, vFC.xyz) - vFC.w + 0.07 * (nz - 0.5));
  float foam = max(fDm, fCr);
  // open cells (~3 mm), and the skin's thickness at the patch's edge (the foam ~1.5 mm under it)
  float cells = fNoise(o * 30.0 + vFDmg.y * 13.0) * 0.7 + fNoise(o * 70.0) * 0.3;
  fHgt = -0.0015 * foam + 0.0009 * cells * foam;
  vec3 foamCol = vec3(0.50, 0.43, 0.12) * (0.78 + 0.4 * cells) * (1.0 - 0.3 * fCr);
  // (a pale rim of stretched skin round each tear)
  float rim = smoothstep(-0.045, -aa, e) * (1.0 - fDm);
  diffuseColor.rgb = mix(diffuseColor.rgb, mix(diffuseColor.rgb, vec3(0.85, 0.78, 0.42), 0.45), rim);
  diffuseColor.rgb = mix(diffuseColor.rgb, foamCol, foam);
  fDm = foam;
}`)
      .replace('#include <roughnessmap_fragment>', `#include <roughnessmap_fragment>
roughnessFactor = mix(roughnessFactor, 0.93, fDm);`)
      .replace('#include <normal_fragment_maps>', `#include <normal_fragment_maps>
if (vFDmg.x > 0.001) {
  if (fCr > 0.0) normal = normalize(mix(normal, normalize(cross(dFdx(vViewPosition), dFdy(vViewPosition))), fCr));
  normal = fBump(normal, fHgt);
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
