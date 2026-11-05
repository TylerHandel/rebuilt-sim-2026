// Motion blur for whatever spins fast on a robot (shooter flywheels, intake and indexer rollers,
// drive and swerve wheels), the way a camera or the eye sees it: past a few degrees a frame the
// sharp part fades into a "blurred" copy of itself, its look averaged round its axis (a ring
// where spokes or fingers sweep is see-through by as much as they leave open), instead of
// strobing or turning backwards (the wagon-wheel effect).
//
// Generic: every robot's mechanisms are posed in model.anim (js/robotModels.js), the swerve
// wheels in Robot.applyVisual (js/robot.js). While those run (begin()/end()) the Euler setters
// note which rotations were written; a part whose angle keeps moving the same way, fast, is a
// spinner. Its blurred copy is built the first time it's needed from its own triangles (sampled
// in cylindrical bins about the spin axis: per axial slab, the outer radius, how much of each
// ring is covered, and the average color) as a stepped surface of revolution, a child of the
// part (so it turns with it, and looks the same at any angle).
// Cost: no extra pass; one small mesh per spinning part (and the sharp part not drawn at all
// once fully blurred). Off on the Lowest preset (js/graphics.js PRESETS spinBlur).
// Visual only: nothing here is read by the simulation.
import * as THREE from 'three';

const TAU = Math.PI * 2;
// rad per frame: the blur starts fading in at LO and is complete at HI (a 6-spoke wheel strobes
// from pi/6 ~ 0.52 a frame)
const LO = 0.1, HI = 0.4;
// (axial bins: about one per cm, between the bounds; radial and angular bins, segments round)
const GRID = [null, { h: [6, 40], nr: 16, na: 96, seg: 24 }, { h: [8, 64], nr: 28, na: 128, seg: 40 }];
const HIDDEN = 30; // a layer no camera (or shadow) renders: the sharp part, once fully blurred

let level = 0, patched = false, active = false;
const touched = new Set();
const live = new Set(); // tracks with a blur showing (to undo when it's switched off)

function patchEuler() {
  if (patched) return;
  patched = true;
  const P = THREE.Euler.prototype;
  for (const k of ['x', 'y', 'z']) {
    const d = Object.getOwnPropertyDescriptor(P, k);
    Object.defineProperty(P, k, { get: d.get, set(v) { d.set.call(this, v); if (active) touched.add(this); }, configurable: true, enumerable: d.enumerable });
  }
  const set0 = P.set;
  P.set = function (...a) { const r = set0.apply(this, a); if (active) touched.add(this); return r; };
}

// the preset's level: 0 off, 1 coarse, 2 fine
export function setSpinBlur(l) {
  l = l | 0;
  if (l === level) return;
  level = l;
  for (const t of [...live]) { reset(t); drop(t); }
  if (level > 0 && typeof window !== 'undefined') patchEuler();
}

export function spinBlurBegin() {
  if (!level) return;
  touched.clear();
  active = true;
}

// root: the robot's drawn model (its rotations' owners are found under it)
export function spinBlurEnd(model, root) {
  if (!active) return;
  active = false;
  const S = model._spinBlur || (model._spinBlur = { owner: new WeakMap(), miss: new WeakSet(), tracks: new Map(), frame: 0 });
  if (++S.frame % 600 === 0) {
    S.miss = new WeakSet();
    // (a robot of a match that's over, let go)
    for (const t of live) { let n = t.o; while (n.parent) n = n.parent; if (!n.isScene) live.delete(t); }
  }
  for (const e of touched) {
    let o = S.owner.get(e);
    if (!o) {
      if (S.miss.has(e)) continue;
      root.traverse((c) => S.owner.set(c.rotation, c));
      o = S.owner.get(e);
      if (!o) { S.miss.add(e); continue; }
    }
    if (!S.tracks.has(o)) S.tracks.set(o, { o, prev: [e.x, e.y, e.z], c: -1, run: 0, sgn: 0, speed: 0, w: 0, ov: null, sig: '', chk: 0, mats: null, layers: null });
  }
  touched.clear();
  for (const t of S.tracks.values()) step(t);
}

// how far a rotation moved since the last frame (an angle kept in [-2pi, 2pi] can wrap round)
function delta(a, b) {
  let d = a - b;
  if (Math.abs(d) > Math.PI && Math.abs(a) <= TAU + 1e-6 && Math.abs(b) <= TAU + 1e-6) d = Math.atan2(Math.sin(d), Math.cos(d));
  return d;
}

function step(t) {
  const r = t.o.rotation, p = t.prev;
  const d = [delta(r.x, p[0]), delta(r.y, p[1]), delta(r.z, p[2])];
  p[0] = r.x; p[1] = r.y; p[2] = r.z;
  let c = 0;
  if (Math.abs(d[1]) > Math.abs(d[c])) c = 1;
  if (Math.abs(d[2]) > Math.abs(d[c])) c = 2;
  const dd = d[c], s = Math.sign(dd);
  // spinning: the same way, frame after frame (a swerve module flipping round, a hood or an
  // intake arm swinging, isn't)
  if (Math.abs(dd) > 0.01 && s === t.sgn && c === t.c) t.run++;
  else if (Math.abs(dd) > 0.01) { t.run = 1; t.sgn = s; if (t.w < 0.01) t.c = c; }
  else t.run = 0;
  t.speed += ((t.run >= 3 && c === t.c ? Math.abs(dd) : 0) - t.speed) * 0.3;
  const x = Math.min(1, Math.max(0, (t.speed - LO) / (HI - LO)));
  t.w += (x * x * (3 - 2 * x) - t.w) * 0.25;
  // (a drawn part its CAD replaced is spun all the same, hidden: nothing to blur)
  if ((t.w < 0.01 && t.speed < LO) || !drawn(t.o)) { if (live.has(t)) reset(t); return; }
  show(t);
}

function drawn(o) {
  for (let n = o; n; n = n.parent) if (!n.visible) return false;
  return true;
}

// the spin axis in the part's own frame: the turned Euler angle's axis, carried through the
// rotations applied after it
function spinAxis(o, c) {
  const e = o.rotation, ord = e.order, k = 'XYZ'[c];
  const v = new THREE.Vector3(c === 0 ? 1 : 0, c === 1 ? 1 : 0, c === 2 ? 1 : 0);
  const AX = { X: new THREE.Vector3(1, 0, 0), Y: new THREE.Vector3(0, 1, 0), Z: new THREE.Vector3(0, 0, 1) };
  for (const f of ord.slice(ord.indexOf(k) + 1)) v.applyAxisAngle(AX[f], -e[f.toLowerCase()]);
  return v.normalize();
}

// the sharp meshes under a part (only what's drawn now)
function sharpMeshes(o, ov) {
  const out = [];
  const walk = (n) => {
    if (!n.visible || n === ov || n.name === 'spinBlur') return;
    if (n.isMesh && !n.isInstancedMesh && n.geometry?.attributes?.position) out.push(n);
    for (const ch of n.children) walk(ch);
  };
  walk(o);
  return out;
}

// (building a blurred copy takes a few ms: no more than about half of the time, so a robot
// whose parts all spin up at once doesn't stall a frame; till then they're drawn sharp)
let nextBuild = 0;
export const spinBlurStats = { builds: 0, ms: 0, maxms: 0 };

function show(t) {
  // (what's drawn under it checked now and then: its CAD arriving replaces the drawn part)
  let meshes = null, sig = t.sig;
  if (!t.ov || --t.chk <= 0) {
    t.chk = 30;
    meshes = sharpMeshes(t.o, t.ov);
    sig = t.c + ':' + meshes.length + ':' + (meshes[0]?.geometry.id ?? '') + ':' + (meshes[meshes.length - 1]?.geometry.id ?? '');
  }
  if (!t.ov || (t.sig !== sig && !t.layers)) {
    const t0 = performance.now();
    if (t0 < nextBuild) { t.chk = 0; return; }
    if (t.ov) { reset(t); drop(t); }
    t.sig = sig;
    t.meshes = meshes;
    t.ov = build(t.o, spinAxis(t.o, t.c), meshes);
    const ms = performance.now() - t0;
    if (ms > spinBlurStats.maxms) spinBlurStats.maxms = ms;
    nextBuild = t0 + 2 * ms;
    spinBlurStats.builds++; spinBlurStats.ms += ms;
    t.none = !t.ov;
    if (!t.ov) { t.ov = new THREE.Object3D(); t.ov.visible = false; }
    t.o.add(t.ov);
    t.mats = null;
  }
  live.add(t);
  if (t.none) return;
  const w = t.w;
  // the blur fades in over the sharp part first (it encloses it), then the sharp part fades out
  t.ov.visible = true;
  t.ov.material.opacity = Math.min(1, 2 * w);
  const fade = Math.min(1, 2 * (1 - w));
  if (w > 0.97) hideSharp(t);
  else {
    unhideSharp(t);
    if (fade < 1) {
      if (!t.mats) t.mats = t.meshes.map((m) => ({ m, orig: m.material, order: m.renderOrder, fade: Array.isArray(m.material) ? m.material.map(fadeable) : fadeable(m.material) }));
      for (const e of t.mats) {
        if (e.m.material !== e.orig && e.m.material !== e.fade) continue; // (changed under us: leave it)
        e.m.material = e.fade;
        e.m.renderOrder = -2;
        t.faded = true;
        for (const [f, o] of Array.isArray(e.fade) ? e.fade.map((f, i) => [f, e.orig[i]]) : [[e.fade, e.orig]]) f.opacity = o.opacity * fade;
      }
    } else unfade(t);
  }
}

function drop(t) {
  if (!t.ov) return;
  t.ov.removeFromParent();
  t.ov.geometry?.dispose();
  t.ov.material?.dispose();
  t.ov = null;
  if (t.mats) for (const e of t.mats) for (const f of [e.fade].flat()) f.dispose();
  t.mats = null;
}

function fadeable(m) {
  const f = m.clone();
  f.transparent = true;
  f.onBeforeCompile = m.onBeforeCompile;
  f.customProgramCacheKey = m.customProgramCacheKey;
  return f;
}

function unfade(t) {
  if (!t.mats || !t.faded) return;
  t.faded = false;
  for (const e of t.mats) if (e.m.material === e.fade) { e.m.material = e.orig; e.m.renderOrder = e.order; }
}

function hideSharp(t) {
  unfade(t);
  // (one merged into a still batch, off every layer, is js/merge.js's: it's left until that
  // puts it back, the frame it sees it move)
  if (t.layers && !t.pending) return;
  if (!t.layers) t.layers = t.meshes.map(() => 0);
  t.pending = 0;
  t.meshes.forEach((m, i) => {
    const l = m.layers.mask;
    if (l && l !== 1 << HIDDEN) { t.layers[i] = l; m.layers.set(HIDDEN); } else if (!l) t.pending++;
  });
}

function unhideSharp(t) {
  if (!t.layers) return;
  t.meshes.forEach((m, i) => { if (t.layers[i] && m.layers.mask & (1 << HIDDEN)) m.layers.mask = t.layers[i]; });
  t.layers = null;
}

function reset(t) {
  unhideSharp(t);
  unfade(t);
  if (t.ov) t.ov.visible = false;
  t.w = 0;
  live.delete(t);
}

// The blurred copy: the part's triangles sampled in bins of axial position h, radius r and
// angle a about the axis u (through the part's origin); a stepped surface of revolution round
// what's there, each band as opaque as its ring is covered and the color of what's in it.
function build(o, u, meshes) {
  if (!meshes.length) return null;
  const G = GRID[level];
  const { nr, na } = G;
  o.updateWorldMatrix(true, true);
  const inv = new THREE.Matrix4().copy(o.matrixWorld).invert();
  const e1 = new THREE.Vector3(1, 0, 0);
  if (Math.abs(u.x) > 0.9) e1.set(0, 1, 0);
  e1.addScaledVector(u, -e1.dot(u)).normalize();
  const e2 = new THREE.Vector3().crossVectors(u, e1);
  // every triangle in the part's frame (strided past a budget)
  let total = 0;
  for (const m of meshes) total += (m.geometry.index ? m.geometry.index.count : m.geometry.attributes.position.count) / 3;
  const stride = Math.max(1, Math.ceil(total / 5000));
  const parts = [];
  let h0 = Infinity, h1 = -Infinity, rmax = 0;
  const v = new THREE.Vector3();
  for (const m of meshes) {
    const rel = new THREE.Matrix4().multiplyMatrices(inv, m.matrixWorld);
    const pos = m.geometry.attributes.position, idx = m.geometry.index;
    const n = idx ? idx.count : pos.count;
    const mats = Array.isArray(m.material) ? m.material : [m.material];
    const groups = Array.isArray(m.material) && m.geometry.groups.length ? m.geometry.groups : [{ start: 0, count: n, materialIndex: 0 }];
    for (const gr of groups) {
      const mat = mats[gr.materialIndex] || mats[0];
      if (!mat || mat.visible === false) continue;
      // (h, x, y) of each corner: the part's frame projected on the axis and across it
      const end = Math.min(n, gr.start + gr.count), first = gr.start - (gr.start % 3);
      const P = new Float32Array(Math.max(0, Math.ceil((end - first) / (3 * stride))) * 9);
      const ia = idx ? idx.array : null, pa = pos.isInterleavedBufferAttribute || pos.normalized ? null : pos.array;
      let o = 0;
      for (let i = first; i + 2 < end && o < P.length; i += 3 * stride) {
        for (let j = 0; j < 3; j++) {
          const vi = ia ? ia[i + j] : i + j;
          if (pa) v.set(pa[3 * vi], pa[3 * vi + 1], pa[3 * vi + 2]); else v.fromBufferAttribute(pos, vi);
          v.applyMatrix4(rel);
          const h = v.dot(u), x = v.dot(e1), y = v.dot(e2);
          P[o++] = h; P[o++] = x; P[o++] = y;
          if (h < h0) h0 = h;
          if (h > h1) h1 = h;
          const r = Math.sqrt(x * x + y * y);
          if (r > rmax) rmax = r;
        }
      }
      if (P.length) parts.push({ P, mat });
    }
  }
  if (!parts.length || rmax < 0.003) return null;
  const pad = 0.0015;
  h0 -= pad; h1 += pad;
  const nh = Math.max(G.h[0], Math.min(G.h[1], Math.round((h1 - h0) / 0.01)));
  const dh = Math.max(h1 - h0, 1e-4) / nh, dr = rmax / nr;
  // angle bins round each ring: fewer toward the axis (each about as long as the outer ones')
  const nA = Array.from({ length: nr }, (_, j) => Math.max(4, Math.round((na * (j + 0.5)) / nr)));
  const occ = new Uint8Array(nh * nr * na);
  const col = new Float32Array(nh * nr * 4);
  const mr = new Float32Array(nh * nr * 2); // (metalness, roughness sums)
  const c = new THREE.Color();
  let a = 1, mt = 0, ro = 0.6;
  const mark = (h, x, y) => {
    const hi = Math.min(nh - 1, Math.max(0, Math.floor((h - h0) / dh)));
    const ri = Math.min(nr - 1, Math.floor(Math.sqrt(x * x + y * y) / dr));
    const ai = Math.floor(((Math.atan2(y, x) / TAU + 1) % 1) * nA[ri]) % nA[ri];
    if (a > 0.5) occ[(hi * nr + ri) * na + ai] = 1;
    const C = 4 * (hi * nr + ri);
    col[C] += c.r * a; col[C + 1] += c.g * a; col[C + 2] += c.b * a; col[C + 3] += a;
    mr[2 * (hi * nr + ri)] += mt * a; mr[2 * (hi * nr + ri) + 1] += ro * a;
  };
  // each triangle split until its sides are under about a bin, its corners and middles marked
  // (the bin size raised for a big part, to keep to a budget)
  let area = 0;
  for (const { P } of parts) for (let i = 0; i < P.length; i += 9) area += triArea(P, i);
  const cell = Math.min(dh, dr, (TAU * rmax) / na);
  const L2 = Math.max(0.5 * cell * cell, (area * 4) / 8000);
  // (cut across its longest side, so a long sliver, as CAD has plenty of, becomes a row of short
  // ones, not a grid of thousands)
  const split = (ah, ax, ay, bh, bx, by, ch, cx, cy, depth) => {
    const ab = (ah - bh) ** 2 + (ax - bx) ** 2 + (ay - by) ** 2, bc = (bh - ch) ** 2 + (bx - cx) ** 2 + (by - cy) ** 2, ca = (ch - ah) ** 2 + (cx - ax) ** 2 + (cy - ay) ** 2;
    if (Math.max(ab, bc, ca) > L2 && depth < 16) {
      if (ab >= bc && ab >= ca) { const mh = (ah + bh) / 2, mx = (ax + bx) / 2, my = (ay + by) / 2; split(ah, ax, ay, mh, mx, my, ch, cx, cy, depth + 1); split(mh, mx, my, bh, bx, by, ch, cx, cy, depth + 1); }
      else if (bc >= ca) { const mh = (bh + ch) / 2, mx = (bx + cx) / 2, my = (by + cy) / 2; split(ah, ax, ay, bh, bx, by, mh, mx, my, depth + 1); split(ah, ax, ay, mh, mx, my, ch, cx, cy, depth + 1); }
      else { const mh = (ch + ah) / 2, mx = (cx + ax) / 2, my = (cy + ay) / 2; split(ah, ax, ay, bh, bx, by, mh, mx, my, depth + 1); split(mh, mx, my, bh, bx, by, ch, cx, cy, depth + 1); }
      return;
    }
    // (a piece of a split one: its middle; a small one whole: its corners too)
    if (!depth) { mark(ah, ax, ay); mark(bh, bx, by); mark(ch, cx, cy); }
    mark((ah + bh + ch) / 3, (ax + bx + cx) / 3, (ay + by + cy) / 3);
  };
  for (const { P, mat } of parts) {
    c.copy(mat.color || c.set(0x808080));
    a = mat.transparent ? mat.opacity : 1;
    mt = mat.metalness ?? 0; ro = mat.roughness ?? 0.6;
    for (let i = 0; i < P.length; i += 9) split(P[i], P[i + 1], P[i + 2], P[i + 3], P[i + 4], P[i + 5], P[i + 6], P[i + 7], P[i + 8], 0);
  }
  // per slab: its outer radius (bins), each ring's cover, the outer band's cover
  const Rout = new Int32Array(nh);
  const cov = new Float32Array(nh * nr);
  for (let i = 0; i < nh; i++) for (let j = 0; j < nr; j++) {
    let k = 0;
    for (let a = 0; a < na; a++) k += occ[(i * nr + j) * na + a];
    cov[i * nr + j] = k / nA[j];
    if (k) Rout[i] = j + 1;
  }
  const ring = (i, j) => {
    const C = 4 * (i * nr + j), n = col[C + 3];
    return n ? [col[C] / n, col[C + 1] / n, col[C + 2] / n] : null;
  };
  // what's seen down the axis at each ring, from either end of a slab: everything beyond it
  // that way (an angle bin covered by any of them is covered: spokes in the middle of a wheel
  // show on the face of its rim)
  const seen = [new Uint8Array(nh * nr * na), new Uint8Array(nh * nr * na)]; // [toward -h (k <= i), toward +h (k >= i)]
  for (let i = 0; i < nh; i++) for (let q = 0; q < nr * na; q++) {
    const a = i * nr * na + q, b = (nh - 1 - i) * nr * na + q;
    seen[0][a] = occ[a] | (i ? seen[0][a - nr * na] : 0);
    seen[1][b] = occ[b] | (i ? seen[1][b + nr * na] : 0);
  }
  // a face's look at ring j of slab i (side -1: facing -h, it shows k >= i; +1: k <= i)
  const faceRGBA = (i, j, side) => {
    const S = seen[side < 0 ? 1 : 0];
    let k = 0;
    for (let a = 0; a < na; a++) k += S[(i * nr + j) * na + a];
    let rgb = null;
    for (let m = i; m >= 0 && m < nh && !rgb; m -= side) rgb = ring(m, j);
    return [...(rgb || [0.5, 0.5, 0.5]), Math.min(1, (k / nA[j]) * 1.15)];
  };
  // a slab's band round its outside: the outermost rings' cover (and, if that's thin, a solid
  // core further in: a tire under its tread)
  const bands = (i) => {
    const R = Rout[i], out = [];
    let k = 0;
    const n1 = nA[R - 1], n2 = R > 1 ? nA[R - 2] : 1;
    for (let q = 0; q < n1; q++) k += occ[(i * nr + R - 1) * na + q] | (R > 1 ? occ[(i * nr + R - 2) * na + Math.floor((q * n2) / n1)] : 0);
    const al = Math.min(1, (k / n1) * 1.15);
    // (its color: the outermost ring's where that covers, the next one's in between)
    const c1 = ring(i, R - 1) || [0.5, 0.5, 0.5], c2 = (R > 1 && ring(i, R - 2)) || c1;
    const f1 = Math.min(1, cov[i * nr + R - 1] / Math.max(k / n1, 1e-6));
    out.push([R, [0, 1, 2].map((q) => c1[q] * f1 + c2[q] * (1 - f1)).concat(al)]);
    if (al < 0.9) for (let j = R - 3; j >= 0; j--) if (cov[i * nr + j] >= 0.9) { out.push([j + 1, [...(ring(i, j) || [0.5, 0.5, 0.5]), 1]]); break; }
    return out;
  };
  // the profile, as segments [r0, h0, r1, h1, rgba, nh] (nh: a face's side, -1 or +1; 0 a band);
  // r grows by a hair so it wraps the sharp part
  const segs = [];
  const RR = (j) => j * dr * 1.012 + 0.0008;
  const H = (i) => h0 + i * dh;
  // (neighbors that look alike, to a 20th in cover and a 40th in color, drawn as one: a plain
  // roller is a few bands, not one per cm)
  const key = (f) => Math.round(f[0] * 40) + ',' + Math.round(f[1] * 40) + ',' + Math.round(f[2] * 40) + ',' + Math.round(f[3] * 20);
  const face = (i, jA, jB, h, side) => { // slab i's face from ring edge jA to jB (either way) at height h
    const s = jB > jA ? 1 : -1;
    let last = null;
    for (let j = jA; j !== jB; j += s) {
      const f = faceRGBA(i, s > 0 ? j : j - 1, side), k = key(f);
      if (f[3] <= 0.02) { last = null; continue; }
      if (last && last[6] === k) { last[2] = RR(j + s); continue; }
      segs.push(last = [RR(j), h, RR(j + s), h, f, side, k]);
    }
  };
  const open = new Map(); // band ring -> its band so far (to carry on into the next slab)
  for (let i = 0; i < nh; i++) {
    if (!Rout[i]) { open.clear(); continue; }
    const prev = i > 0 ? Rout[i - 1] : 0, next = i < nh - 1 ? Rout[i + 1] : 0;
    if (Rout[i] > prev) face(i, prev, Rout[i], H(i), -1); // its face toward -h, where it stands out
    const now = new Map();
    for (const [R, b] of bands(i)) {
      if (b[3] <= 0.02) continue;
      const k = key(b), o = open.get(R);
      if (o && o[6] === k) { o[3] = H(i + 1); now.set(R, o); continue; }
      const sg = [RR(R), H(i), RR(R), H(i + 1), b, 0, k];
      segs.push(sg);
      now.set(R, sg);
    }
    open.clear();
    for (const [R, sg] of now) open.set(R, sg);
    if (Rout[i] > next) face(i, Rout[i], next, H(i + 1), 1); // toward +h
  }
  if (!segs.length) return null;
  const seg = Math.max(12, Math.min(G.seg, Math.round(rmax * 400)));
  const nv = segs.length * seg * 6;
  const pos = new Float32Array(nv * 3), nor = new Float32Array(nv * 3), rgba = new Float32Array(nv * 4);
  const cs = [], sn = [];
  for (let k = 0; k <= seg; k++) { cs.push(Math.cos((k / seg) * TAU)); sn.push(Math.sin((k / seg) * TAU)); }
  let o3 = 0, o4 = 0;
  const put = (r, h, k, nr_, nh_, f) => {
    const x = cs[k], y = sn[k];
    pos[o3] = h * u.x + r * (x * e1.x + y * e2.x); pos[o3 + 1] = h * u.y + r * (x * e1.y + y * e2.y); pos[o3 + 2] = h * u.z + r * (x * e1.z + y * e2.z);
    nor[o3] = nh_ * u.x + nr_ * (x * e1.x + y * e2.x); nor[o3 + 1] = nh_ * u.y + nr_ * (x * e1.y + y * e2.y); nor[o3 + 2] = nh_ * u.z + nr_ * (x * e1.z + y * e2.z);
    rgba[o4] = f[0]; rgba[o4 + 1] = f[1]; rgba[o4 + 2] = f[2]; rgba[o4 + 3] = f[3];
    o3 += 3; o4 += 4;
  };
  for (const [r0, ha, r1, hb, f, side] of segs) {
    // outward normal in the (r, h) plane: a band's out from the axis, a face's away from its slab
    const nr_ = side ? 0 : 1, nh_ = side;
    for (let k = 0; k < seg; k++) {
      // (two triangles, wound so their front faces outward: e1 x e2 = u)
      const A = [r0, ha, k], B = [r1, hb, k], C = [r1, hb, k + 1], D = [r0, ha, k + 1];
      const flip = side ? (side > 0) === (r1 > r0) : false;
      const tri = flip ? [A, B, C, A, C, D] : [A, C, B, A, D, C];
      for (const [r, h, kk] of tri) put(r, h, kk, nr_, nh_, f);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(rgba, 4));
  geo.computeBoundingSphere();
  // (its sheen: what's on the outside, round the rims)
  let met = 0, rough = 0, nm = 0;
  for (let i = 0; i < nh; i++) if (Rout[i]) { const k = i * nr + Rout[i] - 1; met += mr[2 * k]; rough += mr[2 * k + 1]; nm += col[4 * k + 3]; }
  const mat = new THREE.MeshStandardMaterial({ vertexColors: true, transparent: true, metalness: nm ? met / nm : 0.2, roughness: Math.max(0.4, nm ? rough / nm : 0.6), opacity: 0 });
  const ov = new THREE.Mesh(geo, mat);
  ov.castShadow = true;
  // the first of what's see-through (after only the sharp part fading out inside it), and in the
  // depth buffer: a polycarbonate panel in front of it is drawn over it, one behind it isn't
  ov.renderOrder = -1;
  ov.name = 'spinBlur';
  ov.userData.axis = u.clone();
  return ov;
}

function triArea(P, i) {
  const ax = P[i + 3] - P[i], ay = P[i + 4] - P[i + 1], az = P[i + 5] - P[i + 2];
  const bx = P[i + 6] - P[i], by = P[i + 7] - P[i + 1], bz = P[i + 8] - P[i + 2];
  return 0.5 * Math.hypot(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx);
}
