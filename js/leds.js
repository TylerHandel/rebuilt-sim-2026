// The robots' LED strips, drawn from their state (visual only: nothing here touches the physics).
//
// Each robot with LEDs gets one mesh: a small emissive box per LED (and a dark backing under each
// run), all merged into one geometry whose UVs point each LED at its own texel of a tiny
// DataTexture (the material's emissiveMap). A frame's colors are one texture upload of a few
// hundred bytes, made only when they change. It's on the glow layer, so bloom (High and up) makes
// it glow.
//
// Where the strips are and what they show come from each team's public code and photos of the
// 2026 robot (the URLs are with each team below; the photos were only looked at, to place the
// strips). A team whose code and photos show no LED strips has none here: 2910, 341, 581, 604, 6328
// (its code drives 40, but the user saw none on the robot),
// 4414, 1706 (and the robots not asked about: 971, 4946, 3928, 4930).
//
// Robot frame: x forward (the intake end), y up, z across; meters.
import * as THREE from 'three';
import { BLUE, TIMING } from './constants.js';

const PITCH = 1 / 60; // 60 LEDs per meter (the usual WS2812B strip)

// ------------------------------------------------------------------ where they are
// A run: `path` (points the LEDs are spread evenly along) or `arc` ({ c: [x, z], r, y, a0, a1 },
// angles in degrees from +x toward -z, as three.js turns about y); `n` LEDs from index `i0`.
// `dot`: each LED's size along the strip as a fraction of its spacing (1 reads as one diffused
// bar). Runs may share indices (a strip wired in parallel shows the same colors).
export const LED_LAYOUT = {
  // 1114 Simbotics: 60 LEDs on the roboRIO's PWM 9 (RobotConstants.LED_LENGTH, DriverIOSubsystem,
  // github.com/Simbotics/2026-Simbot-Tim). Photos (thebluealliance.com/team/1114/2026: imgur
  // DPhEQBo, UrV2T0c, E8vOusF) show them as a string of separate dots framing the shooter tower's
  // back face: across its top edge and part way down both sides (yellow while driving, red while
  // disabled, as their code has it).
  1114: { n: 60, runs: [{ path: [[-0.296, 0.40, -0.285], [-0.296, 0.69, -0.285], [-0.296, 0.69, 0.285], [-0.296, 0.40, 0.285]], n: 60, i0: 0, dot: 0.45 }] },
  // 1678 Citrus Circuits: a CANdle's own 8 LEDs, then a 12-LED back strip and 5-LED side strips
  // (LEDsConstants: NUM_BACK_STRIP, NUM_SIDE_STRIPS; github.com/frc1678/C2026-Public). Mounted
  // (the user) vertically along the climber's elevator: up the outside of each side's fixed outer
  // tube (js/robotModels.js build1678), the back strip's 12 then a side strip's 5 above them, both
  // sides alike. (The CANdle itself sits with the electronics: not drawn.)
  1678: {
    n: 17,
    runs: [
      { path: [[-0.02, 0.15, -0.330], [-0.02, 0.15 + 12 / 60, -0.330]], n: 12, i0: 0 },
      { path: [[-0.02, 0.15 + 12 / 60, -0.330], [-0.02, 0.15 + 17 / 60, -0.330]], n: 5, i0: 12 },
      { path: [[-0.02, 0.15, 0.330], [-0.02, 0.15 + 12 / 60, 0.330]], n: 12, i0: 0 },
      { path: [[-0.02, 0.15 + 12 / 60, 0.330], [-0.02, 0.15 + 17 / 60, 0.330]], n: 5, i0: 12 },
    ],
  },
  // 1690 Orbit: no public 2026 code. Photos (thebluealliance.com/team/1690/2026: imgur jJiksKT,
  // nGnBPZM) show a diffused strip bent round the front of the turret's bearing ring, just under
  // the turret: cyan, orange while shooting.
  1690: { n: 40, runs: [{ arc: { c: [-0.134, -0.1872], r: 0.165, y: 0.39, a0: -105, a1: 105 }, n: 40, i0: 0, dot: 1 }] },
  // 2056 OP Robotics: a photo (thebluealliance.com/team/2056/2026: imgur B7R1cNB) shows a blue
  // glow round the top of the turret, behind its Limelight; drawn as a ring there (a ring: it
  // reads the same however the turret turns). Their colors aren't public: the default scheme.
  2056: { n: 36, runs: [{ arc: { c: [-0.1334, 0.1841], r: 0.125, y: 0.55, a0: 0, a1: 350 }, n: 36, i0: 0, dot: 1 }] },
  // 7769 Chunk: no public 2026 code. A photo (thebluealliance.com/team/7769/2026: imgur pAzO1fI)
  // shows a short bright blue block low on a corner by the electronics (a CANdle-sized light, not
  // a strip): drawn as 8 LEDs there. Their colors aren't public: the default scheme.
  7769: { n: 8, runs: [{ path: [[-0.31, 0.18, 0.35], [-0.31, 0.26, 0.35]], n: 8, i0: 0, pitch: 0.01 }] },
  // 8793 Pumpkin Bots: a CANdle (its own 8 LEDs, seen lit blue in a photo at the front right:
  // thebluealliance.com/team/8793/2026, imgur vhB9Gr5), then a 60-LED strip (LEDSubsystem.java:
  // LED_END = 7 + LED_COUNT = 67; github.com/Pumpkin-Bots/2026_Robot), mounted (the user) round
  // the turret's cable-chain track: on the outside of its outer wall (js/echain.js TRACK8793:
  // 8.0in from the turret's axis, the wall from 0.318 to 0.343 m up), from the camera mount at the
  // back right round the front to the one at the back left (the user).
  8793: {
    n: 68,
    runs: [
      { path: [[0.27, 0.215, -0.24], [0.27, 0.215, -0.17]], n: 8, i0: 0, pitch: 0.008 },
      { arc: { c: [-0.127, 0], r: 8.0 * 0.0254 + 0.008, y: 0.331, a0: -150, a1: 150 }, n: 60, i0: 8, dot: 0.8 },
    ],
  },
};

// ------------------------------------------------------------------ the mesh
const W = 0.011, T = 0.004; // an LED's face (across the strip) and how far it stands out
// each run's LEDs face outward: away from the robot's center line (a run on the back faces back)
function frames(run) {
  const out = [];
  if (run.arc) {
    const { c, r, y, a0, a1 } = run.arc;
    const da = (a1 - a0) / run.n;
    for (let i = 0; i < run.n; i++) {
      const a = (a0 + (i + 0.5) * da) * Math.PI / 180;
      const p = new THREE.Vector3(c[0] + r * Math.cos(a), y, c[1] - r * Math.sin(a));
      const along = new THREE.Vector3(-Math.sin(a), 0, -Math.cos(a));
      const out_ = new THREE.Vector3(Math.cos(a), 0, -Math.sin(a));
      out.push({ p, along, out: out_, len: Math.abs(da * Math.PI / 180) * r });
    }
    return out;
  }
  const pts = run.path.map((q) => new THREE.Vector3(...q));
  const segs = [];
  let total = 0;
  for (let k = 1; k < pts.length; k++) { const l = pts[k].distanceTo(pts[k - 1]); segs.push(l); total += l; }
  const step = run.pitch ? run.pitch : total / run.n;
  const start = run.pitch ? (total - step * run.n) / 2 : 0;
  for (let i = 0; i < run.n; i++) {
    let s = start + (i + 0.5) * step, k = 0;
    while (k < segs.length - 1 && s > segs[k]) { s -= segs[k]; k++; }
    const a = pts[k], b = pts[k + 1];
    const along = b.clone().sub(a).normalize();
    const p = a.clone().addScaledVector(along, s);
    // outward: across the strip, away from the robot's middle (horizontal for a vertical run)
    const radial = new THREE.Vector3(p.x, 0, p.z);
    const outv = radial.addScaledVector(along, -radial.dot(along));
    if (Math.abs(p.x) > Math.abs(p.z) * 0.9 || outv.lengthSq() < 1e-6) outv.set(Math.sign(p.x) || 1, 0, 0).addScaledVector(along, -along.x);
    if (outv.lengthSq() < 1e-6) outv.set(0, 1, 0);
    out.push({ p, along, out: outv.normalize(), len: step });
  }
  return out;
}

// one box (8 corners) at frame f, dims (len along, W across, T out), all its UVs at texel u
function pushBox(pos, nrm, uv, idx, f, len, w, t, depth, u) {
  const side = new THREE.Vector3().crossVectors(f.along, f.out).normalize();
  const c = f.p.clone().addScaledVector(f.out, depth + t / 2);
  const ax = [f.along.clone().multiplyScalar(len / 2), side.multiplyScalar(w / 2), f.out.clone().multiplyScalar(t / 2)];
  const dirs = [f.along, side.clone().normalize(), f.out];
  // 6 faces, 4 verts each (flat normals)
  for (let k = 0; k < 3; k++) for (const s of [-1, 1]) {
    const n = dirs[k].clone().multiplyScalar(s);
    const a = ax[(k + 1) % 3], b = ax[(k + 2) % 3];
    const base = pos.length / 3;
    const ctr = c.clone().addScaledVector(ax[k], s);
    for (const [i, j] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
      const v = ctr.clone().addScaledVector(a, i).addScaledVector(b, j);
      pos.push(v.x, v.y, v.z); nrm.push(n.x, n.y, n.z); uv.push(u, 0.5);
    }
    // wind so the face looks along n
    const e1 = a.clone(), e2 = b.clone();
    const flip = e1.cross(e2).dot(n) < 0;
    if (flip) idx.push(base, base + 2, base + 1, base, base + 3, base + 2);
    else idx.push(base, base + 1, base + 2, base, base + 2, base + 3);
  }
}

// builds the strips onto a robot model (model.root); returns null for a team without LEDs
let built = 0;
export function addLeds(model, cfg) {
  const lay = LED_LAYOUT[cfg.key];
  if (!lay) return null;
  // three.js gives each new geometry, material, texture and mesh a UUID from Math.random, which
  // the headless matches seed: these draw theirs from a stream of their own, so the match's random
  // numbers (and the play) stay exactly as they were without the LEDs
  const random = Math.random;
  let k = (0x2545f491 + 0x9e3779b9 * ++built) >>> 0;
  Math.random = () => (k = (Math.imul(k, 1664525) + 1013904223) >>> 0) / 4294967296;
  try { return build(model, cfg, lay); } finally { Math.random = random; }
}

function build(model, cfg, lay) {
  const N = lay.n, TX = N + 1; // (the last texel: always dark, for the backing)
  const pos = [], nrm = [], uv = [], idx = [];
  const texel = (i) => (i + 0.5) / TX;
  for (const run of lay.runs) {
    const fr = frames(run);
    fr.forEach((f, i) => {
      const dot = run.dot ?? 0.6;
      pushBox(pos, nrm, uv, idx, f, f.len * 1.001, W + 0.003, 0.002, 0, texel(N)); // backing
      pushBox(pos, nrm, uv, idx, f, f.len * dot, W, T - 0.002, 0.002, texel(run.i0 + i));
    });
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute('normal', new THREE.Float32BufferAttribute(nrm, 3));
  geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
  geo.setIndex(idx);
  const data = new Uint8Array(4 * TX);
  for (let i = 0; i < TX; i++) data[4 * i + 3] = 255;
  const tex = new THREE.DataTexture(data, TX, 1, THREE.RGBAFormat);
  tex.magFilter = tex.minFilter = THREE.NearestFilter;
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.needsUpdate = true;
  const mat = new THREE.MeshStandardMaterial({ color: 0x0b0b0c, roughness: 0.45, metalness: 0, emissive: 0xffffff, emissiveMap: tex, emissiveIntensity: 2.2 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.name = 'leds';
  mesh.castShadow = false;
  mesh.receiveShadow = false;
  mesh.userData.noMerge = true; // (its texture changes every frame: js/merge.js)
  mesh.userData.glow = true; // (bloom: js/graphics.js)
  model.root.add(mesh);
  const leds = { key: cfg.key, n: N, data, tex, mesh, rgb: new Float32Array(3 * N) };
  model.leds = leds;
  return leds;
}

// ------------------------------------------------------------------ what they show
// The robot's state as the LEDs see it (fields the page has in either mode: js/simWorker.js plain)
function stateOf(r, match) {
  const enabled = !!r.enabled && !!match?.robotEnabled;
  const phase = match?.phase ?? 'pre';
  const a = r.alliance;
  return {
    enabled, phase, blue: a === BLUE,
    auto: phase === 'auto',
    climbing: r.climbState && r.climbState !== 'none',
    shooting: (r.feeding ?? 0) > 0,
    wantShoot: !!r.wantShoot || !!r.cmd?.shoot || !!r.cmd?.pass,
    passing: !!r.cmd?.pass,
    ready: !!r.ready,
    intaking: (r.intakeSpeed ?? 0) > 0,
    full: !!r.full,
    has: (r.stored?.length ?? 0) > 0,
    hubActive: match && typeof match.hubActive === 'function' ? match.hubActive(a) : true,
    hubNext: match && typeof match.hubNextChange === 'function' ? match.hubNextChange(a) : null,
    left: phase === 'teleop' ? Math.max(0, TIMING.teleop - (match.phaseTime ?? 0)) : null,
  };
}

const C = {
  off: [0, 0, 0], red: [255, 0, 0], green: [0, 255, 0], blue: [0, 0, 255], white: [255, 255, 255],
  yellow: [255, 255, 0], orange: [255, 165, 0], purple: [128, 0, 128], teal: [0, 128, 128],
  aqua: [0, 255, 255], lime: [0, 255, 0], gold: [255, 215, 0], darkBlue: [0, 0, 139], pink: [255, 105, 180],
};
const ally = (s) => (s.blue ? C.blue : C.red);
const mix = (c1, c2, k) => [c1[0] + (c2[0] - c1[0]) * k, c1[1] + (c2[1] - c1[1]) * k, c1[2] + (c2[2] - c1[2]) * k];
const scale = (c, k) => [c[0] * k, c[1] * k, c[2] * k];
const hsv = (h) => { // full saturation and value, h in [0, 1)
  const f = (n) => { const k = (n + h * 6) % 6; return 255 * (1 - Math.max(0, Math.min(k, 4 - k, 1))); };
  return [f(5), f(3), f(1)];
};
const blink = (t, period) => (t % period) < period / 2; // on for the first half of each period


// each team's own scheme where their code is public; a default for the rest
const SCHEMES = {

  // LEDSubsystem.java (8793): auto, purple breathing; disabled, orange breathing (1 Hz, down to
  // 5%); teleop, solid green while their HUB is active, red while not; in the 5 s before it
  // changes (and the match's last 10 s) a checkerboard of 10-LED blocks swapping with yellow,
  // faster as the change nears (1 to 8 Hz)
  8793(s, set, n, t) {
    const b = 0.05 + (0.5 + 0.5 * Math.sin(2 * Math.PI * t)) * 0.95;
    if (s.enabled && s.auto) { for (let i = 0; i < n; i++) set(i, scale([128, 0, 128], b)); return; }
    if (!s.enabled) { for (let i = 0; i < n; i++) set(i, scale([255, 20, 0], b)); return; }
    const prim = s.hubActive ? C.green : C.red;
    let hz = 0;
    if (s.left !== null && s.left <= 10) hz = 1 + (1 - s.left / 10) * 7;
    else if (s.hubNext !== null && s.hubNext <= 5) hz = 1 + (1 - s.hubNext / 5) * 7;
    if (!hz) { for (let i = 0; i < n; i++) set(i, prim); return; }
    const phase = Math.floor(t * 2 * hz) % 2;
    for (let i = 0; i < n; i++) set(i, ((Math.floor(i / 10) + phase) % 2 === 0) ? prim : [255, 200, 0]);
  },
  // LEDs.java / LEDsConstants.java (1678): disabled, aqua; trigger held, a 90 ms flash of
  // yellow/blue when the shot's verified, off/red when not; intaking, a 80 ms orange/yellow
  // flash; otherwise the side strips show vision (lime: pose stable) and the back is off
  1678(s, set, n, t) {
    if (!s.enabled) { for (let i = 0; i < n; i++) set(i, C.aqua); return; }
    if (s.wantShoot) {
      const on = blink(t, 0.18);
      for (let i = 0; i < n; i++) set(i, s.ready ? (on ? C.yellow : C.blue) : (on ? C.off : C.red));
      return;
    }
    if (s.intaking) { const on = blink(t, 0.16); for (let i = 0; i < n; i++) set(i, on ? C.orange : C.yellow); return; }
    for (let i = 0; i < n; i++) set(i, i >= 12 ? C.lime : C.off);
  },
  // DriverIOSubsystem.java / LEDRequest.java (1114): disabled, red; enabled, the highest request
  // (the enum's order): climbing, a scrolling rainbow; passing, half green half purple; waiting on
  // the shooter, blue; ready to shoot, green; seeing a tag (nearly always), yellow; intaking,
  // orange
  1114(s, set, n, t) {
    if (!s.enabled) { for (let i = 0; i < n; i++) set(i, C.red); return; }
    if (s.climbing) { for (let i = 0; i < n; i++) set(i, hsv(((i - t * 39.37) / 100 % 1 + 1) % 1)); return; }
    if (s.wantShoot && s.passing) { for (let i = 0; i < n; i++) set(i, i < n / 2 ? C.green : C.purple); return; }
    if (s.wantShoot && !s.ready) { for (let i = 0; i < n; i++) set(i, C.blue); return; }
    if (s.wantShoot && s.ready) { for (let i = 0; i < n; i++) set(i, C.green); return; }
    for (let i = 0; i < n; i++) set(i, C.yellow);
  },
  // 1690 (photos only): cyan, orange while shooting; dim cyan breathing while disabled
  1690(s, set, n, t) {
    const cyan = [0, 200, 255];
    if (!s.enabled) { const b = 0.25 + 0.2 * Math.sin(2 * Math.PI * t / 3); for (let i = 0; i < n; i++) set(i, scale(cyan, b)); return; }
    for (let i = 0; i < n; i++) set(i, s.shooting ? [255, 90, 0] : cyan);
  },
};

// the default (teams whose colors aren't public): disabled, the alliance color breathing slowly;
// climbing, a rainbow chase; shooting, a white strobe; aimed and ready, green; aiming, a yellow
// blink; full, purple; intaking, an orange chase; otherwise the alliance color (dim when empty)
function defaultScheme(s, set, n, t) {
  const a = ally(s);
  if (!s.enabled) { const b = 0.2 + 0.25 * (0.5 + 0.5 * Math.sin(2 * Math.PI * t / 2.5)); for (let i = 0; i < n; i++) set(i, scale(a, b)); return; }
  if (s.climbing) { for (let i = 0; i < n; i++) set(i, hsv(((i / n - t * 0.5) % 1 + 1) % 1)); return; }
  if (s.shooting) { const on = blink(t, 0.1); for (let i = 0; i < n; i++) set(i, on ? C.white : scale(C.white, 0.15)); return; }
  if (s.wantShoot) { const c = s.ready ? C.green : blink(t, 0.25) ? C.yellow : C.off; for (let i = 0; i < n; i++) set(i, c); return; }
  if (s.full) { for (let i = 0; i < n; i++) set(i, C.purple); return; }
  if (s.intaking) { const k = Math.floor(t * 20); for (let i = 0; i < n; i++) set(i, (i + k) % 4 < 2 ? C.orange : scale(C.orange, 0.15)); return; }
  for (let i = 0; i < n; i++) set(i, s.has ? a : scale(a, 0.35));
}

// once a frame per robot (t: seconds, any clock: the patterns' timing)
export function updateLeds(r, match, t) {
  const L = r.model?.leds;
  if (!L) return;
  const s = stateOf(r, match);
  const { data, n } = L;
  let changed = false;
  const set = (i, c) => {
    if (i < 0 || i >= n) return;
    const o = 4 * i;
    for (let k = 0; k < 3; k++) {
      const v = Math.max(0, Math.min(255, Math.round(c[k])));
      if (data[o + k] !== v) { data[o + k] = v; changed = true; }
    }
  };
  (SCHEMES[L.key] || defaultScheme)(s, set, n, t);
  if (changed) L.tex.needsUpdate = true;
}
