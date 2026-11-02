// The page's side of the physics thread (js/simWorker.js): starts it, sends it the match and the
// driver's commands, and draws the page's copy of the game from what it posts back.
//
// The page's game is built from the same settings as the worker's (its robots' models, its HUD,
// the camera's target) and never stepped: each frame the latest state is copied in (the match's,
// the robots' and the human players' plain fields, each robot's model drawn from its visual state,
// the FUEL's instance matrices) and the match's events replayed. Matches that need the page's
// thread (Training mode, a neural-net driver, recording your driving) run there as before.
import * as THREE from 'three';
import { BLUE, RED } from './constants.js';

export class SimThread {
  constructor() {
    this.worker = null;
    this.ready = null;    // a promise: the worker's world built
    this.snap = null;     // the latest state, not yet drawn
    this.cost = null;     // its own time: { msPerStep, stepsPerS }
    this.failed = '';
    this.prev = null; this.cur = null; this.gap = 1000 / 60;
  }

  // the worker, its field and FUEL built (once)
  start() {
    if (this.ready) return this.ready;
    this.ready = new Promise((resolve, reject) => {
      let w;
      try { w = new Worker('w/js/simWorker.js', { type: 'module' }); } catch (e) { reject(e); return; }
      this.worker = w;
      const fail = (why) => { this.failed = why; reject(new Error(why)); };
      w.onerror = (e) => fail(e.message || 'the physics thread could not start (it needs serve.py: start.bat / start.command)');
      w.onmessage = (e) => {
        const m = e.data;
        if (m.type === 'ready') resolve(true);
        else if (m.type === 'snap') {
          // (two before a frame takes them: the later one, with both's events)
          if (this.snap) m.events = this.snap.events.concat(m.events);
          m.at = performance.now();
          this.snap = m; this.cost = m.cost;
        }
        else if (m.type === 'error') { this.error = m; console.error('physics thread:', m.message, m.stack); if (this.onError) this.onError(m); }
      };
      w.postMessage({ type: 'init', storage: storage() });
      setTimeout(() => fail('the physics thread did not answer (it needs serve.py: start.bat / start.command)'), 20000);
    });
    return this.ready;
  }

  // a match: settings as createGame takes them (plain data), extra: fields set on the worker's game
  begin(settings, extra = {}) {
    this.snap = null; this.prev = null; this.cur = null;
    this.worker.postMessage({ type: 'start', settings, extra, storage: storage() });
  }
  run(on) { if (this.worker && on !== this.on) { this.on = on; this.worker.postMessage({ type: 'run', on }); } }
  input(msg) { if (this.worker) this.worker.postMessage({ type: 'input', ...msg }); }

  // copy the latest state into the page's game; onEvent(type, data) replays the match's events.
  // false if nothing new came
  apply(game, fuel, onEvent) {
    const s = this.snap;
    if (!s) return false;
    this.snap = null;
    // (kept to draw between: the one before and this one)
    this.prev = this.cur;
    this.cur = { at: s.at, pose: s.robots.map((d) => (d ? [d.vis.pos, d.vis.quat] : null)), inst: s.fuel.inst };
    if (this.prev) this.gap += (Math.min(100, this.cur.at - this.prev.at) - this.gap) * 0.2;
    assign(game.match, s.match);
    game.units.forEach((u, i) => {
      const r = u.robot, d = s.robots[i];
      if (!d) return;
      assign(r, d.r);
      if (r.stored.length !== d.r.storedN) r.stored = new Array(d.r.storedN).fill(null);
      r._vis = d.vis;
      if (u.ai && d.ai) u.ai.label = d.ai.label;
    });
    for (const a of [BLUE, RED]) { assign(game.hps[a], s.hps[a]); game.hps[a].syncVisual(); }
    // the FUEL: drawn where the worker has them (its FuelManager.sync's matrices and dents)
    fuel.mesh.instanceMatrix.array.set(s.fuel.inst);
    fuel.mesh.instanceMatrix.needsUpdate = true;
    fuel.touch.forEach((a, j) => { a.array.set(s.fuel.touch[j]); a.needsUpdate = true; });
    for (const a of [BLUE, RED]) if (fuel.chute[a].length !== s.fuel.chute[a]) fuel.chute[a] = new Array(s.fuel.chute[a]).fill(null);
    for (const [type, d] of s.events) onEvent(type, d);
    return true;
  }

  // Each frame, after apply: the robots and the FUEL drawn between the last two states, by how
  // long ago the last one came (a state's interval behind, so the motion is smooth whenever the
  // frames and the states fall)
  between(game, fuel) {
    const P = this.prev, C = this.cur;
    if (!P || !C) return;
    const a = Math.max(0, Math.min(1, (performance.now() - C.at) / Math.max(1, this.gap)));
    game.units.forEach((u, i) => {
      const r = u.robot, p = P.pose[i], c = C.pose[i];
      if (!p || !c || !r._vis) return;
      const pos = [0, 1, 2].map((k) => p[0][k] + (c[0][k] - p[0][k]) * a);
      if (Math.hypot(c[0][0] - p[0][0], c[0][2] - p[0][2]) > 1) return; // (a jump: a new match, a reset)
      const quat = [0, 0, 0, 1];
      THREE.Quaternion.slerpFlat(quat, 0, p[1], 0, c[1], 0, a);
      r._vis = { ...r._vis, pos, quat };
    });
    const arr = fuel.mesh.instanceMatrix.array, A = P.inst, B = C.inst;
    for (let o = 0; o < B.length; o += 16) {
      // (one shown in both, not moved far: a FUEL put in a chute or let out of a HUB jumps)
      if ((A[o] === 0 && A[o + 1] === 0 && A[o + 2] === 0) || (B[o] === 0 && B[o + 1] === 0 && B[o + 2] === 0)) continue;
      const dx = B[o + 12] - A[o + 12], dy = B[o + 13] - A[o + 13], dz = B[o + 14] - A[o + 14];
      if (dx * dx + dy * dy + dz * dz > 0.25) continue;
      arr[o + 12] = A[o + 12] + dx * a; arr[o + 13] = A[o + 13] + dy * a; arr[o + 14] = A[o + 14] + dz * a;
    }
    fuel.mesh.instanceMatrix.needsUpdate = true;
  }
}

// what the worker's localStorage starts with: the page's (custom autos and the like)
function storage() {
  const out = {};
  try { for (let i = 0; i < localStorage.length; i++) { const k = localStorage.key(i); out[k] = localStorage.getItem(k); } } catch { /* none */ }
  return out;
}

// copy plain data into an object: vectors and quaternions in place, the rest assigned
function assign(t, src) {
  if (!t || !src) return;
  for (const k of Object.keys(src)) {
    const v = src[k], cur = t[k];
    if (v && v.isV3 && cur && cur.isVector3) cur.set(v.x, v.y, v.z);
    else if (v && v.isQ && cur && cur.isQuaternion) cur.set(v.x, v.y, v.z, v.w);
    else if (v && typeof v === 'object' && !Array.isArray(v) && cur && typeof cur === 'object' && !Array.isArray(cur) && Object.getPrototypeOf(cur) === Object.prototype) assign(cur, v);
    else if (typeof cur === 'function') continue;
    else t[k] = v;
  }
}
