// The page's side of the physics thread (js/simWorker.js): starts it, sends it the match and the
// driver's commands, and draws the page's copy of the game from what it posts back.
//
// The page's game is built from the same settings as the worker's (its robots' models, its HUD,
// the camera's target) and never stepped: each frame the latest state is copied in (the match's,
// the robots' and the human players' plain fields, each robot's model drawn from its visual state,
// the FUEL's instance matrices) and the match's events replayed. Matches that need the page's
// thread (Training mode, a neural-net driver, recording your driving) run there as before.
import { BLUE, RED } from './constants.js';

export class SimThread {
  constructor() {
    this.worker = null;
    this.ready = null;    // a promise: the worker's world built
    this.snap = null;     // the latest state, not yet drawn
    this.cost = null;     // its own time: { msPerStep, stepsPerS }
    this.failed = '';
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
        else if (m.type === 'snap') { this.snap = m; this.cost = m.cost; }
      };
      w.postMessage({ type: 'init', storage: storage() });
      setTimeout(() => fail('the physics thread did not answer (it needs serve.py: start.bat / start.command)'), 20000);
    });
    return this.ready;
  }

  // a match: settings as createGame takes them (plain data), extra: fields set on the worker's game
  begin(settings, extra = {}) {
    this.snap = null;
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
