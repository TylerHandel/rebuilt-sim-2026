// The physics thread (Settings → Physics thread: Separate; js/simThread.js is the page's side):
// the whole match (Rapier, every robot's mechanisms, FUEL, AIs, rules, human players) stepped
// here, off the page's thread, at the game's 120 Hz against the clock, two steps then the per-frame
// updates, as the headless matches run it. After each pair it posts what the page draws and shows:
// every robot's visual state (Robot.visualState), the FUEL's instance matrices, the match's, the
// robots' and the human players' plain fields, and the match's events. The page keeps a copy of
// the game built from the same settings (its models, its HUD), never stepped, and draws it from
// these. The page sends the driver's commands every frame.
//
// Loaded as w/js/simWorker.js: serve.py serves w/... with the page's import map applied to the
// modules (a worker has no import map), so this needs serve.py (start.bat / start.command).
import { setStorage } from './workerEnv.js';
import * as THREE from 'three';
import { Physics } from './physics.js';
import { Field } from './field.js';
import { FuelManager } from './fuel.js';
import { createGame, stepGame, frameGame } from './game.js';
import { PHYSICS_DT, BLUE, RED } from './constants.js';

let world = null, game = null, on = false, events = [];
let acc = 0, last = 0, timer = null;
const cost = { ms: 0, steps: 0, t0: performance.now() };

// an object's plain data: numbers, strings, booleans, null, arrays and plain objects of them (not
// class instances: a vector or quaternion as its x, y, z, w), depth-limited
const SKIP = new Set(['cfg', 'model', 'visual', 'body', 'physics', 'scene', 'field', 'fuel', 'match', 'hopper', 'phys', 'swerve', 'drivetrain',
  'loc', 'captured', 'wheelPts', 'climberCfg', 'stored', 'robot', 'foes', 'mates', 'rules', 'info', 'nav', 'brain', 'onEvent', 'units', 'robots']);
function plain(v, depth = 0) {
  if (v === null || typeof v !== 'object') return typeof v === 'function' ? undefined : v;
  if (depth > 3) return undefined;
  if (v.isVector3) return { x: v.x, y: v.y, z: v.z, isV3: 1 };
  if (v.isQuaternion) return { x: v.x, y: v.y, z: v.z, w: v.w, isQ: 1 };
  if (Array.isArray(v)) {
    if (v.length > 64) return undefined;
    const out = [];
    for (const x of v) { const p = plain(x, depth + 1); if (p === undefined && x !== undefined) return undefined; out.push(p); }
    return out;
  }
  const proto = Object.getPrototypeOf(v);
  if (proto !== Object.prototype && proto !== null && depth > 0) return undefined;
  const out = {};
  for (const k of Object.keys(v)) {
    if (depth === 0 && SKIP.has(k)) continue;
    const p = plain(v[k], depth + 1);
    if (p !== undefined) out[k] = p;
  }
  return out;
}

function snapshot() {
  const m = game.match, f = world.fuel;
  f.sync();
  const snap = {
    type: 'snap', t: m.t, over: m.over, events,
    match: plain(m),
    robots: game.units.map((u) => ({ r: { ...plain(u.robot), storedN: u.robot.stored.length }, vis: u.robot.visualState(true), ai: u.ai ? { label: u.ai.label } : null })),
    hps: { [BLUE]: plain(game.hps[BLUE]), [RED]: plain(game.hps[RED]) },
    fuel: { inst: f.mesh.instanceMatrix.array.slice(), touch: f.touch.map((a) => a.array.slice()), dmg: f.dmgAttr.array.slice(), chute: { [BLUE]: f.chute[BLUE].length, [RED]: f.chute[RED].length } },
    cost: { msPerStep: cost.steps ? cost.ms / cost.steps : 0, stepsPerS: (1000 * cost.steps) / Math.max(1, performance.now() - cost.t0) },
  };
  events = [];
  if (performance.now() - cost.t0 > 1000) { cost.ms = 0; cost.steps = 0; cost.t0 = performance.now(); }
  postMessage(snap, [snap.fuel.inst.buffer, snap.fuel.dmg.buffer, ...snap.fuel.touch.map((a) => a.buffer)]);
}

// two steps at a time against the clock (at most 8 a turn: a slower machine plays slower)
function run() {
  timer = null;
  if (!on || !game) return;
  try { turn(); } catch (err) { on = false; postMessage({ type: 'error', message: String(err && err.message || err), stack: String(err && err.stack || '') }); return; }
  timer = setTimeout(run, 2);
}
function turn() {
  const now = performance.now();
  acc += Math.min(0.05, (now - last) / 1000);
  last = now;
  let n = 0;
  while (acc >= 2 * PHYSICS_DT && n < 8 && !game.match.over) {
    const t0 = performance.now();
    stepGame(game, world, PHYSICS_DT);
    stepGame(game, world, PHYSICS_DT);
    frameGame(game, 2 * PHYSICS_DT);
    cost.ms += performance.now() - t0; cost.steps += 2;
    acc -= 2 * PHYSICS_DT;
    n += 2;
  }
  if (n) snapshot();
  if (n >= 8) acc = 0;
}

self.onmessage = async (e) => {
  try { await handle(e.data); } catch (err) { on = false; postMessage({ type: 'error', message: String(err && err.message || err), stack: String(err && err.stack || '') }); }
};
async function handle(msg) {
  if (msg.type === 'init') {
    setStorage(msg.storage);
    const physics = await Physics.create();
    const scene = new THREE.Scene();
    const field = new Field(scene, physics);
    field.build();
    const fuel = new FuelManager(physics, scene, field);
    fuel.stage(8);
    world = { physics, scene, field, fuel };
    postMessage({ type: 'ready' });
  } else if (msg.type === 'start') {
    if (msg.storage) setStorage(msg.storage);
    game = createGame(world, msg.settings, { onEvent: (type, d) => events.push([type, plain(d)]), prev: game });
    Object.assign(game, msg.extra || {});
    acc = 0; last = performance.now();
    snapshot();
    if (on && !timer) timer = setTimeout(run, 0);
  } else if (msg.type === 'run') {
    on = !!msg.on;
    last = performance.now();
    if (on && !timer) timer = setTimeout(run, 0);
  } else if (msg.type === 'input') {
    if (!game) return;
    game.driver = msg.driver;
    game.hpControls = msg.hpControls || {};
    const r = game.robot;
    if (r && msg.climbTarget !== undefined) r.climbTarget = msg.climbTarget;
    if (r && msg.climb) { const said = r.requestClimb(); if (said) events.push(['climbMsg', { text: said }]); }
  }
}
