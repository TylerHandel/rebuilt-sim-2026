// Search for each robot's best AUTO (see BEST_AUTOS in js/auto.js): random plans, then local
// tweaks of the best, each played headless against the other robots' current best AUTOs on
// both alliances. Prints the winners; copy them into BEST_AUTOS.
//   node --import ./tools/node-env.mjs tools/auto-search.mjs [--robots 2910,4414] [--n 40] [--refine 24]
import { Worker } from 'node:worker_threads';
import { cpus } from 'node:os';
import { BEST_AUTOS } from '../js/auto.js';
import { ROBOTS } from '../js/robotConfigs.js';
import { mulberry32 } from './headless.mjs';

const args = process.argv.slice(2);
const opt = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const ROBOT_KEYS = opt('robots', '2910,4414,8793').split(',');
const N = +opt('n', 40), REFINE = +opt('refine', 24), SEEDS = [1, 2];
const rng = mulberry32(+opt('seed', 11));
const pick = (a) => a[Math.floor(rng() * a.length)];
const U = (lo, hi) => lo + (hi - lo) * rng();
const round = (v, k = 100) => Math.round(v * k) / k;

// ------------------------------------------------------------------ worker pool
const workers = [];
const queue = [];
const pending = new Map();
let nextId = 0;
await Promise.all(Array.from({ length: Math.max(1, cpus().length) }, () => new Promise((res) => {
  const w = new Worker(new URL('./auto-worker.mjs', import.meta.url), { execArgv: ['--import', new URL('./node-env.mjs', import.meta.url).pathname] });
  w.busy = false;
  w.on('message', (m) => {
    if (m.ready) { workers.push(w); res(); pump(); return; }
    const p = pending.get(m.id); pending.delete(m.id); w.busy = false;
    if (m.error) p.reject(new Error(m.error)); else p.resolve(m.result);
    pump();
  });
})));
function pump() {
  for (const w of workers) {
    if (w.busy || !queue.length) continue;
    const job = queue.shift(); w.busy = true; pending.set(job.id, job); w.postMessage(job.msg);
  }
}
const run = (a, b, seed) => new Promise((resolve, reject) => { const id = nextId++; queue.push({ id, resolve, reject, msg: { id, a, b, seed } }); pump(); });

// ------------------------------------------------------------------ plans
function randomTrip(key, side) {
  const cap = ROBOTS[key].storage.capacity;
  if (rng() < (cap < 20 ? 0.35 : 0.15)) return { depot: true, speed: round(U(0.25, 0.6)) };
  // sweep a stretch of the FUEL line starting on this side and heading across
  const len = Math.min(5.0, cap < 20 ? U(0.6, 2.2) : U(1.2, 5.0));
  const a = side === 'right' ? U(1.3, 2.4) : U(5.7, 6.7);
  const b = side === 'right' ? Math.min(6.7, a + len) : Math.max(1.3, a - len);
  const home = pick(['trench', 'bump']);
  const homeY = b > 4.03 ? 6.0 : 2.0;
  return { out: pick(['trench', 'bump']), fx: round(U(7.35, 8.7)), a: round(a), b: round(b), speed: round(U(cap < 20 ? 0.2 : 0.35, 0.9)), home, shootAt: [round(U(2.2, 3.4)), round(homeY + U(-0.6, 0.6))] };
}
function randomPlan(key) {
  const start = pick(['rightTrench', 'rightBump', 'hub', 'leftBump', 'leftTrench']);
  const n = pick(ROBOTS[key].storage.capacity < 20 ? [2, 3, 4] : [1, 2, 3]);
  let side = start.startsWith('left') ? 'left' : start.startsWith('right') ? 'right' : pick(['left', 'right']);
  const trips = [];
  for (let i = 0; i < n; i++) {
    const t = randomTrip(key, side);
    trips.push(t);
    if (!t.depot) side = t.b > 4.03 ? 'left' : 'right';
  }
  return { start, preload: pick(['move', 'move', 'stand']), trips };
}
function tweak(plan) {
  const p = JSON.parse(JSON.stringify(plan));
  const t = pick(p.trips);
  const r = rng();
  if (t.depot) t.speed = round(Math.max(0.2, Math.min(0.8, t.speed + U(-0.1, 0.1))));
  else if (r < 0.25) t.fx = round(Math.max(7.3, Math.min(8.75, t.fx + U(-0.25, 0.25))));
  else if (r < 0.5) t.speed = round(Math.max(0.2, Math.min(1, t.speed + U(-0.12, 0.12))));
  else if (r < 0.7) t.b = round(Math.max(1.3, Math.min(6.7, t.b + U(-0.6, 0.6))));
  else if (r < 0.8) t.out = t.out === 'trench' ? 'bump' : 'trench';
  else if (r < 0.9) t.home = t.home === 'trench' ? 'bump' : 'trench';
  else if (p.trips.length > 1 && rng() < 0.5) p.trips.pop();
  else p.trips.push(randomTrip(p.trips.length ? 'x' : 'x', pick(['left', 'right'])));
  return p;
}

const best = { ...BEST_AUTOS };
async function score(key, plan) {
  const opps = ROBOT_KEYS.length ? Object.keys(ROBOTS).filter((k) => k !== key) : [];
  const jobs = [];
  for (const o of opps) for (const s of SEEDS) jobs.push(run({ robot: key, plan }, { robot: o, plan: best[o] }, s));
  const res = await Promise.all(jobs);
  return { mean: res.reduce((s, r) => s + r.a, 0) / res.length, min: Math.min(...res.map((r) => r.a)), fouls: res.flatMap((r) => r.fouls) };
}

for (const key of ROBOT_KEYS) {
  const t0 = performance.now();
  let top = { plan: best[key], ...(await score(key, best[key])) };
  console.log(`\n${key} current: ${top.mean.toFixed(1)} (min ${top.min})`);
  const cands = Array.from({ length: N }, () => randomPlan(key));
  const scored = await Promise.all(cands.map(async (plan) => ({ plan, ...(await score(key, plan)) })));
  for (const s of scored) if (!s.fouls.length && s.mean > top.mean) top = s;
  console.log(`${key} after random: ${top.mean.toFixed(1)} (min ${top.min})`);
  for (let round_ = 0; round_ < REFINE / 4; round_++) {
    const kids = await Promise.all(Array.from({ length: 4 }, async () => { const plan = tweak(top.plan); return { plan, ...(await score(key, plan)) }; }));
    for (const k of kids) if (!k.fouls.length && k.mean > top.mean) top = k;
  }
  best[key] = top.plan;
  console.log(`${key} best: ${top.mean.toFixed(1)} (min ${top.min}) in ${((performance.now() - t0) / 60000).toFixed(1)} min`);
  console.log(`  ${key}: ${JSON.stringify(top.plan)},`);
}
for (const w of workers) w.terminate();
