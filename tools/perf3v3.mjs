// Where a headless match's time goes, and a fingerprint of how it played (for checking that a
// speed-up changed nothing).
//   node --import ./tools/node-env.mjs tools/perf3v3.mjs [--3v3 2910,1678,4414:6328,7769,1690]
//        [--1v1 2910:4414] [--seed 1] [--secs 160] [--noprof] [--hash] [--damage] [--top 40] [--json out.json]
// --damage: how worn the FUEL are at the end (js/fuelDamage.js: drawn only), and by what
// --secs: match time to play (the whole match, 160 s with the pre-match, by default)
// --noprof: no per-part timers (the plain speed); --hash: print a hash of every robot's and FUEL's
// state every 0.5 s of the match (bit-identical runs print the same hashes) and the scores.
// Prints ms per simulated second for each part (self time, and inclusive), calls per second, the
// real-time factor, and GC pauses (Node's own: --expose-gc not needed).
import { PerformanceObserver } from 'node:perf_hooks';
import { writeFileSync } from 'node:fs';
import { createWorld, mulberry32 } from './headless.mjs';
import { createGame, stepGame, frameGame } from '../js/game.js';
import { PHYSICS_DT, BLUE, RED } from '../js/constants.js';
import { Profiler } from '../js/profiler.js';
import { damageStats } from '../js/fuelDamage.js';

const args = process.argv.slice(2);
const arg = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const flag = (k) => args.includes('--' + k);
const seed = +arg('seed', 1), secs = +arg('secs', 1e9), top = +arg('top', 45);

let settings, label;
if (arg('1v1')) {
  const [a, b] = arg('1v1').split(':');
  settings = { matchMode: '1v1', alliance: BLUE, robot: a, auto: 'best', start: 'hub', customSide: 'drawn', driver: 'scorer', driverSkill: 'champs', opponent: 'scorer', oppRobot: b, oppSkill: 'champs' };
  label = `1v1 ${a} v ${b}`;
} else {
  const [blue, red] = arg('3v3', '2910,1678,4414:6328,7769,1690').split(':').map((s) => s.split(','));
  const starts = ['leftTrench', 'hub', 'rightBump'];
  const slot = (robot, i) => ({ robot, auto: 'best', start: starts[i % 3], driver: 'scorer', skill: 'champs' });
  settings = { matchMode: '3v3', slots: [...blue, ...red].map(slot) };
  label = `3v3 ${blue.join(',')} v ${red.join(',')}`;
}

// a hash of the state: every robot's body and every FUEL body's position, rotation and velocities
const f64 = new Float64Array(1), u32 = new Uint32Array(f64.buffer);
let H = 0x811c9dc5;
const mix = (x) => { f64[0] = x; H = Math.imul(H ^ u32[0], 16777619) >>> 0; H = Math.imul(H ^ u32[1], 16777619) >>> 0; };
const mixBody = (b) => {
  const t = b.translation(), q = b.rotation(), v = b.linvel(), w = b.angvel();
  mix(t.x); mix(t.y); mix(t.z); mix(q.x); mix(q.y); mix(q.z); mix(q.w); mix(v.x); mix(v.y); mix(v.z); mix(w.x); mix(w.y); mix(w.z);
};
const hashes = [];

const gcs = { n: 0, ms: 0, major: 0 };
const obs = new PerformanceObserver((list) => { for (const e of list.getEntries()) { gcs.n++; gcs.ms += e.duration; if (e.detail?.kind === 4 || e.kind === 4) gcs.major++; } });
obs.observe({ entryTypes: ['gc'] });

if (flag('damage')) { globalThis.__dmgLog = []; globalThis.__hitLog = []; }
const prof = flag('noprof') ? null : new Profiler().install();
const realRandom = Math.random;
const world = await createWorld();
Math.random = mulberry32(seed);
globalThis.SIM_SEED = seed;
const game = createGame(world, { climber: 'none', preload: 8, hp: 'auto', ...settings });
const m = game.match;
let step = 0;
const t0 = performance.now(), cpu0 = process.cpuUsage();
const maxSteps = Math.min(30000, Math.round(secs / PHYSICS_DT));
let frameMs = 0;
while (!m.over && step < maxSteps) {
  if (prof) prof.enter('stepGame');
  stepGame(game, world, PHYSICS_DT);
  if (prof) prof.exit();
  if (++step % 2 === 0) {
    if (prof) prof.enter('frameGame');
    frameGame(game, 2 * PHYSICS_DT);
    if (prof) prof.exit();
  }
  if (flag('hash') && step % 60 === 0) {
    for (const r of game.robots) mixBody(r.body);
    for (const b of world.fuel.balls) mixBody(b.body);
    hashes.push(H.toString(16).padStart(8, '0'));
  }
}
const wall = performance.now() - t0, cpuU = process.cpuUsage(cpu0), cpu = (cpuU.user + cpuU.system) / 1000;
Math.random = realRandom;
await new Promise((r) => setTimeout(r, 50)); // (the last GC entries)
const simS = step * PHYSICS_DT;
const physRobots = game.robots.filter((r) => r.phys).map((r) => r.cfg.key);
console.log(`${label} seed ${seed}: ${simS.toFixed(1)} s simulated in ${(wall / 1000).toFixed(1)} s wall: ${(wall / simS).toFixed(0)} ms per sim s, real-time factor ${(simS * 1000 / wall).toFixed(2)}x, ${(wall / step).toFixed(2)} ms/step (${(2 * wall / step).toFixed(2)} ms per 60 fps frame of 2 steps)`);
console.log(`CPU time: ${(cpu / simS).toFixed(0)} ms per sim s (wall is noisy on a shared machine; CPU time less so)`);
console.log(`physical robots: ${physRobots.join(' ')}; GC: ${gcs.n} pauses, ${(gcs.ms / simS).toFixed(1)} ms per sim s (${gcs.major} major)`);
console.log(`score: blue ${m.total(BLUE)} red ${m.total(RED)}; shots ${game.robots.map((r) => r.cfg.key + ':' + r.stats.shots + '/' + r.stats.intaked).join(' ')}`);
if (flag('damage')) {
  const c = world.fuel.damageCounts(), e = damageStats.events, r = (x) => x.toFixed(2);
  const hist = [0.02, 0.05, 0.1, 0.2, 0.35, 0.5, 0.75, 1, 1.5].map((x) => `>${x}:${world.fuel.balls.filter((b) => b.dmg > x).length}`).join(' ');
  console.log(`damage: ${c.scuffed} scuffed (>0.1), ${c.torn} torn (>0.5), ${c.broken} broken; max ${r(c.max)}; ${hist}`);
  if (arg('dmglog')) writeFileSync(arg('dmglog'), JSON.stringify({ mech: globalThis.__dmgLog.filter((x) => x[1] > 0 || x[2] > 0 || x[3] > 50), hit: globalThis.__hitLog }));
  console.log(`  by cause (total, events): hit ${r(damageStats.hit)}/${e.hit} nip ${r(damageStats.nip)}/${e.nip} rub ${r(damageStats.rub)}/${e.rub} squeeze ${r(damageStats.sq)}/${e.sq}`);
}
if (flag('hash')) {
  console.log(`hash: ${hashes[hashes.length - 1]} (${hashes.length} samples)`);
  if (arg('hashfile')) writeFileSync(arg('hashfile'), hashes.join('\n') + '\n');
}
if (prof) {
  const rows = prof.rows();
  const per = (ms) => (ms / simS).toFixed(1).padStart(7);
  console.log(`${'part'.padEnd(36)} ${'self'.padStart(7)} ${'incl'.padStart(7)}  ms per sim s   calls/s`);
  for (const r of rows.slice(0, top)) console.log(`${r.label.padEnd(36)} ${per(r.self)} ${per(r.incl)}  ${(r.n / simS).toFixed(0).padStart(12)}`);
  // grouped: by kind across robots
  const group = {};
  for (const r of rows) {
    const g = r.label.startsWith('mech ') ? 'mechanisms (all)' : r.label.startsWith('loc') ? 'localization' : r.label.startsWith('nav') ? 'nav' : r.label.startsWith('fuel') ? 'fuel' : r.label === 'ai' || r.label === 'auto' ? 'ai+auto' : r.label;
    group[g] = (group[g] || 0) + r.self;
  }
  console.log('grouped (self ms per sim s): ' + Object.entries(group).sort((a, b) => b[1] - a[1]).map(([k, v]) => `${k} ${(v / simS).toFixed(0)}`).join(', '));
  if (arg('json')) writeFileSync(arg('json'), JSON.stringify({ label, seed, simS, wall, gc: gcs, rows }, null, 1));
}
world.physics.world.free();
process.exit(0);
